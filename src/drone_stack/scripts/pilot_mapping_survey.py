#!/usr/bin/env python3
"""Simulation survey targets from known layout; measured LIO/FCU control only."""
import argparse
import json
import math
import signal
import threading
import time
from pathlib import Path
import rospy
from std_msgs.msg import String,Bool
from std_srvs.srv import SetBool,Trigger
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State,ExtendedState
from drone_stack.srv import LocalGoal,Takeoff,SetNavigationSpeed,CancelNavigation,MapSession,MapSessionRequest


def survey():
    h=1.03;upper=h+3.6;points=[]
    def add(stage,x,y,z,speed=1.5):points.append(dict(stage=stage,xyz=[x,y,z],speed=speed))
    for x,y in [(-6,-4),(-6,4),(6,4),(6,-4),(0,0)]:add('一楼大厅',x,y,h,1.5)
    for x,y in [(-10,0),(-13,-.25),(-17,-.25),(-20,.25),(-24,.25),(-27.1,0)]:add('一楼左走廊',x,y,h)
    def stair_height(y):return h+min(3.6,max(0.,.6*(y-.9)+.075))
    for y in [0.5,1.2,1.95,2.7,3.45,4.2,4.95,5.7,6.45,7.15]:
        add('左楼梯上楼',-27.1,y,stair_height(y),.4)
    for y in [9.5,12.,13.8]:add('二楼左竖向通道',-27.1,y,upper)
    for x,y in [(-24,13.55),(-20,14.05),(-17,14.05),(-13,13.55),(-10,13.55),(-6,13.8)]:add('二楼左走廊',x,y,upper)
    for x,y in [(-6,9.8),(0,9.8),(6,9.8),(6,17.8),(-6,17.8),(-6,13.8),(6,13.8)]:add('二楼大厅',x,y,upper,1.5)
    for x,y in [(10,13.8),(12,13.8),(19,14.05),(27.1,13.8)]:add('二楼右走廊',x,y,upper)
    for y in [11.,8.,7.15]:add('二楼右竖向通道',27.1,y,upper)
    for y in [6.45,5.7,4.95,4.2,3.45,2.7,1.95,1.2,.5,0.]:add('右楼梯下楼',27.1,y,stair_height(y),.4)
    for x,y in [(24,.25),(17,.25),(12,-.25),(6,0),(0,0)]:add('一楼右走廊/返航',x,y,h)
    return points


class Pilot:
    def __init__(self,args):
        self.args=args;self.latest={};self.lock=threading.RLock();self.owned=0;self.target=None;self.index=-1;self.stopped=False
        self.output=Path(args.output);self.output.mkdir(parents=True,exist_ok=True)
        self.summary=dict(passed=False,completed_points=0,map_saved=False)
        for topic,kind,key in [('/mavros/local_position/odom',Odometry,'odom'),('/mavros/state',State,'state'),
                ('/mavros/extended_state',ExtendedState,'extended'),('/drone/map_transform',PoseStamped,'transform'),
                ('/drone/navigation_goal',PoseStamped,'goal'),('/drone/map_alignment_ready',Bool,'aligned')]:
            rospy.Subscriber(topic,kind,lambda m,k=key:self.receive(k,m),queue_size=1)
        for key in ('flight_state','flight_health','flight_error','navigation_terminal_stage'):
            rospy.Subscriber('/drone/'+key,String,lambda m,k=key:self.receive(k,m.data),queue_size=1)
        self.arm=rospy.ServiceProxy('/drone/arm',Trigger);self.takeoff=rospy.ServiceProxy('/drone/takeoff',Takeoff)
        self.goal=rospy.ServiceProxy('/drone/local_goal',LocalGoal);self.speed=rospy.ServiceProxy('/drone/set_navigation_speed',SetNavigationSpeed)
        self.cancel=rospy.ServiceProxy('/drone/cancel_navigation',CancelNavigation);self.land=rospy.ServiceProxy('/drone/land',Trigger)
        self.auth=rospy.ServiceProxy('/drone/set_authorized',SetBool);self.map=rospy.ServiceProxy('/drone/map_session',MapSession)
        threading.Thread(target=self.telemetry,daemon=True).start()

    def receive(self,key,value):
        with self.lock:self.latest[key]=value

    def event(self,kind,**data):
        row=dict(kind=kind,index=self.index,wall_time=time.time(),sim_time=rospy.Time.now().to_sec(),**data)
        with (self.output/'events.jsonl').open('a') as f:f.write(json.dumps(row,ensure_ascii=False)+'\n')
        print(json.dumps(row,ensure_ascii=False),flush=True)

    def telemetry(self):
        while not self.stopped and not rospy.is_shutdown():
            with self.lock:d=dict(self.latest)
            o=d.get('odom');p=o.pose.pose.position if o else None;v=o.twist.twist.linear if o else None
            row=dict(wall_time=time.time(),sim_time=rospy.Time.now().to_sec(),index=self.index,phase=d.get('flight_state'),
                health=d.get('flight_health'),error=d.get('flight_error'),position_odom=[p.x,p.y,p.z] if p else None,
                velocity=[v.x,v.y,v.z] if v else None)
            with (self.output/'telemetry.jsonl').open('a') as f:f.write(json.dumps(row,ensure_ascii=False)+'\n')
            time.sleep(1.)

    def wait(self,predicate,sim_timeout=90,wall_timeout=900):
        wall=time.monotonic();sim=rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.stopped:raise RuntimeError('操作员/代理停止本轮任务')
            if predicate():return
            if self.stopped:raise RuntimeError('操作员/代理停止本轮任务')
            current=rospy.Time.now().to_sec()
            if sim==0 and current>0:sim=current
            if time.monotonic()-wall>wall_timeout or (sim>0 and current-sim>sim_timeout):raise RuntimeError('等待实际飞行状态/目标结果超时')
            if self.latest.get('flight_state')=='FAILSAFE' or self.latest.get('flight_health')=='SEVERE':raise RuntimeError('严重健康保护，停止后续目标')
            time.sleep(.2)
        raise RuntimeError('ROS已退出')

    @staticmethod
    def accepted(reply):
        if not reply.success:raise RuntimeError(reply.message)

    def target_pose(self,xyz):
        t=self.latest['transform'].pose.position;q=self.latest['transform'].pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));dx=xyz[0]-t.x;dy=xyz[1]-t.y
        pose=PoseStamped();pose.header.frame_id='odom';pose.header.stamp=rospy.Time.now();pose.pose.orientation.w=1.
        pose.pose.position.x=math.cos(yaw)*dx+math.sin(yaw)*dy;pose.pose.position.y=-math.sin(yaw)*dx+math.cos(yaw)*dy;pose.pose.position.z=xyz[2]-t.z
        return pose

    def move(self,point):
        self.wait(lambda:self.latest.get('flight_health')=='HEALTHY' and self.latest.get('flight_state')=='HOLD',30)
        pose=self.target_pose(point['xyz']);self.target=pose.pose.position
        previous=self.latest.get('goal');previous_id=previous.header.stamp.to_nsec() if previous else 0
        point=dict(point)
        key='/drone/survey_hall_speed' if '大厅' in point['stage'] else '/drone/survey_stair_speed' if '楼梯' in point['stage'] else '/drone/survey_corridor_speed'
        point['speed']=float(rospy.get_param(key,point['speed']))
        self.accepted(self.speed(point['speed']));self.accepted(self.goal(pose))
        self.event('goal',**point)
        def acknowledgement():
            g=self.latest.get('goal')
            if g is None or g.header.stamp.to_nsec()<=previous_id:return False
            p=g.pose.position
            if math.dist([p.x,p.y,p.z],[self.target.x,self.target.y,self.target.z])>.02:raise RuntimeError('导航已被其它操作接管')
            self.owned=g.header.stamp.to_nsec();return True
        self.wait(acknowledgement,10,60)
        seen=False;hold_since=None
        def arrived():
            nonlocal seen,hold_since
            g=self.latest.get('goal');phase=self.latest.get('flight_state')
            if g and g.header.stamp.to_nsec()!=self.owned:raise RuntimeError('目标身份改变，停止自主建图，不覆盖手动目标')
            if phase=='NAVIGATING':seen=True
            if phase in ('LANDING','DESCENDING','READY'):raise RuntimeError('飞行中止或已进入降落')
            if not seen or phase!='HOLD':hold_since=None;return False
            if hold_since is None:hold_since=rospy.Time.now().to_sec()
            p=self.latest['odom'].pose.pose.position;v=self.latest['odom'].twist.twist.linear
            distance=math.dist([p.x,p.y,p.z],[self.target.x,self.target.y,self.target.z])
            if self.latest.get('navigation_terminal_stage')!='COMPLETED' or distance>.20:
                if rospy.Time.now().to_sec()-hold_since<.5:return False
                raise RuntimeError('导航进入HOLD但未完成目标：'+str(self.latest.get('flight_error','')))
            return math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)<.15
        self.wait(arrived)
        self.event('arrived',**point);self.owned=0;self.summary['completed_points']=self.index+1

    def run(self):
        try:
            if self.args.resume_index is None or self.args.relaunch_from_ground:
                self.wait(lambda:self.latest.get('flight_state')=='READY' and self.latest.get('flight_health')=='HEALTHY' and
                    self.latest.get('aligned') and self.latest['aligned'].data and all(k in self.latest for k in ('transform','odom','state','extended')),180,900)
                s=self.latest['state'];e=self.latest['extended']
                if s.armed or e.landed_state!=ExtendedState.LANDED_STATE_ON_GROUND:raise RuntimeError('本轮自主建图需要确认地面未解锁起点')
                self.accepted(self.auth(True));self.accepted(self.arm());self.event('arm_requested')
                self.wait(lambda:self.latest['state'].armed and self.latest.get('flight_state')=='ARMED',15)
                height=1.03
                if self.args.relaunch_from_ground and not self.args.prebuilt_transit:
                    current_z=self.latest['odom'].pose.pose.position.z+self.latest['transform'].pose.position.z
                    height=.45 if self.args.low_start else survey()[self.args.resume_index]['xyz'][2]-current_z
                    if not .3<=height<=2.:raise RuntimeError('地面恢复起飞高度不安全')
                self.accepted(self.takeoff(height));self.event('takeoff_requested',height=height)
                self.wait(lambda:self.latest.get('flight_state')=='HOLD',30)
                self.event('takeoff_completed')
                if self.args.low_start:
                    self.index=self.args.resume_index-1
                    lower_z=self.latest['odom'].pose.pose.position.z+self.latest['transform'].pose.position.z
                    self.move(dict(stage='二楼恢复低高度移动',xyz=[0.,9.8,lower_z],speed=.3))
                    self.move(dict(stage='二楼恢复升高（楼梯速度）',xyz=[0.,9.8,4.63],speed=.2))
            else:
                self.wait(lambda:self.latest.get('flight_state')=='HOLD' and self.latest.get('flight_health')=='HEALTHY' and all(k in self.latest for k in ('transform','odom','state','extended')),30)
                if not self.latest['state'].armed or self.latest['state'].mode!='OFFBOARD':raise RuntimeError('恢复建图需要实际解锁的健康OFFBOARD HOLD')
                self.event('resumed',resume_index=self.args.resume_index)
            points=survey();
            if self.args.upper_transit:
                for x,y in [(-19.,14.05),(-10.,13.55),(6.,13.8)]:
                    self.move(dict(stage='二楼已扫描通道接续',xyz=[x,y,4.63],speed=1.5))
            if self.args.prebuilt_transit:
                prefix=[dict(stage='加载地图后前往接续点',xyz=[x,y,1.03],speed=1.5) for x,y in [(-10,0),(-19,-.25),(-24,.25),(-27.1,0)]]+points[11:30]+[dict(stage='加载地图后前往接续点',xyz=[6,13.8,4.63],speed=1.5)]
                self.index=-1
                for point in prefix:self.move(point)
            (self.output/'survey_targets.json').write_text(json.dumps(points,ensure_ascii=False,indent=2))
            for self.index,point in enumerate(points):
                if self.index<(self.args.resume_index or 0) or (self.args.low_start and self.index==self.args.resume_index) or (self.args.skip_completed_upper_hall and 30<=self.index<38) or (self.args.fast_transit and (self.index in (21,22,45,46,48,49,51,52,54,56,57) or 24<=self.index<38)):continue
                self.move(point)
            self.accepted(self.land());self.event('land_requested')
            self.wait(lambda:not self.latest['state'].armed and self.latest['extended'].landed_state==ExtendedState.LANDED_STATE_ON_GROUND,90)
            self.wait(lambda:self.latest.get('flight_state')=='READY' and self.latest.get('flight_health')=='HEALTHY',30)
            self.accepted(self.map(MapSessionRequest(action='save',path=str(Path(self.args.map).resolve()))))
            self.summary.update(passed=True,map_saved=True,map_path=str(Path(self.args.map).resolve()))
            self.event('map_saved',path=self.summary['map_path'])
        except Exception as exc:
            self.summary['reason']=str(exc);self.event('stopped',reason=str(exc))
            if self.owned:
                try:self.cancel(self.owned)
                except rospy.ServiceException:pass
        finally:
            self.stopped=True;(self.output/'summary.json').write_text(json.dumps(self.summary,ensure_ascii=False,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--resume-index',type=int);parser.add_argument('--relaunch-from-ground',action='store_true');parser.add_argument('--low-start',action='store_true');parser.add_argument('--prebuilt-transit',action='store_true');parser.add_argument('--skip-completed-upper-hall',action='store_true');parser.add_argument('--fast-transit',action='store_true');parser.add_argument('--upper-transit',action='store_true');parser.add_argument('--map',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();rospy.init_node('agent_mapping_survey',disable_signals=True)
    if not rospy.get_param('/use_sim_time',False) or rospy.get_param('/drone/simulation_profile','')!='building':raise RuntimeError('仅允许当前Gazebo双层仿真')
    p=Pilot(args)
    def stop(signum,frame):p.stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    p.run()
