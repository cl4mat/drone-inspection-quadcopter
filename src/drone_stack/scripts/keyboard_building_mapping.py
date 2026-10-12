#!/usr/bin/env python3
"""Terminal keys -> bounded navigation goals through FM/A*/EGO, not raw velocity."""
import argparse
import json
import math
import select
import sys
import termios
import threading
import time
import tty
from pathlib import Path
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State,ExtendedState
from std_msgs.msg import String
from std_srvs.srv import Trigger,SetBool
from drone_stack.srv import Takeoff,LocalGoal,RotateHeading,SetNavigationSpeed,BuildingCommand,BuildingCommandRequest


HELP='''
请保持此终端焦点，Gazebo窗口用于观察；按键不用回车。
U 授权并解锁    T 起飞（相对1.03m）    L 降落    B 地面锁定
W/S 地图±X     A/D 地图±Y           R/F 上升/下降
Q/E 原地左/右转15度                 空格 悬停/取消移动
+/- 调移动速度  P 状态               M 落地锁定后保存地图/生成路线
H 帮助          X 退出（需未解锁）
每按一次水平移动0.4m，升降0.2m；执行中重复移动键忽略，不累积目标。
移动仍由规划器决定安全路线及机头方向；无操作时保持悬停，不自动降落。
'''


class KeyboardMapping:
    def __init__(self,args):
        self.args=args;self.latest={};self.receipts={};self.lock=threading.RLock()
        self.motion_pending=False;self.motion_seen=False;self.motion_wall=0.;self.speed=.2;self.last_message='等待健康READY'
        self.last_console=0.;self.log=Path(args.log);self.log.parent.mkdir(parents=True,exist_ok=True)
        for topic,kind,key in [('/mavros/local_position/odom',Odometry,'odom'),('/drone/map_transform',PoseStamped,'transform'),
                ('/mavros/state',State,'state'),('/mavros/extended_state',ExtendedState,'extended')]:
            rospy.Subscriber(topic,kind,self.receive,callback_args=key,queue_size=1)
        for topic,key in [('flight_state','phase'),('flight_health','health'),('flight_error','error')]:
            rospy.Subscriber('/drone/'+topic,String,lambda m,k=key:self.receive(m.data,k),queue_size=1)
        self.auth=rospy.ServiceProxy('/drone/set_authorized',SetBool)
        self.services={k:rospy.ServiceProxy('/drone/'+k,t) for k,t in [('arm',Trigger),('hold',Trigger),('land',Trigger),
            ('disarm',Trigger),('takeoff',Takeoff),('local_goal',LocalGoal),('rotate_heading',RotateHeading),('set_navigation_speed',SetNavigationSpeed)]}
        self.finish=rospy.ServiceProxy('/drone/building/command',BuildingCommand)

    def receive(self,value,key):
        with self.lock:
            self.latest[key]=value;self.receipts[key]=time.monotonic()
            if key=='phase' and self.motion_pending:
                if value!='HOLD':self.motion_seen=True
                if self.motion_seen and value=='HOLD' or value in ('READY','LANDING','DESCENDING','FAILSAFE','DISCONNECTED'):
                    self.motion_pending=False

    def position(self):
        with self.lock:
            o=self.latest.get('odom');t=self.latest.get('transform')
        if not o or not t:return None
        p=o.pose.pose.position;tr=t.pose.position;q=t.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return [math.cos(yaw)*p.x-math.sin(yaw)*p.y+tr.x,math.sin(yaw)*p.x+math.cos(yaw)*p.y+tr.y,p.z+tr.z]

    def record(self,key,message):
        with self.log.open('a') as f:f.write(json.dumps(dict(key=key,message=message,wall_time=time.time(),stamp_ns=rospy.Time.now().to_nsec(),
            phase=self.latest.get('phase'),health=self.latest.get('health'),position_map=self.position()),ensure_ascii=False)+'\n')

    def call(self,name,*values):
        reply=self.services[name](*values)
        if not reply.success:raise RuntimeError(reply.message)
        return reply.message

    def key(self,key):
        key=key.lower()
        if key=='h':print('\n'+HELP,flush=True);return
        if key=='p':self.last_message=str(self.position());return
        if key=='x':
            state=self.latest.get('state')
            if state is not None and state.armed:raise RuntimeError('请先降落并确认锁定，再退出')
            return 'EXIT'
        if key=='u':
            reply=self.auth(True)
            if not reply.success:raise RuntimeError(reply.message)
            self.last_message=self.call('arm')
        elif key=='t':self.last_message=self.call('takeoff',1.03)
        elif key=='l':self.last_message=self.call('land')
        elif key=='b':self.last_message=self.call('disarm')
        elif key==' ':self.last_message=self.call('hold');self.motion_pending=False
        elif key in ('+','=','-'):
            speed=max(.1,min(1.,round(self.speed+(-.1 if key=='-' else .1),2)))
            self.last_message=self.call('set_navigation_speed',speed);self.speed=speed
        elif key=='m':
            reply=self.finish(BuildingCommandRequest(action='finish',path=str(Path(self.args.map).resolve())))
            if not reply.success:raise RuntimeError(reply.message)
            self.last_message=reply.message
        elif key in 'wasdrfqe':
            with self.lock:
                if self.motion_pending or self.latest.get('phase')!='HOLD':return
                if self.latest.get('health')!='HEALTHY' or time.monotonic()-self.receipts.get('odom',0)>1.:
                    raise RuntimeError('定位/飞控未健康，不发送移动指令')
                self.motion_pending=True;self.motion_seen=False;self.motion_wall=time.monotonic()
            try:
                if key in 'qe':self.last_message=self.call('rotate_heading',15. if key=='q' else -15.)
                else:
                    # Fixed map axes avoid repeated S presses flipping the body-relative direction.
                    dx,dy,dz={'w':(.4,0,0),'s':(-.4,0,0),'a':(0,.4,0),'d':(0,-.4,0),'r':(0,0,.2),'f':(0,0,-.2)}[key]
                    with self.lock:o=self.latest['odom'];t=self.latest['transform']
                    q=t.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    goal=PoseStamped();goal.header.frame_id='odom';goal.header.stamp=rospy.Time.now();goal.pose.orientation.w=1.
                    p=o.pose.pose.position;goal.pose.position.x=p.x+math.cos(yaw)*dx+math.sin(yaw)*dy
                    goal.pose.position.y=p.y-math.sin(yaw)*dx+math.cos(yaw)*dy;goal.pose.position.z=p.z+dz
                    self.call('set_navigation_speed',self.speed);self.last_message=self.call('local_goal',goal)
            except Exception:
                self.motion_pending=False;raise
        else:return
        self.record(key,self.last_message)

    def run(self):
        if not sys.stdin.isatty():raise RuntimeError('键盘控制需要交互终端，请在本机终端运行启动脚本')
        print(HELP,flush=True);terminal=sys.stdin.fileno();settings=termios.tcgetattr(terminal);last_record=0.
        try:
            tty.setcbreak(terminal)
            while not rospy.is_shutdown():
                if select.select([sys.stdin],[],[],.1)[0]:
                    key=sys.stdin.read(1)
                    if not key:break
                    if key=='\x1b':
                        sequence=''
                        for _ in range(16):
                            if not select.select([sys.stdin],[],[],.015)[0]:break
                            part=sys.stdin.read(1);sequence+=part
                            if part.isalpha() or part=='~':break
                        key={'[A':'w','[B':'s','[C':'d','[D':'a'}.get(sequence,'')
                        if not key:continue
                    try:
                        if self.key(key)=='EXIT':return
                    except (RuntimeError,rospy.ServiceException,KeyError) as exc:
                        self.last_message=str(exc);self.record(key,self.last_message)
                now=time.monotonic()
                if now-last_record>=1.:
                    self.record('telemetry','');last_record=now
                if now-self.last_console>=.5:
                    self.last_console=now
                    print('\r\033[K%s | %s | %.1fm/s | %s'%(self.latest.get('phase','等待启动'),self.latest.get('health','UNKNOWN'),self.speed,self.last_message),end='',flush=True)
        except KeyboardInterrupt:
            try:
                if self.latest.get('state') and self.latest['state'].armed:self.call('hold')
            except (RuntimeError,rospy.ServiceException):pass
        finally:termios.tcsetattr(terminal,termios.TCSADRAIN,settings);print()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--map',required=True);parser.add_argument('--log',required=True)
    args=parser.parse_args();rospy.init_node('keyboard_building_mapping',disable_signals=True)
    if not rospy.get_param('/use_sim_time',False) or rospy.get_param('/drone/simulation_profile','')!='building':
        raise RuntimeError('键盘入口只允许双层Gazebo仿真')
    KeyboardMapping(args).run()
