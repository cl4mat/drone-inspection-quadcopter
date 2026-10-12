#!/usr/bin/env python3
"""Headless simulation teleoperation through existing safe flight services."""
import argparse
import json
import math
import shlex
import sys
import time
import threading
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State,ExtendedState
from std_msgs.msg import String
from std_srvs.srv import Trigger,SetBool
from drone_stack.srv import Takeoff,LocalGoal,BuildingCommand,BuildingCommandRequest,SetNavigationSpeed


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--map',required=True);parser.add_argument('--log',required=True)
    args=parser.parse_args();rospy.init_node('building_mapping_teleop')
    if not rospy.get_param('/use_sim_time',False) or rospy.get_param('/drone/simulation_profile','')!='building':
        raise RuntimeError('此入口仅允许双层Gazebo仿真，不能用于实机')
    latest={};log=Path(args.log);log.parent.mkdir(parents=True,exist_ok=True)
    for topic,kind,key in [('/mavros/local_position/odom',Odometry,'odom'),('/drone/map_transform',PoseStamped,'transform'),
            ('/mavros/state',State,'fcu'),('/mavros/extended_state',ExtendedState,'extended')]:
        rospy.Subscriber(topic,kind,lambda m,k=key:latest.update({k:m}),queue_size=1)
    for topic,key in [('flight_health','health'),('flight_state','phase'),('flight_error','error')]:
        rospy.Subscriber('/drone/'+topic,String,lambda m,k=key:latest.update({k:m.data}),queue_size=1)
    def map_position():
        p=latest['odom'].pose.pose.position;t=latest['transform'].pose.position;q=latest['transform'].pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return [math.cos(yaw)*p.x-math.sin(yaw)*p.y+t.x,math.sin(yaw)*p.x+math.cos(yaw)*p.y+t.y,p.z+t.z]
    def record(command,result):
        row=dict(stamp_ns=rospy.Time.now().to_nsec(),command=command,result=result,phase=latest.get('phase'),health=latest.get('health'))
        if 'odom' in latest and 'transform' in latest:row['position_map']=map_position()
        with log.open('a') as f:f.write(json.dumps(row,ensure_ascii=False)+'\n')
    def telemetry():
        while not rospy.is_shutdown():
            try:
                row=dict(stamp_ns=rospy.Time.now().to_nsec(),wall_time=time.time(),phase=latest.get('phase'),health=latest.get('health'))
                if 'odom' in latest and 'transform' in latest:row['position_map']=map_position()
                with Path(str(log)+'.telemetry.jsonl').open('a') as f:f.write(json.dumps(row,ensure_ascii=False)+'\n')
            except OSError:pass
            time.sleep(1.)
    threading.Thread(target=telemetry,daemon=True).start()
    auth=rospy.ServiceProxy('/drone/set_authorized',SetBool);arm=rospy.ServiceProxy('/drone/arm',Trigger)
    takeoff=rospy.ServiceProxy('/drone/takeoff',Takeoff);goal=rospy.ServiceProxy('/drone/local_goal',LocalGoal)
    hold=rospy.ServiceProxy('/drone/hold',Trigger);land=rospy.ServiceProxy('/drone/land',Trigger)
    speed=rospy.ServiceProxy('/drone/set_navigation_speed',SetNavigationSpeed)
    finish=rospy.ServiceProxy('/drone/building/command',BuildingCommand)
    rospy.wait_for_service('/drone/arm',timeout=60)
    deadline=time.monotonic()+600
    while latest.get('phase')!='READY' or latest.get('health')!='HEALTHY' or 'transform' not in latest:
        if time.monotonic()>deadline:raise RuntimeError('定位/飞控尚未就绪，未发送飞行指令')
        time.sleep(.2)
    print('遥控建图就绪；arm | takeoff [相对高度，默认1.03] | goto X Y Z（地图坐标） | speed 0.1..1 | hold | land | status | finish | quit',flush=True)
    try:
        while not rospy.is_shutdown():
            line=input('mapping> ').strip()
            if not line:continue
            words=shlex.split(line);action=words[0]
            try:
                if action=='status':
                    result=json.dumps(dict(phase=latest.get('phase'),health=latest.get('health'),position_map=map_position(),error=latest.get('error')),ensure_ascii=False)
                elif action=='arm' and len(words)==1:
                    reply=auth(True)
                    if not reply.success:raise RuntimeError(reply.message)
                    reply=arm();result=reply.message
                    if not reply.success:raise RuntimeError(result)
                elif action=='takeoff' and len(words)<=2:
                    reply=takeoff(float(words[1]) if len(words)==2 else 1.03);result=reply.message
                    if not reply.success:raise RuntimeError(result)
                elif action=='speed' and len(words)==2:
                    reply=speed(float(words[1]));result=reply.message
                    if not reply.success:raise RuntimeError(result)
                elif action=='goto' and len(words)==4:
                    xyz=list(map(float,words[1:]))
                    if not all(math.isfinite(v) for v in xyz):raise ValueError('坐标需为有限值')
                    t=latest['transform'].pose.position;q=latest['transform'].pose.orientation
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));dx=xyz[0]-t.x;dy=xyz[1]-t.y
                    pose=PoseStamped();pose.header.frame_id='odom';pose.header.stamp=rospy.Time.now();pose.pose.orientation.w=1.
                    pose.pose.position.x=math.cos(yaw)*dx+math.sin(yaw)*dy;pose.pose.position.y=-math.sin(yaw)*dx+math.cos(yaw)*dy;pose.pose.position.z=xyz[2]-t.z
                    reply=goal(pose);result=reply.message
                    if not reply.success:raise RuntimeError(result)
                elif action in ('hold','land') and len(words)==1:
                    reply=(hold if action=='hold' else land)();result=reply.message
                    if not reply.success:raise RuntimeError(result)
                elif action=='finish' and len(words)==1:
                    reply=finish(BuildingCommandRequest(action='finish',path=str(Path(args.map).resolve())));result=reply.message
                    if not reply.success:raise RuntimeError(result)
                elif action=='quit' and len(words)==1:
                    if latest.get('fcu') and latest['fcu'].armed:raise RuntimeError('请先land并确认落地锁定后再退出建图入口')
                    record(line,'退出');return
                else:raise ValueError('未知命令/参数数量，请使用上述遥控命令')
                record(line,result);print(result,flush=True)
            except (ValueError,RuntimeError,rospy.ServiceException) as exc:
                record(line,str(exc));print(str(exc),file=sys.stderr,flush=True)
    except (EOFError,KeyboardInterrupt):
        if latest.get('fcu') and latest['fcu'].armed:
            try:hold()
            except rospy.ServiceException:pass
        record('terminal_closed','停止后续遥控；已请求HOLD，已有降落保护继续')


if __name__=='__main__':main()
