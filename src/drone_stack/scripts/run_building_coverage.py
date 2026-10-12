#!/usr/bin/env python3
"""Simulation-only acceptance runner; navigation remains owned by FM/A*/EGO."""
import argparse
import json
import math
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State,ExtendedState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from drone_stack.srv import MapSession,MapSessionRequest,BuildingCommand,BuildingCommandRequest
from building_route import digest,atomic_json


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--map',required=True);parser.add_argument('--output',required=True);parser.add_argument('--reuse-running',action='store_true');parser.add_argument('--resume',action='store_true');parser.add_argument('--airborne-start',action='store_true');parser.add_argument('--wall-timeout',type=float,default=0.)
    args=parser.parse_args();rospy.init_node('building_coverage_acceptance')
    if not rospy.get_param('/use_sim_time',False) or not rospy.get_param('/drone/split_world_launcher',False) or rospy.get_param('/drone/simulation_profile','')!='building':
        raise RuntimeError('此自动验收只允许分离启动的双层Gazebo仿真，禁止用于实机')
    task=json.loads(Path(args.map+'.inspection.json').read_text())
    if task.get('state')!='READY' or task.get('map_digest')!=digest(args.map) or len(task['topology']['floors'])<2:
        raise RuntimeError('需要来自真实建图且验证连通的双层地图/整栋任务')
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True);latest={};receipts={}
    def receive(m,key):
        latest[key]=json.loads(m.data) if isinstance(m,String) else m;receipts[key]=time.monotonic()
    for topic,kind,key in [('/drone/building/status',String,'building'),('/drone/map_session_status',String,'map'),
            ('/drone/flight_state',String,'phase'),('/drone/flight_health',String,'health')]:
        if key in ('phase','health'):
            rospy.Subscriber(topic,kind,lambda m,k=key:latest.update({k:m.data}),queue_size=1)
        else:rospy.Subscriber(topic,kind,receive,callback_args=key,queue_size=1)
    for topic,kind,key in [('/mavros/state',State,'fcu'),('/mavros/extended_state',ExtendedState,'extended'),
            ('/mavros/local_position/odom',Odometry,'odom'),('/drone/map_transform',PoseStamped,'transform')]:
        rospy.Subscriber(topic,kind,receive,callback_args=key,queue_size=1)
    session=rospy.ServiceProxy('/drone/map_session',MapSession);command=rospy.ServiceProxy('/drone/building/command',BuildingCommand)
    auth=rospy.ServiceProxy('/drone/set_authorized',SetBool)
    started=False;complete=False;initial=time.monotonic();last_state='';summary=dict(passed=False,map_digest=task['map_digest'])
    try:
        rospy.wait_for_service('/drone/map_session',timeout=30)
        if args.reuse_running:
            while not latest.get('map',{}).get('ready'):
                if time.monotonic()-initial>45:raise RuntimeError('运行地图未就绪')
                time.sleep(.2)
            if latest['map'].get('map_context',{}).get('digest')!=task['map_digest']:raise RuntimeError('运行地图摘要不匹配')
        else:
            while latest.get('map',{}).get('mode')!='WAITING_POSE':
                if time.monotonic()-initial>45:raise RuntimeError('等待地图初始位姿阶段超时')
                time.sleep(.2)
            reply=session(MapSessionRequest(action='origin',path=str(Path(args.map).resolve())))
            if not reply.success:raise RuntimeError(reply.message)
        stable=None
        while True:
            ready=latest.get('phase')==('HOLD' if args.resume or args.airborne_start else 'READY') and latest.get('health')=='HEALTHY' and latest.get('map',{}).get('ready')
            if ready:
                if stable is None:stable=time.monotonic()
                if time.monotonic()-stable>=3.:break
            else:stable=None
            if time.monotonic()-initial>600:raise RuntimeError('定位/飞控/地图启动检查超时')
            time.sleep(.2)
        reply=auth(True)
        if not reply.success:raise RuntimeError(reply.message)
        if not args.resume:
            reply=command(BuildingCommandRequest(action='load',path=str(Path(args.map).resolve())))
            if not reply.success:raise RuntimeError(reply.message)
        command_sent_ns=rospy.Time.now().to_nsec()
        reply=command(BuildingCommandRequest(action='resume' if args.resume else 'start'))
        if not reply.success:raise RuntimeError(reply.message)
        mission_id=json.loads(reply.task_json)['mission_id']
        started=True;sim_start=rospy.Time.now().to_sec();deadline=initial+args.wall_timeout if args.wall_timeout>0 else float('inf')
        with (output/'samples.jsonl').open('w') as samples:
            while not rospy.is_shutdown():
                status=latest.get('building',{});state=status.get('state','')
                if status.get('mission_id')!=mission_id or int(status.get('stamp_ns',0))<=command_sent_ns:
                    if time.monotonic()>deadline:raise RuntimeError('未收到本次整栋任务状态')
                    time.sleep(.2);continue
                if state!=last_state:
                    print(state,status.get('message',''),flush=True);last_state=state
                row=dict(wall_time=time.time(),sim_time=rospy.Time.now().to_sec(),status=status,phase=latest.get('phase'),health=latest.get('health'))
                if 'odom' in latest:
                    p=latest['odom'].pose.pose.position;row['position_odom']=[p.x,p.y,p.z]
                samples.write(json.dumps(row,ensure_ascii=False)+'\n');samples.flush()
                if state=='COMPLETED':complete=True;break
                if state in ('PAUSED','CANCELED','DRAFT'):
                    raise RuntimeError('覆盖未完成：'+status.get('message',state))
                if time.monotonic()-receipts.get('building',0)>3:raise RuntimeError('整栋管理器失联')
                if time.monotonic()>deadline:raise RuntimeError('完整巡检超过墙钟预算')
                time.sleep(.5)
        fcu=latest.get('fcu');extended=latest.get('extended');odom=latest.get('odom');transform=latest.get('transform')
        if not fcu or not extended or fcu.armed or extended.landed_state!=ExtendedState.LANDED_STATE_ON_GROUND:raise RuntimeError('没有实际落地锁定确认')
        p=odom.pose.pose.position;t=transform.pose.position;q=transform.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        home_error=math.hypot(math.cos(yaw)*p.x-math.sin(yaw)*p.y+t.x,math.sin(yaw)*p.x+math.cos(yaw)*p.y+t.y)
        if home_error>.25:raise RuntimeError('降落位置未返回地图原点25cm范围')
        summary.update(passed=True,home_error_xy_m=home_error,sim_duration_s=rospy.Time.now().to_sec()-sim_start,
            passed_segments=latest['building']['segment'],total_segments=latest['building']['total_segments'],landed=True,armed=False)
    except Exception as exc:
        summary['reason']=str(exc);print(str(exc),file=sys.stderr,flush=True)
        if started and not complete:
            try:command(BuildingCommandRequest(action='pause'))
            except rospy.ServiceException:pass
    finally:atomic_json(output/'summary.json',summary)
    return 0 if summary['passed'] else 1


if __name__=='__main__':sys.exit(main())
