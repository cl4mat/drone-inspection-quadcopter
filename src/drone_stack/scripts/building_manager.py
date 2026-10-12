#!/usr/bin/env python3
"""Measured-map building mission: explicit start, sequential control, resumable HOLD."""
import sys
import copy
import json
import math
import threading
import time
import uuid
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,Path as RosPath
from mavros_msgs.msg import State,ExtendedState
from sensor_msgs.msg import Image
from std_msgs.msg import String,Bool,Header
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
from drone_stack.srv import (BuildingCommand,BuildingCommandResponse,BuildingFlightAction,
    ExecuteInspectionRoute,ExecuteInspectionRouteRequest,MapSession,MapSessionRequest,LocalGoal,SetNavigationSpeed)
from building_route import build,digest,atomic_json,route_window,task_digest
from safe_global_path import SafeGlobalPath,ReferenceSafetyCells
from flight_manager import FlightManager
from voxel_map_client import VoxelMapClient


ACTIVE={'ARMING','TAKEOFF','TRANSITING','INSPECTING','RETURNING','LANDING'}


class StairTubeCells:
    """Replans remain near the measured stair profile, never shortcut the stairwell."""
    def __init__(self,occupied,route,geometry,res):
        self.occupied=occupied;self.allowed=set();radius=math.ceil(.45/res)
        centres=set()
        for a,b in zip(route,route[1:]):centres.update(FlightManager.segment_cells(geometry,a,b))
        for x,y,z in centres:
            for dx in range(-radius,radius+1):
                for dy in range(-radius,radius+1):
                    for dz in range(-radius,radius+1):
                        if dx*dx+dy*dy+dz*dz<=radius*radius:self.allowed.add((x+dx,y+dy,z+dz))
    def __contains__(self,cell):return tuple(cell) not in self.allowed or cell in self.occupied


class BuildingManager:
    def __init__(self):
        self.lock=threading.RLock();self.serial=0;self.state='IDLE';self.reason='尚未保存整栋任务'
        self.mission_id='';self.owner='';self.ai={};self.ai_seen=0.
        self.task=None;self.context={};self.mode='';self.map_ready=False;self.phase='';self.health='UNKNOWN';self.authorized=False
        self.odom=None;self.transform=None;self.fcu=State();self.extended=ExtendedState();self.image=None
        self.index=0;self.progress=0.;self.task_id=0;self.regional_reserved=False;self.receipts={};self.last_reference=0.;self.last_record_wall=0.;self.phase_since=0.
        self.approaching=True;self.last_position=None;self.last_advance=0.;self.record_path=None;self.autoload_key='';self.repair=None
        self.publisher=rospy.Publisher('/drone/building/status',String,queue_size=1,latch=True)
        self.exclusive=rospy.Publisher('/drone/building/exclusive',Bool,queue_size=1,latch=True)
        self.markers=rospy.Publisher('/drone/building/markers',MarkerArray,queue_size=1,latch=True)
        self.flight=rospy.ServiceProxy('/drone/building/flight_action',BuildingFlightAction)
        self.point_sent=False;self.point_target=None;self.point_command_ns=0
        self.point_goal=rospy.ServiceProxy('/drone/building/local_goal',LocalGoal)
        self.point_speed=rospy.ServiceProxy('/drone/set_navigation_speed',SetNavigationSpeed)
        rospy.Subscriber('/drone/navigation_terminal_stage',String,lambda m:self.receive('terminal_stage',m.data),queue_size=1)
        rospy.Subscriber('/drone/navigation_goal',PoseStamped,lambda m:self.receive('navigation_goal',m),queue_size=1)
        self.execute=rospy.ServiceProxy('/drone/execute_inspection_route',ExecuteInspectionRoute)
        self.map_service=rospy.ServiceProxy('/drone/map_session',MapSession)
        for topic,kind,key in [('/mavros/local_position/odom',Odometry,'odom'),('/drone/map_transform',PoseStamped,'transform'),
                ('/mavros/state',State,'fcu'),('/mavros/extended_state',ExtendedState,'extended'),('/drone/down/image_raw',Image,'image')]:
            rospy.Subscriber(topic,kind,lambda m,k=key:self.receive(k,m),queue_size=1)
        for topic,key in [('flight_state','phase'),('flight_health','health')]:
            rospy.Subscriber('/drone/'+topic,String,lambda m,k=key:self.receive(k,m.data),queue_size=1)
        rospy.Subscriber('/drone/map_session_status',String,self.on_map,queue_size=1)
        rospy.Subscriber('/drone/authorized',Bool,lambda m:self.receive('authorized',m.data),queue_size=1)
        rospy.Subscriber('/drone/inspection/exclusive',Bool,lambda m:setattr(self,'regional_reserved',m.data),queue_size=1)
        rospy.Subscriber('/drone/ai/status',String,self.ai_status,queue_size=1)
        rospy.Service('/drone/building/command',BuildingCommand,self.command)
        self.planner=SafeGlobalPath.__new__(SafeGlobalPath)
        p=self.planner;p.origin=[-rospy.get_param('/ego_planner_node/grid_map/map_size_x',40.)/2,-rospy.get_param('/ego_planner_node/grid_map/map_size_y',32.)/2,float(rospy.get_param('/ego_planner_node/grid_map/ground_height',-4.5))]
        p.res=float(rospy.get_param('/ego_planner_node/grid_map/resolution',.1));p.shape=[math.ceil(rospy.get_param('/ego_planner_node/grid_map/map_size_'+a,s)/p.res) for a,s in zip('xyz',[40.,32.,14.])]
        p.lower=list(rospy.get_param('/drone/navigation_min_xyz',[-12.,-10.,.5]));p.upper=list(rospy.get_param('/drone/navigation_max_xyz',[12.,10.,6.6]));p.step=float(rospy.get_param('/drone/global_search_resolution_m',.30));p.max_search_nodes=15000
        p.geometry=SimpleNamespace(map_origin=p.origin,map_resolution=p.res)
        self.map_epoch=0;self.map_revision=0;self.occupied=None;self.map_stamp=rospy.Time(0)
        self.voxel=None;self.voxel_started_wall=0.
        threading.Thread(target=self.loop,daemon=True).start()
        # Independent heartbeat: offline map extraction must not delay flight supervision.
        threading.Thread(target=self.status_loop,daemon=True).start()

    def ai_status(self,message):
        try:self.ai=json.loads(message.data);self.ai_seen=time.monotonic()
        except ValueError:pass

    def owner_valid(self,owner):
        if not owner:return True
        a=self.ai
        return owner=='ai:'+str(a.get('session_id',''))+'|'+str(a.get('request_id','')) and bool(a.get('busy') and a.get('control_authorized') and a.get('client_alive')) and a.get('state') in ('PARSING','EXECUTING') and time.monotonic()-self.ai_seen<=3.

    def receive(self,key,value):
        with self.lock:
            if key=='phase' and value!=self.phase:self.phase_since=rospy.Time.now().to_sec()
            setattr(self,key,value);self.receipts[key]=time.monotonic()

    def activate_voxel(self):
        if self.voxel is None:
            p=self.planner
            self.voxel_started_wall=time.monotonic()
            self.voxel=VoxelMapClient(p.origin,p.res,p.shape,self.on_voxel,lambda:self.receive('map_stamp',rospy.Time(0)))

    def on_voxel(self,occupied,free,stamp,epoch,revision):
        with self.lock:
            if epoch<self.map_epoch or epoch==self.map_epoch and revision<=self.map_revision and self.occupied is not None:return
            self.map_epoch=epoch;self.map_revision=revision;self.occupied=occupied;self.map_stamp=stamp

    def on_map(self,message):
        data=json.loads(message.data)
        with self.lock:
            context=data.get('map_context',{})
            if context.get('digest')!=self.context.get('digest') and self.state in ACTIVE|{'PAUSED'}:
                self.pause('地图内容已切换，任务失效');self.task=None
            self.mode=data.get('mode','');self.map_ready=data.get('ready',False);self.context=context
            key=context.get('digest','')
            if key and key!=self.autoload_key and self.state not in ACTIVE|{'GENERATING','PAUSED'}:
                self.autoload_key=key
                try:self.load_task(context['path'])
                except Exception as exc:self.task=None;self.set_state('IDLE',str(exc))

    def set_state(self,state,reason):
        self.state=state;self.reason=reason;self.state_since=rospy.Time.now().to_sec();self.record('event',dict(state=state,message=reason,segment=self.index));rospy.loginfo('Building %s: %s',state,reason);self.publish_status()

    def load_task(self,path):
        source=Path(str(path)+'.inspection.json')
        if not source.is_file():raise ValueError('此地图尚无整栋巡检任务，请先完成建图生成')
        if source.stat().st_size>32*1024*1024:raise ValueError('整栋任务文件过大')
        task=json.loads(source.read_text())
        if task.get('version')!=1 or task.get('map_digest')!=digest(path):raise ValueError('任务版本/地图摘要不匹配')
        if task.get('state')!='READY':raise ValueError('任务草稿不可执行：'+task.get('message','需补建图'))
        if task.get('task_digest')!=task_digest(task):raise ValueError('任务内容已被改动，请修正拓扑后重新生成')
        if task.get('execution_mode','continuous')!=task['topology'].get('execution_mode','continuous'):raise ValueError('执行模式与校验拓扑不一致')
        segments=task.get('segments',[])
        if not 1<=len(segments)<=20000:raise ValueError('任务段数无效')
        for s in segments:
            points=np.asarray(s.get('points',[]),float)
            if s.get('kind') not in ('hall','corridor','stair','home') or points.ndim!=2 or points.shape[1]!=3 or not 2<=len(points)<=100000 or not np.isfinite(points).all() or not .1<=s['speed']<=float(rospy.get_param('/drone/max_navigation_speed_mps',2.)):raise ValueError('任务路径/速度无效')
        task['map_path']=str(Path(path).resolve());self.autoload_key=task['map_digest']
        self.base_task=copy.deepcopy(task);self.task=task;self.index=0;self.progress=0.;self.set_state('READY','整栋任务已加载，点击一键开始前请授权');self.preview()

    def ground(self):
        return self.extended.landed_state==ExtendedState.LANDED_STATE_ON_GROUND and not self.fcu.armed

    def command(self,req):
        with self.lock:
            try:
                if req.action in ('finish','regenerate'):
                    if self.state in ACTIVE|{'GENERATING','PAUSED'} or not self.ground():raise ValueError('请先落地、锁定并结束任务再生成地图路线')
                    correction=json.loads(req.topology_json) if req.topology_json else None
                    if req.action=='finish':
                        if self.mode!='ONLINE':raise ValueError('完成建图仅用于在线建图模式')
                        result=self.map_service(MapSessionRequest(action='save',path=req.path))
                        if not result.success:raise ValueError(result.message)
                    self.serial+=1;serial=self.serial;self.set_state('GENERATING','已保存地图，正在识别楼层和生成路线')
                    threading.Thread(target=self.generate,args=(req.path,correction,serial),daemon=True).start()
                elif req.action=='load':
                    if self.state in ACTIVE|{'PAUSED','GENERATING'}:raise ValueError('请先取消当前任务')
                    if req.path!=self.context.get('path'):raise ValueError('请先在综合页加载同一地图')
                    self.load_task(req.path)
                elif req.action=='start':
                    if not self.owner_valid(req.owner):raise ValueError('AI执行授权/任务代次已失效')
                    self.owner=req.owner
                    if self.state!='READY' or not self.task:raise ValueError('请先生成/加载有效整栋任务')
                    self.check_start();self.activate_voxel();self.prepare_entry();self.mission_id=uuid.uuid4().hex;self.task['mission_id']=self.mission_id;self.index=0;self.progress=0.;self.task_id=0;self.point_sent=False;self.approaching=True;self.last_position=None;self.last_advance=rospy.Time.now().to_sec()
                    self.serial+=1;self.exclusive.publish(Bool(True))
                    self.record_path=Path(self.task['map_path']+'.mission_progress.json')
                    self.set_state('ARMING' if self.ground() else 'TRANSITING','整栋任务开始，已授权后依次解锁/起飞/巡检/返航/降落')
                    self.arm_sent=False
                elif req.action=='pause':
                    if self.state not in ACTIVE:raise ValueError('当前没有执行中的任务')
                    self.pause('操作员暂停，保留已通过任务段')
                elif req.action=='resume':
                    if not self.owner_valid(req.owner):raise ValueError('AI执行授权已失效')
                    self.owner=req.owner
                    if self.state!='PAUSED':raise ValueError('当前没有暂停的任务')
                    self.check_start(airborne=True);self.activate_voxel();self.serial+=1;self.task_id=0;self.point_sent=False;self.approaching=self.task.get('execution_mode')!='waypoints';self.last_position=None;self.last_advance=rospy.Time.now().to_sec()
                    self.set_state('TRANSITING','恢复剩余任务，检查最新障碍后重接路线')
                elif req.action=='cancel':
                    if req.owner and req.owner!=self.owner:raise ValueError('任务已由操作员接管，没有取消新任务')
                    if self.state in ACTIVE:self.pause('操作员取消')
                    self.serial+=1;self.task_id=0;self.set_state('CANCELED','任务已取消，保留实验记录');self.exclusive.publish(Bool(False))
                elif req.action!='status':raise ValueError('未知整栋巡检操作')
                return BuildingCommandResponse(True,self.reason,json.dumps(self.task or {},ensure_ascii=False))
            except Exception as exc:return BuildingCommandResponse(False,str(exc),'')

    def prepare_entry(self):
        self.task=copy.deepcopy(getattr(self,'base_task',self.task))
        if getattr(self,'odom',None) is None:return
        current_z=self.odom.pose.pose.position.z+self.transform.pose.position.z
        floors=[f for f in self.task['topology']['floors'] if f['ground_z']<current_z-.05]
        if floors and max(floors,key=lambda f:f['ground_z'])['id']>0:
            # A manual upper-floor birth first follows the saved descent to the
            # canonical entry; no vertical shortcut down the open stairwell.
            suffix=[]
            for segment in reversed(self.task['segments']):
                if segment['kind']!='stair':break
                suffix.insert(0,copy.deepcopy(segment))
            if not suffix:raise RuntimeError('地图缺少预存下楼连接，不能从二楼连接巡检入口')
            self.task['segments']=suffix+self.task['segments']
            self.task['expected_duration_s']+=sum(sum(math.dist(a,b) for a,b in zip(s['points'],s['points'][1:]))/s['speed'] for s in suffix)

    def check_start(self,airborne=False):
        if self.regional_reserved:raise ValueError('区域巡检正在占用控制权限')
        if not self.map_ready or self.mode!='PRIOR_NAV' or not self.task or self.task['map_digest']!=self.context.get('digest'):raise ValueError('一键巡检只支持匹配的预建地图')
        if digest(self.context['path'])!=self.context['digest']:raise ValueError('地图文件已变化，请重新加载')
        if not self.authorized or self.health!='HEALTHY':raise ValueError('请先授权并等待定位/飞控健康')
        if self.odom is None or time.monotonic()-self.receipts.get('odom',0)>1 or self.transform is None:raise ValueError('当前位姿/地图变换过期')
        if self.phase=='READY' and not self.ground():raise ValueError('地面启动需确认落地且未解锁')
        if self.phase not in (('HOLD',) if airborne else ('READY','HOLD')):raise ValueError('需在地面READY或空中健康HOLD开始')

    def generate(self,path,correction,serial):
        try:
            build(path,correction)
            with self.lock:
                if serial==self.serial:self.load_task(path)
        except Exception as exc:
            with self.lock:
                if serial==self.serial:self.task=None;self.set_state('DRAFT','地图已保留，路线需补建图/修正：'+str(exc))

    def rotation(self):
        q=self.transform.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])

    def to_odom(self,points):
        t=self.transform.pose.position;return (np.asarray(points)-[t.x,t.y,t.z])@self.rotation()

    def position(self):
        p=self.odom.pose.pose.position;return np.array([p.x,p.y,p.z])

    def call_flight(self,action,height=0.):
        reply=self.flight(action,height)
        if not reply.success:raise RuntimeError(reply.message)

    def pause(self,reason):
        self.serial+=1;self.task_id=0;self.repair=None;self.set_state('PAUSED',reason)
        try:self.call_flight('hold')
        except (RuntimeError,rospy.ServiceException):pass
        self.save_progress()

    def record(self,kind,data):
        if not self.record_path:return
        try:
            payload=dict(stamp_ns=rospy.Time.now().to_nsec(),wall_time=time.time(),mission_id=getattr(self,'mission_id',''),**data)
            with Path(str(self.record_path)+'.'+kind+'.jsonl').open('a') as f:f.write(json.dumps(payload,ensure_ascii=False)+'\n')
        except OSError as exc:rospy.logwarn_throttle(5.,'Building record write: %s',exc)

    def save_progress(self):
        if self.record_path:
            try:atomic_json(self.record_path,dict(map_digest=(self.task or {}).get('map_digest'),segment=self.index,arc_m=self.progress,
                state=self.state,reason=self.reason,mission_id=getattr(self,'mission_id',''),stamp_ns=rospy.Time.now().to_nsec(),wall_time=time.time()))
            except OSError as exc:rospy.logwarn_throttle(5.,'Building progress write: %s',exc)

    def preview(self):
        markers=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;markers.markers.append(clear)
        if self.task:
            for i,segment in enumerate(self.task['segments']):
                m=Marker(header=Header(frame_id='map'),ns='building',id=i,type=Marker.LINE_STRIP,action=Marker.ADD)
                m.pose.orientation.w=1.;m.scale.x=.035;m.color.r=.1;m.color.g=.8;m.color.b=.9;m.color.a=.7
                m.points=[Point(*p) for p in segment['points']];markers.markers.append(m)
        self.markers.publish(markers)

    def publish_status(self):
        self.exclusive.publish(Bool(self.state in ACTIVE|{'PAUSED'}))
        task=self.task
        self.publisher.publish(String(json.dumps(dict(state=self.state,message=self.reason,stamp_ns=rospy.Time.now().to_nsec(),
            segment=self.index,total_segments=len(task['segments']) if task else 0,arc_m=self.progress,
            map_digest=self.context.get('digest',''),mission_id=self.mission_id,task_id=self.task_id),ensure_ascii=False)))

    def status_loop(self):
        while not rospy.is_shutdown():
            self.publish_status();time.sleep(.2)

    def loop(self):
        while not rospy.is_shutdown():
            try:
                self.tick()
                # Atomic detach with command(start): the idle loop must not
                # unregister the subscriber just created for a new mission.
                retired=None
                with self.lock:
                    if self.state not in ACTIVE|{'PAUSED'} and self.voxel is not None:
                        retired=self.voxel;self.voxel=None;self.occupied=None;self.map_stamp=rospy.Time(0)
                if retired is not None:retired.subscriber.unregister()
            except Exception as exc:
                with self.lock:
                    retryable=any(v in str(exc) for v in ('最新障碍阻断','路线存在需要停止换向','发送前最新地图已阻断'))
                    if self.state in ACTIVE and rospy.get_param('/drone/sim_aggressive',False) and rospy.get_param('/use_sim_time',False) and retryable:
                        try:self.call_flight('hold')
                        except (RuntimeError,rospy.ServiceException):pass
                        self.task_id=0;self.last_reference=time.monotonic()+1.;self.state='TRANSITING'
                        self.reason='自动HOLD并重规划：'+str(exc);self.publish_status()
                    elif self.state in ACTIVE:self.pause('整栋任务暂停：'+str(exc))
                rospy.logwarn_throttle(2.,'Building mission: %s',exc)
            time.sleep(.1)

    def point_tick(self,now):
        segment=self.task['segments'][self.index]
        route=self.to_odom(segment['points']).tolist();position=self.position().tolist()
        target=route[0] if self.approaching else route[-1]
        distance=math.dist(position,target)
        if getattr(self,'point_distance',float('inf'))-distance>.025:self.last_advance=now
        self.point_distance=distance
        if self.point_sent:
            observed=getattr(self,'navigation_goal',None)
            owned=(observed is not None and observed.header.stamp.to_nsec()>=self.point_command_ns and
                   math.dist([observed.pose.position.x,observed.pose.position.y,observed.pose.position.z],self.point_target)<.02)
            if owned:self.task_id=observed.header.stamp.to_nsec()
            if owned and self.phase=='HOLD' and getattr(self,'terminal_stage','')=='COMPLETED' and distance<=.22:
                self.point_sent=False;self.task_id=0;self.last_advance=now;self.point_distance=float('inf')
                if self.approaching:self.approaching=False
                else:self.index+=1;self.progress=0.;self.save_progress()
                self.publish_status();return
            if self.phase=='HOLD' and owned and getattr(self,'terminal_stage','')!='COMPLETED':
                self.point_sent=False;self.task_id=0
            else:
                if now-self.last_advance>float(rospy.get_param('/drone/building/progress_timeout_s',90.)):raise RuntimeError('单目标长时间无进展')
                return
        if now-self.last_advance>float(rospy.get_param('/drone/building/progress_timeout_s',90.)):
            raise RuntimeError('单目标长时间无进展')
        if self.phase!='HOLD' or self.health!='HEALTHY':return
        if self.approaching and distance<=.15:
            self.approaching=False;self.point_distance=float('inf');return
        speed=self.point_speed(segment['speed'])
        if not speed.success:raise RuntimeError(speed.message)
        pose=PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id='odom'))
        pose.pose.orientation.w=1.;pose.pose.position=Point(*target)
        self.point_command_ns=rospy.Time.now().to_nsec()
        reply=self.point_goal(pose)
        if not reply.success:
            self.reason='等待可执行巡检目标：'+reply.message;self.publish_status();return
        self.point_target=target;self.point_sent=True;self.point_distance=distance
        self.state='RETURNING' if self.index==len(self.task['segments'])-1 else 'INSPECTING'
        self.reason='真实A*/EGO执行%s转折点，目标%d/%d'%(segment['kind'],self.index+1,len(self.task['segments']))
        self.record('goal',dict(segment=self.index,target_map=segment['points'][-1],target_odom=target,speed=segment['speed']))
        self.publish_status()

    def tick(self):
        if self.state not in ACTIVE:return
        serial=self.serial
        if self.state!='LANDING' and not self.owner_valid(self.owner):self.pause('AI授权/连接失效，保留任务进度');return
        now=rospy.Time.now().to_sec()
        if self.state=='TRANSITING' and not self.task_id and now-self.state_since<.15:return
        if self.phase in ('LANDING','DESCENDING','FAILSAFE') and self.state!='LANDING':
            self.set_state('CANCELED','飞行保护/操作员已启动降落，整栋任务结束');self.save_progress();return
        if self.health!='HEALTHY' and self.phase=='HOLD':self.pause('健康保护进入HOLD，任务进度已保留');return
        if self.state=='LANDING':
            if self.ground():self.set_state('COMPLETED','已巡检并返回地图原点、落地锁定');self.save_progress()
            return
        if self.state=='ARMING':
            if self.phase=='READY' and not self.arm_sent and now-self.state_since>.5:
                self.call_flight('arm');self.arm_sent=True
            if self.phase=='ARMED':
                p=self.position();t=self.transform.pose.position
                current_z=p[2]+t.z
                floors=[f for f in self.task['topology']['floors'] if f['ground_z']<current_z-.05]
                if not floors:raise RuntimeError('当前出生楼面不在已识别地图中')
                target=max(floors,key=lambda f:f['ground_z'])['altitude']
                self.call_flight('takeoff',target-current_z);self.set_state('TAKEOFF','起飞至当前大厅巡检高度 %.2fm'%float(self.task['topology'].get('hall_height_above_floor_m',1.2)))
            elif now-self.state_since>12:self.pause('解锁未完成，请检查飞控拒绝原因')
            return
        if self.state=='TAKEOFF':
            if self.phase=='HOLD':self.set_state('TRANSITING','连接预存巡检入口');self.last_advance=now
            elif now-self.state_since>20:self.pause('起飞未完成')
            return
        if self.odom is None or (rospy.Time.now()-self.odom.header.stamp).to_sec()>.7:raise RuntimeError('定位数据过期')
        if self.image is None or (rospy.Time.now()-self.image.header.stamp).to_sec()>.5:raise RuntimeError('下视相机数据过期')
        if self.occupied is None or not -.02<=(rospy.Time.now()-self.map_stamp).to_sec()<=2:
            if self.state=='TRANSITING' and not self.task_id and time.monotonic()-self.voxel_started_wall<10.:
                self.reason='等待首次完整体素快照，保持HOLD';return
            raise RuntimeError('最新地图过期')
        if self.index>=len(self.task['segments']):
            if self.phase!='HOLD':return
            self.call_flight('land');self.set_state('LANDING','已返回地图原点上方，执行降落');return
        if self.task.get('execution_mode')=='waypoints':
            self.point_tick(now);return
        if time.monotonic()-self.last_record_wall>=1.:
            self.last_record_wall=time.monotonic();self.record('trajectory',dict(position_odom=self.position().tolist(),segment=self.index,arc_m=self.progress,health=self.health,phase=self.phase))
        segment=self.task['segments'][self.index];route=self.to_odom(segment['points']).tolist();position=self.position().tolist()
        # Persistent directed projection; a repeated crossing cannot skip a later loop.
        limit=self.progress+(max(.25,math.dist(position,self.last_position)+.10) if self.last_position is not None else .35)
        tail,progress=self.planner.project_forward(position,route,self.progress,limit)
        if self.approaching:
            if self.phase=='HOLD' and math.dist(position,route[0])<=.15:
                self.approaching=False;self.task_id=0;self.last_advance=now
            else:tail=[position,route[0]];progress=self.progress
        if progress>self.progress+.025:self.last_advance=now
        self.progress=progress;self.last_position=position
        length=sum(math.dist(a,b) for a,b in zip(route,route[1:]));v=self.odom.twist.twist.linear
        if not self.approaching and self.phase=='HOLD' and self.progress>=length-.2 and math.dist(position,route[-1])<=.15 and math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)<=.15:
            self.index+=1;self.progress=0.;self.task_id=0;self.last_position=None;self.last_advance=now;self.repair=None;self.save_progress();return
        if self.task_id and self.phase=='HOLD' and not self.approaching:raise RuntimeError('执行会话已中断，请点击继续恢复剩余任务')
        if now-self.last_advance>float(rospy.get_param('/drone/building/progress_timeout_s',25.)):raise RuntimeError('通道/楼梯长时间无法通过，保持HOLD等待处理')
        if time.monotonic()-self.last_reference<.5:return
        self.last_reference=time.monotonic()
        p=self.planner;p.lower=list(rospy.get_param('/drone/navigation_min_xyz',p.lower));p.upper=list(rospy.get_param('/drone/navigation_max_xyz',p.upper));p.upper[2]=float(rospy.get_param('/drone/odom_max_flight_height_m',6.6))-.2
        occupied=self.occupied
        if segment['kind']=='stair' and not self.approaching:occupied=StairTubeCells(occupied,route,self.planner.geometry,self.planner.res)
        cells=(ReferenceSafetyCells(occupied),None)
        # Next 10m only: connect actual vehicle to saved route, preserve future segments.
        selected=[position,route[0]] if self.approaching else route_window(route,self.progress,10.)
        if len(selected)<2:
            if math.dist(position,route[-1])<.015:return
            selected=[position,route[-1]]
        if self.approaching:
            if abs(position[2]-route[0][2])>.3:raise RuntimeError('入口不在当前楼层，需预存楼梯连接')
            p.lower[2]=max(p.lower[2],min(position[2],route[0][2])-.15);p.upper[2]=min(p.upper[2],max(position[2],route[0][2])+.15)
        anchored=p.connect_reference(position,selected,cells)
        if len(anchored)<2 or not p.route_safe(anchored,cells):
            # Repair toward a saved forward point, using latest occupancy.
            target=selected[-1];result=None
            for candidate in p.astar_search(position,target,cells):
                if candidate is not None:result=candidate;break
            anchored=p.smooth_route(result,cells) if result else None
            if not anchored:raise RuntimeError('最新障碍阻断当前通道/楼梯，无法安全绕行')
        else:
            # Collinear simplification followed by authoritative Bezier rounding.
            clean=[anchored[0]]
            for a,b,c in zip(anchored,anchored[1:],anchored[2:]):
                u=np.asarray(b)-a;w=np.asarray(c)-b
                if np.linalg.norm(np.cross(u,w))>1e-7 or np.dot(u,w)<0:clean.append(b)
            clean.append(anchored[-1]);rounded=p.smooth_route(clean,cells)
            if rounded is None:raise RuntimeError('路线存在需要停止换向的尖角，请修正拓扑/分段')
            anchored=rounded
        path=RosPath(header=Header(stamp=rospy.Time.now(),frame_id='odom'))
        for point in anchored:
            pose=PoseStamped(header=path.header);pose.pose.orientation.w=1.;pose.pose.position=Point(*point);path.poses.append(pose)
        continuous=not self.approaching and math.dist(anchored[-1],route[-1])>.02
        with self.lock:
            if self.serial!=serial or self.state not in ACTIVE:return
            if not -.02<=(rospy.Time.now()-self.map_stamp).to_sec()<=2:raise RuntimeError('规划完成时地图过期')
            if not self.planner.route_safe(anchored,(ReferenceSafetyCells(self.occupied),None)):raise RuntimeError('发送前最新地图已阻断路线，保持HOLD')
            reply=self.execute(ExecuteInspectionRouteRequest(route=path,speed=segment['speed'],task_id=self.task_id,continuous=continuous,owner='building'))
            if not reply.success:raise RuntimeError(reply.message)
            self.task_id=reply.task_id
            self.state='RETURNING' if self.index==len(self.task['segments'])-1 else 'INSPECTING'
        self.reason='执行预存%s段；黄线/紫线为真实控制路径'%segment['kind']


if __name__=='__main__':
    rospy.init_node('drone_building_manager');BuildingManager();rospy.spin()
