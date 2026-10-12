#!/usr/bin/env python3
"""Run separately scored obstacle exposures during one real navigation flight."""
import argparse,csv,json,math,os,signal,subprocess,threading,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import rospy
from drone_stack.srv import RotateHeading
from dynamic_obstacle_experiment import Driver,BOX
from dynamic_obstacle_injector import yaw_of
from voxel_map_client import NumpyVoxelSnapshot
from run_dynamic_obstacle_tests import MemorySampler,ROW_FIELDS,write_report


def save_summary(base,records,sampler,finished=False):
    valid=[r for r in records if r.get('code') != 'F7_INFRASTRUCTURE']
    success=sum(bool(r.get('collision_free', r.get('perception_hits',0)>0 and r.get('min_clearance') is not None and r['min_clearance']>0)) for r in valid)
    goal_completed=sum(r.get('code')=='SUCCESS' for r in valid)
    stats=dict(records=len(records),counted=len(valid),success=success,
               rate=round(100.*success/len(valid),2) if valid else None,
               infrastructure=len(records)-len(valid),goal_completed=goal_completed,
               collision_free_rate=round(100.*success/len(valid),2) if valid else None,
               goal_completion_rate=round(100.*goal_completed/len(valid),2) if valid else None,
               peak_sim_rss_mb=round(sampler.peak_rss,1),
               min_available_mb=round(sampler.min_available,1),
               finished=finished,protocol='groups 01-05 cold start; groups 06-07 flight A; groups 08-10 flight B with empty transfers to fixed FCU start')
    (base/'summary.json').write_text(json.dumps(dict(summary=stats,groups=records),indent=2))
    with (base/'results.csv').open('w',newline='') as handle:
        w=csv.DictWriter(handle,fieldnames=ROW_FIELDS);w.writeheader()
        for r in records:w.writerow({k:r.get(k) for k in ROW_FIELDS})
    write_report(base,SimpleNamespace(feasibility=False),records,stats)
    return stats


def clear_obstacle(driver,group):
    """Clear by actual LiDAR observations, with FCU/observed-map yaw control."""
    hits=driver.voxels.hits(BOX)
    if not hits: raise RuntimeError('No observed obstacle evidence for clearing')
    target=np.mean([h[2] for h in hits],axis=0)
    odom=driver.value('odom');position=driver.drone_odom()
    heading=math.atan2(target[1]-position[1],target[0]-position[0])
    delta=math.atan2(math.sin(heading-yaw_of(odom.pose.pose.orientation)),math.cos(heading-yaw_of(odom.pose.pose.orientation)))
    if abs(math.degrees(delta)) >= .1:
        driver.call('/drone/rotate_heading',RotateHeading,math.degrees(delta))
    def aligned():
        odom=driver.value('odom')
        if odom is None or driver.text('flight_state')!='HOLD':return False
        error=heading-yaw_of(odom.pose.pose.orientation)
        return abs(math.atan2(math.sin(error),math.cos(error))) < math.radians(12)
    driver.wait(aligned,180.,'turn toward observed obstacle for clearing')
    before=driver.voxels.active_count(BOX)
    driver.injector.remove(BOX)
    driver.wait(lambda: driver.injector.model(BOX) is None,15.,'Gazebo obstacle removal')
    driver.wait(lambda: driver.voxels.active_count(BOX)==0 and driver.voxels.removed_count(BOX)>0,
                120.,'LiDAR did not clear the previous obstacle occupancy')
    evidence=dict(active_before=before,active_after=driver.voxels.active_count(BOX),
                  removed=driver.voxels.removed_count(BOX),sim=driver.sim_now(),
                  yaw_delta_deg=math.degrees(delta),method='observed map cells + FCU yaw; actual occupied_removed deltas')
    (Path(driver.output)/'between_groups_clearance.json').write_text(json.dumps(evidence,indent=2))
    print('Previous obstacle confirmed cleared: '+json.dumps(evidence),flush=True)
    driver.voxels.release(BOX)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    parser.add_argument('--start-index',type=int,default=6);parser.add_argument('--last-index',type=int,default=10)
    parser.add_argument('--airborne',action='store_true')
    parser.add_argument('--positive-exposures',action='store_true')
    parser.add_argument('--clear-initial-group',type=int)
    opt=parser.parse_args();base=Path(opt.output).resolve()
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:11311';os.environ['ROS_HOSTNAME']='127.0.0.1'
    rospy.init_node('continuous_dynamic_experiment')
    records=json.loads((base/'summary.json').read_text()).get('groups',[])
    if len(records)!=opt.start_index-1:raise RuntimeError('Existing summary must end before start-index')
    sampler=MemorySampler(base/'memory_continuous.jsonl',append=True)
    for trace in [base/'memory.jsonl',base/'memory_continuous.jsonl']:
        if trace.exists():
            for line in trace.read_text().splitlines():
                row=json.loads(line)
                sampler.peak_rss=max(sampler.peak_rss,row['sim_rss_mb'])
                sampler.min_available=min(sampler.min_available,row['available_mb'])
    sampler.start()
    driver=None;bag=None;last_stats=None
    first=json.loads((base/'groups/group_06/group.json').read_text()) if (base/'groups/group_06/group.json').exists() else None
    origin=None
    if first:
        e=next(e for e in first['events'] if e['event']=='goal_issued')
        origin=np.asarray(e['goal_odom'])-np.array([4.2,1.5,0.])
    try:
        if (base/'continuous_failure.json').exists():
            (base/'continuous_failure.json').unlink()
        for i in range(opt.start_index,opt.last_index+1):
            folder=base/'groups'/('group_%02d'%i);folder.mkdir(parents=True,exist_ok=True)
            if (folder/'group.json').exists():raise RuntimeError('Refusing to overwrite finished group')
            forward=4.2 if (i-opt.start_index)%2==0 else -4.2
            args=SimpleNamespace(output=str(folder),map_mode='PRIOR_NAV',forward=forward,
                lateral=(-1.5,0.,1.5)[(i-1)%3],height=1.2,obstacle_ahead=2.2,
                obstacle_size=[.3,2.,1.8],goal_retries=8,ready_timeout=1200.,
                inject_timeout=360.,perception_timeout=90.,arrive_timeout=360.,
                keep_airborne=i<opt.last_index,reuse_airborne=opt.airborne or i>opt.start_index,
                protocol='continuous shared flight',ready_settle_wall_s=1.,obstacle_inflation=.5)
            (folder/'parameters.json').write_text(json.dumps(vars(args),indent=2))
            if driver is None:driver=Driver(args)
            else:
                driver.args=args;driver.output=str(folder);driver.events=[];driver.group={}
                driver.goal_odom=None;driver.arrival_sim=None;driver._stop=threading.Event()
                driver._closed=False;driver._thread=None
            if opt.positive_exposures:
                if i==opt.start_index:
                    driver.ready()
                    if not opt.airborne:driver.arm_and_takeoff()
                    args.reuse_airborne=True
                    if opt.clear_initial_group:
                        old=json.loads((base/'groups'/('group_%02d'%opt.clear_initial_group)/'group.json').read_text())
                        driver.watch_obstacle(old['injection']['centre'])
                        snap=rospy.ServiceProxy('/grid_map/get_voxel_snapshot',NumpyVoxelSnapshot)()
                        if not snap.success:raise RuntimeError('Initial obstacle snapshot failed')
                        driver.voxels._receive(snap.update)
                        clear_obstacle(driver,old)
                # Transfer to the fixed FCU start without an obstacle; not an exposure.
                current=driver.drone_odom()
                driver.send_goal(tuple(origin-current));driver.await_terminal()
                (folder/'preparation_events.json').write_text(json.dumps(driver.events,indent=2))
                driver.events=[];driver.group={};args.reuse_airborne=True
            if origin is not None:
                driver.wait(lambda: driver.drone_odom() is not None,15.,'No FCU odom for fixed stations')
                current=driver.drone_odom()
                high=opt.positive_exposures or current[0]<origin[0]+2.1
                target=origin+np.array([4.2 if high else 0.,args.lateral,0.])
                args.forward=float(target[0]-current[0]);args.lateral=float(target[1]-current[1])
                args.goal_offset_z=float(target[2]-current[2])
                args.fixed_goal_odom=target.tolist()
                (folder/'parameters.json').write_text(json.dumps(vars(args),indent=2))
            with (folder/'rosbag.log').open('w') as log:
                bag=subprocess.Popen(['rosbag','record','--lz4','-O',str(folder/'telemetry.bag'),
                    '/clock','/gazebo/model_states','/mavros/state','/mavros/local_position/odom',
                    '/drone/flight_state','/drone/flight_health','/drone/navigation_stage',
                    '/drone/navigation_terminal_stage','/drone/navigation_goal'],stdout=log,stderr=subprocess.STDOUT)
                print('=== continuous group_%02d ==='%i,flush=True)
                start=time.monotonic()
                try:
                    group=driver.run_group();group['ok']=group.get('code')=='SUCCESS'
                except Exception as exc:
                    driver.stop_sampling();driver.emergency()
                    group=dict(driver.group,ok=False,code='F7_INFRASTRUCTURE',reason=str(exc))
                group.update(events=list(driver.events),wall_seconds=round(time.monotonic()-start,1),protocol='continuous shared flight')
                (folder/'group.json').write_text(json.dumps(group,indent=2,sort_keys=True))
                bag.send_signal(signal.SIGINT)
                try:bag.wait(timeout=20.)
                except subprocess.TimeoutExpired:bag.terminate();bag.wait(timeout=5.)
                bag=None
            row=dict(group=folder.name,lateral=args.lateral,forward=args.forward)
            row.update({k:group[k] for k in ROW_FIELDS if k in group});records.append(row)
            last_stats=save_summary(base,records,sampler)
            print(json.dumps(row),flush=True)
            if group.get('code')=='F7_INFRASTRUCTURE':break
            if i<opt.last_index:
                clear_obstacle(driver,group)
                if driver.text('flight_state')!='HOLD':raise RuntimeError('Shared vehicle not in HOLD')
        if driver and driver.value('state') and driver.value('state').armed:driver.land()
        (base/'continuous_session.json').write_text(json.dumps(dict(events=driver.events if driver else [],
            final_armed=bool(driver.value('state').armed) if driver and driver.value('state') else None,
            completed_groups=len(records),clearance_protocol='Remove box; face observed obstacle; wait active occupancy zero'),indent=2))
    except Exception as exc:
        (base/'continuous_failure.json').write_text(json.dumps(dict(error=str(exc),records=len(records)),indent=2))
        if driver:
            driver.stop_sampling()
            try:driver.land()
            except Exception:driver.emergency()
        raise
    finally:
        if bag is not None and bag.poll() is None:bag.send_signal(signal.SIGINT)
        sampler.stop.set();sampler.join(timeout=3)
        if records:last_stats=save_summary(base,records,sampler,finished=len(records)==opt.last_index)
    print(json.dumps(last_stats),flush=True)

if __name__=='__main__':main()
