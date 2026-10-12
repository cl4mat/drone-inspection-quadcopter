#!/usr/bin/env python3
"""Export measured CSV statistics, geometric camera coverage, figures and PDF."""
import argparse
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pandas as pd
from tf.transformations import quaternion_matrix, euler_matrix, quaternion_slerp


def scene_geometry(path, resolution=.1, inset=.8):
    world = ET.parse(path).getroot().find('world')
    floors, boxes, treads = [], [], []
    for model in world.findall('model'):
        if model.findtext('static') != 'true':
            continue
        for link in model.findall('link'):
            name = link.get('name', '')
            collision = link.find('collision')
            if collision is None or collision.find('geometry/box') is None:
                continue
            size = np.array(list(map(float, collision.findtext('geometry/box/size').split())))
            pose = list(map(float, link.findtext('pose', '0 0 0 0 0 0').split()))
            rotation = euler_matrix(*pose[3:])[:3, :3]
            centre = np.array(pose[:3])
            if 'hall_floor' in name or 'corridor_floor' in name or 'upper_return_corridor' in name:
                floors.append(dict(name=name, kind='hall' if 'hall_floor' in name else 'corridor',
                    z=float(centre[2]+size[2]/2), lo=(centre[:2]-size[:2]/2).tolist(),
                    hi=(centre[:2]+size[:2]/2).tolist()))
            if '_stair_step_' in name:
                treads.append(dict(name=name,z=float(centre[2]+size[2]/2),lo=centre[:2]-size[:2]/2,hi=centre[:2]+size[:2]/2))
            if True:  # All static collision boxes can occlude the camera.
                corners = np.array([[a,b,c] for a in (-1,1) for b in (-1,1) for c in (-1,1)])*size/2
                world_corners = corners @ rotation.T + centre
                boxes.append(dict(name=name, centre=centre, rotation=rotation, half=size/2,
                    lo=world_corners.min(axis=0), hi=world_corners.max(axis=0)))
    levels = sorted(set(round(f['z'], 4) for f in floors))
    xlo = min(f['lo'][0] for f in floors)
    xhi = max(f['hi'][0] for f in floors)
    ylo = min(f['lo'][1] for f in floors)
    yhi = max(f['hi'][1] for f in floors)
    xx, yy = np.meshgrid(np.arange(xlo+resolution/2,xhi,resolution),
                         np.arange(ylo+resolution/2,yhi,resolution), indexing='xy')
    xy = np.column_stack((xx.ravel(), yy.ravel()))
    regions = []
    for f in floors:
        mask = np.all((xy >= f['lo']) & (xy <= f['hi']), axis=1)
        for box in boxes:
            if 'fixed_obstacle' in box['name'] and box['lo'][2] <= f['z']+.1 <= box['hi'][2]:
                mask &= ~np.all((xy >= box['lo'][:2]) & (xy <= box['hi'][:2]), axis=1)
        safe = mask.copy()
        if f['kind'] == 'hall':
            safe &= np.all((xy >= np.array(f['lo'])+inset) & (xy <= np.array(f['hi'])-inset), axis=1)
        regions.append(dict(**f, mask=mask, safety_mask=safe, floor=levels.index(round(f['z'],4))))
    heights=[np.full(len(xy),z) for z in levels]
    for side in ('left','right'):
        surface=np.full(len(xy),np.nan);mask=np.zeros(len(xy),bool)
        for tread in treads:
            if tread['name'].startswith(side+'_stair_step_'):
                hit=np.all((xy>=tread['lo'])&(xy<=tread['hi']),axis=1)
                surface[hit]=tread['z'];mask|=hit
        if mask.any():
            regions.append(dict(name=side+'_stair_treads',kind='stair',z=None,floor=len(levels),mask=mask,safety_mask=mask))
            levels.append(None);heights.append(surface)
    return xy, xx.shape, regions, boxes, levels, heights


def visible_points(points, camera, boxes):
    visible = np.ones(len(points), dtype=bool)
    lo = np.minimum(points.min(axis=0), camera)
    hi = np.maximum(points.max(axis=0), camera)
    for box in boxes:
        if np.any(box['hi'] < lo) or np.any(box['lo'] > hi):
            continue
        start = (camera-box['centre']) @ box['rotation']
        end = (points-box['centre']) @ box['rotation']
        delta = end-start
        moving = np.abs(delta)>1e-10
        t0 = np.divide(-box['half']-start,delta,out=np.full_like(delta,-np.inf),where=moving)
        t1 = np.divide(box['half']-start,delta,out=np.full_like(delta,np.inf),where=moving)
        enter = np.max(np.minimum(t0,t1),axis=1)
        leave = np.min(np.maximum(t0,t1),axis=1)
        parallel_outside = np.any(~moving & ((start < -box['half']) | (start > box['half'])),axis=1)
        blocked = ~parallel_outside & (enter<=leave) & (leave>.001) & (enter<.999)
        visible &= ~blocked
    return visible


def coverage(data, metadata, task, root):
    xy, shape, regions, boxes, levels, heights = scene_geometry(metadata['world_file'],
        inset=task['topology'].get('hall_inset_m',.8))
    covered = np.zeros((len(levels),len(xy)),bool)
    swath = covered.copy()
    floor_masks = [np.logical_or.reduce([r['mask'] for r in regions if r['floor']==i])
                   for i in range(len(levels))]
    rotation_camera = euler_matrix(0., math.pi/2, 0.)[:3,:3]
    th = math.tan(metadata['camera_horizontal_fov_rad']/2)
    tv = th*metadata['camera_height']/metadata['camera_width']
    rays = np.array([[1.,a*th,b*tv] for a,b in [(-1,-1),(-1,1),(1,1),(1,-1)]])
    series, last, last_series, accepted = [], None, -1e9, 0
    radius=float(metadata['strip_spacing_m'])/2
    truth_data=data[data.truth_valid].drop_duplicates('sim_time_s').sort_values('sim_time_s')
    truth_time=truth_data.sim_time_s.to_numpy()
    truth_pose=truth_data[['gt_x_m','gt_y_m','gt_z_m','gt_qx','gt_qy','gt_qz','gt_qw']].to_numpy(float)
    active = data[data.building_state.isin(['TRANSITING','INSPECTING','RETURNING']) &
                  data.truth_valid & data.armed].copy()
    for row in active.itertuples():
        p = np.array([row.gt_x_m,row.gt_y_m,row.gt_z_m])
        q = [row.gt_qx,row.gt_qy,row.gt_qz,row.gt_qw]
        camera_valid = math.isfinite(row.camera_age_s) and -.02<=row.camera_age_s<=.5
        if not camera_valid:
            last = None
            continue
        camera_p,camera_q=p,q
        pair=int(np.searchsorted(truth_time,row.camera_stamp_s))
        if 0<pair<len(truth_time) and truth_time[pair]-truth_time[pair-1]<=.6:
            u=(row.camera_stamp_s-truth_time[pair-1])/(truth_time[pair]-truth_time[pair-1])
            camera_p=truth_pose[pair-1,:3]*(1-u)+truth_pose[pair,:3]*u
            camera_q=quaternion_slerp(truth_pose[pair-1,3:],truth_pose[pair,3:],u)
        else:
            last=None;continue
        rotation = quaternion_matrix(camera_q)[:3,:3]
        camera = camera_p+rotation @ np.array([0.,0.,-.15])
        directions = rays @ (rotation @ rotation_camera).T
        if np.all(directions[:,2]<-.02) and camera[2]>.02:
            # The lowest floor gives a conservative XY bounding rectangle.
            # Each actual floor/tread is then tested at its own true elevation.
            distance = (0.-camera[2])/directions[:,2]
            corners = camera + directions*distance[:,None]
            lo, hi = corners[:,:2].min(axis=0), corners[:,:2].max(axis=0)
            for floor in range(len(levels)):
                ids=np.flatnonzero(floor_masks[floor]&np.all((xy>=lo)&(xy<=hi),axis=1))
                if not len(ids):continue
                points=np.column_stack((xy[ids],heights[floor][ids]+.005))
                local=(points-camera) @ (rotation @ rotation_camera)
                frustum=((local[:,0]>.05)&(local[:,0]<35)&
                    (np.abs(local[:,1])<=local[:,0]*th)&(np.abs(local[:,2])<=local[:,0]*tv))
                selected=ids[frustum]
                if len(selected):
                    selected=selected[visible_points(points[frustum],camera,boxes)]
                    covered[floor,selected]=True
            accepted+=1
        a=last[0][:2] if last and row.sim_time_s-last[1]<=.6 else p[:2]
        b=p[:2];lo,hi=np.minimum(a,b)-radius,np.maximum(a,b)+radius
        for floor in range(len(levels)):
            above=p[2]-heights[floor]
            ids=np.flatnonzero(floor_masks[floor]&(above>.3)&(above<2.)&np.all((xy>=lo)&(xy<=hi),axis=1))
            if len(ids):
                delta=b-a
                u=np.clip((xy[ids]-a) @ delta/max(float(delta@delta),1e-12),0,1)
                hit=np.linalg.norm(xy[ids]-(a+u[:,None]*delta),axis=1)<=radius
                swath[floor,ids[hit]]=True
        last=(p,row.sim_time_s)
        if row.sim_time_s-last_series>=1.:
            last_series=row.sim_time_s
            for r in regions:
                mask=r['mask'];safe=r['safety_mask'];i=r['floor']
                series.append(dict(sim_time_s=row.sim_time_s,region=r['name'],floor=i,
                    kind=r['kind'],floor_area_m2=int(mask.sum())*.01,
                    safety_area_m2=int(safe.sum())*.01,
                    camera_covered_m2=int((covered[i]&mask).sum())*.01,
                    safety_camera_covered_m2=int((covered[i]&safe).sum())*.01,
                    nominal_swath_m2=int((swath[i]&mask).sum())*.01))
    pd.DataFrame(series).to_csv(root/'csv'/'coverage_timeseries.csv',index=False,encoding='utf-8-sig')
    results=[]
    for r in regions:
        i=r['floor'];full=r['mask'];safe=r['safety_mask']
        results.append(dict(region=r['name'],floor=i,kind=r['kind'],
            floor_area_m2=int(full.sum())*.01,safety_area_m2=int(safe.sum())*.01,
            camera_covered_m2=int((covered[i]&full).sum())*.01,
            camera_coverage_pct=100*float((covered[i]&full).sum())/max(1,full.sum()),
            safety_camera_coverage_pct=100*float((covered[i]&safe).sum())/max(1,safe.sum()),
            nominal_swath_coverage_pct=100*float((swath[i]&full).sum())/max(1,full.sum())))
    table=pd.DataFrame(results)
    table.to_csv(root/'csv'/'coverage_regions.csv',index=False,encoding='utf-8-sig')
    np.savez_compressed(root/'coverage_grid.npz',xy=xy,shape=shape,covered=covered,swath=swath,
                        floor_masks=np.array(floor_masks),floor_z=np.array([np.nan if z is None else z for z in levels]))
    return table,xy,shape,covered,swath,floor_masks,levels,accepted


def numeric_stats(values):
    a=np.asarray(values,dtype=float);a=a[np.isfinite(a)]
    return dict(n=len(a),mean=float(a.mean()),median=float(np.median(a)),
                p95=float(np.percentile(a,95)),max=float(a.max()),rmse=float(np.sqrt(np.mean(a*a)))) if len(a) else dict(n=0)


def export_report(root):
    root=Path(root)
    task=json.loads((root/'task.json').read_text())
    meta=json.loads((root/'metadata.json').read_text())
    data=pd.read_csv(root/'csv'/'samples.csv')
    events=pd.read_csv(root/'csv'/'events.csv')
    # Trim preflight waiting while retaining takeoff and terminal actual landing.
    mission=data[data.building_state.isin(['ARMING','TAKEOFF','TRANSITING','INSPECTING','RETURNING','LANDING','COMPLETED'])]
    if mission.empty:
        mission=data
    start=float(mission.sim_time_s.min());end=float(mission.sim_time_s.max())
    data=data[data.sim_time_s>=start].copy()
    data['elapsed_sim_s']=data.sim_time_s-start
    targets=[]
    for mission_id,attempt in data.dropna(subset=['mission_id']).groupby('mission_id',sort=False):
        for index,segment in enumerate(task['segments']):
            samples=attempt[(attempt.segment==index)&attempt.building_state.isin(['TRANSITING','INSPECTING','RETURNING'])]
            if samples.empty:
                continue
            advanced=attempt[(attempt.segment>index)&attempt.building_state.isin(['TRANSITING','INSPECTING','RETURNING','LANDING','COMPLETED'])]
            complete=not advanced.empty
            terminal=float(advanced.iloc[0].sim_time_s) if complete else float(samples.iloc[-1].sim_time_s)
            at=attempt.iloc[int(np.argmin(np.abs(attempt.sim_time_s.to_numpy()-terminal)))]
            goal=np.array(segment['points'][-1]);gt=at[['gt_x_m','gt_y_m','gt_z_m']].to_numpy(dtype=float)
            estimate=at[['fcu_map_x_m','fcu_map_y_m','fcu_map_z_m']].to_numpy(dtype=float)
            targets.append(dict(mission_id=mission_id,segment=index,kind=segment['kind'],result='COMPLETED' if complete else 'INCOMPLETE',
                start_sim_s=float(samples.iloc[0].sim_time_s),end_sim_s=terminal,
                navigation_sim_duration_s=terminal-float(samples.iloc[0].sim_time_s),
                navigation_wall_duration_s=float(at.wall_time_s-samples.iloc[0].wall_time_s),
                target_map_x_m=goal[0],target_map_y_m=goal[1],target_map_z_m=goal[2],
                actual_world_x_m=gt[0],actual_world_y_m=gt[1],actual_world_z_m=gt[2],
                true_arrival_error_m=float(np.linalg.norm(gt-goal-[0.,0.,.17])),
                estimated_arrival_error_m=float(np.linalg.norm(estimate-goal)),
                lio_ate_at_arrival_m=float(at.lio_ate_m),speed_cap_mps=segment['speed'],
                planned_length_m=sum(math.dist(a,b) for a,b in zip(segment['points'],segment['points'][1:]))))
    target_table=pd.DataFrame(targets)
    target_table.to_csv(root/'csv'/'navigation_targets.csv',index=False,encoding='utf-8-sig')
    motion=data[data.truth_valid].copy()
    xyz=motion[['gt_x_m','gt_y_m','gt_z_m']].to_numpy(float)
    steps=np.r_[0.,np.linalg.norm(np.diff(xyz,axis=0),axis=1)] if len(xyz) else np.zeros(0)
    dt=motion.sim_time_s.diff().fillna(0).to_numpy()
    steps[(dt<0)|(dt>.6)]=0.
    inspecting=motion.building_state.isin(['TRANSITING','INSPECTING','RETURNING']).to_numpy()
    motion['true_step_length_m']=steps
    motion['groundtruth_speed_mps']=np.divide(steps,dt,out=np.zeros_like(steps),where=dt>0)
    motion['cumulative_true_length_m']=np.cumsum(steps)
    motion['cumulative_inspection_length_m']=np.cumsum(steps*inspecting)
    motion[['sim_time_s','elapsed_sim_s','building_state','segment','true_step_length_m',
            'cumulative_true_length_m','cumulative_inspection_length_m','groundtruth_speed_mps']].to_csv(
        root/'csv'/'path_length.csv',index=False,encoding='utf-8-sig')
    cov,xy,shape,covered,swath,masks,levels,frames=coverage(data,meta,task,root)
    valid=data[data.lio_pair_valid & data.building_state.isin(['TAKEOFF','TRANSITING','INSPECTING','RETURNING','LANDING'])]
    completed=target_table[target_table.result=='COMPLETED'] if not target_table.empty else target_table
    summary=dict(mission_completed=meta['mission_completed'],map_digest=meta['map_digest'],
        duration_sim_s=end-start,duration_wall_s=float(mission.wall_time_s.max()-mission.wall_time_s.min()),
        planned_path_length_m=task['length_m'],actual_flight_length_m=float(steps.sum()),
        actual_inspection_length_m=float((steps*inspecting).sum()),samples=len(data),
        synchronized_lio_samples=len(valid),valid_camera_evaluation_frames=frames,
        lio_translation_error_m=numeric_stats(valid.lio_ate_m),
        lio_attitude_error_deg=numeric_stats(valid.lio_attitude_error_deg),
        arrival_error_m=numeric_stats(completed.true_arrival_error_m) if len(completed) else dict(n=0),
        navigation_duration_sim_s=numeric_stats(completed.navigation_sim_duration_s) if len(completed) else dict(n=0),
        completed_targets=len(completed),total_segments=len(task['segments']),
        effective_camera_floor_coverage_pct=100*sum(int((covered[i]&masks[i]).sum()) for i in range(len(masks)))/max(1,sum(int(m.sum()) for m in masks)),
        unique_floor_area_m2=sum(int(m.sum()) for m in masks)*.01,
        unique_camera_covered_m2=sum(int((covered[i]&masks[i]).sum()) for i in range(len(masks)))*.01,
        regions=cov.to_dict('records'),definitions=dict(
            lio_drift='Ground-truth relative pose error after one fixed initial SE(3) alignment; includes no target error.',
            arrival_error='Ground-truth body centre minus segment endpoint in map-origin world coordinates.',
            navigation_time='Segment entry to completion, including planning/approach/pauses; simulation and wall clocks separate.',
            coverage='Union of down-camera frustum floor cells with static wall/cabinet/rail occlusion; not detector recognition.',
            nominal_swath='Actual flown centreline buffered by configured strip spacing / 2; independent of camera footprint.',
            floor_area='Actual scene floor area excluding cabinet footprints; horizontal stair treads included. Global coverage deduplicates overlapping floor links; safety hall inset is stored in metadata. Layers 0/1 are floors, 2/3 are stairs.'))
    (root/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False))
    rows=[]
    for name,value in summary.items():
        if isinstance(value,dict) and 'n' in value:
            rows.append(dict(metric=name,**value))
    pd.DataFrame(rows).to_csv(root/'csv'/'statistics.csv',index=False,encoding='utf-8-sig')
    positions=motion[['gt_x_m','gt_y_m','gt_z_m']].to_numpy(float)
    _,grid_shape,regions,_,_,heights=scene_geometry(meta['world_file'],inset=task['topology'].get('hall_inset_m',.8))
    ix=np.floor((positions[:,0]-xy[:,0].min()+.05)/.1).astype(int)
    iy=np.floor((positions[:,1]-xy[:,1].min()+.05)/.1).astype(int)
    in_grid=(ix>=0)&(ix<grid_shape[1])&(iy>=0)&(iy<grid_shape[0]);ids=np.clip(iy,0,grid_shape[0]-1)*grid_shape[1]+np.clip(ix,0,grid_shape[1]-1)
    ground=np.full(len(motion),np.nan)
    for layer in range(len(masks)):
        elevation=heights[layer][ids];hit=in_grid&masks[layer][ids]&np.isfinite(elevation)&(elevation<positions[:,2]-.15)
        ground[hit]=np.fmax(ground[hit],elevation[hit])
    motion['true_height_above_floor_m']=positions[:,2]-ground
    motion[['sim_time_s','segment','segment_kind','groundtruth_speed_mps','true_height_above_floor_m']].to_csv(root/'csv'/'speed_and_height.csv',index=False,encoding='utf-8-sig')
    terrain=[]
    for kind,group in motion[motion.building_state.isin(['TRANSITING','INSPECTING','RETURNING'])].groupby('segment_kind'):
        terrain.append(dict(kind=kind,speed_mean_mps=float(group.groundtruth_speed_mps.mean()),speed_p95_mps=float(group.groundtruth_speed_mps.quantile(.95)),speed_max_mps=float(group.groundtruth_speed_mps.max()),height_mean_m=float(group.true_height_above_floor_m.mean())))
    pd.DataFrame(terrain).to_csv(root/'csv'/'terrain_statistics.csv',index=False,encoding='utf-8-sig')
    summary['terrain_statistics']=terrain
    (root/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False))
    make_figures(root,data,motion,target_table,cov,xy,shape,covered,masks,levels,task,start)
    make_documents(root,summary,meta)
    archive=shutil.make_archive(str(root), 'zip',root_dir=root.parent,base_dir=root.name)
    print(json.dumps(dict(report=str(root/'REPORT.pdf'),archive=archive,summary=summary),ensure_ascii=False),flush=True)
    return summary


def make_figures(root,data,motion,targets,cov,xy,shape,covered,masks,levels,task,start):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':11,'axes.grid':True,'grid.alpha':.25,'savefig.dpi':240})
    out=root/'figures';out.mkdir(exist_ok=True)
    fig,axes=plt.subplots(2,1,figsize=(13.33,7.5),sharex=True)
    axes[0].plot(motion.elapsed_sim_s,motion.groundtruth_speed_mps,label='Actual ground-truth speed')
    axes[0].plot(data.elapsed_sim_s,data.speed_cap_mps,'--',label='Requested speed cap',alpha=.7)
    axes[0].set_ylabel('Speed (m/s)');axes[0].legend()
    axes[1].plot(motion.elapsed_sim_s,motion.true_height_above_floor_m,label='Actual height above local floor')
    axes[1].set_ylabel('Height above floor (m)');axes[1].set_xlabel('Simulation elapsed time (s)');axes[1].legend()
    fig.suptitle('Measured speed and local inspection height')
    fig.tight_layout();fig.savefig(out/'speed_and_height.png');plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(13.33,7.5),sharex=True)
    valid=data[data.lio_pair_valid]
    axes[0].plot(valid.elapsed_sim_s,valid.lio_ate_m,label='3D position')
    axes[0].plot(valid.elapsed_sim_s,valid.lio_ate_xy_m,label='XY position',alpha=.8)
    axes[0].set_ylabel('Position error (m)');axes[0].legend()
    axes[1].plot(valid.elapsed_sim_s,valid.lio_attitude_error_deg,color='#9b59b6')
    axes[1].set_ylabel('Attitude error (deg)');axes[1].set_xlabel('Simulation elapsed time (s)')
    fig.suptitle('Faster-LIO drift against synchronized Gazebo ground truth')
    fig.tight_layout();fig.savefig(out/'lio_drift.png');plt.close(fig)
    targets=targets[targets.result=='COMPLETED'] if not targets.empty else targets
    if not targets.empty:
        fig,axes=plt.subplots(2,1,figsize=(13.33,7.5),sharex=True)
        x=targets.segment.to_numpy()
        axes[0].bar(x,targets.true_arrival_error_m,label='Actual body vs target',color='#e67e22')
        axes[0].plot(x,targets.estimated_arrival_error_m,'o-',label='Estimated body vs target',markersize=3)
        axes[0].set_ylabel('Arrival error (m)');axes[0].legend()
        axes[1].bar(x,targets.navigation_sim_duration_s,color='#2980b9')
        axes[1].set_ylabel('Navigation duration (sim s)');axes[1].set_xlabel('Executed target / segment')
        fig.suptitle('Actual target accuracy and navigation time');fig.tight_layout()
        fig.savefig(out/'navigation_targets.png');plt.close(fig)
    fig,ax=plt.subplots(figsize=(13.33,7.5))
    labels=[name.replace('lower_','L1 ').replace('upper_','L2 ').replace('_',' ') for name in cov.region]
    x=np.arange(len(cov));width=.25
    ax.bar(x-width,cov.camera_coverage_pct,width,label='Camera / complete floor')
    ax.bar(x,cov.safety_camera_coverage_pct,width,label='Camera / safety region')
    ax.bar(x+width,cov.nominal_swath_coverage_pct,width,label='Nominal %.1f m swath / floor'%task['topology'].get('strip_spacing_m',1.8))
    ax.set_xticks(x);ax.set_xticklabels(labels,rotation=22,ha='right')
    ax.set_ylim(0,105);ax.set_ylabel('Measured coverage (%)');ax.legend()
    ax.set_title('Actual coverage: denominator and footprint are explicit')
    fig.tight_layout();fig.savefig(out/'coverage_comparison.png');plt.close(fig)
    extent=[xy[:,0].min()-.05,xy[:,0].max()+.05,xy[:,1].min()-.05,xy[:,1].max()+.05]
    flat=[(i,z) for i,z in enumerate(levels) if z is not None]
    fig,axes=plt.subplots(len(flat),1,figsize=(16,9),squeeze=False)
    for panel,(floor,z) in enumerate(flat):
        ax=axes[panel,0];image=np.full(len(xy),np.nan)
        image[masks[floor]]=0.;image[covered[floor]&masks[floor]]=1.
        ax.imshow(image.reshape(shape),origin='lower',extent=extent,cmap='RdYlGn',vmin=0,vmax=1,alpha=.45)
        points=motion[(motion.gt_z_m>z+.25)&(motion.gt_z_m<z+2.)]
        ax.plot(points.gt_x_m,points.gt_y_m,color='#008060',linewidth=1,label='Actual ground-truth flight')
        for s in task['segments']:
            p=np.asarray(s['points']);height=p[:,2]+.17
            if s.get('floor_id')==floor and s['kind']!='stair':
                ax.plot(p[:,0],p[:,1],color='#b7950b',linewidth=.65,alpha=.8)
        ax.set_aspect('equal');ax.set_ylabel('World Y (m)');ax.set_title('Floor %d: green observed / red uncovered'%(floor+1))
        ax.legend(loc='upper right')
    axes[-1,0].set_xlabel('World X (m)')
    fig.tight_layout();fig.savefig(out/'coverage_and_actual_route.png');plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(13.33,7.5))
    axes[0].plot(motion.elapsed_sim_s,motion.cumulative_inspection_length_m,label='Actual inspection path')
    axes[0].plot(motion.elapsed_sim_s,motion.cumulative_true_length_m,label='Actual including takeoff/landing',alpha=.7)
    axes[0].set_xlabel('Simulation elapsed (s)');axes[0].set_ylabel('Path length (m)');axes[0].legend()
    duration=data.groupby('building_state').sim_time_s.agg(['min','max'])
    # Sampling occupancy, rather than max-min spans that double-count resumptions.
    weights=data.sim_time_s.diff().shift(-1).fillna(0).clip(lower=0)
    phases=weights.groupby(data.building_state).sum()
    phases.to_csv(root/'csv'/'phase_durations.csv',header=['duration_sim_s'],encoding='utf-8-sig')
    phases=phases[phases.index.isin(['TAKEOFF','TRANSITING','INSPECTING','RETURNING','LANDING','PAUSED'])]
    if len(phases):axes[1].pie(phases,labels=phases.index,autopct='%1.1f%%',textprops={'fontsize':9})
    axes[1].set_title('Mission sampled time composition')
    fig.tight_layout();fig.savefig(out/'path_length_and_time.png');plt.close(fig)


def make_documents(root,s,meta):
    def fmt(v):
        return '%.3f'%v if isinstance(v,(float,int)) else str(v)
    lines=['# 双层无人机巡检仿真实验报告','',
        '任务结果：'+('完成并落地锁定' if s['mission_completed'] else '未完成（保留真实失败数据）'),'',
        '|指标|实测结果|','|---|---|',
        '|仿真总耗时|%s s|'%fmt(s['duration_sim_s']),
        '|实际墙钟耗时|%s s|'%fmt(s['duration_wall_s']),
        '|规划路径总长|%s m|'%fmt(s['planned_path_length_m']),
        '|实飞巡检路径总长|%s m|'%fmt(s['actual_inspection_length_m']),
        '|实际相机楼面覆盖率|%s %%|'%fmt(s['effective_camera_floor_coverage_pct']),
        '|完成目标/总段数|%s / %s|'%(s['completed_targets'],s['total_segments']),'',
        '## 指标口径','']
    definitions=[
        'Faster-LIO漂移：首次同步地面样本固定SE(3)对齐后，与按采集时间配对的Gazebo真值相比。到达实际位置与目标点的差距另记为导航到达误差，不能全部归因于LIO。',
        '单目标定义为真实执行的巡检任务段终点；中间10m控制窗口更新不重复算新目标。耗时包含连接入口、规划等待、运动和收敛；仿真时钟与墙钟分别报告。',
        '大厅条带间距上限%s m，大厅速度上限2m/s、走廊1m/s、楼梯0.4m/s。上限不是平均速度。'%meta['strip_spacing_m'],
        '覆盖率基于实际轨迹、按图像采集时间插值的下视相机姿态/FOV与新鲜图像，投影到真实楼面，并剔除墙、柜体和扶手遮挡；只代表几何可视覆盖，不代表检测算法识别成功。',
        '同时报告完整楼面与大厅内缩%s m的安全巡检区域；另外导出按配置宽度缓冲的名义条带覆盖。整体覆盖率对相交楼面去重。'%meta.get('hall_inset_m',.8),
        '真值及世界碰撞几何只用于离线评估，不输入定位、规划、EKF或飞行控制。ModelStates没有采集时间戳，使用ROS时钟回调时间，超过配对时间差门槛的LIO样本无效并保留标记。',
        '本轮仿真门槛：位置约束%s，障碍膨胀%s m，HOLD %s s、降落%s s；保留严重异常保护。只用于仿真。'%(meta['simulation_lio_min_support'],meta.get('obstacles_inflation_m',.4),meta.get('health_hold_s',1),meta.get('health_land_s',2))]
    lines += ['- '+v for v in definitions]
    lines += ['', '## 统计', '']
    for key in ['lio_translation_error_m','lio_attitude_error_deg','arrival_error_m','navigation_duration_sim_s']:
        values=s[key]
        lines += ['- '+key+': '+', '.join(k+'='+fmt(v) for k,v in values.items())]
    lines += ['', '## 数据和图表', '',
        'csv/samples.csv：采样原始值、真值、LIO误差、实际/估计目标距离、相机与几何健康。',
        'csv/navigation_targets.csv：每目标完成结果、到达误差、仿真/墙钟耗时。',
        'csv/coverage_regions.csv、coverage_timeseries.csv：逐区域面积及覆盖率和时间序列。',
        'csv/path_length.csv、statistics.csv、events.csv：实飞长度、指标统计、运行状态事件。',
        'figures/：240 dpi PNG，可直接用于报告/PPT；task.json和metadata.json保存路线及评估配置。']
    text='\n'.join(lines)+'\n'
    (root/'REPORT.md').write_text(text)
    from reportlab.pdfgen import canvas
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.cidfonts import UnicodeCIDFont
    from reportlab.lib.pagesizes import A4
    pdfmetrics.registerFont(UnicodeCIDFont('STSong-Light'))
    c=canvas.Canvas(str(root/'REPORT.pdf'),pagesize=A4)
    c.setTitle('双层无人机巡检仿真实验报告')
    width,height=A4;y=height-45
    for line in lines:
        # Simple CJK wrapping; pipe table is retained in the Markdown/raw CSV.
        if not line:line=' '
        for begin in range(0,len(line),48):
            if y<45:c.showPage();y=height-45
            c.setFont('STSong-Light',10);c.drawString(35,y,line[begin:begin+48]);y-=16
    for file in sorted((root/'figures').glob('*.png')):
        c.showPage();c.setFont('STSong-Light',12);c.drawString(35,height-40,file.stem)
        c.drawImage(str(file),30,100,width=width-60,height=height-170,preserveAspectRatio=True,anchor='c')
    c.save()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory')
    export_report(parser.parse_args().directory)
