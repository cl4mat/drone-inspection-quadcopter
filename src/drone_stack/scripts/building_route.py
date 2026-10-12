#!/usr/bin/env python3
"""Offline building topology/route from measured DRONE_GRID archives only."""
import hashlib
import heapq
import json
import math
import os
import tempfile
import time
from pathlib import Path
import numpy as np
from scipy import ndimage as ndi
from scipy.spatial import cKDTree


def digest(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for block in iter(lambda:f.read(1048576),b''):h.update(block)
    return h.hexdigest()


def task_digest(task):
    payload={k:task[k] for k in ('version','map_digest','topology','segments','origin','home_approach','length_m','expected_duration_s')}
    return hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def atomic_json(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=None
    try:
        with tempfile.NamedTemporaryFile('w',dir=path.parent,delete=False) as f:
            tmp=Path(f.name);json.dump(data,f,ensure_ascii=False,indent=2,allow_nan=False);f.flush();os.fsync(f.fileno())
        tmp.replace(path)
    finally:
        if tmp is not None:tmp.unlink(missing_ok=True)


def route_window(route,progress,horizon=10.):
    result=[];arc=0.;end=progress+horizon
    for a,b in zip(route,route[1:]):
        length=math.dist(a,b)
        if length<1e-9:continue
        if arc+length>=progress and arc<=end:
            lo=max(0.,(progress-arc)/length);hi=min(1.,(end-arc)/length)
            start=(np.array(a)+(np.array(b)-a)*lo).tolist();stop=(np.array(a)+(np.array(b)-a)*hi).tolist()
            if not result:result.append(start)
            if math.dist(result[-1],stop)>1e-6:result.append(stop)
        arc+=length
        if arc>=end:break
    return result or [route[-1]]


def skeleton(mask):
    """Vectorised Zhang-Suen thinning, no extra image processing dependency."""
    image=np.pad(mask.astype(bool),1)
    for _ in range(max(mask.shape)):
        changed=False
        for second in (False,True):
            c=image[1:-1,1:-1]
            p=[image[:-2,1:-1],image[:-2,2:],image[1:-1,2:],image[2:,2:],
               image[2:,1:-1],image[2:,:-2],image[1:-1,:-2],image[:-2,:-2]]
            n=sum(v.astype(np.uint8) for v in p)
            transitions=sum((~p[i]&p[(i+1)%8]).astype(np.uint8) for i in range(8))
            a,b=(p[0]&p[2]&p[6],p[0]&p[4]&p[6]) if second else (p[0]&p[2]&p[4],p[2]&p[4]&p[6])
            remove=c&(n>=2)&(n<=6)&(transitions==1)&~a&~b
            if remove.any():c[remove]=False;changed=True
        if not changed:break
    return image[1:-1,1:-1]


class BuildingMap:
    def __init__(self,path):
        self.path=str(Path(path).resolve());self.hash=digest(self.path)
        if Path(path).stat().st_size>256*1024*1024:raise ValueError('地图超过256MB')
        with open(path) as f:
            h=f.readline().split()
            if len(h) not in (10,14) or h[0] not in ('DRONE_GRID_V1','DRONE_GRID_V2') or h[1]!='odom':raise ValueError('地图格式不兼容')
            self.res=float(h[2]);self.origin=np.array(h[3:6],float);self.shape=tuple(map(int,h[6:9]));count=int(h[9])
            if not .05<=self.res<=.25 or any(v<=0 for v in self.shape) or np.prod(self.shape)>50000000 or not 1<=count<=np.prod(self.shape):raise ValueError('地图网格无效')
            self.translation=np.array(h[10:13],float) if len(h)==14 else np.zeros(3)
            yaw=math.radians(float(h[13])) if len(h)==14 else 0.
            self.rotation=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1.]])
            if not np.isfinite(np.r_[self.origin,self.translation,yaw]).all():raise ValueError('地图坐标元数据无效')
            occupied=np.zeros(self.shape,bool);free=np.zeros(self.shape,bool);seen=np.zeros(occupied.size,bool)
            offset=0
            while offset<count:
                length=min(250000,count-offset)
                records=np.loadtxt(f,dtype=np.int32,max_rows=length,ndmin=2)
                if records.shape!=(length,3):raise ValueError('地图记录不完整')
                address,evidence,known=records.T
                if (np.any(address<0) or np.any(address>=occupied.size) or
                    np.unique(address).size!=length or np.any(seen[address]) or
                    np.any((evidence<0)|(evidence>3)) or np.any((known<0)|(known>1)) or
                    np.any((evidence!=0)==(known!=0))):raise ValueError('地图记录非法')
                seen[address]=True;occupied.flat[address]=evidence!=0;free.flat[address]=known!=0
                offset+=length
            if f.read().strip():raise ValueError('地图有多余记录')
        # Conservative cell-centre mask for live inflation plus 5 cm clearance.
        padding=[math.ceil(.4/self.res)+1]*2+[math.ceil(.25/self.res)+1]
        self.safe=free&~ndi.maximum_filter(occupied,size=tuple(2*v+1 for v in padding),mode='constant',cval=1)
        self.occupied=occupied
        self.free=free
        # Horizontal surface support: a wall/rail column alone is not a floor.
        horizontal=ndi.convolve(occupied.astype(np.uint8),np.ones((3,3,1),np.uint8),mode='constant')>=5
        self.horizontal_surface=horizontal
        rise=round(1.2/self.res)
        supported=np.zeros(self.shape,bool)
        # Interpolate only short (20 cm) holes in the measured tread surface.
        # This is support classification, not adding occupied/free map voxels.
        local_surface=ndi.maximum_filter(horizontal,size=(5,5,1),mode='constant')
        supported[:,:,rise:]=local_surface[:,:,:-rise]
        # Adjacent treads differ by a small rise; permit smoothing within +/-20cm.
        # 1.0..1.4 m nominal support band plus 10 cm measured-surface
        # uncertainty; occupied/free collision evidence is unchanged.
        band=math.ceil(.3/self.res-1e-9)
        self.supported=self.safe&ndi.maximum_filter1d(supported,size=2*band+1,axis=2,mode='constant')
        self.support_deviation=np.full(self.shape,255,np.uint8)
        for offset in range(-band,band+1):
            if offset>=0:
                dest=self.support_deviation[:,:,offset:];source=supported[:,:,:self.shape[2]-offset]
            else:
                dest=self.support_deviation[:,:,:offset];source=supported[:,:,-offset:]
            dest[source]=np.minimum(dest[source],abs(offset))



    def xyz(self,index):
        return ((self.origin+(np.asarray(index)+.5)*self.res)@self.rotation.T+self.translation).tolist()

    def index(self,point):
        return tuple(np.floor(((np.asarray(point)-self.translation)@self.rotation-self.origin)/self.res).astype(int))

    def valid(self,index):
        return all(0<=v<n for v,n in zip(index,self.shape)) and self.safe[index]

    def nearest(self,point,mask=None,max_distance=.30):
        index=self.index(point)
        if mask is None and self.valid(index):return index
        if mask is not None and all(0<=v<n for v,n in zip(index[:2],mask.shape)) and mask[index[:2]]:return (*index[:2],index[2])
        radius=math.ceil(max_distance/self.res);candidates=[]
        for dx in range(-radius,radius+1):
            for dy in range(-radius,radius+1):
                k=(index[0]+dx,index[1]+dy,index[2])
                if (self.valid(k) if mask is None else 0<=k[0]<mask.shape[0] and 0<=k[1]<mask.shape[1] and mask[k[:2]]):
                    d=math.dist(self.xyz(k),point)
                    if d<=max_distance:candidates.append((d,k))
        if not candidates:raise ValueError('入口/原点/修正点不在已观测安全自由空间，需补建图')
        return min(candidates)[1]

    def line_cells(self,start,end):
        cell=np.array(start,dtype=int);goal=np.array(end,dtype=int);delta=goal-cell
        step=np.sign(delta).astype(int)
        td=np.divide(1.,np.abs(delta),out=np.full(3,np.inf),where=delta!=0)
        tm=td*.5
        yield tuple(cell)
        while not np.array_equal(cell,goal):
            edge=float(tm.min());axes=np.flatnonzero(np.abs(tm-edge)<1e-10)
            # Closed-voxel supercover at simultaneous face/edge crossings.
            for subset in range(1,1<<len(axes)):
                neighbour=cell.copy()
                for j,axis in enumerate(axes):
                    if subset&(1<<j):neighbour[axis]+=step[axis]
                yield tuple(neighbour)
            cell[axes]+=step[axes];tm[axes]+=td[axes]

    def shortcut(self,route,available):
        if len(route)<3:return route
        result=[route[0]];i=0
        while i<len(route)-1:
            following=i+1
            for j in range(len(route)-1,i,-1):
                if all(available(k) for k in self.line_cells(route[i],route[j])):following=j;break
            result.append(route[following]);i=following
        return result

    def astar(self,start,end,mask=None,supported=False):
        if start==end:return [start]
        def available(k):
            return (self.valid(k) and (not supported or self.supported[k])) if mask is None else all(0<=k[i]<mask.shape[i] for i in (0,1)) and mask[k[:2]]
        if all(available(k) for k in self.line_cells(start,end)):return [start,end]
        deadline=time.monotonic()+20.;queue=[(math.dist(start,end),0.,start)];cost={start:0.};parent={};closed=set()
        # Axial graph prevents diagonal corner cuts. Dense sampling is retained.
        moves=[(a,d) for a in range(2 if mask is not None else 3) for d in (-1,1)]
        while queue:
            _,g,node=heapq.heappop(queue)
            if node in closed:continue
            if node==end:
                route=[node]
                while node!=start:node=parent[node];route.append(node)
                return self.shortcut(route[::-1],available)
            closed.add(node)
            if len(closed)>400000 or len(closed)%1024==0 and time.monotonic()>deadline:raise ValueError('连通搜索预算耗尽，需补图/修正连接，不生成可执行任务')
            for axis,delta in moves:
                candidate=list(node);candidate[axis]+=delta;candidate=tuple(candidate)
                valid=(self.valid(candidate) and (not supported or self.supported[candidate])) if mask is None else all(0<=candidate[i]<mask.shape[i] for i in (0,1)) and mask[candidate[:2]]
                step_cost=1.+(float(self.support_deviation[candidate])*.5 if valid and supported else 0.)
                next_cost=g+step_cost
                if valid and next_cost<cost.get(candidate,float('inf')):
                    cost[candidate]=next_cost;parent[candidate]=node;heapq.heappush(queue,(next_cost+math.dist(candidate,end),next_cost,candidate))
        raise ValueError('已观测安全空间不连通：%s → %s (supported=%s)'%(self.xyz(start),self.xyz(end),supported))

    def topology(self,height=1.2):
        floors=[]
        # Choose broad floor modes, not the first sparse/noisy horizontal slice.
        candidates=[]
        for k in range(self.shape[2]-round(height/self.res)):
            surfaces=np.count_nonzero(self.horizontal_surface[:,:,k])
            target=k+round(height/self.res)
            area=np.count_nonzero(self.supported[:,:,target])
            if surfaces>=100 and area*self.res**2>=10.:
                candidates.append((area,k))
        chosen=[]
        for area,k in sorted(candidates,reverse=True):
            if all(abs(k-old)*self.res>=2. for old in chosen):chosen.append(k)
            if len(chosen)>=4:break
        for k in sorted(chosen):
            z=self.xyz((0,0,k))[2];target=k+round(height/self.res)
            mask=self.supported[:,:,target]
            floor=dict(id=len(floors),ground_z=z,altitude=self.xyz((0,0,target))[2],regions=[])
            clearance=ndi.distance_transform_edt(mask)*self.res
            cores,n=ndi.label(clearance>=.9)
            for label in range(1,n+1):
                coords=np.argwhere(cores==label)
                if len(coords)*self.res**2<1.:continue
                lo=np.maximum(0,coords.min(axis=0)-round(1.4/self.res));hi=np.minimum(np.array(mask.shape)-1,coords.max(axis=0)+round(1.4/self.res))
                polygon=[self.xyz((*q,target))[:2] for q in [(lo[0],lo[1]),(hi[0],lo[1]),(hi[0],hi[1]),(lo[0],hi[1])]]
                floor['regions'].append(dict(id=f'hall_{len(floors)}_{label}',kind='hall',polygon=polygon))
            floors.append(floor)
        if not floors:raise ValueError('未识别到具有已观测自由空间的楼面，请先完成建图')
        return dict(version=1,map_digest=self.hash,map_path=self.path,origin=[0.,0.,0.],height_above_floor=height,
                    hall_inset_m=.8,speeds=dict(hall=2.,corridor=1.,stair=.4,home=1.),strip_spacing_m=1.,floors=floors,connections=[])

    def floor_traversal(self,floor):
        k=self.index([0,0,floor['altitude']])[2]
        ground=k-round((floor['altitude']-floor['ground_z'])/self.res)
        # Floor classification tolerates local surface-height scatter. Flight
        # clearance still requires the exact observed-free, inflated-safe slice.
        band=max(1,math.ceil(.4/self.res))
        support=np.any(self.horizontal_surface[:,:,max(0,ground-band):min(self.shape[2],ground+band+1)],axis=2)
        support=ndi.maximum_filter(support,size=3)
        mask=self.safe[:,:,k]&support
        labels,n=ndi.label(mask)
        if n!=1:
            sizes=np.bincount(labels.ravel())[1:]
            # Wall tops at a lower storey can form disconnected horizontal
            # patches at this height. Keep the component containing the hall;
            # never silently join them across unobserved/occupied space.
            centres=[self.index([*np.asarray(r['polygon']).mean(axis=0),floor['altitude']])
                     for r in floor.get('regions',[]) if r.get('kind')=='hall']
            hall_labels={int(labels[c[:2]]) for c in centres if all(0<=c[i]<labels.shape[i] for i in (0,1)) and labels[c[:2]]}
            if len(hall_labels)>1:raise ValueError('大厅之间的已观测安全通道未连通')
            selected=next(iter(hall_labels)) if hall_labels else int(np.argmax(sizes)+1)
            floor['excluded_components_m2']=[float(v*self.res**2) for i,v in enumerate(sizes,1) if i!=selected]
            mask=labels==selected
        regions=[r for r in floor.get('regions',[]) if r.get('kind')=='hall']
        if not regions:raise ValueError('楼层没有可巡检大厅区域')
        centre=np.asarray(regions[0]['polygon']).mean(axis=0)
        root=self.nearest([*centre,floor['altitude']],mask,max_distance=1.)
        walk=[root]
        if not getattr(self,'corridors_in_connections',False):
            # Visit the terminal of each corridor component, instead of DFS
            # over pixel skeleton cycles/spurs that create hundreds of stops.
            outside=mask.copy()
            for region in regions:
                polygon=np.asarray(region['polygon']);lo=polygon.min(axis=0);hi=polygon.max(axis=0)
                ia=self.index([*lo,floor['altitude']]);ib=self.index([*hi,floor['altitude']])
                outside[max(0,ia[0]):min(mask.shape[0],ib[0]+1),max(0,ia[1]):min(mask.shape[1],ib[1]+1)]=False
            labels,n=ndi.label(outside)
            for label in range(1,n+1):
                points=np.argwhere(labels==label)
                if len(points)*self.res**2<1.:continue
                endpoint=max(points,key=lambda q:math.dist(q,root[:2]));target=(*endpoint,k)
                walk+=self.astar(walk[-1],target,mask)[1:]
                walk+=self.astar(walk[-1],root,mask)[1:]
        # Hall lawnmower strips are added to the measured corridor skeleton.
        for region in floor.get('regions',[]):
            if region.get('kind')!='hall':continue
            polygon=np.asarray(region['polygon'],float)
            if polygon.shape!=(4,2) or not np.isfinite(polygon).all():raise ValueError('大厅边界需为四个有限坐标点')
            inset=float(self.current_inset)
            previous=np.roll(polygon,1,axis=0)-polygon;following=np.roll(polygon,-1,axis=0)-polygon
            edge=np.linalg.norm(following,axis=1)
            if np.any(edge<=2*inset):raise ValueError('大厅太窄，无法按内缩边界生成巡检环线')
            inward=polygon+previous/np.linalg.norm(previous,axis=1)[:,None]*inset+following/edge[:,None]*inset
            spacing=float(getattr(self,'strip_spacing',1.))
            if not math.isfinite(spacing) or not .3<=spacing<=2.:raise ValueError('条带间距需为0.3～2m')
            # Inset quadrilateral, 1 m maximum adjacent lane spacing. This
            # reference is stored and used by the flight controller, not display.
            width=max(np.linalg.norm(inward[3]-inward[0]),np.linalg.norm(inward[2]-inward[1]))
            lanes=max(1,math.ceil(width/spacing))
            targets=[]
            for lane in range(lanes+1):
                u=lane/lanes
                ends=[inward[0]*(1-u)+inward[3]*u,inward[1]*(1-u)+inward[2]*u]
                if lane%2:ends.reverse()
                targets.extend(self.nearest([*xy,floor['altitude']],mask,max_distance=1.) for xy in ends)
            centre=self.nearest([*polygon.mean(axis=0),floor['altitude']],mask)
            stop=min(range(len(walk)),key=lambda i:math.dist(walk[i],centre))
            start=walk[stop];addition=[];last=start
            for target in targets+[start]:
                addition+=self.astar(last,target,mask)[1:];last=target
            walk[stop+1:stop+1]=addition
        return walk

    def split_turns(self,indices):
        """Simplify axial stairs then split reversals: stop/turn at actual landings."""
        if len(indices)<3:return [indices]
        clean=[indices[0]]
        for a,b,c in zip(indices,indices[1:],indices[2:]):
            u=np.asarray(b)-a;v=np.asarray(c)-b
            if np.any(np.cross(u,v)) or np.dot(u,v)<0:clean.append(b)
        clean.append(indices[-1]);expanded=[clean[0]];stops=set()
        for i in range(1,len(clean)-1):
            expanded.append(clean[i])
            if i+2>=len(clean):continue
            incoming=np.asarray(clean[i][:2])-clean[i-1][:2]
            outgoing=np.asarray(clean[i+2][:2])-clean[i+1][:2]
            if clean[i][2]==clean[i+1][2] and np.linalg.norm(incoming)>0 and np.linalg.norm(outgoing)>0 and np.dot(incoming,outgoing)/(np.linalg.norm(incoming)*np.linalg.norm(outgoing))<-.5:
                midpoint=tuple(((np.asarray(clean[i])+clean[i+1])//2).tolist())
                if midpoint not in (clean[i],clean[i+1]):expanded.append(midpoint);stops.add(len(expanded)-1)
        expanded.append(clean[-1])
        for i in range(1,len(expanded)-1):
            u=np.asarray(expanded[i][:2])-expanded[i-1][:2];v=np.asarray(expanded[i+1][:2])-expanded[i][:2]
            if np.linalg.norm(u)>0 and np.linalg.norm(v)>0 and np.dot(u,v)/(np.linalg.norm(u)*np.linalg.norm(v))<-.5:stops.add(i)
        bounds=[0]+sorted(stops)+[len(expanded)-1]
        return [expanded[a:b+1] for a,b in zip(bounds,bounds[1:]) if b>a]

    def generate(self,topology):
        if topology.get('map_digest')!=self.hash:raise ValueError('拓扑与地图内容不匹配')
        proof=topology.get('validated_flight_connections')
        if proof:
            if proof.get('map_digest')!=self.hash or proof.get('source_topic')!='/mavros/local_position/odom':
                raise ValueError('已飞连接证据与地图不匹配')
            radius=float(proof.get('radius_m',0.))
            if not .1<=radius<=.4:raise ValueError('已飞连接管道半径无效')
            extent=math.ceil(radius/self.res)
            offsets=np.array([(x,y,z) for x in range(-extent,extent+1)
                for y in range(-extent,extent+1) for z in range(-extent,extent+1)
                if (x*x+y*y+z*z)*self.res**2<=radius**2])
            tube=np.zeros(self.shape,bool)
            for connection in proof.get('connections',[]):
                points=np.asarray(connection.get('points',[]),float)
                if (connection.get('completed_mapping_goals',0)<2 or points.ndim!=2 or
                    points.shape[1]!=3 or not 2<=len(points)<=100000 or not np.isfinite(points).all()):
                    raise ValueError('需要已完成建图目标的实际位姿连接证据')
                for source in connection.get('sources',[]):
                    for name in ('events','telemetry'):
                        if digest(source[name])!=source[name+'_sha256']:
                            raise ValueError('建图连接原始日志摘要不匹配')
                indices=np.array([self.index(p) for p in points])
                cells=(indices[:,None,:]+offsets[None,:,:]).reshape(-1,3)
                cells=cells[np.all((cells>=0)&(cells<np.array(self.shape)),axis=1)]
                tube[tuple(cells.T)]=True
            # Proven, previously flown free connections supplement missing
            # tread observations. Never create free cells or erase occupancy.
            self.supported=self.supported|(tube&self.safe)
            self.flight_connection_tree=cKDTree(np.concatenate([np.asarray(c['points'],float) for c in proof['connections']]))
        self.current_inset=float(topology.get('hall_inset_m',.8))
        self.strip_spacing=float(topology.get('strip_spacing_m',1.))
        self.corridors_in_connections=bool(topology.get('corridors_in_connections',False))
        if not math.isfinite(self.current_inset) or not .6<=self.current_inset<=2.:raise ValueError('大厅内缩距离需为0.6～2.0m')
        floors=topology.get('floors',[])
        if not 1<=len(floors)<=4:raise ValueError('楼层数量需为1～4')
        for i,f in enumerate(floors):
            if f.get('id')!=i or not all(math.isfinite(float(f.get(k,float('nan')))) for k in ('altitude','ground_z')):raise ValueError('楼层ID/高度无效')
            if not .8<=f['altitude']-f['ground_z']<=2.2:raise ValueError('离地巡检高度需为0.8～2.2m')
            ground=self.index([0,0,f['ground_z']])[2]
            if not 0<=ground<self.shape[2] or np.count_nonzero(self.horizontal_surface[:,:,ground])<100:raise ValueError('修正楼面没有实际水平观测支持')
        speeds=topology.get('speeds',dict(hall=.4,corridor=.3,stair=.2,home=.3))
        limits=dict(hall=2.,corridor=1.,stair=.4,home=1.)
        if not all(k in speeds and math.isfinite(float(speeds[k])) and .1<=float(speeds[k])<=limits[k] for k in limits):raise ValueError('速度上限：大厅2、走廊1、楼梯0.4m/s')
        for height in sorted(set(round(f['altitude']-f['ground_z'],6) for f in floors)):
            if abs(height-1.2)<1e-6:continue
            rise=round(height/self.res);band=math.ceil(.3/self.res-1e-9)
            surface=ndi.maximum_filter(self.horizontal_surface,size=(5,5,1),mode='constant')
            elevated=np.zeros(self.shape,bool);elevated[:,:,rise:]=surface[:,:,:-rise]
            self.supported|=self.safe&ndi.maximum_filter1d(elevated,size=2*band+1,axis=2,mode='constant')
        connections=topology.get('connections',[])
        generated_connections=[]
        home=self.nearest([0.,0.,float(floors[0]['altitude'])]);segments=[];current=home
        def add(kind,indices,floor_id=None):
            if len(indices)<2:return
            # Preserve sampled stair profile; no smooth shortcut through walls.
            pieces=([indices] if kind=='hall' else self.split_turns(indices))
            if topology.get('execution_mode')=='waypoints':
                clean=[indices[0]]
                for a,b,c in zip(indices,indices[1:],indices[2:]):
                    u=np.asarray(b)-a;v=np.asarray(c)-b
                    if np.linalg.norm(np.cross(u,v))>1e-6 or np.dot(u,v)<0:clean.append(b)
                clean.append(indices[-1])
                pieces=[];last=clean[0]
                for point in clean[1:]:
                    if math.dist(last,point)*self.res>=.15:pieces.append([last,point]);last=point
                if pieces and math.dist(pieces[-1][-1],clean[-1])*self.res>1e-6:pieces[-1][-1]=clean[-1]
            for piece in pieces:
                segments.append(dict(kind=kind,floor_id=floor_id,speed=float(speeds[kind]),points=[self.xyz(i) for i in piece]))
        def add_connector(indices,floor_id=None):
            if len(indices)<2:return
            dense=[indices[0]]
            for a,b in zip(indices,indices[1:]):
                count=max(1,math.ceil(math.dist(a,b)*self.res/.25))
                dense.extend(tuple(np.array(a)+(np.array(b)-a)*i/count) for i in range(1,count+1))
            group=[dense[0]];kind=None
            for point in dense[1:]:
                location=self.xyz(point);z=location[2]
                if hasattr(self,'flight_connection_tree'):
                    new_kind='stair' if self.flight_connection_tree.query(location)[0]<=.45 else 'corridor'
                else:
                    new_kind='corridor' if min(abs(z-f['altitude']) for f in floors)<.20 else 'stair'
                if kind is not None and new_kind!=kind:
                    add(kind,group,floor_id);group=[group[-1]]
                group.append(point);kind=new_kind
            add(kind,group,floor_id)
        for floor in floors:
            walk=self.floor_traversal(floor)
            crossing=abs(current[2]-walk[0][2])*self.res>1.
            correction=next((c for c in connections if c.get('from_floor')==floor['id']-1 and c.get('to_floor')==floor['id'] and c.get('anchors')),None)
            anchors=[current]+[self.nearest(p) for p in correction['anchors']]+[walk[0]] if correction else [current,walk[0]]
            connector=[current]
            for start,end in zip(anchors,anchors[1:]):connector+=self.astar(start,end,supported=crossing)[1:]

            if crossing:
                generated_connections.append(dict(kind='stair',from_floor=floor['id']-1,to_floor=floor['id'],
                    supported_height_band_m=[1.,1.4],anchors=correction.get('anchors',[]) if correction else [],points=[self.xyz(i) for i in connector]))
            add_connector(connector,floor['id'])
            # Width class comes from measured clearance, not Gazebo object labels.
            k=walk[0][2];clearance=ndi.distance_transform_edt(self.safe[:,:,k])*self.res
            group=[walk[0]];kind='hall' if self.corridors_in_connections or clearance[walk[0][:2]]>=.9 else 'corridor'
            for point in walk[1:]:
                new_kind='hall' if self.corridors_in_connections or clearance[point[:2]]>=.9 else 'corridor'
                group.append(point)
                if new_kind!=kind:
                    add(kind,group,floor['id']);group=[point];kind=new_kind
            add(kind,group,floor['id']);current=walk[-1]
            floor['corridors']=[s['points'] for s in segments if s['kind']=='corridor' and s['floor_id']==floor['id']]
        anchors=[current]+[self.nearest(p) for p in topology.get('return_connection',{}).get('anchors',[])]+[home]
        connector=[current]
        for start,end in zip(anchors,anchors[1:]):connector+=self.astar(start,end,supported=len(floors)>1)[1:]
        add_connector(connector,0)
        topology['connections']=generated_connections
        result=dict(version=1,map_digest=self.hash,map_path=self.path,origin=[0.,0.,0.],home_approach=self.xyz(home),
                    state='READY',topology=topology,segments=segments,execution_mode=topology.get('execution_mode','continuous'),created_at=time.time())
        if not segments:raise ValueError('没有可执行路线')
        result['length_m']=sum(math.dist(a,b) for s in segments for a,b in zip(s['points'],s['points'][1:]))
        # Sufficient duration for large buildings; absence of progress has its own timeout.
        result['expected_duration_s']=sum(sum(math.dist(a,b) for a,b in zip(s['points'],s['points'][1:]))/s['speed'] for s in segments)+120
        result['task_digest']=task_digest(result)
        return result


def build(path,correction=None,required_floors=2):
    grid=BuildingMap(path);topology=correction or grid.topology()
    atomic_json(str(path)+'.topology.json',topology)
    task_path=str(path)+'.inspection.json'
    try:
        if len(topology.get('floors',[]))<required_floors:raise ValueError('尚未观测到全部%d层楼面，需继续建图'%required_floors)
        task=grid.generate(topology)
    except Exception as exc:
        atomic_json(task_path,dict(version=1,map_digest=grid.hash,state='DRAFT',message=str(exc),topology=topology))
        raise
    if digest(path)!=grid.hash:raise ValueError('生成期间地图被更改，请重新保存')
    atomic_json(str(path)+'.topology.json',task['topology'])
    atomic_json(task_path,task);return task
