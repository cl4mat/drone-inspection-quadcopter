#!/usr/bin/env python3
"""Open, offset two-floor inspection scene; collision geometry is not a nav map."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def generate(path):
    root=ET.Element('sdf',version='1.6');world=ET.SubElement(root,'world',name='inspection_building')
    ET.SubElement(world,'gravity').text='0 0 -9.81'
    ET.SubElement(ET.SubElement(world,'include'),'uri').text='model://sun'
    # No infinite ground plane: only the actual hall/corridor/stair floors exist.
    physics=ET.SubElement(world,'physics',name='ode',type='ode')
    ET.SubElement(physics,'max_step_size').text='.002';ET.SubElement(physics,'real_time_update_rate').text='500'
    scene=ET.SubElement(world,'scene');ET.SubElement(scene,'shadows').text='false'
    ET.SubElement(scene,'ambient').text='.65 .65 .65 1';ET.SubElement(scene,'background').text='.78 .83 .90 1'
    gui=ET.SubElement(world,'gui',fullscreen='false');camera=ET.SubElement(gui,'camera',name='user_camera')
    ET.SubElement(camera,'pose').text='0 6.8 70 0 1.5707 0'
    ET.SubElement(camera,'view_controller').text='orbit'
    # One static model, multiple links: fewer model-state and rendering updates.
    model=ET.SubElement(world,'model',name='open_two_floor_building');ET.SubElement(model,'static').text='true'
    def box(name,xyz,size,color='.62 .67 .71 1',rpy=(0,0,0)):
        link=ET.SubElement(model,'link',name=name)
        ET.SubElement(link,'pose').text=' '.join(map(str,(*xyz,*rpy)))
        for tag in ('collision','visual'):
            element=ET.SubElement(link,tag,name=name+'_'+tag)
            ET.SubElement(ET.SubElement(ET.SubElement(element,'geometry'),'box'),'size').text=' '.join(map(str,size))
            if tag=='visual':
                ET.SubElement(element,'cast_shadows').text='false';material=ET.SubElement(element,'material')
                for field in ('ambient','diffuse'):ET.SubElement(material,field).text=color
    def floor(name,x0,x1,y0,y1,z,color):box(name,((x0+x1)/2,(y0+y1)/2,z-.06),(x1-x0,y1-y0,.12),color)
    def wall_x(name,x,y0,y1,z0=0,z1=3.6):box(name,(x,(y0+y1)/2,(z0+z1)/2),(.2,y1-y0,z1-z0))
    def wall_y(name,y,x0,x1,z0=0,z1=3.6):box(name,((x0+x1)/2,y,(z0+z1)/2),(x1-x0,.2,z1-z0))
    # Both halls 16x12m. Lower hall at origin; 20x1.8m side corridors.
    floor('lower_hall_floor',-8,8,-6,6,0,'.74 .62 .39 1')
    floor('lower_left_corridor_floor',-28,-8,-.9,.9,0,'.79 .70 .48 1')
    floor('lower_right_corridor_floor',8,28,-.9,.9,0,'.79 .70 .48 1')
    wall_y('lower_hall_south_wall',-6.1,-8.1,8.1)
    wall_y('lower_hall_north_wall',6.1,-8.1,8.1)
    for x,side in ((-8.1,'left'),(8.1,'right')):
        wall_x('lower_hall_'+side+'_south',x,-6,-.9)
        wall_x('lower_hall_'+side+'_north',x,.9,6)
    wall_y('lower_left_corridor_south',-1,-28,-8)
    wall_y('lower_right_corridor_south',-1,8,28)
    wall_y('lower_left_corridor_north',1,-26.2,-8)
    wall_y('lower_right_corridor_north',1,8,26.2)
    # Upper hall is displaced toward +Y: no floor slab above the lower hall.
    floor('upper_hall_floor',-8,8,7.8,19.8,3.6,'.34 .61 .77 1')
    floor('upper_left_corridor_floor',-28,-8,12.9,14.7,3.6,'.48 .69 .81 1')
    floor('upper_right_corridor_floor',8,28,12.9,14.7,3.6,'.48 .69 .81 1')
    wall_y('upper_hall_front_wall',7.7,-8.1,8.1,3.6,7.2)
    wall_y('upper_hall_back_wall',19.9,-8.1,8.1,3.6,7.2)
    for x,side in ((-8.1,'left'),(8.1,'right')):
        wall_x('upper_hall_'+side+'_front',x,7.8,12.9,3.6,7.2)
        wall_x('upper_hall_'+side+'_back',x,14.7,19.8,3.6,7.2)
    wall_y('upper_left_corridor_front',12.8,-26.2,-8,3.6,7.2)
    wall_y('upper_right_corridor_front',12.8,8,26.2,3.6,7.2)
    wall_y('upper_left_corridor_back',14.8,-28,-8,3.6,7.2)
    wall_y('upper_right_corridor_back',14.8,8,28,3.6,7.2)
    # Two 1.5m stairs at x=+-27.1; turn into upper return corridors.
    for x,side in ((-27.1,'left'),(27.1,'right')):
        floor(side+'_stair_base',x-.9,x+.9,.9,6.9,0,'.72 .61 .39 1')
        for sign in (-1,1):
            floor(side+'_upper_stair_ledge_'+str(sign),x+sign*.825-.075,x+sign*.825+.075,.9,6.9,3.6,'.34 .61 .77 1')
        floor(side+'_upper_return_corridor',x-.9,x+.9,6.9,14.7,3.6,'.48 .69 .81 1')
        inner=x+1 if x<0 else x-1
        outer=x-1 if x<0 else x+1
        wall_x(side+'_shaft_outer_wall',outer,-1,14.8,0,7.2)
        wall_x(side+'_shaft_inner_wall',inner,.8,12.9,0,7.2)
        for step in range(24):
            top=(step+1)*.15
            box(side+'_stair_step_'+str(step),(x,1.025+step*.25,top/2),(1.5,.25,top),'.83 .48 .29 1')
        slope=math.atan2(3.6,6.)
        for sign in (-1,1):
            box(side+'_stair_rail_'+str(sign),(x+sign*.8,3.9,2.25),(.1,math.hypot(6.,3.6),.9),'.42 .47 .51 1',(slope,0,0))
    # Two alternating wall-side cabinets per horizontal corridor.
    # Each intrudes only .35m; remaining physical clear width is 1.425m.
    for z,level,y_center in ((0.,'lower',0.),(3.6,'upper',13.8)):
        for side,sign in (('left',-1),('right',1)):
            for i,(distance,lateral) in enumerate(((14.,.70),(21.,-.70))):
                box(level+'_'+side+'_fixed_obstacle_'+str(i),(sign*distance,y_center+lateral,z+.75),(.7,.35,1.5),'.84 .27 .18 1' if i==0 else '.25 .40 .72 1')
    # Low visual origin marker has no collision and does not affect lidar.
    origin=ET.SubElement(model,'link',name='origin_marker');ET.SubElement(origin,'pose').text='0 0 .004 0 0 0'
    visual=ET.SubElement(origin,'visual',name='origin_marker_visual')
    ET.SubElement(ET.SubElement(ET.SubElement(visual,'geometry'),'box'),'size').text='.45 .45 .004'
    material=ET.SubElement(visual,'material');ET.SubElement(material,'ambient').text='.95 .16 .14 1';ET.SubElement(material,'diffuse').text='.95 .16 .14 1'
    Path(path).write_text(ET.tostring(root,encoding='unicode')+'\n')


if __name__=='__main__':generate(Path(__file__).resolve().parents[1]/'worlds/inspection_building.world')
