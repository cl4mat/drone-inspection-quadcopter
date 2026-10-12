#!/usr/bin/env python3
"""Display-only floor slicing. Planning and mapping never consume these topics."""
import copy
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Float64


class FloorView:
    def __init__(self):
        self.ground=-.17;self.offset=0.;self.cache={};self.publishers={}
        for source,target in [('drone/cloud_fcu_world','drone/view/cloud'),('grid_map/occupancy_inflate','drone/view/voxels'),('drone/prior_map_preview','drone/view/prior')]:
            self.publishers[source]=rospy.Publisher('/'+target,PointCloud2,queue_size=1,latch=target.endswith('prior'))
            rospy.Subscriber('/'+source,PointCloud2,self.receive,callback_args=source,queue_size=1)
        rospy.Subscriber('/drone/display_floor_ground',Float64,self.floor,queue_size=1)
        rospy.Subscriber('/drone/map_transform',PoseStamped,lambda m:setattr(self,'offset',m.pose.position.z),queue_size=1)

    def floor(self,m):
        if not np.isfinite(m.data):return
        self.ground=m.data
        for source,message in list(self.cache.items()):self.receive(message,source)

    def receive(self,message,source):
        self.cache[source]=message;pub=self.publishers[source]
        if not pub.get_num_connections() and not source.endswith('preview'):return
        if self.ground<-100 or not message.width:pub.publish(message);return
        field=next((f for f in message.fields if f.name=='z'),None)
        if field is None or field.datatype!=PointField.FLOAT32:return
        try:
            records=np.ndarray((message.height,message.width,message.point_step),np.uint8,buffer=message.data,
                               strides=(message.row_step,message.point_step,1))
            z=np.ndarray((message.height,message.width),dtype='>f4' if message.is_bigendian else '<f4',buffer=message.data,
                         offset=field.offset,strides=(message.row_step,message.point_step))
            map_z=z+(self.offset if message.header.frame_id=='odom' else 0.)
            keep=np.isfinite(z)&(map_z>=self.ground-.2)&(map_z<=self.ground+2.8)
            output=copy.copy(message);output.height=1;output.width=int(keep.sum());output.row_step=output.width*output.point_step
            output.data=records[keep].tobytes();pub.publish(output)
        except (ValueError,TypeError):rospy.logwarn_throttle(5.,'Display cloud layout is invalid')


if __name__=='__main__':
    rospy.init_node('drone_map_floor_view');FloorView();rospy.spin()
