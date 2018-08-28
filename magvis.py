#!/usr/bin/env python
import rospy
from sensor_msgs.msg import ChannelFloat32
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import numpy
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt 

magscale = pow(10,5)

pub_points = None
pub_markers = None

#X = []
#Y = []
#Z = []

# fig = None
point_cloud = PointCloud()
point_cloud.points = []
point_cloud.channels = []

def callback(data):
    x = magscale*data.magnetic_field.x
    y = magscale*data.magnetic_field.y
    z = magscale*data.magnetic_field.z
    s = numpy.sqrt(x*x + y*y + z*z)
    rospy.loginfo(" Magnetic Field(%d) : %.6lf %.6lf %.6lf %.6lf, (scale : %.6lf)", 
            len(point_cloud.points),
            s,
            x,
            y,
            z,
            magscale)

    global point_cloud
    point_cloud.header.frame_id = 'map'
    point_cloud.header.stamp = rospy.Time.now() 

    ch = ChannelFloat32()
    ch.name = 'intensity'
    ch.values = s,
    point_cloud.channels = ch,

    p = Point32();
    p.x = x
    p.y = y
    p.z = z
    point_cloud.points = p,

    pub_points.publish(point_cloud)

    marker_array = MarkerArray()
    marker_array.markers = []

    marker_x = Marker()
    marker_x.header.frame_id = 'map'
    marker_x.header.stamp = rospy.Time.now()
    marker_x.id = 0
    marker_x.type = marker_x.ARROW
    marker_x.action = marker_x.ADD
    marker_x.pose.orientation.w = 1
    marker_x.scale.x = 0.1
    marker_x.scale.y = 0.1
    marker_x.scale.z = 0.1
    marker_x.color.a = 1.0
    marker_x.color.r = 1.0
    marker_x.color.g = 0.0
    marker_x.color.b = 0.0
    marker_x.points = [Point32(0, 0, 0), Point32(s, 0, 0)]
    marker_array.markers.append(marker_x)

    marker_y = Marker()
    marker_y.header.frame_id = 'map'
    marker_y.header.stamp = rospy.Time.now()
    marker_y.id = 1
    marker_y.type = marker_y.ARROW
    marker_y.action = marker_y.ADD
    marker_y.pose.orientation.w = 1
    marker_y.scale.x = 0.1
    marker_y.scale.y = 0.1
    marker_y.scale.z = 0.1
    marker_y.color.a = 1.0
    marker_y.color.r = 0.0
    marker_y.color.g = 1.0
    marker_y.color.b = 0.0
    marker_y.points = [Point32(0, 0, 0), Point32(0,s, 0)]
    marker_array.markers.append(marker_y)

    marker_z = Marker()
    marker_z.header.frame_id = 'map'
    marker_z.header.stamp = rospy.Time.now()
    marker_z.id = 2
    marker_z.type = marker_z.ARROW
    marker_z.action = marker_z.ADD
    marker_z.pose.orientation.w = 1
    marker_z.scale.x = 0.1
    marker_z.scale.y = 0.1
    marker_z.scale.z = 0.1
    marker_z.color.a = 1.0
    marker_z.color.r = 0.0
    marker_z.color.g = 0.0
    marker_z.color.b = 1.0
    marker_z.points = [Point32(0, 0, 0), Point32(0,0,s)]
    marker_array.markers.append(marker_z)

    pub_markers.publish(marker_array)


#    if len(X) > 100:
#        del X[0]
#        del Y[0]
#        del Z[0] 

    #X.append(x)
    #Y.append(y)
    #Z.append(z)
    
    # global ax
    # global count
    #if count%10 == 0:
        # ax.clear
        # ax.plot(X)
        # plt.pause(0.0000000001)

def listener():
    global pub_points, pub_markers
    rospy.init_node('magvis', anonymous=True)
    rospy.Subscriber("/mavros/imu/mag", MagneticField, callback, queue_size=100)
    pub_points = rospy.Publisher("/magvis/point_visual", PointCloud, queue_size = 10) 
    pub_markers = rospy.Publisher("/magvis/axis_visual", MarkerArray, queue_size = 10)

    # global fig, ax
    # plt.ion()
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d'),
    # ax = fig.add_subplot(111)
    # plt.show(block=True)
    rospy.spin()

if __name__ == '__main__':
    listener()

