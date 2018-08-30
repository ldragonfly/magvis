#!/usr/bin/env python
import rospy
from sensor_msgs.msg import ChannelFloat32
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_conjugate, quaternion_multiply

import math
import numpy
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt 

from threading import Thread, Lock

mutex_mag = Lock()
mag_x = 0
mag_y = 0
mag_z = 0

magscale = pow(10,5)

point_cloud = PointCloud()
point_cloud.points = []
point_cloud.channels = []

def mag_callback(data):
    global mag_x, mag_y, mag_z
    mutex_mag.acquire()
    mag_x = data.magnetic_field.x
    mag_y = data.magnetic_field.y
    mag_z = data.magnetic_field.z
    mutex_mag.release()

    x = magscale*data.magnetic_field.x
    y = magscale*data.magnetic_field.y
    z = magscale*data.magnetic_field.z
    s = numpy.sqrt(x*x + y*y + z*z)
#    rospy.loginfo(" Magnetic Field(%d) : %.6lf %.6lf %.6lf %.6lf, (scale : %.6lf)", 
#            len(point_cloud.point),
#            s,
#            x,
#            y,
#            z,
#            magscale)

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

    pub_point.publish(point_cloud)

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

def qv_mult(q1, v1):
    q2 = list(v1)
    q2.append(0.0)
    return quaternion_multiply(
            quaternion_multiply(q1, q2),
            quaternion_conjugate(q1)
            )[:3]

def predict(att, P_att):
    Q = numpy.diag([0.1, 0.1, 0.1])
    P_att = P_att + Q
    return (att, P_att)

def update(att, P_att):
    pass


def odom_callback(data): 

    mutex_mag.acquire()
    mag = [mag_x, mag_y, mag_z]
    mutex_mag.release()

    #(att, P_att) = predict(5, numpy.diag([0.1, 0.1, 0.1]))
    #print(P_att)

    q = (
        data.pose.pose.orientation.x, 
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    mag_global = qv_mult(q, mag)

    euler = euler_from_quaternion(q)
    q_rp = quaternion_from_euler(euler[0], euler[1], 0)

    mag_yaw = qv_mult(q_rp, mag)
    yaw = math.atan2(mag_yaw[0], mag_yaw[1])
    
    print(yaw)
    q_rpy = quaternion_from_euler(euler[0], euler[1], yaw)
    q_rp = quaternion_from_euler(euler[0], euler[1], 0)
    q_yaw = quaternion_from_euler(0, 0, yaw)
    print(q_rpy)

    point_plane_cloud = PointCloud()
    point_plane_cloud.header.frame_id = 'map'
    point_plane_cloud.header.stamp = rospy.Time.now() 
    p = Point32();
    p.x = magscale*mag_yaw[0]
    p.y = magscale*mag_yaw[1]
    p.z = 0
    point_plane_cloud.points = p,
    pub_point_plane.publish(point_plane_cloud)

    point_global_cloud = PointCloud()
    point_global_cloud.header.frame_id = 'map'
    point_global_cloud.header.stamp = rospy.Time.now() 
    p = Point32();
    p.x = magscale*mag_global[0]
    p.y = magscale*mag_global[1]
    p.z = magscale*mag_global[2]
    point_global_cloud.points = p,
    pub_point_global.publish(point_global_cloud)

    odom_rp = Odometry()
    odom_rp.header.frame_id = 'map'
    odom_rp.header.stamp = rospy.Time.now()
    odom_rp.pose.pose.orientation.x = q_rp[0]
    odom_rp.pose.pose.orientation.y = q_rp[1]
    odom_rp.pose.pose.orientation.z = q_rp[2]
    odom_rp.pose.pose.orientation.w = q_rp[3]
    pub_odom_rp.publish(odom_rp)


    odom_rpy = Odometry()
    odom_rpy.header.frame_id = 'map'
    odom_rpy.header.stamp = rospy.Time.now()
    odom_rpy.pose.pose.orientation.x = q_rpy[0]
    odom_rpy.pose.pose.orientation.y = q_rpy[1]
    odom_rpy.pose.pose.orientation.z = q_rpy[2]
    odom_rpy.pose.pose.orientation.w = q_rpy[3]
    pub_odom_rpy.publish(odom_rpy)

    odom_yaw = Odometry()
    odom_yaw.header.frame_id = 'map'
    odom_yaw.header.stamp = rospy.Time.now()
    odom_yaw.pose.pose.orientation.x = q_yaw[0]
    odom_yaw.pose.pose.orientation.y = q_yaw[1]
    odom_yaw.pose.pose.orientation.z = q_yaw[2]
    odom_yaw.pose.pose.orientation.w = q_yaw[3]
    pub_odom_yaw.publish(odom_yaw)



def listener():
    global pub_point, pub_point_plane, pub_point_global, pub_markers, pub_odom_rp, pub_odom_rpy, pub_odom_yaw
    rospy.init_node('magvis', anonymous=True)
    rospy.Subscriber("/vins_estimator/odometry", Odometry, odom_callback, queue_size=1)
    rospy.Subscriber("/mavros/imu/mag", MagneticField, mag_callback, queue_size=1)
    pub_point = rospy.Publisher("/magvis/point_visual", PointCloud, queue_size = 10)  
    pub_point_plane = rospy.Publisher("/magvis/point_plane_visual", PointCloud, queue_size = 10) 
    pub_point_global = rospy.Publisher("/magvis/point_global_visual", PointCloud, queue_size = 10)
    pub_markers = rospy.Publisher("/magvis/axis_visual", MarkerArray, queue_size = 10)
    pub_odom_rpy = rospy.Publisher("/magvis/odometry_rpy", Odometry, queue_size = 10)
    pub_odom_rp = rospy.Publisher("/magvis/odometry_rp", Odometry, queue_size = 10)
    pub_odom_yaw = rospy.Publisher("/magvis/odometry_yaw", Odometry, queue_size = 10)

    rospy.spin()

if __name__ == '__main__':
    listener()

