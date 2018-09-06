#!/usr/bin/env python
import rospy
from sensor_msgs.msg import ChannelFloat32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_conjugate, quaternion_multiply

import math
import numpy
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt 


R = 6400*1000


def gps_callback(data): 
    global lat_0, lon_0, alt_0
    if gps_callback.count == 0 :
        lat_0 = data.latitude;
        lon_0 = data.longitude;
        alt_0 = data.altitude;

    gps_callback.count = gps_callback.count + 1

    lat = data.latitude
    lon = data.longitude
    alt = data.altitude

    dlat = (lat - lat_0) * math.pi/180
    dlon = (lon - lon_0) * math.pi/180
    dalt = alt - alt_0

    dx = R*math.cos(lat)*dlon;
    dy = R*dlat;
    dz = dalt;
    
    print("------------")
    print(dx)
    print(dy)
    print(dz)

    global pub_point
    point = PointCloud()
    point.header.frame_id = 'map'
    point.header.stamp = rospy.Time.now()
    p = Point32()
    p.x = dx
    p.y = dy
    p.z = dz
    point.points = p,
    pub_point.publish(point)

def listener():
    global pub_point
    rospy.init_node('gpsvis', anonymous=True)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, gps_callback, queue_size=5)
    pub_point = rospy.Publisher("/gpsvis/point", PointCloud, queue_size=10)

    gps_callback.count = 0;
    rospy.spin()

if __name__ == '__main__':
    listener()

