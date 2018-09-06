#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField

def publisher():
    pub = rospy.Publisher('/magvis/odometry', Odometry, queue_size=1)
    pub_mag = rospy.Publisher('/magvis/test_mag', MagneticField, queue_size=1)
    rospy.init_node('test_publisher', anonymous=True)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.orientation.x = 0.0

        mag = MagneticField()
        mag.header.frame_id = 'map'
        mag.header.stamp = rospy.Time.now()
        mag.magnetic_field.x = 0#0.707
        mag.magnetic_field.y = 1#0.707
        mag.magnetic_field.z = 0

        pub.publish(odom)
        pub_mag.publish(mag)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

