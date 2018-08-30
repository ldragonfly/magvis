#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def publisher():
    pub = rospy.Publisher('/magvis/odometry', Odometry, queue_size=1)
    rospy.init_node('test_publisher', anonymous=True)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.orientation.x = 0.0

        pub.publish(odom)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

