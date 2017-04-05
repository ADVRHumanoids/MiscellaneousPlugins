#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

import math

if __name__ == '__main__':

    rospy.init_node('IkRefPublisher', anonymous=True)
    pub_left = rospy.Publisher('w_T_left_ee', PoseStamped, queue_size=1)
    pub_right = rospy.Publisher('w_T_right_ee', PoseStamped, queue_size=1)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        msg = PoseStamped()
        msg.pose.position.x = 0.5
        msg.pose.position.y = 0.4
        msg.pose.position.z = -0.5 * math.sin(rospy.Time.now().to_sec())
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1

        pub_left.publish(msg)

        msg = PoseStamped()
        msg.pose.position.x = 0.5
        msg.pose.position.y = -0.4
        msg.pose.position.z = 0.5 * math.sin(rospy.Time.now().to_sec())
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1

        pub_right.publish(msg)

        rate.sleep()






