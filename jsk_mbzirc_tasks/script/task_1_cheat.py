#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import sys
import rospy
from geometry_msgs.msg import Twist

__author__ = 'kei.okada@gmail.com (Kei Okada)'

if __name__ == '__main__':
    rospy.init_node('task_1_cheat', anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # for for 1 sec
    while rospy.get_time() < 2 :
        rospy.sleep(0.01)
    rospy.loginfo("start program %f"%rospy.get_time())
    msg = Twist()
    msg.linear.z = 1.0
    rospy.loginfo("send %f"%msg.linear.z)
    pub.publish(msg)
    rospy.sleep(2)
    msg.linear.z = -3.0
    rospy.loginfo("send %f"%msg.linear.z)
    pub.publish(msg)

    rospy.spin()
