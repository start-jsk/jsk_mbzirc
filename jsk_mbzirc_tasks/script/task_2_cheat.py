#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import sys
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import math
from moveit_commander import MoveGroupCommander, conversions
from std_msgs.msg import Float64

__author__ = 'kei.okada@gmail.com (Kei Okada)'

def open_hand():
    msg = Float64()
    msg.data = 0.4
    rospy.loginfo("send %f"%msg.data)
    for i in range(10):
        pub.publish(msg)
        rospy.sleep(0.1)

def close_hand():
    msg = Float64()
    msg.data = -1.0
    for i in range(30):
        pub.publish(msg)
        rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('task_2_cheat', anonymous=True)

    # for for 5 sec
    rospy.sleep(5)

    rospy.loginfo("start program %f"%rospy.get_time())
    arm = MoveGroupCommander("ur5_arm")
    arm.set_planner_id('RRTConnectkConfigDefault')
    pub = rospy.Publisher("/r_gripper_controller/command", Float64, queue_size=1)
    client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    rospy.loginfo("init pose")
    msg = FollowJointTrajectoryGoal()
    msg.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
    msg.trajectory.joint_names = ['ur5_arm_shoulder_pan_joint', 'ur5_arm_shoulder_lift_joint', 'ur5_arm_elbow_joint', 'ur5_arm_wrist_1_joint', 'ur5_arm_wrist_2_joint', 'ur5_arm_wrist_3_joint']
    msg.trajectory.points.append(JointTrajectoryPoint(positions=[-1.57,-0.1745,-2.79,-1.57,0,0], time_from_start = rospy.Duration(2)))
    client.send_goal(msg)
    client.wait_for_result()


    # open
    open_hand()
    # reach
    rospy.loginfo("reach")
    arm.set_pose_target([0.90, 0.16, 0.233, 0, 0, 0])
    arm.plan() and arm.go()
    arm.plan() and arm.go()
    # approach
    rospy.loginfo("approach")
    arm.set_pose_target([1.13, 0.16, 0.233, 0, 0, 0])
    arm.plan() and arm.go()
    # rotate
    for i in range(2):
        # close
        rospy.loginfo("close")
        close_hand()
        # rotate
        angles = arm.get_current_joint_values()
        import numpy
        start_angle = angles[5]
        print("current angles=", start_angle)
        for r in numpy.arange(start_angle, start_angle-3.14*2, -1.0):
            rospy.loginfo(angles)
            angles[5] = r
            rospy.loginfo("rotate (%f)"%(r))
            msg.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
            msg.trajectory.points = [JointTrajectoryPoint(positions=angles, time_from_start = rospy.Duration(1))]
            client.send_goal(msg)
            client.wait_for_result()

        # open
        rospy.loginfo("open")
        open_hand()
        # back
        angles[5] = start_angle
        arm.set_joint_value_target(angles)
        rospy.loginfo("rotate (%f)"%(r))
        arm.plan() and arm.go()
    
    rospy.loginfo("done")

