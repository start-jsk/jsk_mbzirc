#!/usr/bin/env python
# Software License Agreement (BSD License)
#
from __future__ import print_function

import sys
import time
import unittest

import rospy
import rostest
from std_msgs.msg import String

NAME = 'check_mission'

class MissionCompleted(unittest.TestCase):
    def __init__(self, *args):
        super(MissionCompleted, self).__init__(*args)
        rospy.init_node(NAME)

    def setUp(self):
        self.message_received = False

    def test_score(self):
        #
        sub = rospy.Subscriber('/score', String, self.callback)

        while not self.message_received :
            rospy.sleep(1)

        assert(self.message_received)
        
    def callback(self, msg):
        #
        rospy.loginfo("recevied message (%s)", msg)
        if msg.data == "Mission Completed" :
            self.message_received = True

if __name__ == '__main__':
    # A dirty hack to work around an apparent race condition at startup
    # that causes some hztests to fail.  Most evident in the tests of
    # rosstage.
    time.sleep(0.75)
    try:
        rostest.run('check_mission', NAME, MissionCompleted, sys.argv)
    except KeyboardInterrupt:
        pass
    print("exiting")

        
