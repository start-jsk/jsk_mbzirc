#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)

# Copyright (c) 2016, JSK Robotics Laboratory, The University of Tokyo
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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

        
