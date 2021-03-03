#!/usr/bin/env python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2008, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

# Author: Blaise Gassend

import roslib; roslib.load_manifest('fingertip_pressure')

import rospy
import sys
import time
import unittest
import rostest

from fingertip_pressure.msg import PressureInfo

## Tests the PressureInfo message produced by sensor_info.py
class PressureInfoTest(unittest.TestCase):
    def __init__(self, *args):
        super(PressureInfoTest, self).__init__(*args)
        rospy.init_node('pressure_info_test')
        
        # Read one message from the topic
        self.msg = None
        sub = rospy.Subscriber('pressure/r_gripper_motor_info',
                PressureInfo, self.callback)
        timeout_t = rospy.get_time() + 2
        print('waiting for message')
        while self.msg == None and timeout_t > rospy.get_time():
            rospy.sleep(0.1)
        print('done waiting for message')
        sub.unregister()
        self.assertNotEquals(self.msg, None)
                                    
    def callback(self, msg):
        print('got message')
        # Account for offset origin of sensor origin.
        for i in range(0,2):
            for j in range(0,22):
                fact = [1,-1][i]
                msg.sensor[i].center[j].x = msg.sensor[i].center[j].x + 0.004
                msg.sensor[i].center[j].y = msg.sensor[i].center[j].y + 0.015 * fact
        self.msg = msg

    def test_array_sizes(self):
        self.assertEquals(len(self.msg.sensor), 2, "Should have two PressureInfoElements")
        self.assertEquals(len(self.msg.sensor[1].center), 22, "Should have 22 centers.")
        self.assertEquals(len(self.msg.sensor[1].halfside1), 22, "Should have 22 halfside1")
        self.assertEquals(len(self.msg.sensor[1].halfside2), 22, "Should have 22 halfside2")
        self.assertEquals(len(self.msg.sensor[1].force_per_unit), 22, "Should have 22 pressure")

    def test_cross_products(self):
        for j in range(0,2):
            yori = [1, -1][j]
            for i in range(0,22):
                a = self.msg.sensor[j].halfside1[i]
                b = self.msg.sensor[j].halfside2[i]
                c = self.msg.sensor[j].center[i]
                c.y = c.y - yori * .005 # Ensure that we are inside the sensor
                det = a.x * b.y * c.z + b.x * c.y * a.z + c.x * a.y * b.z\
                     -a.x * c.y * b.z - b.x * a.y * c.z - c.x * b.y * a.z
                self.assertTrue(det > 0, 
                        "Wrong orientation for sensor %i on tip %i, det=%e, %e %e %e %e %e %e %e %e %e"%(i, j, det, a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z))

    def test_bounding_box(self):
        for j in range(0,2):
            yori = [1, -1][j]
            for i in range(0,22):
                a = self.msg.sensor[j].halfside1[i]
                b = self.msg.sensor[j].halfside2[i]
                c = self.msg.sensor[j].center[i]
                msg = "Bound box error sensor %i, tip %i, %e %e %e %e %e %e %e %e %e"%(i, j, a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z)
                self.assertTrue(c.x - abs(a.x) > 0, msg)
                self.assertTrue(c.x - abs(b.x) > 0, msg)
                self.assertTrue(c.x + abs(a.x) <= 0.035, msg)
                self.assertTrue(c.x + abs(b.x) <= 0.035, msg)
                self.assertTrue(yori * c.y - abs(a.y) >= 0, msg)
                self.assertTrue(yori * c.y - abs(b.y) >= 0, msg)
                self.assertTrue(yori * c.y + abs(a.y) < 11, msg)
                self.assertTrue(yori * c.y + abs(b.y) < 11, msg)
                self.assertTrue(abs(c.z) + abs(a.z) <= 11.5, msg)
                self.assertTrue(abs(c.z) + abs(b.z) <= 11.5, msg)


if __name__ == '__main__':
    import rostest
    time.sleep(0.75)
    try:
        rostest.rosrun('fingertip_pressure', 'pressure_info_test', PressureInfoTest)
    except KeyboardInterrupt as e:
        pass
    print("exiting")



