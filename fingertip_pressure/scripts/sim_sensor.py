#! /usr/bin/python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
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

# Simulates the presence of a fingertip sensor

import roslib
roslib.load_manifest('fingertip_pressure')
import rospy
from math import sin, cos
import threading

from pr2_msgs.msg import PressureState

class pressureSimulator:
    def callback(self, pressurestate):
        #print "callback"
        self.l_finger_tip = pressurestate.l_finger_tip
        self.r_finger_tip = pressurestate.r_finger_tip
        self.datatimestamp = pressurestate.header.stamp
        self.dataready = True

    def publish(self):
        ps = PressureState()
        ps.header.stamp = rospy.get_rostime();
        ps.l_finger_tip = []
        ps.r_finger_tip = []
        t = rospy.get_time()
        for i in range(0,22):
            ph = .1 * t * (i / 22. + 1)
            ps.l_finger_tip.append(4000*(1+sin(ph)))
            ps.r_finger_tip.append(4000*(1+cos(ph)))
        self.pub.publish(ps)

    def __init__(self, dest):
        rospy.init_node('sim_sensor', anonymous=True)
        rospy.sleep(.2)
        
        self.pub = rospy.Publisher(dest, PressureState)
        

if __name__ == '__main__':
    #@todo it would be nice to read an xml configuration file to get these parameters.
    s1 = pressureSimulator('pressure/r_gripper_motor')
    s2 = pressureSimulator('pressure/l_gripper_motor')
        
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        s1.publish()
        s2.publish()
