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

# Publish information needed to interpret fingertep pressure sensor data:
# - Frame id
# - 

import roslib
roslib.load_manifest('fingertip_pressure')
import rospy

from fingertip_pressure.msg import PressureInfo
from fingertip_pressure.fingertip_geometry import *


class pressureInformationPublisher:
    def __init__(self, dest, frame0, frame1):
        self.info = []
        self.info.append(pressureInformation(frame0, 1))
        self.info.append(pressureInformation(frame1, -1))
        self.publisher = rospy.Publisher(dest, PressureInfo, latch=True, queue_size=1000)

    def publish(self):
        self.publisher.publish(self.info)

    
if __name__ == '__main__':

    #@todo it would be nice to read an xml configuration file to get these parameters.
    rospy.init_node('pressure_sensor_info')
    
    pip1=pressureInformationPublisher('pressure/r_gripper_motor_info', 
            'r_gripper_l_finger_tip_link', 'r_gripper_r_finger_tip_link')
    pip2=pressureInformationPublisher('pressure/l_gripper_motor_info', 
            'l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link')

    pip1.publish()
    pip2.publish()
    
    while not rospy.is_shutdown():
        rospy.spin()

