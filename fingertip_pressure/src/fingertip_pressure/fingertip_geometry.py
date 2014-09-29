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

import roslib
roslib.load_manifest('fingertip_pressure')

from fingertip_pressure.msg import PressureInfoElement
from geometry_msgs.msg import Vector3

force_per_unit_table = [ 
        600, # 0 bottom
        400, # 1 side
        600, # 2 corner
        600, # 3 front
        600, # 4 front
        600, # 5 corner
        400, # 6 side
        1600, 1600, 1600, 
        1600, 1600, 1600, 
        1600, 1600, 1600, 
        1600, 1600, 1600, 
        1600, 1600, 1600, 
        ]

# coordinates are in mm here, and get converted to meters for publishing.
coordinates = [
        # center             half-side 1        half-side 2
        [ 29.3, 11.0,  0.0,   0.0,  0.0, 10.0,   2.8,  0.0,  0.0 ],    # 0
        [ 16.5,  5.2, 11.5,  12.0,  0.0,  0.0,   0.0,  3.0,  0.0 ],    # 1
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 2 fused 
        [ 35.0,  4.7,  3.5,   0.0,  0.0,  3.5,   0.0, -2.5,  0.0 ],    # 3 CHK x
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 4 mirrored 
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 5 mirrored 
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 6 mirrored 
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 7 translated
        [ 30.5,  0.0,  0.0,   0.0,  0.0, -2.8,   3.0,  0.0,  0.0 ],    # 8 CHK
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 9 translated 
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 10
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 11 translated 
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 12 translated 
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 13 translated 
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 14 translated  
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 15 translated  
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 16 translated  
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 17 translated  
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 18 translated  
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 19 translated  
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 20 translated  
        [  0.0,  0.0,  0.0,   0.0,  0.0,  0.0,   0.0,  0.0,  0.0 ],    # 21 translated  
        ]                                                                                 

# Generate #2 to go from #1 to #3
coordinates[2][0] = (coordinates[1][0] + coordinates[1][3] + coordinates[3][0]) / 2
coordinates[2][1] = coordinates[3][1]
coordinates[2][2] = (coordinates[1][2] + coordinates[3][2] + coordinates[3][5]) / 2
coordinates[2][3] = (coordinates[1][0] + coordinates[1][3] - coordinates[3][0]) / 2
coordinates[2][4] = 0
coordinates[2][5] = (coordinates[1][2] - coordinates[3][2] - coordinates[3][5]) / 2
for i in range(6,9):
    coordinates[2][i] = coordinates[3][i]

# Same sensor on other side.
def mirror(src, dest):
    coordinates[dest] = list(coordinates[src])
    coordinates[dest][2] = -coordinates[dest][2]
    coordinates[dest][5] = -coordinates[dest][5]
    # Flip half-edge 2 this way so that cross product points out.
    coordinates[dest][6] = -coordinates[dest][6]
    coordinates[dest][7] = -coordinates[dest][7]

# Generate one sidewall from the other
mirror(1, 6)
mirror(2, 5)
mirror(3, 4)

# Step 2 * dim times half-side k.
def translate(src, dest, dir, k):
    coordinates[dest] = list(coordinates[src])
    for i in range(0,3):
        coordinates[dest][i] = coordinates[dest][i] + 2 * dir * coordinates[dest][i + 3 * k]

# Generate the main array from #8
translate(8, 7, 1, 1)
translate(8, 9, -1, 1)
for i in range(10, 22):
   translate(i-3, i, -1, 2)

# Adjust for actual origin and flip Z
for i in range(0,22):
    # Move origin
    coordinates[i][0] = coordinates[i][0] - 4
    coordinates[i][1] = coordinates[i][1] - 15
    # Flip Z
    coordinates[i][2] = -coordinates[i][2]
    coordinates[i][5] = -coordinates[i][5]
    coordinates[i][6] = -coordinates[i][6]
    coordinates[i][7] = -coordinates[i][7]

def multorientation(data, ori):
    for i in range(0, len(data)):
        data[i].y = data[i].y * ori
        data[i].z = data[i].z * ori

def extractvec(i):
    out = [];
    for j in range(0,len(coordinates)):
        v = Vector3()
        v.x = coordinates[j][i] / 1000.;
        v.y = coordinates[j][i+1] / 1000.;
        v.z = coordinates[j][i+2] / 1000.;
        out.append(v)
    return out

def pressureInformation(frame_id, orientation):
    msg = PressureInfoElement()
    msg.frame_id = frame_id
    msg.force_per_unit = force_per_unit_table
    msg.center = extractvec(0)
    msg.halfside1 = extractvec(3)
    msg.halfside2 = extractvec(6)
    multorientation(msg.center, orientation)
    multorientation(msg.halfside1, orientation)
    multorientation(msg.halfside2, orientation)
    #print "%e %e %e  %e %e %e  %e %e %e"%(msg.center[7].x, msg.center[7].y,
    #        msg.center[7].z, msg.halfside1[7].x, msg.halfside1[7].y,
    #        msg.halfside1[7].z, msg.halfside2[7].x, msg.halfside2[7].y,
    #        msg.halfside2[7].z)
    return msg
