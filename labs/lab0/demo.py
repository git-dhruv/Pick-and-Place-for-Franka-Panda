"""
Date: 08/26/2021

Purpose: This script creates an ArmController and uses it to command the arm's
joint positions and gripper.

Try changing the target position to see what the arm does!

"""

import sys
import rospy
import numpy as np
from math import pi

from core.interfaces import ArmController

rospy.init_node('demo')

arm = ArmController()
arm.set_arm_speed(0.3)

arm.close_gripper()

q = arm.neutral_position()
# arm.safe_move_to_position(q)
# arm.open_gripper()

"""
A1, A3, A5, A7: -166/166
A2: -101/101
A4: -176/-4
A6: -1/215
"""
#[0,-1 ,0,-2,0,1,1]
"""
#1
[ 0., -1.76278254,  0., -1.57079633,  0.08726646,  0.01745329, 1.74532925]
[0,-101 ,0,-90,5,1,100] degrees

#2
0,0 ,0,-72,-0,47,0
 0.          0. , 0. , -1.25663706 , 0. ,0.82030475,0.   ]

#3
0,-67 ,45,-20,-0,0,90


#4
[166.00315123 101.00045038 103.13246897  -3.99957527 166.00297575
 215.00204445 166.00306625]

#5
[-166.00320107 -101.00112122 -166.00351877   -3.99928722   11.45786177
    4.26542983  -91.68077852]
"""
q = np.array([166, 101 ,103 , -4, 166, 215, 166])*pi/180 # TODO: try changing this!


q = [1,-1,-0,-1,1.1,0,1]



arm.safe_move_to_position(q)
arm.close_gripper()
print(arm.get_positions()*180/pi)