from ast import Invert
import roboticstoolbox as rtb

import numpy as np
from math import *
from calculateFK import FK 
from copy import deepcopy
fk = FK()

"""
Things that work: nothing now

"""
robot1 = rtb.models.Panda()
robot2 = rtb.models.Panda()

# matches figure in the handout #
# All 0 is the config for prelab
q = np.array([0,0,0,-pi/2,0,0,pi/2])

joint_positions, End_effector = fk.forward(q)

d1 = 0.333
a3 = 0.0825
d5 = 0.384
d3 = 0.316
d7 = 0.21
a6 = 0.088
# robot1.plot(q)
import time

# time.sleep(1000)
#Invert the end effector
Inverted_Toe = np.linalg.inv(End_effector)

#Get the wrist center
Oc = Inverted_Toe[0:3,-1] + Inverted_Toe[0:3,2]*(d1*np.ones(3))
# print(Oc)

##----Q7----##
Oc_wrist = deepcopy(Oc)
q7 = pi - atan2(-Oc[1],Oc[0])+pi/4
q7 =  3*pi/4 - np.arctan2(-(Oc[1]),(Oc[0])) 
q7 = -q7

q7 = np.arctan2(sin(q7),cos(q7))

# print(Oc)

T7 = fk.translx(a6)@(fk.rotz(-pi/4)@fk.single_frame_transform(6,q7))
Inverted_Toe = T7@Inverted_Toe #t60

#new wrist center
Oc = np.array([[Oc[0],Oc[1],Oc[2],1]]).T
Oc = T7@Oc 
print("Wrist")
print(Oc,np.arctan2(-(Oc[1]),(Oc[0]))*180/pi )

if 1:#abs(q7)<=pi/2:
    print("aiyo")
    Oy = float(Oc[0])#float((np.sign(Oc[0]))*(abs(Oc[0])+a6))
    Ox = float(Oc[2])
else:
    Oy = -float(Oc[0]-a6)
    Ox = float(Oc[2])
print(Ox,Oy)

#-- Theta 4 --#
l1 = sqrt(d5**2 + a3**2)
l2 = sqrt(d3**2 + a3**2)
D = ((Ox**2 + Oy**2 - l1**2 - l2**2)/(2*l1*l2))
# t2 = np.arccos(D)
# D = plus minus 1
t2 = np.arctan2(sqrt(1-D**2),-D)

# t1 = np.arctan2(Ox,Oy) - np.arctan2((l2*sin(t2)),l1+l2*cos(t2))
t1 = np.arctan2(Oy,Ox) - np.arctan2((l2*sin(t2)),l1+l2*cos(t2))

# q4 = ( np.arctan2(d5,a3) + np.arctan2(d3,a3) + t2 -pi
q4 = np.arctan2(d5,a3) + np.arctan2(d3,a3) + t2 -pi
# 
#-- Theta 6 --#
q6 =  (t1 + np.arctan2(a3,d5) - pi/2)

num = d3*sin(q4) - (a3*cos(q4)-a3)
den = d5+a3*sin(q4)+d3*cos(q4)
# q6 =  - (atan2(Oy,Ox) - atan2(num,den)) 

# q4 = -q4

# r = np.linalg.norm([Oc_wrist[0],Oc_wrist[1]])
# Oc = np.round([-(r+a6),Oc_wrist[2]+d7],5)
# # Oc = np.round([(r-a6),Oc_wrist[2]+d7],5)

# print(Ox,Oy)

# num = a3**2 + d5**2 + a3**2 + d3**2 - (Ox**2+Oy**2)
# den = 2*sqrt(a3**2 + d5**2)*sqrt(a3**2 + d3**2)
# num = a3*a3 + d5*d5 + a3*a3 + d3*d3 - Oc[0]**2 - Oc[1]**2
# den = 2* (np.sqrt(a3**2 + d5**2) * np.sqrt(a3**2 + d3**2))

# temp = num/den
# beta =  (np.arctan2(sqrt(1-temp**2),temp)) #This 2pi also keeps on changing
# alpha = np.arctan(d5/a3)   
# gamma = np.arctan(d3/a3) 
# theta4_1 =  2*pi - (alpha+beta+gamma) 
# theta4_1 = np.arctan2(sin(theta4_1),cos(theta4_1))

# #Solution 2 for theta4
# beta = -beta
# theta4_2 = 2*pi - (alpha+beta+gamma)
# theta4_2 = atan2(sin(theta4_2),cos(theta4_2))


# theta4 = theta4_1
# num = d3*sin(theta4) - (a3*cos(theta4)-a3)
# den = d5+a3*sin(theta4)+d3*cos(theta4)
# theta_6 = pi/2 - (atan2(Oc[1],Oc[0]) - atan2(num,den))  #d5,d3+a3 #This quadrant needs to keep on changing #there was a minus here!?
# theta_6_1 = theta_6


# #Solution 2
# theta4 = theta4_2
# num = d3*sin(theta4) - (a3*cos(theta4)-a3)
# den = d5+a3*sin(theta4)+d3*cos(theta4)
# theta_6_2 = pi/2 - (atan2(Oc[1],Oc[0]) - atan2(num,den))






# robot1.plot(q)
print(np.round(q*180/pi),3)

q = np.array([0,0,0,q4,0,q6,q7])

_,R = fk.general_transformer(3,6,q)
R_orient = R[0:3,0:3]@np.linalg.inv(End_effector[0:3,0:3])
# R_orient = R_orient[0:3,0:3]

print(R_orient)

R_orient = R_orient.T
q2 = np.arccos(R_orient[2,1])
q3 = -np.arctan2(R_orient[2,2],R_orient[2,0])
q1 = np.arctan2(R_orient[1,1],R_orient[0,1])

q = np.array([0,0,0,q4,0,q6,q7])

_,R_new = fk.general_transformer(0,2,q)
# print(np.round(R_new,5))
# print(np.round(R_orient.T,5))



print(q*180/pi)
# robot2.plot(q)
_, J = fk.forward(q)
print(np.round(J,5))
print(np.round(End_effector,5))