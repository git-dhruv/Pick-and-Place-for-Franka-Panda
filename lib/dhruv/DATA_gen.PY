"""
Cleaner code

Bug fixers: 
Last angle: all other angles as 0, the last angle will be correct all the time
6th angle: All other angles as 0, theta7 as pi/4, it will be correct
4th angle: Same

All angles: never work 
"""
from matplotlib import spines
import roboticstoolbox as rtb
from spatialmath import SE3


import numpy as np
from math import *
from l1 import FK 
from copy import deepcopy
fk = FK()

"""
Things that work: nothing now

"""
robot1 = rtb.models.Panda()
robot2 = rtb.models.Panda()

# for i in range(-176,4,3):
#     for j in range(0,215,3):
# matches figure in the handout #All 0 is the config for prelab
# q = np.array([pi,-pi/2,pi/2,-90*pi/180,0,3*pi/2,pi])
q = np.array([0,0,0,0,0,0,-pi/4])

joint_positions, End_effector = fk.forward(q)
End_effector[:,2] *= -1

d1 = 0.333
a3 = 0.0825
d5 = 0.384
d3 = 0.316
d7 = 0.21
a6 = 0.088
robot1.plot(q)
print(np.round(End_effector,3))

#Invert the end effector
Inverted_Toe = np.linalg.inv(End_effector)

#Get the wrist center
Oc_wrist = Inverted_Toe[0:3,-1] + Inverted_Toe[0:3,2]*(d1*np.ones(3))
#Calculate theta_7
#Solution1
theta_7_1 = atan2(Oc_wrist[1],Oc_wrist[0]) +pi/4 + pi/2
#Solution2
theta_7_2 = theta_7_1 + pi
r = np.linalg.norm([Oc_wrist[0],Oc_wrist[1]])
for i in range(2):
    if i==0:
        Oc = np.round([(r-a6),Oc_wrist[2]+d7],5)
        theta_7 = theta_7_1
    else:
        Oc = np.round([-(r+a6),Oc_wrist[2]+d7],5)
        theta_7 = theta_7_2

    t = float(End_effector[0][-1])
    if t>=0:
        print("Always")
        activation = 1

        num = a3*a3 + d5*d5 + a3*a3 + d3*d3 - Oc[0]**2 - Oc[1]**2
        den = 2* (np.sqrt(a3**2 + d5**2) * np.sqrt(a3**2 + d3**2))

        beta =  (np.arccos(num/den)) #This 2pi also keeps on changing
        alpha = np.arctan(d5/a3)   
        gamma = np.arctan(d3/a3)

        theta4_1 =  2*pi - (alpha+beta+gamma) 
        theta4_1 = np.arctan2(sin(theta4_1),cos(theta4_1))

        beta2 = -beta
        theta4_2 = 2*pi - (alpha+beta2+gamma) 
        theta4_2 = np.arctan2(sin(theta4_2),cos(theta4_2))


        if activation:

            theta4 = theta4_1
            num = d3*sin(theta4) - (a3*cos(theta4)-a3)
            den = d5+a3*sin(theta4)+d3*cos(theta4)
            theta_6 = pi/2 - (atan2(Oc[1],Oc[0]) - atan2(num,den))  #d5,d3+a3 #This quadrant needs to keep on changing #there was a minus here!?
            theta_6_1 = theta_6


            #Solution 2
            theta4 = theta4_2
            num = d3*sin(theta4) - (a3*cos(theta4)-a3)
            den = d5+a3*sin(theta4)+d3*cos(theta4)
            theta_6_2 = pi/2 - (atan2(Oc[1],Oc[0]) - atan2(num,den))  #d5,d3+a3 #This quadrant needs to keep on changing #there was a minus here!?
            for j in range(2):
                if j==0:
                    theta_6 = theta_6
                    theta4 = -theta4_1
                else:
                    theta_6 = theta_6_2
                    theta4 = -theta4_2
                # robot1.plot(q)
                print(theta4*180/pi)
                q = np.array([0,0,0,theta4,0,theta_6,theta_7])
                q = [pi,-pi/2,pi,0,0,pi/6,0]
                _,R = fk.general_transformer(3,6,q)
                R_orient = R[0:3,0:3]@np.linalg.inv(End_effector[0:3,0:3])
                # R_orient = R_orient[0:3,0:3]


                R_orient = R_orient.T
                q2 = np.arccos(R_orient[2,1])
                q3 = -np.arctan2(R_orient[2,2],R_orient[2,0])
                q1 = np.arctan2(R_orient[1,1],R_orient[0,1])
                
                if i==0:
                    if j==0:
                        Q1 = [q1,q2,q3,-theta4_1,0,theta_6_1,theta_7_1]
                    else:
                        Q2 = [q1,q2,q3,-theta4_2,0,theta_6_2,theta_7_1]
                else:
                    if j==0:
                        Q3 = [q1,q2,q3,-theta4_1,0,theta_6_1,theta_7_2]
                    else:
                        Q4 = [q1,q2,q3,-theta4_2,0,theta_6_2,theta_7_2]



    else:
        activation = 1

        num = a3*a3 + d5*d5 + a3*a3 + d3*d3 - Oc[0]**2 - Oc[1]**2
        den = 2* (np.sqrt(a3**2 + d5**2) * np.sqrt(a3**2 + d3**2))

        beta =  2*pi - (np.arccos(num/den)) #This 2pi also keeps on changing
        # beta = atan2(sin(beta),cos(beta))
        alpha = np.arctan(d5/a3)   
        gamma = np.arctan(d3/a3)

        theta4_1 =  2*pi -  (alpha+beta+gamma) 
        theta4_1 = np.arctan2(sin(theta4_1),cos(theta4_1))


        if activation:

            theta4 = theta4_1
            num = d3*sin(theta4) - (a3*cos(theta4)-a3)
            den = d5+a3*sin(theta4)+d3*cos(theta4)
            theta_6 =   (atan2(Oc[0],Oc[1]) + atan2(num,den))  #d5,d3+a3 #This quadrant needs to keep on changing #there was a minus here!?
            print(atan2(Oc[1],Oc[0]), atan2(num,den))
            print("====Solution 1=====")
            print(f"Theta 4: {round(theta4*180/pi,2)}")
            print(f"Theta 6: {round(theta_6*180/pi,2)}")
            print(f"Theta 7: {round(theta_7*180/pi,2)}")
            Q1 = [q[0],q[1],q[2],-theta4_1,0,theta_6,theta_7_1]
            # Q1 = [q[0],q[1],q[2],-theta4_1,0,theta_6,theta_7_1]
            
    
    





# Q = np.array([Q1,Q2,Q3,Q4])
Q = Q1
print(np.round([i*180/pi for i in Q],2))
Q = np.array([Q1,Q2,Q3,Q4])
print(np.round(Q*180/pi,2))

# print(q*180/pi)
robot2.plot(Q1)
import time
time.sleep(1000)
robot3 = rtb.models.Panda()
robot4 = rtb.models.Panda()
robot5 = rtb.models.Panda()
robot3.plot(Q2)
robot4.plot(Q3)
robot5.plot(Q4)

# _, J = fk.forward(q)
# print(np.round(J,5))
# print(np.round(End_effector,5))
 
import time
time.sleep(100)


# x = []
# y = []
# z= []
# for i in range(-176,4,3):
#     for j in range(0,215,3):
#         # for k in range(0,180,40):
#             _,T = fk.forward([0,0,0,i*pi/180,0,j*pi/180,0])
#             pos = T[0:3,-1]
#             x.append(float(pos[0]))
#             y.append(float(pos[1]))
#             z.append(float(pos[2]))
# import matplotlib.pyplot as plt

# fig = plt.figure(figsize=(12, 12))
# ax = fig.add_subplot(projection='3d')

# ax.scatter(x,y,z)
# x = []
# y = []
# z= []
# for i in range(-225,0,3):
#     for j in range(-225,-45,3):
#         # for k in range(0,180,40):
#             _,T = fk.forward([pi,0,0,i*pi/180,0,j*pi/180,0])
#             pos = T[0:3,-1]
#             x.append(float(pos[0]))
#             y.append(float(pos[1]))
#             z.append(float(pos[2]))
# print("huh")
# ax.scatter(x,y,z, c ='b')
# plt.show()

        

# 