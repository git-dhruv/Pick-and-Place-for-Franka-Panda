import sys
import numpy as np
import rospy
from math import cos, sin, pi
import matplotlib.pyplot as plt
import geometry_msgs

from core.interfaces import ArmController
from core.utils import time_in_seconds

from lib.IK_velocity import IK_velocity
from lib.calculateFK import FK

from gazebo_msgs.msg import LinkStates
from copy import deepcopy

import time
class JacobianDemo():
    """
    Demo class for testing Jacobian and Inverse Velocity Kinematics.
    Contains trajectories and controller callback function
    """
    active = False # When to stop commanding arm
    start_time = 0 # start time
    dt = 0.03 # constant for how to turn velocities into positions
    fk = FK()
    point_pub = rospy.Publisher('/vis/trace', geometry_msgs.msg.PointStamped, queue_size=10)
    counter = 0
    x0 = np.array([0.307, 0, 0.487]) # corresponds to neutral position
    last_iteration_time = None

    flag = 0


    #Modified
    error = 0
    no_iterations = 0
    xold = np.array([0.307, 0, 0.487])
    x = np.array([0.307, 0, 0.487])
    q1 = []
    q2 = []
    q3 = []
    q4 = []
    q5 = []
    q6 = []
    q7 = []
    v_accum = 0
    v_counter = 0

    ##################
    ## TRAJECTORIES ##
    ##################

    def eight(t,fx=1,fy=2,rx=.15,ry=.1):
        """
        Calculate the position and velocity of the figure 8 trajector

        Inputs:
        t - time in sec since start
        fx - frequecny in rad/s of the x portion
        fy - frequency in rad/s of the y portion
        rx - radius in m of the x portion
        ry - radius in m of the y portion

        Outputs:
        xdes = 0x3 np array of target end effector position in the world frame
        vdes = 0x3 np array of target end effector linear velocity in the world frame
        """

        # Lissajous Curve
        x0 = np.array([0.307, 0, 0.487]) # corresponds to neutral position
        xdes = x0 + np.array([rx*sin(fx*t),ry*sin(fy*t),0])
        vdes = np.array([rx*fx*cos(fx*t),ry*fy*cos(fy*t),0])
        return xdes, vdes

    def ellipse(t,f=1,ry=.15,rz=.15):
        """
        Calculate the position and velocity of the figure ellipse trajector

        Inputs:
        t - time in sec since start
        f - frequecny in rad/s of the trajectory
        rx - radius in m of the x portion
        ry - radius in m of the y portion

        Outputs:
        xdes = 0x3 np array of target end effector position in the world frame
        vdes = 0x3 np array of target end effector linear velocity in the world frame
        """

        x0 = np.array([0.307, 0, 0.487]) # corresponds to neutral position
        

        ## STUDENT CODE GOES HERE
        xdes = x0 + np.array([0,ry*cos(f*t)-ry,rz*sin(f*t)])
        vdes = np.array([0,-ry*f*sin(f*t),rz*f*cos(f*t)])

        ## END STUDENT CODE

        return xdes, vdes

    def line(t,f=1,L=.2):
        """
        Calculate the position and velocity of the line trajector

        Inputs:
        t - time in sec since start
        f - frequecny in Hz of the line trajectory
        L - length of the line in meters
        
        Outputs:
        xdes = 0x3 np array of target end effector position in the world frame
        vdes = 0x3 np array of target end effector linear velocity in the world frame
        """
        ## STUDENT CODE GOES HERE
        #I believe we can find xdes and then differentiate?
        x0 = JacobianDemo.x0
        # x0 = np.array([0.307, -L*2, 0.487])

        # xdes = x0 + np.array([0,L*sin(f*t),0])
        # vdes = np.array([0,L*f*cos(f*t),0])
        xdes = x0 + np.array([0,0, L*sin(f*t)])
        vdes = np.array([0,0,L*f*cos(f*t)])
        # TODO: replace these!
        # xdes = JacobianDemo.x0
        # vdes = np.array([0,0,0])

        ## END STUDENT CODE

        return xdes, vdes

    ###################
    ## VISUALIZATION ##
    ###################

    def show_ee_position(self):
        msg = geometry_msgs.msg.PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'endeffector'
        msg.point.x = 0
        msg.point.y = 0
        msg.point.z = 0
        self.point_pub.publish(msg)

    ################
    ## CONTROLLER ##
    ################

    def follow_trajectory(self, state, trajectory):

        if self.active:

            try:
                

                t = time_in_seconds() - self.start_time

                # get desired trajectory position and velocity
                xdes, vdes = trajectory(t)
                

                # get current end effector position
                q = state['position']
                joints, T0e = self.fk.forward(q)

                self.xold = deepcopy(self.x)
                self.x = (T0e[0:3,3])

                # First Order Integrator, Proportional Control with Feed Forward
                kp = 20 #0.0002427265629863269 0.00022433558505511385
                # vdes = 0
                v = vdes + kp * (xdes - self.x)

                # Velocity Inverse Kinematics #np.nan,np.nan,np.nan
                dq = IK_velocity(q,v,np.array([np.nan,np.nan,np.nan]))

                # Get the correct timing to update with the robot
                if self.last_iteration_time == None:
                    self.last_iteration_time = time_in_seconds()
                
                self.dt = time_in_seconds() - self.last_iteration_time
                self.last_iteration_time = time_in_seconds()
                
                new_q = q + self.dt * dq

                # arm.safe_move_to_position(new_q)
                
                arm.safe_set_joint_positions_velocities(new_q, dq)

                """
                This part is famously known as Dhruv's Mess
                """
                test2 = 0
                test3 = 1
                if test2:
                    """
                    This is for test2! - Line trajectory!
                    """
                    if (abs(self.x[1]-0.2)<=0.0001):
                        print(f"Side 1: {time_in_seconds()}")
                        self.v_accum = 0
                        self.v_counter = 0
                    if (abs(self.x[1]+0.2)<=0.0001):
                        print(f"Side 2: {time_in_seconds()}")
                        print(self.v_accum/self.v_counter)
                    self.v_accum += v
                    self.v_counter += 1
                # /gazebo/link_states
                # rospy.Subscriber('/gazebo/link_states', LinkStates, self.joint_callback)
                if test3:
                    v_calc = (self.x - self.xold) #Calculate velocity
                    # print(self.dt)
                    # self.error += np.linalg.norm((v*self.dt - v_calc))
                    #stuff - for position - not for experimentations
                    q = state['position']
                    joints, T0e = self.fk.forward(q)
                    x = (T0e[0:3,3])
                    self.error += np.linalg.norm(x-xdes)                    
                    self.no_iterations += 1

                # self.q1.append(q[0])
                # self.q2.append(q[1])
                # self.q3.append(q[2])
                # self.q4.append(q[3])
                # self.q5.append(q[4])
                # self.q6.append(q[5])
                # self.q7.append(q[6])
                    if t>=20 and self.flag==0:
                        self.flag = 1
                        print("Error after 20 seconds of trajectory")
                        print(self.error/self.no_iterations)
                        print(self.dt)
                    
                    # a = [max(self.q1)-min(self.q1),max(self.q2)-min(self.q2),max(self.q3)-min(self.q3),max(self.q4)-min(self.q4),max(self.q5)-min(self.q5),max(self.q6)-min(self.q6),max(self.q7)-min(self.q7)]
                    # print([round(i*180/pi,3) for i in a] )

                



                # Downsample visualization to reduce rendering overhead
                self.counter = self.counter + 1
                if self.counter == 10:
                    self.show_ee_position()
                    self.counter = 0

            except rospy.exceptions.ROSException:
                pass

    def joint_callback(self,data): # data of type JointState
        # Each subscriber gets 1 callback, and the callback either
        # stores information and/or computes something and/or publishes
        # It _does not!_ return anything
        global g_joint_states, g_positions, posx,posy,posz
        g_joint_states = data
        # rostopic echo -n 1 /gazebo/model_states
        g_positions = data.pose
        print(g_positions[0].position.x)
        posx = g_positions[0].position.x
        posy = g_positions[0].position.y
        posz = g_positions[0].position.z
        print(posx,posy,posz)


if __name__ == "__main__":

    if len(sys.argv) < 2:
        print("usage:\n\tpython jacobianDemo.py line\n\tpython jacobianDemo.py ellipse\n\tpython jacobianDemo.py eight")
        exit()

    rospy.init_node("follower")

    JD = JacobianDemo()

    if sys.argv[1] == 'line':
        callback = lambda state : JD.follow_trajectory(state, JacobianDemo.line)
    elif sys.argv[1] == 'ellipse':
        callback = lambda state : JD.follow_trajectory(state, JacobianDemo.ellipse)
    elif sys.argv[1] == 'eight':
        callback = lambda state : JD.follow_trajectory(state, JacobianDemo.eight)
    else:
        print("invalid option")
        exit()

    arm = ArmController(on_state_callback=callback)

    # reset arm
    print("resetting arm...")
    arm.safe_move_to_position(arm.neutral_position())

    # start tracking trajectory
    JD.active = True
    JD.start_time = time_in_seconds()

    input("Press Enter to stop")
