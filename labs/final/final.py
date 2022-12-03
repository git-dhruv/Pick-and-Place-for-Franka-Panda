import sys
import numpy as np
from copy import deepcopy
from math import pi, atan2, sin,cos,sqrt

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds


#Team Libraries
from lib.calculateFK import FK
from lib.IK_velocity import IK_velocity
        

from ik import IK

class Final():
    def __init__(self):
        #ROS Classes

        self.team = "red" #default

        try:
            self.team = rospy.get_param("team") # 'red' or 'blue'
        except KeyError:
            print('Team must be red or blue - make sure you are running final.launch!')
            exit()

        rospy.init_node("team_script")
        self.arm = ArmController()
        self.detector = ObjectDetector()

        self.fk = FK()

        #!-- States --!#
        #List of states that will be populated in the main code. 
        self.JointPositions = None
        self.T0e = None
        #Current angle of the robot
        self.currentangle = self.arm.get_positions()
        #Previous angle - t_(i-1)
        self.prevangle = self.arm.get_positions()
        #Current Position of end effector - xyz
        self.eepos = np.eye(4)
        #Previous Position of end effector
        self.eeprevpos = np.array([0,0,0])

        #Time
        self.t1 = time_in_seconds()
        self.dt = 1e-10

        #Array of 4 Static Cubes
        self.cubeStaticCamera = np.zeros((4,4,4))
        #Array of Dynamic Cubes - Undecided
        self.cubeDynamicCamera = np.zeros((8,4,4))
        
        #Array of 4 Static Cubes from Base
        self.cubeStaticBase = np.zeros((4,4,4))
        self.approachAxis = np.zeros((4,1))

        self.cubeDynamicBase = np.zeros((8,4,4))

        #Block of Interest
        self.dynamicBlockofInterest = None
        self.staticBlockofInterest = None

        self.StaticBlocksIhave = []

        #Number of cubes that are detected
        self.numStaticCubes = sum([np.linalg.norm(self.cubeStaticCamera[i,:,:])>0 for i in range(4)])


        #!-Thread Architecture-!#
        self.cmdthread = None
        self.mainloopthread = None

        self.ik = IK()



    #----------------------------------#
    # Please discuss with me before modifications, somethings might already be calculated but not stored
    #----------------------------------#

    #!-- State Estimation Functions--!#
    def calculateForwardKinematics(self,q=None,ret=False):
        """
        Calculates FK. If angle not provide, it take the current angle
        """
        #Take Current angle by default
        if q is None:
            q = self.currentangle
        
        self.JointPositions, self.T0e = self.fk.forward(q)
        if ret:
            return self.T0e

    def updateAllStates(self):
        """
        Call this method to calculate everything - First few readings will be bad
        Updates Angular Velocities, Angles, End effector Velocities and position
        """
        self.updateDT()
        self.updateAngularStates()
        self.updateLateralStates()

    
    def updateAngularStates(self):
        """
        Updates Angles and Angular Velocities. Also stores the previous positions
        """
        self.prevangle = self.currentangle
        self.currentangle = self.arm.get_positions()
        self.qdot = self.arm.get_velocities()
        self.roll, self.pitch, self.yaw = self.getRPYfromRmat(self.eepos[0:3,0:3])

    def getRPYfromRmat(self,Rmat):
        yaw=atan2(Rmat[1,0],Rmat[0,0])
        pitch=atan2(-Rmat[2,0],sqrt(Rmat[2,1]**2+ Rmat[2,2]**2))
        roll=atan2(Rmat[2,1],Rmat[2,2])
        return roll,pitch,yaw


    def updateLateralStates(self):
        """
        Updates Lateral States of end effector -> You need min snap? Modify this
        """
        self.eeprevpos = self.calculateForwardKinematics(self.prevangle,ret=True) 
        self.eepos = self.calculateForwardKinematics(self.currentangle,ret=True)
        self.eevel = (self.eepos[0:3,-1] - self.eeprevpos[0:3,-1])/self.dt

    def calculateCam2Base(self):
        """
        Camera wrt to the Base
        """
        #If we have not yet calculated anything, take the current angles
        if self.T0e is None:
            q = self.arm.get_positions()
            self.calculateForwardKinematics(q)
         
        #Cam to End effector
        self.Cam2EE = self.detector.get_H_ee_camera()
        self.Cam2Base = self.T0e@self.Cam2EE

    def populate_raw_blocks(self):
        """
        Block wrt to the Camera
        """
        #Since its just 4 times loop its fine..
        for (name, pose) in self.detector.get_detections():
            self.nameParser(name,pose)
        self.numStaticCubes = sum([np.linalg.norm(self.cubeStaticCamera[i,:,:])>0 for i in range(4)])

    def get_block_from_base(self):
        """
        Call this to get the block transformation wrt to the base frame
        """

        if self.numStaticCubes<1: 
            print("Sir, Cubes not detected or populated from populate raw blocks!")
            
        else:
            #Block to Cam - can someone help me vectorize this?
            #We also apply a LPF but need a warm start on this
            weight = 0.7
            for block in self.StaticBlocksIhave:
                self.cubeStaticBase[block,:,:] = weight*self.cubeStaticBase[block,:,:] + (1-weight)*(self.Cam2Base)@self.cubeStaticCamera[block,:,:]

        for block in range(8):
            self.cubeDynamicBase[block,:,:] = (self.Cam2Base)@self.cubeDynamicCamera[block,:,:]
    
    

    def LeastSquaresEstimate(self):
        """
        We know one of the axis of the cube has to be upright - unknown direction
        We use that and calculate other axes using Least squares 
        """
        for block in self.StaticBlocksIhave:
            #Get the current static Estimate 
            R = self.cubeStaticBase[block,0:3,0:3]

            #Base approach vector - sign doesn't matter
            basevector = [0,0,1]
            #Get the error
            err = [np.linalg.norm(basevector-abs(R[:,i].flatten())) for i in range(3)] #if err is zero then GTFO
            #If error for one axis is zero that means that we are probably getting a very good reading
            if min(err)<=1e-5:
                return
            
            #Get Approach Axis
            approachAxis = err.index(min(err))
            #Fix the approach Axis and do Procustes
            self.cubeStaticBase[block,:,approachAxis] = np.round(self.cubeStaticBase[block,:,approachAxis])
            U,_,Vt = np.linalg.svd(self.cubeStaticBase[block,0:3,0:3])
            Smod = np.eye(3)
            Smod[-1,-1] = np.linalg.det(U@Vt)
            self.cubeStaticBase[block,0:3,0:3] = U@Smod@Vt

            self.approachAxis[block,0] = approachAxis


    def Block2BasePipeline(self):
        """
        If you are lazy, call this. If calculates everything and then gives you Cube wrt to the base frame
        Just be cautious as its inefficient to do Matmul everytime for no reason at all.
        """
        #Check if we are moving
        if np.linalg.norm(self.eevel)>2:
            print("High Speed Motion - Blurry Image or bad estimate possibility")
            return

        self.calculateCam2Base()
        self.populate_raw_blocks()
        self.get_block_from_base()
        self.LeastSquaresEstimate()

    def nameParser(self,string,value):
        """
        Parse the cube strings
        """
        if string[6]=='s':
            self.cubeStaticCamera[int(string[4])-1,:,:] = value

            #Also populate Static Blocks for index accessibility
            if int(string[4]) not in self.StaticBlocksIhave:
                self.StaticBlocksIhave.append(int(string[4])-1)
        if 'dynamic' in string:
            if string[5].isdigit:
                num = int(string[4])*10 + int(string[5]) - 4
            else:
                num = int(string[4]) -  4
            self.cubeDynamicCamera[num-1,:,:] = value
            
            

    def updateDT(self):
        """
        Under construction - But gets the time step dt
        """
        self.t2 = time_in_seconds()
        self.dt = self.t2-self.t1
        if self.dt ==0:
            self.dt = 1e-6
        else:
            self.t1 = self.t2
    def makeAlinetraj(self,maxt,startpos,endpos):
        t = np.linspace(0, maxt, num=1000)
        xdes =  np.array([(endpos[0]-startpos[0])*t/maxt + startpos[0],(endpos[1]-startpos[1])*t/maxt + startpos[1],(endpos[2]-startpos[2])*t/maxt + startpos[2]])
        vdes = np.array([((endpos[0]-startpos[0])+0*t)/maxt,((endpos[1]-startpos[1])+0*t)/maxt,((endpos[2]-startpos[2])+0*t)/maxt])  

        return xdes, vdes

    def run(self):
        for i in range(10):
                self.updateAllStates()
    
        #Move to neutral Position
        self.commanded_q = self.arm.neutral_position()  
        self.commanded_q[-2] += 0.4 
        self.commanded_q[0] += pi/2
        self.arm.safe_move_to_position(self.commanded_q)
        self.arm.open_gripper()
        
        print("Updating States")
        #State Estimation Pipeline
        self.updateAllStates()      #Update Angular and Lateral States of Robot
        for i in range(50):
            self.Block2BasePipeline()   #Update Blocks wrt Base (and Camera and whatever in the process)

        # Jacobian based control system
        blockno = 6
        self.TargetPos = self.cubeDynamicBase[blockno,:,:]
         
        xdes, vdes = self.makeAlinetraj(100,self.eepos[0:3,-1],self.TargetPos[0:3,-1])

        # Get the target rpy
        troll, tpitch, tyaw = self.getRPYfromRmat(self.TargetPos[0:3,0:3])

        # Verify This   - We only do nonsense with the aligned trajectory         
        if self.approachAxis[blockno,0]==1: 
            tyaw = troll

        if self.approachAxis[blockno,0] ==0:
            tyaw = tpitch

        if self.approachAxis[blockno,0] ==2:
            tyaw = troll

        # Make the desired angles and desired angular omega
        angdes,omegades = self.makeAlinetraj(100,[self.pitch,self.roll,self.yaw],[ tpitch,troll, tyaw])

        q = self.currentangle
        self.last_iteration_time = None
        new_q = q

        #!-We need a while loop here that has a tolerance on error rather than this iterator--!#        
        for i in range(1000):
            #Update            
            self.updateAllStates()
            new_q = self.currentangle
            #P gain
            kp = 10
            v = vdes[:,i] + kp * (xdes[:,i] - self.eepos[0:3,-1])
            # Verify This   2 - Make Z trajectory slower
            v[-1] = 0.4*v[-1]

            #Controller for orientation
            w = omegades[:,i] + .1*(angdes[:,i] - [self.pitch,self.roll,self.yaw])

            # Velocity Inverse Kinematics
            dq = IK_velocity(new_q,v,np.array([np.nan,np.nan,w[-1]]))

            # Get the correct timing to update with the robot
            if self.last_iteration_time == None:
                self.last_iteration_time = time_in_seconds()
            
            self.dt = time_in_seconds() - self.last_iteration_time
            self.last_iteration_time = time_in_seconds()
            
            new_q += self.dt * dq
            
            self.arm.safe_set_joint_positions_velocities(new_q, dq)

        # Verify This   3 - The last bit of Z that is left
        
        for i in range(100):
            self.updateAllStates()

            new_q = self.currentangle
            
            kp = 3
            v = vdes[:,i] + kp * (xdes[:,-1] - self.eepos[0:3,-1])
            
            w = omegades[:,i] + .1*(angdes[:,-1] - [self.pitch,self.roll,self.yaw])
            # Velocity Inverse Kinematics
            dq = IK_velocity(new_q,v,np.array([np.nan,np.nan,w[-1]]))

            # Get the correct timing to update with the robot
            if self.last_iteration_time == None:
                self.last_iteration_time = time_in_seconds()
            
            self.dt = time_in_seconds() - self.last_iteration_time
            self.last_iteration_time = time_in_seconds()
            
            new_q += self.dt * dq
            
            self.arm.safe_set_joint_positions_velocities(new_q, dq)
        
        self.arm.close_gripper()


        print(self.eepos-self.cubeStaticBase[blockno,:,:])



    
    



if __name__ == "__main__":

    # print("\n****************")
    # if team == 'blue':
    #     print("** BLUE TEAM  **")
    # else:
    #     print("**  RED TEAM  **")
    #Make stuff
    game = Final()
    # #This is our main method being called. 
    game.run()




























"""
#This is for generating Plots for report - To my future self, put this after LeastSquare Block
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# TODO: update this with real results
ax.scatter(0,0,0,s=0.25) # plot the point (1,1,1)
plt.xlim([-5,5])
plt.ylim([-5,5])
ax.set_zlim([-5,5])
axis = H[0:3,0:3]@np.eye(3)
x_pos = [0, 0,0]
y_pos = [0, 0,0]
x_direct = axis[:,0]
y_direct = axis[:,1]
z_direct = axis[:,2]
ax.quiver(x_pos,y_pos,x_pos,x_direct,y_direct,z_direct,color='g',cmap="Reds")
axis = self.cubeStaticBase[block,0:3,0:3]@np.eye(3)
x_direct = axis[:,0]
y_direct = axis[:,1]
z_direct = axis[:,2]
ax.quiver(x_pos,y_pos,x_pos,x_direct,y_direct,z_direct,color='r',cmap="Reds")

axis = noisyH[0:3,0:3]@np.eye(3)
x_direct = axis[:,0]
y_direct = axis[:,1]
z_direct = axis[:,2]
ax.quiver(x_pos,y_pos,x_pos,x_direct,y_direct,z_direct,color='b',cmap="Reds")
plt.show()
"""