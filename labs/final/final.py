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
from ik1 import IK

try:
	import bezier
except:
	print("Don't use bezier because u need to install library")

import time
"""
0.2 0.0001 - widths
0.01 50 Forces

"""

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

		print("The team is : ",self.team)

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
		self.DynamicApproachAxis = np.zeros((8,1))

		self.cubeDynamicBase = np.zeros((8,4,4))

		#Block of Interest
		self.dynamicBlockofInterest = None
		self.staticBlockofInterest = None

		self.StaticBlocksIhave = []
		self.DynamicBlocksInView = []

		#Number of cubes that are detected
		self.numStaticCubes = sum([np.linalg.norm(self.cubeStaticCamera[i,:,:])>0 for i in range(4)])
		self.DynamicLastUpdate = np.zeros([1,8]).flatten()

		#!-Thread Architecture-!#
		self.cmdthread = None
		self.mainloopthread = None


		#Caching and Seeds
		self.cacheInterAfterPicking = None
		self.cachePickingIk = None

		self.dictOfTags = {}

		
		
		self.ik = IK()


	#----------------------------------#
	# 520 FINAL COMPETITION CODE	   #
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
		self.getEEpose()

	def getRPYfromRmat(self,Rmat):
		yaw=atan2(Rmat[1,0],Rmat[0,0])
		pitch=atan2(-Rmat[2,0],sqrt(Rmat[2,1]**2 + Rmat[2,2]**2))
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
	
		q = self.arm.get_positions()
		self.calculateForwardKinematics(q)
		 
		#Cam to End effector
		self.Cam2EE = self.detector.get_H_ee_camera()
		self.Cam2Base = self.eepos@self.Cam2EE

	def populate_raw_blocks(self):
		"""
		Block wrt to the Camera
		"""
		#Since its just 4 times loop its fine.. get over it
		self.numstaticblocks = 0
		self.numdynblocks = 0
		self.DynamicTags = []
		self.dictOfTags = {}
		
		for (name, pose) in self.detector.get_detections():
			self.nameParser(name,pose)
		self.numStaticCubes = sum([np.linalg.norm(self.cubeStaticCamera[i,:,:])>0 for i in range(4)])

	def cleanupDynamicBlocks(self):
		"""
		Clean the Dynamic Blocks
		"""
		t1 = time_in_seconds()
		updateMask = [(t1 - i)>.5 for i in self.DynamicLastUpdate]
		self.cubeDynamicBase[updateMask,:,:] = np.zeros((4,4))
		indices_where_updated = np.argwhere(np.array(updateMask)==False).flatten()
		self.DynamicBlocksInView = indices_where_updated
		# listofkeys = list(self.dictOfTags.keys())
		# print(listofkeys, updateMask)
		# for i in range(len(updateMask)):
		# 	if updateMask[i]:
		# 		print(i)
		# 		print(listofkeys[i])
		# 		self.dictOfTags.pop(listofkeys[i])

	def get_block_from_base(self):
		"""
		Call this to get the block transformation wrt to the base frame
		"""

		if self.numStaticCubes<1: 
			pass			
		else:
			#Block to Cam - can someone help me vectorize this?

			#We also apply a LPF for some reason
			weight = 0 #Current disabled since its performing okayish
			for block in self.StaticBlocksIhave:
				self.cubeStaticBase[block,:,:] = weight*self.cubeStaticBase[block,:,:] + (1-weight)*(self.Cam2Base)@self.cubeStaticCamera[block,:,:]
		keys = list(self.dictOfTags.keys())
		for block in range(8):
			self.cubeDynamicBase[block,:,:] = (self.Cam2Base)@self.cubeDynamicCamera[block,:,:]

		#Pray to the god if this last minute corrections work
		for block in keys:
			self.dictOfTags[block] = (self.Cam2Base)@self.dictOfTags[block]
	
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
			err = [np.linalg.norm(basevector-abs(R[:,i].flatten())) for i in range(3)] #if err is zero

			#Store the approachAxis before returning
			approachAxis = err.index(min(err))
			self.approachAxis[block,0] = approachAxis

			#If error for one axis is zero that means that we are probably getting a very good reading
			if min(err)<=1e-4:
				pass
			else:
				#Fix the approach Axis and do Procustes
				self.cubeStaticBase[block,:,approachAxis] = np.round(self.cubeStaticBase[block,:,approachAxis])
				U,_,Vt = np.linalg.svd(self.cubeStaticBase[block,0:3,0:3])
				Smod = np.eye(3)
				Smod[-1,-1] = np.linalg.det(U@Vt)
				self.cubeStaticBase[block,0:3,0:3] = U@Smod@Vt

			

		#For dynamic blocks#
		for block in self.DynamicBlocksInView:

			R = self.cubeDynamicBase[block,0:3,0:3]
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
			self.cubeDynamicBase[block,:,approachAxis] = np.round(self.cubeDynamicBase[block,:,approachAxis])
			U,_,Vt = np.linalg.svd(self.cubeDynamicBase[block,0:3,0:3])
			Smod = np.eye(3)
			Smod[-1,-1] = np.linalg.det(U@Vt)
			self.cubeDynamicBase[block,0:3,0:3] = U@Smod@Vt

			self.DynamicApproachAxis[block,0] = approachAxis

		keys = list(self.dictOfTags.keys())
		for block in keys:

			R = self.dictOfTags[block][0:3,0:3]
			#Base approach vector - sign doesn't matter
			basevector = [0,0,1]
			#Get the error
			err = [np.linalg.norm(basevector-abs(R[:,i].flatten())) for i in range(3)] #if err is zero then GTFO
			#If error for one axis is zero that means that we are probably getting a very good reading
			if min(err)<=1e-5:
				return
			cubeDynamicBase = self.dictOfTags[block]
			#Get Approach Axis
			approachAxis = err.index(min(err))
			#Fix the approach Axis and do Procustes
			cubeDynamicBase[:,approachAxis] = np.round(cubeDynamicBase[:,approachAxis])
			U,_,Vt = np.linalg.svd(cubeDynamicBase[0:3,0:3])
			Smod = np.eye(3)
			Smod[-1,-1] = np.linalg.det(U@Vt)
			cubeDynamicBase[0:3,0:3] = U@Smod@Vt
			self.dictOfTags[block] = deepcopy(cubeDynamicBase)

		

	def Block2BasePipeline(self):
		"""
		If you are lazy, call this. If calculates everything and then gives you Cube wrt to the base frame
		Just be cautious as its inefficient to do Matmul everytime for no reason at all.
		"""
		#Check if we are moving
		if np.linalg.norm(self.eevel)>2:
			print("High Speed Motion - Blurry Image or bad estimate possibility")
			pass
		self.updateAllStates()
		self.calculateCam2Base()
		self.populate_raw_blocks()
		self.get_block_from_base()
		self.LeastSquaresEstimate()
		self.cleanupDynamicBlocks()

	def nameParser(self,string,value):
		"""
		Parse the cube strings
		"""
		if 'static' in string:
			self.cubeStaticCamera[self.numstaticblocks,:,:] = value
			#Also populate Static Blocks for index accessibility
			if self.numstaticblocks not in self.StaticBlocksIhave:
				self.StaticBlocksIhave.append(self.numstaticblocks)
			self.numstaticblocks += 1

		if 'dynamic' in string:
			
			if string[5].isdigit():
				num = int(string[4])*10 + int(string[5])
			else:
				num = int(string[4])
			self.dictOfTags[num] = value
			self.DynamicTags.append(num)
			num = deepcopy(self.numdynblocks)
			self.cubeDynamicCamera[num,:,:] = value
			self.DynamicLastUpdate[num] = time_in_seconds()
			self.numdynblocks+=1

	def moveIKFaster(self, finalq, Kp=1.2):
		#Mini Angular P control - Because why not
		while np.linalg.norm(self.currentangle - finalq)>=0.2:
			#Feedback
			self.updateAllStates()
			dq = -1*(self.currentangle - finalq)
			ang = self.currentangle + dq*0.01			
			self.arm.safe_set_joint_positions_velocities(ang, dq)

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
				   
	def makeAlinetraj(self,startpos,endpos,numpts=200):
		maxt = numpts
		t = np.linspace(0, maxt, num=numpts)
		xdes =	np.array([(endpos[0]-startpos[0])*t/maxt + startpos[0],(endpos[1]-startpos[1])*t/maxt + startpos[1],(endpos[2]-startpos[2])*t/maxt + startpos[2]])
		vdes = np.array([((endpos[0]-startpos[0])+0*t)/maxt,((endpos[1]-startpos[1])+0*t)/maxt,((endpos[2]-startpos[2])+0*t)/maxt])	 

		return xdes, vdes


	def makeAQuadratictraj(self, maxt, startpos, midpos, endpos, numpts = 500):
		"""
		An attempt at a worse and simpler 2nd degree curve
		"""
		t = np.linspace(0, maxt, num=numpts)

		#Equation for X
		inter = int(numpts/2)
		coeffMat = np.array([[t[0]**2,t[0],1], 
						 [t[-1]*t[-1], t[-1],1], 
						 [t[inter]**2 , t[inter] , 1]  ]) 
		coeffs = (np.linalg.inv(coeffMat)@np.array([  [startpos[0]], [endpos[0]],[midpos[0]]  ])).flatten()

		#x = at2+bt+c
		xc = startpos[0]
		xa = coeffs[0]
		xb = coeffs[1]

		#Equation for Y
		inter = int(numpts/2)
		coeffs = (np.linalg.inv(coeffMat)@np.array([ [startpos[1]],[endpos[1]],[midpos[1]] ])).flatten()
		#y = at2+bt+c
		yc = startpos[1]
		ya = coeffs[0]
		yb = coeffs[1]

		#Equation for Z
		inter = int(numpts/2)
		coeffs = (np.linalg.inv(coeffMat)@np.array([ [startpos[2]],[endpos[2]],[midpos[2]] ])).flatten()
		#z = at2+bt+c
		zc = startpos[2]
		za = coeffs[0]
		zb = coeffs[1]


		xdes =	np.array([xa*t*t + xb*t + xc, ya*t*t + yb*t + yc, za*t*t + zb*t + zc])
		vdes = np.array([2*xa*t + xb, 2*ya*t + yb , 2*za*t + zb])

		return xdes, vdes

	def get_bezierCurve(self, start_pos, mid_pos, end_pos, num_points):
		
		nodes = np.asfortranarray([start_pos.tolist(), (np.array([1,1,0])*start_pos+np.array([0,0,1])*max(start_pos[-1], mid_pos[-1], end_pos[-1])).tolist(),
							mid_pos.tolist(), (np.array([1,1,0])*end_pos+np.array([0,0,1])*max(start_pos[-1], mid_pos[-1], end_pos[-1])).tolist(), end_pos.tolist()], dtype = object)

		curve = bezier.Curve(nodes.T, degree = 4)
		xdes = curve.evaluate_multi(np.linspace(0,1,num_points))

		vmax = 1.5/3**0.5
		vdes = np.linspace(np.zeros(3),vmax,num_points).T

		return xdes, vdes


	def getEEpose(self):

		R = self.eepos[0:3,0:3]
		base = np.eye(3)
		self.pitch = self.wrapper(np.arccos( R[:,0]@base[:,0]) )
		self.roll = self.wrapper(np.arccos( R[:,1]@base[:,1]) ) 
		self.yaw = self.wrapper(np.arccos( R[:,2]@base[:,2]) ) 

	def rotz(self,a):
		"""
		 rotation matrix around Z
		"""
		return np.array([[cos(a), -sin(a),	0],[sin(a), cos(a),	 0],[0,0,1]])
	
	def wrapper(self, val):
		return np.arctan2(np.sin(val),np.cos(val))

	def rotateBlock(self,block):
		for i in range(3):
			if np.all(block[:2,i] < 1e-5):
				if block[2,i] > 0.9:
					block[:3,i] = -1*block[:3,i]
					block[:3,(i+2)%3] = -1*block[:3,(i+2)%3]
		return block

	def get_path(self, Target, StackPos, num_points = 3):
		"""
		Gives the target pos angle but breaks done ik in several
		"""
		midpoint = (Target + StackPos)/2
		
		path = np.vstack([np.linspace(Target, midpoint, num_points), np.linspace(midpoint, StackPos, num_points)])
		q_path = np.zeros((len(path),7))
		self.updateAllStates()
		q_path[0] = self.currentangle
		for i, T in enumerate(path):
			if i==0:
				continue
			q_path[i] = self.ik.inverse(T, q_path[i-1])
		
		return q_path[-1]
	def alignTarget(self, TargetPos):
		TargetPos = self.rotateBlock(TargetPos)
		T = np.zeros((4,4))
		T[:3,2] = np.array([0,0,-1.0])
		T[-1] = np.array([0,0,0,1.0])
		T[:3,3] =TargetPos[:3,3]
		
		errors = abs(TargetPos[:3,:3]) - np.array([[0,0,1],[0,0,1],[0,0,1]]).T
		z_error = np.array([np.linalg.norm(errors[:3, i]) for i in range(3)])
		z_index = np.where(z_error == z_error.min())[0][0]

		T[:3,0] = TargetPos[:3,(z_index+1)%3]
		T[:3,1] = TargetPos[:3,(z_index+2)%3]

		#Ensure that the Z axis is 0,0,-1 or 0,0,1
		success = False
		if abs(round(T[2,2]))==1:
			success=True

		return T,success


	def optlastangle(self, config, currentq):
		qtarget = config
		error = -(currentq[-1] - qtarget[-1])*180/pi #num
		tmperr = abs(error)%90
		error = tmperr*(error/abs(error))
		config[-1] = currentq[-1] + (error*pi/180)
		return config
	
	def checkvalidBlock(self):
		for block in self.StaticBlocksIhave:
			T = self.cubeStaticBase[block,:,:]
			_, success = self.alignTarget(T)
			if not success:
				return False
		return True

	def searchStaticBlocks(self,blockNoFault = 0):
		itr = 0
		while itr<20:
			# self.arm.safe_move_to_position(self.scoutposition)
			self.Block2BasePipeline()
			validSol = self.checkvalidBlock()
			if validSol:
				if blockNoFault==0:
					return
				if blockNoFault==1:
					print(self.numStaticCubes)
					if self.numStaticCubes>3:
						return
					else:
						print(self.numStaticCubes)
						fault = 1
			else:
				fault = 1

			if fault:
				q = deepcopy(self.scoutposition)
				#Yes, I am ashamed of this
				import random
				q[0] = (q[0] + random.random()-0.5)*0.5
				q[-2] += (random.random()-0.5)*0.1
				self.arm.safe_move_to_position(q)
			itr += 1
		return 

	def JacobianBasedControl(self,xdes,vdes, kp=20, ascendLogic=False):
		#Getting current angle		
		q = self.currentangle
		self.last_iteration_time = None
		new_q = q
		for i in range(len(xdes[0,:])):
			#BY DEFAULT WE ARE ALWAYS GOING DOWN MAYDAY
			ascend = False

			#Update			   
			self.updateAllStates()
			#New q
			new_q = self.currentangle

			#If we are supposed to descend or ascend
			if self.eepos[2,-1]<xdes[2,i]:
				ascend = True
			
			if ascendLogic == False:
				ascend = True

			#If we are ascending then do some things
			if ascend:
				kp = kp/2
			
			# PD Controller
			v = vdes[:,i] + kp * (xdes[:,i] - self.eepos[0:3,-1])
			if not ascend: 
				#Dont descend like a smarty
				if self.eepos[2,-1]<0.2 and v[-1]<0:
					v[-1] = 0

			# Normalize the Velocity and restrict to 2 m/s - Allows for better Kp gain
			if np.linalg.norm(v)>=2:
				v = v*2/np.linalg.norm(v)

			if not ascend:			
				# Make Z trajectory slower
				v[-1] = 0.7*v[-1]

			# Velocity Inverse Kinematics
			dq = IK_velocity(new_q,v,np.array([np.nan,np.nan,np.nan]))
			
			# Get the correct timing to update with the robot
			if self.last_iteration_time == None:
				self.last_iteration_time = time_in_seconds()
			
			self.dt = time_in_seconds() - self.last_iteration_time
			self.last_iteration_time = time_in_seconds()
			

			new_q += self.dt * dq
			
			self.arm.safe_set_joint_positions_velocities(new_q, dq)


	def StaticCubeControlPipeline(self,TargetPos, blockno):
		"""
		Static Cube Controller Give a block and it will complete picking and placing
		
		Was not allowed :(
		"""

		print("Block No: ", blockno)

		#If we are picking first block, then go with y=mx+c
		if self.numblockStacked==0:
			xdes, vdes = self.makeAlinetraj(self.eepos[0:3,-1],TargetPos[0:3,-1],numpts=500)
			xdes = (xdes[:,-1].reshape(3,1))*np.ones((3,500))
			print(xdes.shape)
			# cdes = xdes[-1,:]*np.ones((200,3))
			kp = 10
		
		#Else we are reaching from stacking position 
		else:
			#Mid position of bezier 
			midpos = deepcopy(self.eepos[0:3,-1])
			#Mid position should be above the start position
			midpos[-1] = midpos[-1] + 0.1

			#Target Position should be slightly above
			TargetPosForBezier = deepcopy(TargetPos[0:3,-1])
			TargetPosForBezier[-1] += 0.1 

			#Make the trajectories
			_, vdes = self.makeAQuadratictraj(1, self.eepos[0:3,-1], midpos, TargetPosForBezier, 500)
			xdes,_ = self.get_bezierCurve(self.eepos[0:3, -1], midpos, TargetPosForBezier, 500)
			kp = 20


		# Transforming block orientation in terms of end effector reachable orientation
		TargetPos = self.rotateBlock(TargetPos)
		TargetPos,_ = self.alignTarget(TargetPos)

		print("=====Picking=====")
		
		self.JacobianBasedControl(xdes,vdes,kp,ascendLogic=True)

		print("Picking")
		#=#- Picking with IK -#=#
		self.updateAllStates()
		tmp = deepcopy(self.currentangle)
		print("Block no: ",blockno, " ; IK on block ")
		TargetPos2 = deepcopy(TargetPos)
		print(TargetPos2,self.eepos)
		TargetPos2[2,-1] = self.eepos[2,-1]
		if self.cachePickingIk is None:
			self.cachePickingIk = self.currentangle

		q = self.ik.inverse(TargetPos2, self.currentangle)

		error = (tmp[-1] - q[-1])*180/pi
		error = error%90
		q[-1] = tmp[-1] + self.wrapper(error*pi/180)
		lastjt = deepcopy(q[-1])

		self.arm.safe_move_to_position(q)

		#Dataloggers
		# with open("new_file.csv","a") as my_csv:
			# csvWriter = csv.writer(my_csv,delimiter=',')
			# csvWriter.writerow(q)
		q= self.ik.inverse(TargetPos, q)		
		self.cachePickingIk = deepcopy(q)		

		q[-1] = lastjt
		self.arm.safe_move_to_position(q)
		self.arm.exec_gripper_cmd(0.0499, 50)

		#This has to be adaptive for red and black
		if self.team=="red":
			TargetPos = np.array([[1,0,0,0.562],
											[0,-1,0,0.144],
											[0,0,-1,0.25 + 0.06*(self.numblockStacked)],
											[0,0,0,1]])
		else:
			TargetPos = np.array([[1,0,0,0.562],
											[0,-1,0,-0.144],
											[0,0,-1,0.25 + 0.06*(self.numblockStacked)],
											[0,0,0,1]])


		if self.cacheInterAfterPicking is None:
			self.cacheInterAfterPicking = q

		
		print("Dropping - IK")
		T_inter = deepcopy(TargetPos)
		T_inter[2,3]+= 0.05
		q_inter = self.ik.inverse(T_inter, self.cacheInterAfterPicking)
		q = self.ik.inverse(TargetPos, self.cacheInterAfterPicking)

		self.cacheInterAfterPicking = deepcopy(q)
		#Stacked!
		self.updateAllStates()
		# self.moveIKFaster(q_inter)
		# xdes,vdes = self.makeAlinetraj(self.eepos[0:3,-1],T_inter[0:3,-1])
		# self.JacobianBasedControl(xdes,vdes*500,15)
		self.arm.safe_move_to_position(q_inter)
		print("IK here!!!")
		self.arm.safe_move_to_position(q)
		self.arm.exec_gripper_cmd(0.075, 10)
		print("===================Finished==================")



	def SafeStaticCubeControlPipeline(self,TargetPos, blockno):
		"""
		Allow me to explain
		"""

		#Target Conditioning
		TargetPos,success = self.alignTarget(TargetPos)
		if success == False:
			print("Scanner has begun!")
			successAgain = self.searchStaticBlocks()
			if successAgain:
				TargetPos,success = self.alignTarget(TargetPos)
			# Vision Pipeline giving noisy reading, leave the block
			else:
				return

		if self.numblockStacked < 6:
			if self.team=="red":
				StackPos = np.array([[1,0,0,0.562],
											[0,-1,0,0.144],
											[0,0,-1,0.225 + 0.05*(self.numblockStacked)+0.01],
											[0,0,0,1]])
			else:
				StackPos = np.array([[1,0,0,0.562],
											[0,-1,0,-0.144],
											[0,0,-1,0.225 + 0.05*(self.numblockStacked)+0.01],
											[0,0,0,1]])
		else:
			if self.team=="red":
				StackPos = np.array([[0,	0,	1,	0.562],
								 [0,	1,	0,	0.144],
								 [-1,	0,	0,	0.225 + 0.05*(self.numblockStacked)+0.01],
								 [0,	0,	0,	1]])
			else:
				StackPos = np.array([[0,	0,	1,	0.562],
								 [0,	1,	0,	-0.144],
								 [-1,	0,	0,	0.225 + 0.05*(self.numblockStacked)+0.01],
								 [0,	0,	0,	1]])
			
		self.updateAllStates()
		#Path to block
		StackAscent = deepcopy(self.eepos)
		StackAscent[2,3] = StackPos[2,3]+0.07
		Targettmp = deepcopy(TargetPos)
		Targettmp[2,3] = StackPos[2,3]+0.1
		path2block = np.vstack([self.get_path(self.eepos, StackAscent), self.get_path(StackAscent,Targettmp), self.get_path(Targettmp, TargetPos)])
		
		#Path to Stack
		StackPostmp = deepcopy(StackPos)
		BlockAscent = deepcopy(TargetPos)
		if self.numblockStacked <6:
			StackPostmp[2,3] += 0.05 #
			BlockAscent[2,3] = StackPostmp[2,3]
			path2Stack = np.vstack([self.get_path(TargetPos, BlockAscent),self.get_path(BlockAscent, StackPostmp),self.get_path(StackPostmp, StackPos)])
		else: 
			StackPostmp[2,3] += 0.1
			BlockAscent[2,3] = StackPostmp[2,3]
			path2Stack = np.vstack([self.get_path(TargetPos, BlockAscent),self.get_path(BlockAscent, StackPostmp),self.get_path(StackPostmp, StackPos)])
		
		print("==============Block No: ", blockno)

		#moving to block - 10.91
		currentq = self.arm.get_positions()
		thresh = [0.2,0.1,0.001]
		i = 0
		for config in path2block:
			if i==1:
				config[-1] = path2block[-1,-1]
			config = self.optlastangle(config,currentq)
			self.arm.safe_move_to_position(config,threshold=thresh[i])
			i = i+1

		self.arm.exec_gripper_cmd(0.049, 50) # Here we have picked the block
		print("Picked")

		currentq = self.arm.get_positions()
		i = 0
		thresh[0] = thresh[2]*50
		thresh[1] = thresh[2]*10
		for config in path2Stack:
			config = self.optlastangle(config,currentq)
			self.arm.safe_move_to_position(config,threshold = thresh[i])
			i = i+1

		self.arm.exec_gripper_cmd(0.075, 50)
		print("===================Finished==================")


	def DynamicGetTarget(self):
		
		#Path to Scout position
		self.updateAllStates()

		for i in range(5):
			self.Block2BasePipeline()
		listofkeys1 = list(self.dictOfTags.keys())
		b1 = self.dictOfTags
		blocks1 = []
		blocks2 = []

		#self.dictOfTags[listofkeys]
		
		initTime = time_in_seconds()
		
		#Measuring the differential of the transformation matrices
		#First reading
		# blocks1 = deepcopy(self.cubeDynamicBase[self.DynamicBlocksInView])
		# print("Blocks: \n", blocks1.round(4))
		time.sleep(1.2)

		#Second reading
		for i in range(5):
			self.Block2BasePipeline()

		dt = max(time_in_seconds()-initTime,1e-7)
		listofkeys2 = list(self.dictOfTags.keys())
		b2 = self.dictOfTags

		for i in list(set(listofkeys1) & set(listofkeys2)):
			blocks1.append(b1[i])
			blocks2.append(b2[i])
		blocks1 = np.array(blocks1)
		blocks2 = np.array(blocks2)

		minlen = min(len(blocks1),len(blocks2))
		blocks1 = blocks1[:minlen]
		blocks2 = blocks2[:minlen]
		for i in range( minlen ):
			blocks1[i,:,:],_ = self.alignTarget(blocks1[i,:,:])
			blocks2[i,:,:],_ = self.alignTarget(blocks2[i,:,:])
		
		w = (blocks2-blocks1)/dt #omega in terms of SO3

		#Prediction
		time_advance = 7 # how many seconds advance to predict
		PredictedBlocks = blocks1+w*time_advance

		#Filtering the closest bloack
		# try:
		x_array = abs(PredictedBlocks[:,0,3].flatten()).tolist()
		target_index = x_array.index(min(x_array))
		TargetBlock = PredictedBlocks[target_index]
			
			# if self.team == "red":
			# 	B_Xneg = PredictedBlocks[[(block[0,3] > 0)  for block in PredictedBlocks]]
			# 	TargetBlock = B_Xneg[[block[1,3] == min(B_Xneg[:,1,3]) for block in B_Xneg]][0]
				
			# else:
			# 	B_Xneg = PredictedBlocks[[(block[0,3] > 0)  for block in PredictedBlocks]]
			# 	TargetBlock = B_Xneg[[block[1,3] == max(B_Xneg[:,1,3]) for block in B_Xneg]][0]

		# except:
		# 	self.DynamicGetTarget()

		return initTime,TargetBlock
		
	def DynamicPicking(self, table_neutral, time_array=None):
		self.arm.safe_move_to_position(table_neutral,threshold=0.1)
		initTime, TargetBlock = self.DynamicGetTarget()
		
		t1 = time_in_seconds()
		#Planning motion to target block
		self.updateAllStates()
		EEDescend = deepcopy(TargetBlock)
		EEDescend[2,3] += 0.1
		path2approach = np.vstack([self.get_path(self.eepos,TargetBlock, 5)])

		while ~(np.linalg.norm(TargetBlock[0:3,3]-self.fk.forward(path2approach[-1])[1][0:3,3]) < 0.2):
			initTime, TargetBlock = self.DynamicGetTarget()
			EEDescend = deepcopy(TargetBlock)
			EEDescend[2,3] += 0.1
			path2approach = np.vstack([self.get_path(self.eepos,TargetBlock, 5)])
			print(abs(TargetBlock-self.fk.forward(path2approach[-1])[1]))
			print("Failed to get valid Target")

		#Moving to Approach Point
		t1 = time_in_seconds()
		for i, config in enumerate(path2approach):
			if i == len(path2approach)-1:
				err = (config[-1] - pi/4)
				print(err)
				config[-1] = self.wrapper(pi/4 + max(err,(err+pi/2)))
				print(config[-1])
				self.arm.safe_move_to_position(config,threshold=0.001)
			self.arm.safe_move_to_position(config,threshold=0.3)
		
		#picking the block
		while 1:
			if time_in_seconds() >= initTime+2:
				self.arm.close_gripper()
				x = self.arm.get_gripper_state()['position']
				dist = abs(x[1] + x[0]) 
				if (dist<= 0.045 or dist>= 0.07): #Failed Grip
					self.arm.exec_gripper_cmd(0.18, 50)
					self.arm.safe_move_to_position(config,threshold=0.01)
					self.arm.close_gripper()
					x = self.arm.get_gripper_state()['position']
					dist = abs(x[1] + x[0]) 
					if (dist<= 0.045 or dist>= 0.07):
						self.arm.exec_gripper_cmd(0.15, 50)
						self.arm.safe_move_to_position(config,threshold=0.1)
						self.DynamicPicking(table_neutral)

				break

		self.arm.safe_move_to_position(table_neutral)

		x=self.arm.get_gripper_state()['position']
		dist = abs(x[0] + x[1])
		if (dist <=0.045):
			self.arm.exec_gripper_cmd(0.12, 50)
			self.DynamicPicking(table_neutral)
		return 

	def SafeDynamicPipeline(self,blockno):
		tmp = self.arm.get_positions()
		#Post Static
		a = time_in_seconds()
		time_array = []
		self.updateAllStates()
		EEtmp = deepcopy(self.eepos)
		EEtmp[2,3] += 0.1
		path2approach = np.vstack([self.get_path(self.eepos,EEtmp)])

		for config in path2approach:
			self.arm.safe_move_to_position(config)

		#Scout Position
		if self.team == 'red':
			table_neutral = np.array([pi/2, 0,0, -pi/2+0.2, 0, pi/2+0.25, pi/4])
		else:
			table_neutral = np.array([-pi/2, 0,0, -pi/2+0.2, 0, pi/2+0.25, pi/4])
		self.DynamicPicking(table_neutral, time_array)
		self.arm.safe_move_to_position(table_neutral)

		#Defining the Positions on the static table
		self.updateAllStates()
		
		if self.team=="red":
			StackPos = np.array([[0,	0,	1,	0.562],
								 [0,	1,	0,	0.144],
								 [-1,	0,	0,	0.225 + 0.05*(self.numblockStacked)+0.01],
								 [0,	0,	0,	1]])
		else:
			StackPos = np.array([[0,	0,	1,	0.562],
								 [0,	1,	0,	-0.144],
								 [-1,	0,	0,	0.225 + 0.05*(self.numblockStacked)+0.01],
								[0,	0,	0,	1]])

		self.numblockStacked += 1
		#!Modified!#	
		
		tmp = self.ik.inverse(StackPos, tmp)
		tmpFK = self.fk.forward(tmp)[1]
		while (np.linalg.norm(tmpFK[:3,3]-StackPos[:3,3]) > 0.5):
			tmp = self.ik.inverse(StackPos, tmp)
			tmpFK = self.fk.forward(tmp)[1]

		
		self.arm.safe_move_to_position(tmp)
		self.arm.exec_gripper_cmd(0.15,50)
	
	def fitRotMatrix(self, mat):
		
		U,_,Vt = np.linalg.svd(mat)
		Smod = np.eye(3)
		Smod[-1,-1] = np.linalg.det(U@Vt)
		return U@Smod@Vt

	def fixLastAngle(self,q, targetPos):

		targetR = targetPos[0:3,0:3]
		temp = self.fk.single_frame_transform(6,q[-1])@self.fk.rotz(-pi/4)
		Fk = targetPos@np.linalg.inv(temp)
		dq = np.linspace(-180,180, num=1500)
		

		dist = [np.linalg.norm(  np.abs(Fk@(self.fk.single_frame_transform(6,q*pi/180)@self.fk.rotz(-pi/4)))[0:2,0:2] - np.abs(targetR[0:2,0:2]) ) for q in dq]
		minIndex = dist.index(min(dist))

		return (dq[minIndex]*pi/180)

	def moveIKFaster(self, finalq, Kp=1.2):
		"""
		Yo Yo Yo This is peak optimization and unsafe moves
		"""
		#Mini Angular P control - Because why not
		while np.linalg.norm(self.currentangle - finalq)>=0.2:
			#Feedback
			self.currentangle = self.arm.get_positions()
			dq = -Kp*(self.currentangle - finalq)
			ang = self.currentangle + dq*0.01			
			self.arm.safe_set_joint_positions_velocities(ang, dq)

	def sortStaticBlocksToPick(self):

		#First block should be nearest to eepos and closer to the 0,0,0
		robotBase = np.array([0,-0.99,0])

		#Distances from Origin
		distx = [np.linalg.norm(self.cubeStaticBase[i,0,-1]) for i in self.StaticBlocksIhave]
		disty = [np.linalg.norm(self.cubeStaticBase[i,1,-1] + robotBase[1]) for i in self.StaticBlocksIhave]

		index = sorted(range(len(disty)), key=lambda k: disty[k])

		leftblock1 = index[0]
		leftblock2 = index[1]

		self.StaticCubesArranged = []
		if distx[leftblock1]<distx[leftblock2]:
			self.StaticCubesArranged.append(leftblock1)
			self.StaticCubesArranged.append(leftblock2)
		else:
			self.StaticCubesArranged.append(leftblock2)
			self.StaticCubesArranged.append(leftblock1)
		for i in range((self.numstaticblocks)-2):	
			self.StaticCubesArranged.append(index[i+2])
		
		return self.StaticCubesArranged

	def StaticBlocksFinal(self):		
		#These are the scouters
		self.netq[-2] += 0.4 
		self.netq[3] += 0.3
		self.netq[1] += 0.4
		#Color specific scout angles
		if self.team=="red":
			self.netq[0] += -0.2
		else:
			self.netq[0] += 0.2
		self.scoutposition = deepcopy(self.netq)
		#Move
		self.arm.safe_move_to_position(self.netq,threshold = 0.01)
		
		#State Estimation Pipeline
		self.updateAllStates()		#Update Angular and Lateral States of Robot
		for i in range(20):
			self.Block2BasePipeline()	#Update Blocks wrt Base (and Camera and whatever in the process)

		if self.numStaticCubes<4:
			print("Can't Find Last block yo")
			self.searchStaticBlocks(blockNoFault=1)
			for i in range(20):
				t = deepcopy(self.cubeStaticBase)
				self.Block2BasePipeline()
				for j in range(4):
					self.cubeStaticBase[j,:,:] = 0.7*t[j,:,:] + 0.3*self.cubeStaticBase[j,:,:]

		

		# Run a quick Block Sort 
		if self.team == "red":
			sort = self.sortStaticBlocksToPick()
		else:
			sort = [self.sortStaticBlocksToPick()[-(i+1)] for i in range((self.numstaticblocks))]

		for blockno in sort:
			#Get the target Cube
			self.TargetPos = deepcopy(self.cubeStaticBase[blockno,:,:])
			#Static Cube Controller - simple ik
			self.SafeStaticCubeControlPipeline(self.TargetPos, blockno)
			#Jacobian Based Controller - Comment this out!
			# self.StaticCubeControlPipeline(self.TargetPos, blockno)
			self.numblockStacked +=1
			if time_in_seconds()-self.timer>150:
				print("We have already lost if we have to reach here... might as well press the software stop")
				return
	



	def run(self):
		#Move to neutral Position
		self.netq = self.arm.neutral_position()	 
		self.arm.safe_move_to_position(self.netq)
		self.arm.exec_gripper_cmd(0.2, 50)
		print(f"The team is : {self.team}")		
		input("\nWaiting for start... Press ENTER to begin! ...Pizzas are getting cold\n") # get set!
		print("Go!")
		print("----------!Starting!--------")
		
		print("Static Blocks")
		self.numblockStacked = 0
		self.timer = time_in_seconds() #watch dog timer
		self.StaticBlocksFinal()
		for i in range(8):
			self.SafeDynamicPipeline(i)
			if i==3:
				#Would never reach here with this amount of testing
				print("||-----Thank you for the trophy and pizza-----||")


if __name__ == "__main__":
	#Make stuff
	game = Final()
	# #This is our main method being called. 
	game.run()
