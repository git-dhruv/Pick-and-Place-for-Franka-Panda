import numpy as np
from math import pi, acos,sin,cos,atan2
from scipy.linalg import null_space
from copy import deepcopy

import random
from lib.calcJacobian import calcJacobian,calcGenJacobian
from lib.calculateFK import FK
from lib.detectCollision import detectCollision,plotBox
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
from lib.loadmap import loadmap
'''
from calcJacobian import calcJacobian
from calculateFK import FK
from detectCollision import detectCollision
from loadmap import loadmap
'''
# export PYTHONPATH=$PYTHONPATH:`pwd`

class PotentialFieldPlanner:

    # JOINT LIMITS
    lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    center = lower + (upper - lower) / 2 # compute middle of range of motion of each joint
    fk = FK()

    def __init__(self, tol=1e-4, max_steps=500, min_step_size=1e-5):
        """
        Constructs a potential field planner with solver parameters.

        PARAMETERS:
        tol - the maximum distance between two joint sets
        max_steps - number of iterations before the algorithm must terminate
        min_step_size - the minimum step size before concluding that the
        optimizer has converged
        """

        # YOU MAY NEED TO CHANGE THESE PARAMETERS

        # solver parameters
        self.tol = tol
        self.max_steps = max_steps
        self.min_step_size = min_step_size

        #Potential Field Parameters - Spong Convention

        #Attractor Parameters
        self.d = 0.6
        self.zeta = [500,20,20,10,5,20,200]
        # self.zeta = [1]*10#[5,2,2000,1000,500,2000,20000]


        #Repulsive Parameters
        self.eta = 10
        self.pho_0 = .1

        self.alpha = 0.1

        self.data_logger = []



    ######################
    ## Helper Functions ##
    ######################
    # The following functions are provided to you to help you to better structure your code
    # You don't necessarily have to use them. You can also edit them to fit your own situation 

    # @staticmethod
    def attractive_force(self,target, current, i):
        """
        Helper function for computing the attactive force between the current position and
        the target position for one joint. Computes the attractive force vector between the 
        target joint position and the current joint position 

        INPUTS:
        target - 3x1 numpy array representing the desired joint position in the world frame
        current - 3x1 numpy array representing the current joint position in the world frame

        OUTPUTS:
        att_f - 3x1 numpy array representing the force vector that pulls the joint 
        from the current position to the target position 
        """

        ## STUDENT CODE STARTS HERE
        att_f = np.zeros((3, 1)) 
        dist = np.linalg.norm(target-current)

        if dist<=self.d:
            #Parabolic
            att_f = self.zeta[i]*(target - current)
        else:
            att_f = self.d*self.zeta[i]*(target - current)/dist
        ## END STUDENT CODE

        return att_f

    # @staticmethod
    def repulsive_force(self,obstacle, current, pho,unitvec=np.zeros((3,1))):
        """
        Helper function for computing the repulsive force between the current position
        of one joint and one obstacle. Computes the repulsive force vector between the 
        obstacle and the current joint position 

        INPUTS:
        obstacle - 1x6 numpy array representing the an obstacle box in the world frame
        current - 3x1 numpy array representing the current joint position in the world frame
        unitvec - 3x1 numpy array representing the unit vector from the current joint position 
        to the closest point on the obstacle box 

        OUTPUTS:
        rep_f - 3x1 numpy array representing the force vector that pushes the joint 
        from the obstacle
        """

        ## STUDENT CODE STARTS HERE

        rep_f = np.zeros((3, 1))
        if pho>self.pho_0:
            return rep_f

        if pho==0:
            pho = 0.00001

        rep_f = (self.eta*((1/pho) - 1/self.pho_0)*-unitvec/(pho**2))

        ## END STUDENT CODE

        return rep_f

    @staticmethod
    def dist_point2box(p, box):
        """
        Helper function for the computation of repulsive forces. Computes the closest point
        on the box to a given point 
    
        INPUTS:
        p - nx3 numpy array of points [x,y,z]
        box - 1x6 numpy array of minimum and maximum points of box

        OUTPUTS:
        dist - nx1 numpy array of distance between the points and the box
                dist > 0 point outside
                dist = 0 point is on or inside box
        unit - nx3 numpy array where each row is the corresponding unit vector 
        from the point to the closest spot on the box
            norm(unit) = 1 point is outside the box
            norm(unit)= 0 point is on/inside the box

         Method from MultiRRomero
         @ https://stackoverflow.com/questions/5254838/
         calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
        """
        # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU

        # Get box info
        boxMin = np.array([box[0], box[1], box[2]])
        boxMax = np.array([box[3], box[4], box[5]])
        boxCenter = boxMin*0.5 + boxMax*0.5
        p = np.array(p)

        # Get distance info from point to box boundary
        dx = np.amax(np.vstack([boxMin[0] - p[:, 0], p[:, 0] - boxMax[0], np.zeros(p[:, 0].shape)]).T, 1)
        dy = np.amax(np.vstack([boxMin[1] - p[:, 1], p[:, 1] - boxMax[1], np.zeros(p[:, 1].shape)]).T, 1)
        dz = np.amax(np.vstack([boxMin[2] - p[:, 2], p[:, 2] - boxMax[2], np.zeros(p[:, 2].shape)]).T, 1)

        # convert to distance
        distances = np.vstack([dx, dy, dz]).T
        dist = np.linalg.norm(distances, axis=1)

        # Figure out the signs
        signs = np.sign(boxCenter-p)

        # Calculate unit vector and replace with
        unit = distances / dist[:, np.newaxis] * signs
        unit[np.isnan(unit)] = 0
        unit[np.isinf(unit)] = 0
        return dist, unit

    def compute_forces(self,target, obstacle, current):
        """
        Helper function for the computation of forces on every joints. Computes the sum 
        of forces (attactive, repulsive) on each joint. 

        INPUTS:
        target - 3x7 numpy array representing the desired joint/end effector positions 
        in the world frame
        obstacle - nx6 numpy array representing the obstacle box min and max positions
        in the world frame
        current- 3x7 numpy array representing the current joint/end effector positions 
        in the world frame

        OUTPUTS:
        joint_forces - 3x7 numpy array representing the force vectors on each 
        joint/end effector
        """

        ## STUDENT CODE STARTS HERE

        joint_forces = np.zeros((3, 7)) 
        F_att = np.zeros((3,7))
        F_rep = np.zeros((3,7))
        
        #For each joint
        for j in range(7):        
            #Calculate attractive force            
            F_att[:,j] = self.attractive_force(target[:,j],current[:,j],j)

            #For each obstacle
            obstacle = obstacle.reshape((len(obstacle),6))
            for i in range(len(obstacle)):
                #Calculate Repulsive Force for that joint
                pho,unitvec = PotentialFieldPlanner.dist_point2box(current.T,obstacle[i,:])    
                F_rep[:,j] += self.repulsive_force(obstacle[i,:],current[:,j],0.98*pho[j],unitvec[j,:].reshape(3,1)).reshape(3,)


        #Store in the array
        joint_forces = np.add(F_att, F_rep)    
        ## END STUDENT CODE

        return joint_forces
    
    @staticmethod
    def compute_torques(joint_forces, q):
        """
        Helper function for converting joint forces to joint torques. Computes the sum 
        of torques on each joint.

        INPUTS:
        joint_forces - 3x7 numpy array representing the force vectors on each 
        joint/end effector
        q - 1x7 numpy array representing the current joint angles

        OUTPUTS:
        joint_torques - 1x7 numpy array representing the torques on each joint 
        """

        ## STUDENT CODE STARTS HERE

        joint_torques = np.zeros((1, 7))

        #For each joints
        for i in range(7):
            # J = calcGenJacobian(q,i+1)
            J = calcGenJacobian(q,i+1)
            
            t = (J.T@joint_forces[:,i]).reshape(i+1,1)
            #Appending zeros to torque when not calculated
            t = np.vstack((t,np.zeros((7-(i+1),1))))

            joint_torques = np.add(joint_torques,t.T)

        ## END STUDENT CODE

        return joint_torques

    @staticmethod
    def q_distance(target, current):
        """
        Helper function which computes the distance between any two
        vectors.

        This data can be used to decide whether two joint sets can be
        considered equal within a certain tolerance.

        INPUTS:
        target - 1x7 numpy array representing some joint angles
        current - 1x7 numpy array representing some joint angles

        OUTPUTS:
        distance - the distance between the target and the current joint sets 

        """

        ## STUDENT CODE STARTS HERE

        distance = np.array([np.linalg.norm(target[i]-current[i]) for i in range(7)])

        ## END STUDENT CODE

        return distance
    
    def compute_gradient(self,q, target, map_struct):
        """
        Computes the joint gradient step to move the current joint positions to the
        next set of joint positions which leads to a closer configuration to the goal 
        configuration 

        INPUTS:
        q - 1x7 numpy array. the current joint configuration, a "best guess" so far for the final answer
        target - 1x7 numpy array containing the desired joint angles
        map_struct - a map struct containing the obstacle box min and max positions

        OUTPUTS:
        dq - 1x7 numpy array. a desired joint velocity to perform this task
        """

        ## STUDENT CODE STARTS HERE

        dq = np.zeros((1, 7))
        
        o_current,current_pos = FK().forward(q)
        o_final,final_pos = FK().forward(target)

        obstacle = np.array(map_struct.obstacles)

        joint_forces = self.compute_forces(o_final[1:,:].T, obstacle, o_current[1:,:].T)

        joint_torques = PotentialFieldPlanner.compute_torques(joint_forces,q)
        
        dq = self.alpha*(joint_torques)/np.linalg.norm(joint_torques)

        ## END STUDENT CODE

        return dq

    def constrainAngles(self,q):
        for i in range(7):
            q[i] = PotentialFieldPlanner.constrain(q[i],self.upper[i],self.lower[i])
        return q

    @staticmethod
    def constrain(q,high,low):

        if q>high:
            return high    
        elif q<low:
            return low
        
        return q
    
    @staticmethod
    def wrapper(angle):
        return atan2(sin(angle),cos(angle))

    ###############################
    ### Potential Feild Solver  ###
    ###############################

    def plan(self, map_struct, start, goal):
        """
        Uses potential field to move the Panda robot arm from the startng configuration to
        the goal configuration.

        INPUTS:
        map_struct - a map struct containing min and max positions of obstacle boxes 
        start - 1x7 numpy array representing the starting joint angles for a configuration 
        goal - 1x7 numpy array representing the desired joint angles for a configuration

        OUTPUTS:
        q - nx7 numpy array of joint angles [q0, q1, q2, q3, q4, q5, q6]. This should contain
        all the joint angles throughout the path of the planner. The first row of q should be
        the starting joint angles and the last row of q should be the goal joint angles. 
        """

        q_path = np.array([]).reshape(0,7)

        q = deepcopy(start)
        qf = goal
        q_path = np.concatenate((q_path,[q]))
        q[-1] = qf[-1]
        # q_path = np.concatenate((q_path,[qf]))

        final,_ = FK().forward(qf)
        itr = 0 

        # fig = plt.figure()
        # ax = Axes3D(fig)
        # for j in range(6):
        #     ax.plot([0,final[j+1,0]],[0,final[j+1,1]],[0,final[j+1,2]])

        while True:

            ## STUDENT CODE STARTS HERE
            #Debugger
            if(itr%100==0):
                print(np.linalg.norm(q-qf))
            
            #datalogger
            if (itr%10==0):
                self.data_logger.append(np.linalg.norm(q-qf))
                
            
            # The following comments are hints to help you to implement the planner
            # You don't necessarily have to follow these steps to complete your code 
            
            # Compute gradient 
            dq = self.compute_gradient(q,goal,map_struct)
            dq[0][-1] = 0 #Changing last joint angle won't be benificial and add to errors
            
            
            # q = q+dq


            # YOU NEED TO CHECK FOR COLLISIONS WITH OBSTACLES
            # TODO: Figure out how to use the provided function 
            if True: #REMEMBER TO COMMEND Q+DQ AND UNCOMMEND COLLIDE=0
                current,_ = FK().forward(q)
                obstacle = np.array(map_struct.obstacles)
                newq = q+dq    
                new_pos,_ = FK().forward(newq[0])
                collide = 0
                obstacle = obstacle.reshape((len(obstacle),6))
                for i in range(len(obstacle)):
                    line_pt1 = current[1:,:]
                    line_pt2 = new_pos[1:,:]
                    box = np.array(obstacle[i,:])
                    collide = np.any(detectCollision(line_pt1,line_pt2,box))
                if collide:
                    q = [q]
                    forcerandomwalk = True
                else:
                    q = q+dq 
                    forcerandomwalk = False
            
            if False:                
                for i in range(7):
                    pho,unitvec = PotentialFieldPlanner().dist_point2box(current[i+1,:].reshape((1,3)),box)
                    if detectCollision(line_pt1, line_pt2, box)[i]:
                        ax.plot([line_pt1[i,0], line_pt2[i,0]], [line_pt1[i,1], line_pt2[i,1]], [line_pt1[i,2], line_pt2[i,2]], 'r')
                    else:
                        ax.plot([line_pt1[i,0], line_pt2[i,0]], [line_pt1[i,1], line_pt2[i,1]], [line_pt1[i,2], line_pt2[i,2]], 'b')
                    # line_pt1 = line_pt2
                    # i = 0
                    # ax.plot([line_pt1[i,0], unitvec[0,0]], [line_pt1[i,1], unitvec[0,1]], [line_pt1[i,2], unitvec[0,2]],'g')
                for j in range(6):
                    ax.plot([final[j,0],final[j+1,0]],[final[j+1,1],final[j,1]],[final[j,2],final[j+1,2]])
                box1 = [[box[0], box[1], box[2]],
                        [box[0+3], box[1], box[2]],
                        [box[0+3], box[1+3], box[2]],
                        [box[0], box[1+3], box[2]]]
                box2 = [[box[0], box[1], box[2]],
                        [box[0+3], box[1], box[2]],
                        [box[0+3], box[1], box[2+3]],
                        [box[0], box[1], box[2+3]]]
                box3 = [[box[0], box[1], box[2]],
                        [box[0], box[1+3], box[2]],
                        [box[0], box[1+3], box[2+3]],
                        [box[0], box[1], box[2+3]]]
                box4 = [[box[0], box[1], box[2+3]],
                        [box[0+3], box[1], box[2+3]],
                        [box[0+3], box[1+3], box[2+3]],
                        [box[0], box[1+3], box[2+3]]]
                box5 = [[box[0], box[1+3], box[2]],
                        [box[0+3], box[1+3], box[2]],
                        [box[0+3], box[1+3], box[2+3]],
                        [box[0], box[1+3], box[2+3]]]
                box6 = [[box[0+3], box[1], box[2]],
                        [box[0+3], box[1+3], box[2]],
                        [box[0+3], box[1+3], box[2+3]],
                        [box[0+3], box[1], box[2+3]]]
                plotBox(ax, box1)
                plotBox(ax, box2)
                plotBox(ax, box3)
                plotBox(ax, box4)
                plotBox(ax, box5)
                plotBox(ax, box6)
                ax.set_xlim([-1/2,1/2])
                ax.set_ylim([-1/2,1/2])
                ax.set_zlim([0,1])
                plt.pause(.0010)
                ax.clear()



            #Adaptive Gradiant Descent - Much better control
            if np.linalg.norm(q-qf)<1: 
                self.alpha = 0.05  
                randomobs = 0.0005
            else:
                self.alpha = 0.1
                randomobs = 0.05


            # Termination Conditions
            if np.linalg.norm(q-qf)<0.05 or itr>15000: 
                if itr<15000:
                    #Might as well converge fully
                    q_path = np.concatenate((q_path,[qf]))
                print((qf-q),itr)
                break # exit the while loop if conditions are met!
            

            # YOU MAY NEED TO DEAL WITH LOCAL MINIMA HERE
            # TODO: when detect a local minima, implement a random walk
            if itr>10:#0.0001 #0.0008
                bitmask = np.linalg.norm(q_path[-1,:]-q_path[-9,:])<=randomobs
                # print("Random Walk",itr)
                if(bitmask) or forcerandomwalk:
                    print("Random Walk",itr)
                    current,_ = FK().forward(q[0])
                    obstacle = np.array(map_struct.obstacles)
                    walkitr = 0
                    while True:
                        walkitr = walkitr+1
                        danger = 0
                        newdq = random.sample(range(0, 100), 6)
                        newdq.append(0)
                        newq = q + [(-0.5+newdq[i]/100)*5 for i in range(7)]
                        newq[0][-1] = q[0][-1]
                        new_pos,_ = FK().forward(newq[0])
                        obstacle = obstacle.reshape((len(obstacle),6))
                        for i in range(len(obstacle)):
                        # for i in range(len(obstacle[:,0])):
                            line_pt1 = current[1:,:]
                            line_pt2 = new_pos[1:,:]
                            box = np.array(obstacle[i,:])
                            danger = danger or np.any(detectCollision(line_pt1,line_pt2,box))
                        olderr = np.linalg.norm(qf-q[0])
                        newerr = np.linalg.norm(qf-newq[0])
                        if walkitr>20000:
                            q = [qf]
                            break
                        if danger==False and (newerr<olderr or walkitr>1000):
                            q = deepcopy(newq)
                            break

                    #q += [2*(-0.5+random.random()) if i<6 else 0 for i in range(7)] #[(-0.5+random.random()) for i in range(7)]
            

            #Constraining Joint angles
            q = self.constrainAngles(q[0])
            #Wrapping between -pi to pi
            q = [PotentialFieldPlanner.wrapper(i) for i in q]
            #Add to path
            q_path = np.concatenate((q_path,[q]))
            #Increase iteration
            itr = itr+1

            ## END STUDENT CODE
        return q_path#[0::20,:],self.data_logger

################################
## Simple Testing Environment ##
################################

if __name__ == "__main__":

    np.set_printoptions(suppress=True,precision=5)

    planner = PotentialFieldPlanner()
    
    # inputs 
    map_struct = loadmap("../maps/map2.txt")
    start = np.array([0, -1.4, 0, -2, 0, 1.57, 0.707])#np.array([0.0,-1,0,-2,0,1.57,0.7])

    start =  np.array([0.0,-1,0,-2,0,1.57,0.7])
    goal = np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    # goal =  np.array([1.9, 1.57, -1.57, -1.57, 1.57, 1.57, 0.707])

    
    # potential field planning
    q_path,data = planner.plan(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    
    # show results
    for i in range(q_path.shape[0]):
        error = PotentialFieldPlanner.q_distance(q_path[i,:], goal)
        # print('iteration:',i,' q =', q_path[i, :], f' error={error}')

    import matplotlib.pyplot as plt

    plt.plot(data)
    plt.show()
    # print("q path: ", q_path)
    import roboticstoolbox as rtb
    robot = rtb.models.Panda()
    robot2 = rtb.models.Panda()
    # robot.plot(q_path[0::2,:])
    # robot2.plot(goal)

