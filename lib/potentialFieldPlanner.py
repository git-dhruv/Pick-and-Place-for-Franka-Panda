import numpy as np
from math import pi, acos,sin,cos,atan2
from scipy.linalg import null_space
from copy import deepcopy

import random
try:
    from lib.calcJacobian import calcJacobian,calcGenJacobian
    from lib.calculateFK import FK
    from lib.detectCollision import detectCollision,plotBox,detectCollisionOnce
    from lib.loadmap import loadmap
except:
    from calcJacobian import calcJacobian,calcGenJacobian
    from calculateFK import FK
    from detectCollision import detectCollision,plotBox,detectCollisionOnce
    from loadmap import loadmap
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt

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
        self.d = 0.65
        self.zeta = [.1,.1,.1,.1,.5,.5,.2] #[500,20,20,10,5,20,200]

        #Repulsive Parameters
        self.eta = 10
        self.pho_0 = .1

        #Take the first step very slowly
        self.alpha = 0.0005

        self.delta = 0.12
        self.data_logger = []

        self.obstacle = None
        self.MAXITER = 15000


        self.DhruvPlot = 0



    ######################
    ## Helper Functions ##
    ######################
    # The following functions are provided to you to help you to better structure your code
    # You don't necessarily have to use them. You can also edit them to fit your own situation 

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
            pho = 1e-10

        rep_f = (self.eta*((1/pho) - 1/self.pho_0)*-unitvec/(pho**2))

        ## END STUDENT CODE

        return rep_f

    @staticmethod
    def dist_point2box(p, box, itr=0):
        """
        Helper function for the computation of repulsive forces. Computes the closest point
        on the box to a given point 
    
        INPUTS:
        return distance
    
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
        return distance
    

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
            obstacle = self.obstacle
            for i in range(len(obstacle)):
                #Calculate Repulsive Force for that joint
                pho,unitvec = PotentialFieldPlanner.dist_point2box(current.T,obstacle[i,:])    
                F_rep[:,j] += self.repulsive_force(obstacle[i,:],current[:,j],pho[j],unitvec[j,:].reshape(3,1)).reshape(3,)

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
    
    def inflateObs(self,obstacles,lowerdelta=0):
        obstacles = obstacles.reshape((len(obstacles),6))
        for i in range(len(obstacles)):
            if lowerdelta:
                obstacles[i,:] = np.add(obstacles[i,:],[-self.delta*.2,-self.delta*.2,-self.delta*.2,self.delta*.2,self.delta*.2,self.delta*.2])
            else:
                obstacles[i,:] = np.add(obstacles[i,:],[-self.delta,-self.delta,-self.delta,self.delta,self.delta,self.delta])
        return obstacles

    def checkCollision(self,current):
        collide = 0
        for i in range(len(self.obstacle)):
            pho,_ = PotentialFieldPlanner().dist_point2box(current.T,self.obstacle[i,:],self.itr)
            collide = collide or np.any(pho<=0.01)
        return collide
                


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

        #Get the start in q
        q = deepcopy(start)
        #Get goal in qf
        qf = goal
        #Start path from start
        q_path = np.concatenate((q_path,[q]))
        #Set the last angle as final angle - no impact on position
        q[-1] = qf[-1]
        #Counters for randomwalk and iterations
        randomwalk = 0
        itr = 0

        #My data logging capabilities are demonstrated here (jk)
        final,_ = FK().forward(qf)
        #Plotter Like a pyplot simulation
        if self.DhruvPlot:
            fig = plt.figure()
            ax = Axes3D(fig)
            for j in range(6):
                ax.plot([0,final[j+1,0]],[0,final[j+1,1]],[0,final[j+1,2]])

        self.obstacle = self.inflateObs(np.array(map_struct.obstacles))
        obstacle = deepcopy(self.obstacle)
        while True:
            self.itr = itr
            ## STUDENT CODE STARTS HERE

            #!==Debugging and Datalogging==!#
            #Debugger
            if(itr%1000==0):
                print(np.linalg.norm(q-qf))
            #Datalogger
            if (itr%1==0):
                self.data_logger.append(np.linalg.norm(q-qf))
                            
            #!==Compute gradient==!# 
            
            dq = self.compute_gradient(q,goal,map_struct)
            dq[0][-1] = 0 #Changing last joint angle won't be benificial and add to errors
            
            #Get current joint position and future joint position            
            current,_ = FK().forward(q)
            newq = q + dq    
            new_pos,_ = FK().forward(newq[0])

            #!==Collision Avoidance==!#
            #if current position is colliding then we are done
            # if self.checkCollision(current.T):
            #     return q_path
            
            #Check1 - Collision of new joint position and obstacle
            collide =  self.checkCollision(new_pos.T)

            #Check 2 - Collision when moving from joint position 1 to joint position 2
            for i in range(len(obstacle)):
                line_pt1 = current
                line_pt2 = new_pos
                box = np.array(obstacle[i,:])
                collide = collide or np.any(detectCollision(line_pt1,line_pt2,box))
                #Check 3 - Collision of robot linkages with the obstacle
                collide = collide or np.any([detectCollisionOnce(new_pos[j,:],new_pos[j+1,:],box) for j in range(7)])
            
            #If collide then force a random walk, and don't change q
            if collide:
                print("Collided:", itr)
                q = [q]
                forcerandomwalk = True
            
            #Else change the q
            else:
                q = q+dq 
                forcerandomwalk = False

            #IGNORE THIS
            if self.DhruvPlot:                
                for i in range(7):
                    # pho,unitvec = PotentialFieldPlanner().dist_point2box(current[i+1,:].reshape((1,3)),box)
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
                if False and forcerandomwalk:
                    plt.pause(1000)
                plt.pause(.0010)
                ax.clear()


            #Adaptive Gradiant Descent - Much better control
            if np.linalg.norm(q-qf)<.8: 
                self.alpha = 0.05
                randomobs = 0.001    
            else:
                self.alpha = 0.1
                randomobs = 0.075
                

            # Termination Conditions
            if round(np.linalg.norm(q-qf),3)<0.1 or itr>self.MAXITER: 
                # if itr<self.MAXITER: #-We area converging only when norm condition is satisfied not the maxiter. So we don't converge when there is no solution.
                #     q_path = np.concatenate((q_path,[qf]))
                break 
            

            #!==Random Walk==!#
            if itr>10:
                #Bitmask to check if stuck for local minima
                bitmask = np.linalg.norm(q_path[-1,:]-q_path[-9,:])<=randomobs
                #If in local minima or obstacle collision
                if bitmask or forcerandomwalk:
                    randomwalk = randomwalk+1
                    print("Random Walk",itr)
                    
                    #Get current joint positions
                    current,_ = FK().forward(q[0])
                    #Random Walk iterator
                    walkitr = 0
                    while True:
                        walkitr = walkitr+1
                        #Sample new dq
                        newdq = random.sample(range(0, 100), 6)
                        #last joint angle needs no change
                        newdq.append(0)

                        #If we have forced the random walk that means that we are in collision - move slowly
                        if forcerandomwalk:
                            dqrandom = [(-0.5+newdq[i]/100)*2 for i in range(7)]
                        else:
                            dqrandom = [(-0.5+newdq[i]/100)*5 for i in range(7)]

                        #New Q
                        newq = q + dqrandom
                        #Again just confirming that last angle needs no change
                        newq[0][-1] = q[0][-1]

                        #New position
                        new_pos,_ = FK().forward(newq[0])

                        #Check 1 - Collision of new pos with obstacle box
                        danger = self.checkCollision(new_pos.T)
                        #Check 2 - Collision of line from old pos to new pos with obstacle
                        for i in range(len(obstacle)):
                            line_pt1 = current
                            line_pt2 = new_pos
                            #We will inflate again because why not
                            box = self.inflateObs(np.array(obstacle[i,:]).reshape(1,6),1)[0]
                            danger = danger or np.any(detectCollision(line_pt1,line_pt2,box))
                            #Check 3 - Collision of linkages
                            danger = danger or np.any([detectCollisionOnce(new_pos[j,:],new_pos[j+1,:],box) for j in range(7)])

                        #Old error and new error
                        olderr = np.linalg.norm(qf-q[0])
                        newerr = np.linalg.norm(qf-newq[0])

                        #Break out of random walk if its taking too long. Normally this is collision and nothing can be done. 
                        #If we are forcing random walk this means the walkitr can be reduced at this point we have no hope of recovery
                        bool1 = (forcerandomwalk and walkitr>1000)
                        if walkitr>1e3 or bool1:
                            #increase itr to break out and loose on this test case
                            print("!!Cant find collision free path!!")
                            return q_path#,self.data_logger#[0:itr-2,:] #Remove some positions that are very close to the obstacle
                            q = [q_path[-1]]
                            itr = self.MAXITER
                            break
                        bool2 = (newerr<=olderr or walkitr>5e3)
                        if danger==False and bool2:
                            q =deepcopy(newq)
                            break
            
            
            #Wrapping between -pi to pi
            q = [PotentialFieldPlanner.wrapper(i) for i in q[0]]

            #Constraining Joint angles
            q = self.constrainAngles(q)

            #Add to path
            q_path = np.concatenate((q_path,[q]))

            #Increase iteration
            itr = itr+1

            ## END STUDENT CODE
        print(randomwalk, itr)
        return q_path#,self.data_logger

################################
## Simple Testing Environment ##
################################

if __name__ == "__main__":

    np.set_printoptions(suppress=True,precision=5)

    planner = PotentialFieldPlanner()
    
    # inputs 
    map_struct = loadmap("../maps/map1.txt")

    starts = [np.array([0, -1, 0, -2, 0, 1.57, 0]),
            np.array([0, 0.4, 0, -2.5, 0, 2.7, 0.707]),
            np.array([-2, 1.57, 1.57, -2.07, -1.57, 1.57, 0.7]),
            np.array([-2, 1.57, 1.57, -2.07, -1.57, 1.57, 0.7])]
    goals = [np.array([1.2, 1.57, 1.57, -2.07, -1.57, 1.57, 0.7]),
            np.array([2.4, 1.57, -1.57, -1.57, 1.57, 1.57, 0.707]),
            np.array([2.4, 1.57, -1.57, -1.57, 1.57, 1.57, 0.707]),
            np.array([2.8, 1.57, 0, -1.57, 1.57, 1.57, 0.707])]   

    # lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    # upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    start = starts[0]
    goal = goals[0] 
    # potential field planning
    q_path,data = planner.plan(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    
    # show results
    for i in range(q_path.shape[0]):
<<<<<<< HEAD
        error = PotentialFieldPlanner.q_distance(q_path[i,:], goal)
        # print('iteration:',i,' q =', q_path[i, :], f' error={error}')

    import matplotlib.pyplot as plt
=======
        error = PotentialFieldPlanner.q_distance(q_path[i, :], goal)
        print('iteration:',i,' q =', q_path[i, :], ' error={error}'.format(error=error))
>>>>>>> 6dba5d06276aa2ce62f3ad5ea1546c32fe357f70

    plt.plot(data)
    plt.xlabel("Iterations")
    plt.ylabel("Error (radians)")
    plt.show()