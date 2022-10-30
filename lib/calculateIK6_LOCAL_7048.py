import numpy as np
from math import *
from copy import deepcopy
class IK:
    """
    Solves the 6 DOF (joint 5 fixed) IK problem for panda robot arm
    """
    # offsets along x direction 
    a1 = 0 
    a2 = 0
    a3 = 0.0825
    a4 = 0.0825
    a5 = 0 
    a6 = 0.088
    a7 = 0

    # offsets along z direction 
    d1 = 0.333
    d2 = 0 
    d3 = 0.316 
    d4 = 0
    d5 = 0.384
    d6 = 0 
    d7 = 0.210
    
    # This variable is used to express an arbitrary joint angle 
    Q0 = 0.123


    def panda_ik(self, target):
        """
        Solves 6 DOF IK problem given physical target in x, y, z space
        Args:
            target: dictionary containing:
                'R': numpy array of the end effector pose relative to the robot base 
                't': numpy array of the end effector position relative to the robot base 

        Returns:
             q = nx7 numpy array of joints in radians (q5: joint 5 angle should be 0)
        """
        Q = np.zeros((8, 7))
        
        soltrack = 0
        # Student's code goes in between: 

        # joint_positions, End_effector = fk.forward(q) np.row_stack((Inverted_Toe2,np.array([0,0,0,1])))
        End_effector = np.row_stack((np.column_stack((target['R'],target['t'])),np.array([0,0,0,1])))

        d1 = 0.333
        a3 = 0.0825
        d5 = 0.384
        d3 = 0.316
        d7 = 0.21
        Q0 = 0.123
        a6 = 0.088

        #Invert the end effector
        Inverted_Toe = np.linalg.inv(End_effector)
        #Calculate theta_7
        fk = FK()
        
        Oc_wrist = Inverted_Toe[0:3,-1] + Inverted_Toe[0:3,2]*(d1*np.ones(3))


        theta_7_1 = pi - atan2(Oc_wrist[1],Oc_wrist[0]) + pi/4
        # theta_7_1 = 3*pi/4 - np.arctan2(-(Oc_wrist[1]),(Oc_wrist[0])) 

        theta_7_1 = -(np.arctan2(Oc_wrist[1],Oc_wrist[0]) - pi - pi/4)
        if theta_7_1 > 2*pi:
            theta_7_1  = theta_7_1 - 2*pi
        elif theta_7_1 > pi and theta_7_1<=2*pi:
            theta_7_1 = theta_7_1 - 2*pi

        if theta_7_1 > 0:
            theta_7_2 = theta_7_1 -  pi
        else:
            theta_7_2 = theta_7_1 + pi
        
        if Oc_wrist[0] == 0 and Oc_wrist[1] ==0:
            theta_7_1 = Q0
            theta_7_2 = Q0
        # theta_7_1 = -theta_7_1
        #Solution2
        # theta_7_2 = theta_7_1 + pi
        r = np.linalg.norm([Oc_wrist[0],Oc_wrist[1]])
        sol_no = 0



        for i in range(2):
            if i==0:

                T7 = fk.single_frame_transform(6,theta_7_1-pi/4)

                #new wrist center
                Oc = np.array([[Oc_wrist[0],Oc_wrist[1],Oc_wrist[2],1]]).T
                Oc = T7@Oc 

                # r = np.linalg.norm([Oc[0],Oc[1]])
                Oc = np.round([float(Oc[0])+a6,float(Oc[2])],5)

                theta_7 = theta_7_1
            else:
                T7 = fk.single_frame_transform(6,theta_7_2-pi/4)

                #new wrist center
                Oc = np.array([[Oc_wrist[0],Oc_wrist[1],Oc_wrist[2],1]]).T
                print("Hutt")
                print(Oc)
                Oc = T7@Oc
                print(Oc) 

                # r = np.linalg.norm([Oc[0],Oc[1]])
                Oc = np.round([float(Oc[0])+a6,float(Oc[2])])
                theta_7 = theta_7_2

            Oy = float(Oc[1]) #Our z6 becomes z6
            Ox = float(Oc[0]) #our x6 becomes z6
            
            l1 = sqrt(d5**2 + a3**2)
            l2 = sqrt(d3**2 + a3**2)
            D = ((Ox**2 + Oy**2) - (l1**2 + l2**2) )/(2*l1*l2) 
            print(D)
            if abs(D)>1:
                pass
            else:
                


                t2 = np.arccos(D)
                t1 = np.arctan2(Oy,Ox) - np.arctan2( (l2*sin(t2)),(l1+(l2*cos(t2))) )
                
                theta4_1 = (np.arctan2(d5,a3) + np.arctan2(d3,a3) + t2) - pi
                theta_6_1 =  (t1 + np.arctan2(a3,d5)  - pi/2) 

                t2 =  2*pi - np.arccos(D)
                t1 = np.arctan2(Oy,Ox) - np.arctan2( (l2*sin(t2)),(l1+(l2*cos(t2))) )
                theta_6_2 = (t1 - pi/2 + np.arctan2(a3,d5))
                theta4_2 = (np.arctan2(d5,a3) + np.arctan2(d3,a3) + t2) - pi

                # theta_6_1 = theta_6_1 + pi/2
                # theta_6_2 = theta_6_2 + pi/2

                if theta_6_1<-pi/2:
                    theta_6_1 += 2*pi
                if theta_6_2<-pi/2:
                    theta_6_2 += 2*pi
                


                for j in range(2):
                    if j==0:
                        theta_6 = theta_6_1
                        theta4 = theta4_1
                    else:
                        theta_6 = theta_6_2
                        theta4 = theta4_2
                    # robot1.plot(q)
                    print(theta4*180/pi)
                    q = np.array([0,0,0,theta4,0,theta_6,theta_7])

                    _,R = fk.general_transformer(2,6,q)
                    R_orient = R[0:3,0:3]@np.linalg.inv(End_effector[0:3,0:3])
                    R_orient = R_orient.T
                    q2 = np.arccos(R_orient[2,2])

                    arbi = 0
                    if abs(R_orient[2,2])==1:
                        q3 = Q0
                        q1 = Q0
                        arbi = 1
                    else:
                        q3 = -np.arctan2(R_orient[2,0],R_orient[2,1])
                        q1 = np.arctan2(-R_orient[0,2],R_orient[1,2])
                    q2,q3,q1 = 0,0,0
                    if i==0:
                        if j==0:                
                            Q[sol_no,:] = np.array([q1,q2,q3,-theta4_1,0,theta_6_1,theta_7_1])
                            if arbi==0:
                                Q[sol_no+1,:] = np.array([q1-pi,-q2,q3-pi,-theta4_1,0,theta_6_1,theta_7_1])
                                sol_no = sol_no+2
                            else:
                                sol_no = sol_no+1
                        else:
                            Q[sol_no,:] = np.array([q1,q2,q3,-theta4_2,0,theta_6_2,theta_7_1])
                            if arbi==0:
                                Q[sol_no+1,:] = np.array([q1-pi,-q2,q3-pi,-theta4_2,0,theta_6_2,theta_7_1])
                                sol_no = sol_no+2
                            else:
                                sol_no = sol_no+1

                    else:
                        if j==0:
                            
                            Q[sol_no,:] = np.array([q1,q2,q3,-theta4_1,0,theta_6_1,theta_7_2])
                            if arbi==0:
                                Q[sol_no+1,:] = np.array([q1-pi,-q2,q3-pi,-theta4_1,0,theta_6_1,theta_7_2])
                                sol_no = sol_no+2
                            else:
                                sol_no = sol_no+1
                        else:
                            Q[sol_no,:] = np.array([q1,q2,q3,-theta4_2,0,theta_6_2,theta_7_2])
                            if arbi==0:
                                Q[sol_no+1,:] = np.array([q1-pi,-q2,q3-pi,-theta4_2,0,theta_6_2,theta_7_2])
                                sol_no = sol_no+2    
                            else:
                                sol_no = sol_no+1



        


        # q = q[0:soltrack,:]
        q_scheme = np.zeros((8,7))        
        return Q[0:sol_no,:]
        if sol_no==0:
            return np.array([[]])


        else:
            joints = 0
            for i in range(sol_no):
                if self.joint_lim(Q[i,:]):
                    joints = joints+1
                    q_t = np.array([atan2(sin(i),cos(i)) for i in Q[i,:]])
                    q_scheme[joints-1,:] = q_t
        

            q = q_scheme[0:joints,:]

        if joints==0:
            return np.array([[]])
        # Student's code goes in between:

        ## DO NOT EDIT THIS PART 
        # This will convert your joints output to the autograder format
        q = self.sort_joints(q)
        ## DO NOT EDIT THIS PART
        return q

    def joint_lim(self,q):
        # print([i*180/pi for i in q])
        q = [atan2(sin(i),cos(i)) for i in q]
        q1_bool = abs(q[0])<=166*pi/180
        q2_bool = abs(q[1])<= 101*pi/180
        q3_bool = abs(q[2]) <= 166*pi/180
        
        q4_bool = (q[3] <=-4*pi/180 and q[3]>=-176*pi/180) or (q[3]>=-pi and q[3]<=-145*pi/180)
        q6_bool = (q[5])>=-1*pi/180 and q[5]<= pi
        q7_bool = abs(q[6])<=166*pi/180
        print(q1_bool , q2_bool , q3_bool , q4_bool , q6_bool , q7_bool)

        return (q1_bool and q2_bool and q3_bool and q4_bool and q6_bool and q7_bool)
    
    def wrap(self,angle):
        return np.arctan2(sin(angle),cos(angle))
    def kin_decouple(self, target):
        """
        Performs kinematic decoupling on the panda arm to find the position of wrist center
        Args: 
            target: dictionary containing:
                'R': numpy array of the end effector pose relative to the robot base 
                't': numpy array of the end effector position relative to the robot base 

        Returns:
             wrist_pos = 3x1 numpy array of the position of the wrist center in frame 7
        """
        wrist_pos = []
        return wrist_pos 

    def ik_pos(self, wrist_pos):
        """
        Solves IK position problem on the joint 4, 6, 7 
        Args: 
            wrist_pos: 3x1 numpy array of the position of the wrist center in frame 7

        Returns:
             joints_467 = nx3 numpy array of all joint angles of joint 4, 6, 7
        """
        joints_467 = []
        return joints_467

    def ik_orient(self, R, joints_467):
        """
        Solves IK orientation problem on the joint 1, 2, 3
        Args: 
            R: numpy array of the end effector pose relative to the robot base 
            joints_467: nx3 numpy array of all joint angles of joint 4, 6, 7

        Returns:
            joints_123 = nx3 numpy array of all joint angles of joint 1, 2 ,3
        """
        joints_123 = [] 
        return joint_123
    
    def sort_joints(self, q, col=0):
        """
        Sort the joint angle matrix by ascending order 
        Args: 
            q: nx7 joint angle matrix 
        Returns: 
            q_as = nx7 joint angle matrix in ascending order 
        """
        if col==7:
            return q[q[:, col-1].argsort()]
        q_as = q[q[:, col].argsort()]
        for i in range(q_as.shape[0]-1):
            if (q_as[i, col] < q_as[i+1, col]):
                # do nothing
                pass
            else:
                for j in range(i+1, q_as.shape[0]):
                    if q_as[i, col] < q_as[j, col]:
                        idx = j
                        break
                    elif j == q_as.shape[0]-1:
                        idx = q_as.shape[0]

                q_as_part = self.sort_joints(q_as[i:idx, :], col+1)
                q_as[i:idx, :] = q_as_part
        return q_as









class FK():

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout
        """
        Defining DH Parameters
        """
        self.alpha = [-pi/2,pi/2,pi/2,-pi/2,pi/2,pi/2,0]
        self.a = [0,0,0.0825,-0.0825,0,0.088,0] 
        self.d = [0.333,0.0,0.316,0,0.125+0.259,0.,0.21]

        #For joint coordinates only! Should not affect the final transformation
        # self.coinciding_frame_offset = [0,0,0.195,0,0.125,-0.015,0.051,0.]
        self.coinciding_frame_offset = np.array([[0,0,0,0,0,0,0,0],
                                                [0,0,0,0,0,0,0,0.015],
                                                [0,0,0.195,0,0.125,-0.015,0.051,0.]])




    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your Lab 1 code starts here

        jointPositions = np.zeros((8,3))
        T0e = np.identity(4)

        t = q


        
        for i in range(len(t)):
            H = self.single_frame_transform(i,q[i])

            #We need to pre multiply the last frame by -pi/4 because of an offset in the end effector angle 
            if i==6:
                T0e = np.dot(T0e,self.rotz(-pi/4))

            #Post Multiply current transformation
            T0e = np.dot(T0e,H)

            #Joint angles
            """
            Basically - since we have coinciding frames, we have to translate the coincided frame to its
            actual location for the original joint coordinates. 

            Mathematically
            Joint_Location = T0e.Tz(offset).Ty(offset).Tx(offset)
            """
            joint_coordinates = np.dot(T0e,self.generate_joint_offset_matrix(i+1))

            jointPositions[i+1,0] = float(joint_coordinates[0][3])
            jointPositions[i+1,1] = float(joint_coordinates[1][3])
            jointPositions[i+1,2] = float(joint_coordinates[2][3])

        #This is for first Joint - this is the reason you see i+1 everywhere            
        jointPositions[0,0] = 0
        jointPositions[0,1] = 0
        jointPositions[0,2] = 0.141

        # Your code ends here

        return jointPositions, T0e


    def backward_for_2nd(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your Lab 1 code starts here

        jointPositions = np.zeros((8,3))
        T0e = np.identity(4)

        t = q


        
        for i in range(len(t)):
            H = self.single_frame_transform(i,q[i],0)

            #We need to pre multiply the last frame by -pi/4 because of an offset in the end effector angle 
            if i==6:
                T0e = np.dot(T0e,self.rotz(-pi/4))
                inv2 = deepcopy(np.dot(self.rotz(-pi/4),H))

        
            #Post Multiply current transformation
            T0e = np.dot(T0e,H)

            if i==5:
                inv_1 = np.linalg.inv(self.single_frame_transform(i,0,1))
            
            #Joint angles
            """
            Basically - since we have coinciding frames, we have to translate the coincided frame to its
            actual location for the original joint coordinates. 

            Mathematically
            Joint_Location = T0e.Tz(offset).Ty(offset).Tx(offset)
            """
            joint_coordinates = np.dot(T0e,self.generate_joint_offset_matrix(i+1))

            jointPositions[i+1,0] = float(joint_coordinates[0][3])
            jointPositions[i+1,1] = float(joint_coordinates[1][3])
            jointPositions[i+1,2] = float(joint_coordinates[2][3])

        #This is for first Joint - this is the reason you see i+1 everywhere            
        jointPositions[0,0] = 0
        jointPositions[0,1] = 0
        jointPositions[0,2] = 0.141

        inverted_toe = np.linalg.inv(np.dot(inv_1,inv2))


        # Your code ends here

        return jointPositions, inv_1,inv2

    # feel free to define additional helper methods to modularize your solution for lab 1


    def general_transformer(self,begin,end,q):
        # Your Lab 1 code starts here

        jointPositions = np.zeros((8,3))
        T0e = np.identity(4)

        t = q


        
        for i in range(begin,end+1):
            H = self.single_frame_transform(i,q[i])

            #We need to pre multiply the last frame by -pi/4 because of an offset in the end effector angle 
            if i==6:
                T0e = np.dot(T0e,self.rotz(-pi/4))
        
            #Post Multiply current transformation
            T0e = np.dot(T0e,H)

            if i==5:
                inv_1 = deepcopy(H)
            
            #Joint angles
            """
            Basically - since we have coinciding frames, we have to translate the coincided frame to its
            actual location for the original joint coordinates. 

            Mathematically
            Joint_Location = T0e.Tz(offset).Ty(offset).Tx(offset)
            """
            joint_coordinates = np.dot(T0e,self.generate_joint_offset_matrix(i+1))

            jointPositions[i+1,0] = float(joint_coordinates[0][3])
            jointPositions[i+1,1] = float(joint_coordinates[1][3])
            jointPositions[i+1,2] = float(joint_coordinates[2][3])

        #This is for first Joint - this is the reason you see i+1 everywhere            
        jointPositions[0,0] = 0
        jointPositions[0,1] = 0
        jointPositions[0,2] = 0.141




        # Your code ends here

        return jointPositions, T0e
    def translz(self,x):
        """
        Homogenous translation matrix
        """
        return np.array([[1,0,0,0],[0,1,0,0],[0,0,1,x],[0,0,0,1]])

    def transly(self,x):
        return np.array([[1,0,0,0],[0,x,0,0],[0,0,1,0],[0,0,0,1]])

    def translx(self,x):
        return np.array([[1,0,0,x],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    def rotz(self,a):
        return np.array([[cos(a), -sin(a), 0, 0],[sin(a), cos(a), 0, 0],[0,0,1,0],[0,0,0,1]])
    def rotx(self,a):
        return np.array([[1,0,0,0],[0,cos(a),-sin(a),0],[0,sin(a),cos(a),0],[0,0,0,1]])

    def single_frame_transform(self, current_index, joint_angle,no_off=0):
        if no_off:
            from copy import deepcopy
            temp = deepcopy(self.alpha[current_index])
            self.alpha[current_index] = 0
        H = np.array([[cos(joint_angle), -sin(joint_angle)*cos(self.alpha[current_index]), sin(self.alpha[current_index])*sin(joint_angle), self.a[current_index]*cos(joint_angle)],
            [sin(joint_angle), cos(joint_angle)*cos(self.alpha[current_index]), -sin(self.alpha[current_index])*cos(joint_angle), self.a[current_index]*sin(joint_angle)],
            [0, sin(self.alpha[current_index]), cos(self.alpha[current_index]), self.d[current_index]],
            [0,0,0,1]])
        if no_off:
            self.alpha[current_index] = temp
        return H

    def generate_joint_offset_matrix(self,current_index):
        return np.dot(np.dot(self.translz(self.coinciding_frame_offset[2][current_index]),self.transly(self.coinciding_frame_offset[1][current_index])),self.translx(self.coinciding_frame_offset[0][current_index]))
         

    
    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
def main(): 
    
    # fk solution code
    fk = FK()

    # input joints  
    q1 = 0
    q2 = 0
    q3 = 0
    q4 = -pi/4
    q6 = 0
    q7 = np.pi/4
    
    q_in  = np.array([q1, q2, q3, q4, 0, q6, q7])
    # q_in = np.array([-pi,-pi/2,pi/2,40*pi/180,0,-3*pi/2,pi/4])

    [_, T_fk] = fk.forward(q_in)

    # input of IK class
    target = {'R': T_fk[0:3, 0:3], 't': T_fk[0:3, 3]}
    print(target)

    ik = IK()
    q = ik.panda_ik(target)
    
    # verify IK solutions 
    for i in range(q.shape[0]):
        [_, T_ik] = fk.forward(q[i, :])
        print(q[i,:]*180/pi)
        print('Matrix difference = ')
        print(T_fk - T_ik)
        print()

if __name__ == '__main__':
    main()