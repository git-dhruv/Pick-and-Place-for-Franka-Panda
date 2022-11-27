import numpy as np
from math import pi,sin,cos,atan2 #Bread and butter

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
        self.coinciding_frame_offset = np.array([[0,0,0,0,0,0,0,0],
                                                [0,0,0,0,0,0,0,0.015],
                                                [0.141,0,0.195,0,0.125,-0.015,0.051,0.]])

        self.approach = np.zeros((3,8))
        
        self.approach[:,0] = np.array([[0],[0],[1]]).reshape(3,)




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
       
        for i in range(len(q)):
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

            #Don't mind me ruining old code
            self.approach[:,i+1] = T0e[0:3,2].reshape(3,)

        #This is for first Joint - this is the reason you see i+1 everywhere            
        jointPositions[0,0] = 0
        jointPositions[0,1] = 0
        jointPositions[0,2] = 0.141

        # Your code ends here

        return jointPositions, T0e

    # feel free to define additional helper methods to modularize your solution for lab 1
    def get_approach_for_all(self):
        return self.approach

    def translz(self,x):
        """
        Homogenous translation matrix for Z
        """
        return np.array([[1,0,0,0],[0,1,0,0],[0,0,1,x],[0,0,0,1]])

    def transly(self,x):
        """
        Homogenous translation matrix for Y
        """
        return np.array([[1,0,0,0],[0,x,0,0],[0,0,1,0],[0,0,0,1]])

    def translx(self,x):
        """
        Homogenous translation matrix for X
        """
        return np.array([[x,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    def rotz(self,a):
        """
        Homogenous rotation matrix around Z
        """
        return np.array([[cos(a), -sin(a), 0, 0],[sin(a), cos(a), 0, 0],[0,0,1,0],[0,0,0,1]])


    def single_frame_transform(self, current_index, joint_angle):
        """
        Matrix from DH parameters for current index and joint angle
        When intialised the current index parameters are defined there
        """
        H = np.array([[cos(joint_angle), -sin(joint_angle)*cos(self.alpha[current_index]), sin(self.alpha[current_index])*sin(joint_angle), self.a[current_index]*cos(joint_angle)],
            [sin(joint_angle), cos(joint_angle)*cos(self.alpha[current_index]), -sin(self.alpha[current_index])*cos(joint_angle), self.a[current_index]*sin(joint_angle)],
            [0, sin(self.alpha[current_index]), cos(self.alpha[current_index]), self.d[current_index]],
            [0,0,0,1]])
        return H

    def generate_joint_offset_matrix(self,current_index):
        """
        Multiplies all the translation matrices for given coinciding frame offsets
        """
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
    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    q = np.array([0,0,0,pi/2,0,0,pi/4])

    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
