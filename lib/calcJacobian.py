import numpy as np
try:
    from lib.calculateFK import FK
except:
    from calculateFK import FK

def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    J = np.zeros((6, 7))

    ## STUDENT CODE GOES HERE
    fk = FK()
    jointPositions,T0e =    fk.forward(q_in)

    translation_vector = T0e[0:3,-1]

    approaches = fk.get_approach_for_all()

    #Build the Jacobian
    for i in range(len(q_in)):
        J[0:3,i] =get_skew_symmetric_matrix(approaches[:,i])@(np.add(translation_vector,-jointPositions[i,:]))
        J[3:6,i] = approaches[:,i]

    return J

def calcGenJacobian(q_in,joint_no=7):
    '''
    Only calculate velocity Jacob
    joint_no: Assume that joint_no for base frame is 0, so it belongs to (1,7)
    q_in: You have to give all q_in because I won't change FK for this
    '''

    J = np.zeros((3,joint_no))
    fk = FK()
    jointPositions,T0e = fk.forward(q_in)

    #This is our T0e
    translation_vector = jointPositions[joint_no-1,:]

    # translation_vector = T0e[0:3,-1]

    #These are approaches
    approaches = fk.get_approach_for_all()


    for i in range(joint_no-1):
        J[0:3,i] =get_skew_symmetric_matrix(approaches[:,i])@(np.add(translation_vector,-jointPositions[i,:]))

    return J


def get_skew_symmetric_matrix(vector):
    a = vector[0]
    b = vector[1]
    c = vector[2]

    return np.array([[0,-c,b],
                    [c,0,-a],
                    [-b,a,0]])






import numpy as np
import math
from math import *
from numpy import *

def calcJacobian2(q_in):

	J = np.zeros((6, 7))
	
	t1= q_in[0]
	t2= q_in[1]
	t3= q_in[2]
	t4= q_in[3]
	t5= q_in[4]
	t6= q_in[5]
	t7= q_in[6]

	J11 = 0.21000001*(((-sin(t1)*cos(t2)*cos(t3) - sin(t3)*cos(t1))*cos(t4) - sin(t1)*sin(t2)*sin(t4))*cos(t5) - (-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*sin(t5))*sin(t6) + 0.088*(((-sin(t1)*cos(t2)*cos(t3) - sin(t3)*cos(t1))*cos(t4) - sin(t1)*sin(t2)*sin(t4))*cos(t5) - (-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*sin(t5))*cos(t6) + 0.088*(-(-sin(t1)*cos(t2)*cos(t3) - sin(t3)*cos(t1))*sin(t4) - sin(t1)*sin(t2)*cos(t4))*sin(t6) - 0.21000001*(-(-sin(t1)*cos(t2)*cos(t3) - sin(t3)*cos(t1))*sin(t4) - sin(t1)*sin(t2)*cos(t4))*cos(t6) - 0.384000001*(-sin(t1)*cos(t2)*cos(t3) - sin(t3)*cos(t1))*sin(t4) - 0.083*(-sin(t1)*cos(t2)*cos(t3) - sin(t3)*cos(t1))*cos(t4) + 0.083*sin(t1)*sin(t2)*sin(t4) - 0.384000001*sin(t1)*sin(t2)*cos(t4) - 0.316*sin(t1)*sin(t2) - 0.082000001*sin(t1)*cos(t2)*cos(t3) - 0.082000001*sin(t3)*cos(t1)
	J12 = 0.21000001*((-sin(t2)*cos(t1)*cos(t3)*cos(t4) + sin(t4)*cos(t1)*cos(t2))*cos(t5) + sin(t2)*sin(t3)*sin(t5)*cos(t1))*sin(t6) + 0.088*((-sin(t2)*cos(t1)*cos(t3)*cos(t4) + sin(t4)*cos(t1)*cos(t2))*cos(t5) + sin(t2)*sin(t3)*sin(t5)*cos(t1))*cos(t6) + 0.088*(sin(t2)*sin(t4)*cos(t1)*cos(t3) + cos(t1)*cos(t2)*cos(t4))*sin(t6) - 0.21000001*(sin(t2)*sin(t4)*cos(t1)*cos(t3) + cos(t1)*cos(t2)*cos(t4))*cos(t6) + 0.384000001*sin(t2)*sin(t4)*cos(t1)*cos(t3) + 0.083*sin(t2)*cos(t1)*cos(t3)*cos(t4) - 0.082000001*sin(t2)*cos(t1)*cos(t3) - 0.083*sin(t4)*cos(t1)*cos(t2) + 0.384000001*cos(t1)*cos(t2)*cos(t4) + 0.316*cos(t1)*cos(t2)
	J13 =0.21000001*(-(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t5) + (-sin(t1)*cos(t3) - sin(t3)*cos(t1)*cos(t2))*cos(t4)*cos(t5))*sin(t6) + 0.088*(-(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t5) + (-sin(t1)*cos(t3) - sin(t3)*cos(t1)*cos(t2))*cos(t4)*cos(t5))*cos(t6) - 0.088*(-sin(t1)*cos(t3) - sin(t3)*cos(t1)*cos(t2))*sin(t4)*sin(t6) + 0.21000001*(-sin(t1)*cos(t3) - sin(t3)*cos(t1)*cos(t2))*sin(t4)*cos(t6) - 0.384000001*(-sin(t1)*cos(t3) - sin(t3)*cos(t1)*cos(t2))*sin(t4) - 0.083*(-sin(t1)*cos(t3) - sin(t3)*cos(t1)*cos(t2))*cos(t4) - 0.082000001*sin(t1)*cos(t3) - 0.082000001*sin(t3)*cos(t1)*cos(t2)
	J14=0.21000001*(-(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) + sin(t2)*cos(t1)*cos(t4))*sin(t6)*cos(t5) + 0.088*(-(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) + sin(t2)*cos(t1)*cos(t4))*cos(t5)*cos(t6) + 0.088*(-(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) - sin(t2)*sin(t4)*cos(t1))*sin(t6) - 0.21000001*(-(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) - sin(t2)*sin(t4)*cos(t1))*cos(t6) + 0.083*(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) - 0.384000001*(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) - 0.384000001*sin(t2)*sin(t4)*cos(t1) - 0.083*sin(t2)*cos(t1)*cos(t4)
	J15 = 0.21000001*(-((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*sin(t5) - (sin(t1)*cos(t3) + sin(t3)*cos(t1)*cos(t2))*cos(t5))*sin(t6) + 0.088*(-((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*sin(t5) - (sin(t1)*cos(t3) + sin(t3)*cos(t1)*cos(t2))*cos(t5))*cos(t6)
	J16 = -0.088*(((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*cos(t5) - (sin(t1)*cos(t3) + sin(t3)*cos(t1)*cos(t2))*sin(t5))*sin(t6) + 0.21000001*(((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*cos(t5) - (sin(t1)*cos(t3) + sin(t3)*cos(t1)*cos(t2))*sin(t5))*cos(t6) + 0.21000001*(-(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) + sin(t2)*cos(t1)*cos(t4))*sin(t6) + 0.088*(-(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) + sin(t2)*cos(t1)*cos(t4))*cos(t6)

	J17 =0

	J21 =0.21000001*(((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*cos(t5) - (sin(t1)*cos(t3) + sin(t3)*cos(t1)*cos(t2))*sin(t5))*sin(t6) + 0.088*(((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*cos(t5) - (sin(t1)*cos(t3) + sin(t3)*cos(t1)*cos(t2))*sin(t5))*cos(t6) + 0.088*(-(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) + sin(t2)*cos(t1)*cos(t4))*sin(t6) - 0.21000001*(-(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) + sin(t2)*cos(t1)*cos(t4))*cos(t6) - 0.384000001*(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) - 0.083*(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) - 0.082000001*sin(t1)*sin(t3) - 0.083*sin(t2)*sin(t4)*cos(t1) + 0.384000001*sin(t2)*cos(t1)*cos(t4) + 0.316*sin(t2)*cos(t1) + 0.082000001*cos(t1)*cos(t2)*cos(t3)

	J22 =0.21000001*((-sin(t1)*sin(t2)*cos(t3)*cos(t4) + sin(t1)*sin(t4)*cos(t2))*cos(t5) + sin(t1)*sin(t2)*sin(t3)*sin(t5))*sin(t6) + 0.088*((-sin(t1)*sin(t2)*cos(t3)*cos(t4) + sin(t1)*sin(t4)*cos(t2))*cos(t5) + sin(t1)*sin(t2)*sin(t3)*sin(t5))*cos(t6) + 0.088*(sin(t1)*sin(t2)*sin(t4)*cos(t3) + sin(t1)*cos(t2)*cos(t4))*sin(t6) - 0.21000001*(sin(t1)*sin(t2)*sin(t4)*cos(t3) + sin(t1)*cos(t2)*cos(t4))*cos(t6) + 0.384000001*sin(t1)*sin(t2)*sin(t4)*cos(t3) + 0.083*sin(t1)*sin(t2)*cos(t3)*cos(t4) - 0.082000001*sin(t1)*sin(t2)*cos(t3) - 0.083*sin(t1)*sin(t4)*cos(t2) + 0.384000001*sin(t1)*cos(t2)*cos(t4) + 0.316*sin(t1)*cos(t2)

	J23 =0.21000001*((-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*cos(t4)*cos(t5) - (sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t5))*sin(t6) + 0.088*((-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*cos(t4)*cos(t5) - (sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t5))*cos(t6) - 0.088*(-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*sin(t4)*sin(t6) + 0.21000001*(-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*sin(t4)*cos(t6) - 0.384000001*(-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*sin(t4) - 0.083*(-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*cos(t4) - 0.082000001*sin(t1)*sin(t3)*cos(t2) + 0.082000001*cos(t1)*cos(t3)

	J24 =0.21000001*(-(sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t4) + sin(t1)*sin(t2)*cos(t4))*sin(t6)*cos(t5) + 0.088*(-(sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t4) + sin(t1)*sin(t2)*cos(t4))*cos(t5)*cos(t6) + 0.088*(-(sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) - sin(t1)*sin(t2)*sin(t4))*sin(t6) - 0.21000001*(-(sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) - sin(t1)*sin(t2)*sin(t4))*cos(t6) + 0.083*(sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t4) - 0.384000001*(sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) - 0.384000001*sin(t1)*sin(t2)*sin(t4) - 0.083*sin(t1)*sin(t2)*cos(t4)

	J25 =0.21000001*(-((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4))*sin(t5) - (sin(t1)*sin(t3)*cos(t2) - cos(t1)*cos(t3))*cos(t5))*sin(t6) + 0.088*(-((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4))*sin(t5) - (sin(t1)*sin(t3)*cos(t2) - cos(t1)*cos(t3))*cos(t5))*cos(t6)


	J26 =-0.088*(((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4))*cos(t5) - (sin(t1)*sin(t3)*cos(t2) - cos(t1)*cos(t3))*sin(t5))*sin(t6) + 0.21000001*(((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4))*cos(t5) - (sin(t1)*sin(t3)*cos(t2) - cos(t1)*cos(t3))*sin(t5))*cos(t6) + 0.21000001*(-(sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t4) + sin(t1)*sin(t2)*cos(t4))*sin(t6) + 0.088*(-(sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t4) + sin(t1)*sin(t2)*cos(t4))*cos(t6)

	J27 =0

	J31 =0
	J32 =0.21000001*((-sin(t2)*sin(t4) - cos(t2)*cos(t3)*cos(t4))*cos(t5) + sin(t3)*sin(t5)*cos(t2))*sin(t6) + 0.088*((-sin(t2)*sin(t4) - cos(t2)*cos(t3)*cos(t4))*cos(t5) + sin(t3)*sin(t5)*cos(t2))*cos(t6) + 0.088*(-sin(t2)*cos(t4) + sin(t4)*cos(t2)*cos(t3))*sin(t6) - 0.21000001*(-sin(t2)*cos(t4) + sin(t4)*cos(t2)*cos(t3))*cos(t6) + 0.083*sin(t2)*sin(t4) - 0.384000001*sin(t2)*cos(t4) - 0.316*sin(t2) + 0.384000001*sin(t4)*cos(t2)*cos(t3) + 0.083*cos(t2)*cos(t3)*cos(t4) - 0.082000001*cos(t2)*cos(t3)

	J33 =0.21000001*(sin(t2)*sin(t3)*cos(t4)*cos(t5) + sin(t2)*sin(t5)*cos(t3))*sin(t6) + 0.088*(sin(t2)*sin(t3)*cos(t4)*cos(t5) + sin(t2)*sin(t5)*cos(t3))*cos(t6) - 0.088*sin(t2)*sin(t3)*sin(t4)*sin(t6) + 0.21000001*sin(t2)*sin(t3)*sin(t4)*cos(t6) - 0.384000001*sin(t2)*sin(t3)*sin(t4) - 0.083*sin(t2)*sin(t3)*cos(t4) + 0.082000001*sin(t2)*sin(t3)

	J34 =0.21000001*(sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4))*sin(t6)*cos(t5) + 0.088*(sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4))*cos(t5)*cos(t6) + 0.088*(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*sin(t6) - 0.21000001*(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*cos(t6) - 0.083*sin(t2)*sin(t4)*cos(t3) + 0.384000001*sin(t2)*cos(t3)*cos(t4) - 0.384000001*sin(t4)*cos(t2) - 0.083*cos(t2)*cos(t4)

	J35 =0.21000001*(-(-sin(t2)*cos(t3)*cos(t4) + sin(t4)*cos(t2))*sin(t5) + sin(t2)*sin(t3)*cos(t5))*sin(t6) + 0.088*(-(-sin(t2)*cos(t3)*cos(t4) + sin(t4)*cos(t2))*sin(t5) + sin(t2)*sin(t3)*cos(t5))*cos(t6)

	J36 =-0.088*((-sin(t2)*cos(t3)*cos(t4) + sin(t4)*cos(t2))*cos(t5) + sin(t2)*sin(t3)*sin(t5))*sin(t6) + 0.21000001*((-sin(t2)*cos(t3)*cos(t4) + sin(t4)*cos(t2))*cos(t5) + sin(t2)*sin(t3)*sin(t5))*cos(t6) + 0.21000001*(sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4))*sin(t6) + 0.088*(sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4))*cos(t6)

	J37 =0

	J = np.array([[(J11), (J12), (J13), (J14), (J15), (J16), (J17)], [(J21), (J22), (J23), J24, (J25), (J26), J27], [(J31), (J32), (J33), (J34), (J35), (J36), (J37)]])	
	return J
	



if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    # q = np.array([0,0,0,0,0,0,0])
    print(np.round(calcJacobian(q),3))
