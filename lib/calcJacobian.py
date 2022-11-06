import numpy as np
from lib.calculateFK import FK

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
        print(-translation_vector)
        # print(np.add(translation_vector,-jointPositions[i,:]))
        J[0:3,i] =get_skew_symmetric_matrix(approaches[:,i])@(np.add(translation_vector,-jointPositions[i,:]))
        J[3:6,i] = approaches[:,i]

    return J
def get_skew_symmetric_matrix(vector):
    a = vector[0]
    b = vector[1]
    c = vector[2]

    return np.array([[0,-c,b],
                    [c,0,-a],
                    [-b,a,0]])


if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    # q = np.array([0,0,0,0,0,0,0])
    print(np.round(calcJacobian(q),3))
