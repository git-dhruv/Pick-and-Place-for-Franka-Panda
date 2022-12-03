import numpy as np 
from math import sin,cos,pi,atan2
from lib.calcJacobian import calcJacobian
     
def IK_velocity(q_in, v_in, omega_in):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param v_in: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega_in: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 1 x 7 vector corresponding to the joint velocities. If v_in and omega_in
         are infeasible, then dq should minimize the least squares error. If v_in
         and omega_in have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """



    v_in = v_in.reshape((3,1))
    omega_in = omega_in.reshape((3,1))

    twist = np.vstack((v_in,omega_in))

    

    nan_indices = np.argwhere(np.isnan(twist.reshape(6,)))

    J = calcJacobian(q_in)

    twist = np.delete(twist,nan_indices,axis=0)
    J = np.delete(J,nan_indices,axis=0)


    if np.linalg.det(J@J.T)<=0.0001:
          dq = np.linalg.lstsq(J,twist,rcond=None)
          dq = dq[0]       

    else:
          dq = np.linalg.pinv(J)@twist
          
    dq = dq.reshape(7,)
    return dq

