import numpy as np
<<<<<<< HEAD
import math
=======

>>>>>>> 8bb54494a7d62144463048fe5b7ad5b46ed5994c

def linear_solver(A, b):
    """
    Solve for x Ax=b. Assume A is invertible.
    Args:
        A: nxn numpy array
        b: 0xn numpy array

    Returns:
        x: 0xn numpy array
    """
    # Insert student code here
<<<<<<< HEAD
    x = np.dot(np.linalg.inv(A),b)
    return x
=======
    return b
>>>>>>> 8bb54494a7d62144463048fe5b7ad5b46ed5994c


def angle_solver(v1, v2):
    """
    Solves for the magnitude of the angle between v1 and v2
    Args:
        v1: 0xn numpy array
        v2: 0xn numpy array

    Returns:
        theta = scalar >= 0 = angle in radians
    """
    # Insert student code here
<<<<<<< HEAD
    temp = np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2))
    
    #sanity check to ensure input of acos between -1 and 1
    if temp<=-1:
        temp = -1
    elif temp>=1:
        temp = 1
    
    #since we need absolute value
    theta = math.fabs(math.acos(temp))

    return theta
=======
    return 0
>>>>>>> 8bb54494a7d62144463048fe5b7ad5b46ed5994c


def linear_euler_integration(A, x0, dt, nSteps):
    """
    Integrate the ode x'=Ax using euler integration where:
    x_{k+1} = dt (A x_k) + x_k
    Args:
        A: nxn np array describing linear differential equation
        x0: 0xn np array Initial condition
        dt: scalar, time step
        nSteps: scalar, number of time steps

    Returns:
        x: state after nSteps time steps (np array)
    """
<<<<<<< HEAD
    #initial state
    x = x0
    #nSteps simulation
    for i in range(nSteps):
        #Hoping that dimensionality is correct for operations
        x = np.add(x,np.dot(A,x)*dt)
        
    return x
=======
    # Insert student code here
    return x0
>>>>>>> 8bb54494a7d62144463048fe5b7ad5b46ed5994c


if __name__ == '__main__':
    # Example call for linear solver
    A = np.array([[1, 2], [3, 4]])
    b = np.array([1, 2])
    print(linear_solver(A, b))

    # Example call for angles between vectors
    v1 = np.array([1, 0])
    v2 = np.array([0, 1])
    print(angle_solver(v1, v2))

    # Example call for euler integration
    A = np.random.rand(3, 3)
    x0 = np.array([1, 1, 1])
    dt = 0.01
    nSteps = 100
    print(linear_euler_integration(A, x0, dt, nSteps))
