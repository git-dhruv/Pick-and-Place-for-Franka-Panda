import roboticstoolbox as rtb
robot = rtb.models.Panda()
robot2 = rtb.models.Panda()

import numpy as np
q = np.array([4.3150e-01 , 5.5620e-01 , 6.0400e-01 ,-1.57,  6.2100e-01  ,2.5625e+00,7.0700e-01])
q2 = np.array([1.9, 1.57, -1.57, -1.57, 1.57, 1.57, 0.707])
print(q*180/3.14,q2*180/3.14)


from math import sin,cos,atan2
q = -2.666788734561893
print(atan2(sin(q),cos(q)))



q = [ 1.89573997  ,1.56893163 ,-1.56808957 ,-1.57137022  ,1.56247901  ,1.57417205,0.707     ]
# q2 = [ 1.9    1.57  -1.57  -1.57   1.57   1.57   0.707]
robot.plot(q)
robot2.plot(q2)
