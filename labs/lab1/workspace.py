from lib.calculateFK import FK
from core.interfaces import ArmController

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fk = FK()

# the dictionary below contains the data returned by calling arm.joint_limits()
limits = [
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -1.7628, 'upper': 1.7628},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -3.0718, 'upper': -0.0698},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -0.0175, 'upper': 3.7525},
    {'lower': -2.8973, 'upper': 2.8973}
 ]

# TODO: create plot(s) which visualize the reachable workspace of the Panda arm,
# accounting for the joint limits.
#
# We've included some very basic plotting commands below, but you can find
# more functionality at https://matplotlib.org/stable/index.html
import numpy as np


q1 = np.arange(limits[0]['lower'], limits[0]['upper'], 1.) 
q2 = np.arange(limits[1]['lower'], limits[1]['upper'], 1.) 
q3 = np.arange(limits[2]['lower'], limits[2]['upper'], 0.8) 
q4 = np.arange(limits[3]['lower'], limits[3]['upper'], 1.) 
q5 = np.arange(limits[4]['lower'], limits[4]['upper'], 0.8) 
q6 = np.arange(limits[5]['lower'], limits[5]['upper'], 1.) 

x = []
y = []
z = []


for a in q1:
    for b in q2:
        for c in q3:
            for d in q4:
                for e in q5:
                    for f in q6:
                        _,T = fk.forward([a,b,c,d,e,f,0])
                        x.append(float(T[0,3]))
                        y.append(float(T[1,3]))
                        z.append(float(T[2,3]))




fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# TODO: update this with real results
ax.scatter(x,y,z,s=0.25) # plot the point (1,1,1)
plt.xlim([-1,1])
plt.ylim([-1,1])
ax.set_zlim([-1,1])

x_pos = [0, 0,0]
y_pos = [0, 0,0]
x_direct = [1/2, 0,0]
y_direct = [0, 1/2,0]
z_direct = [0,0,1/2]
ax.scatter(0,0,0,s=30)
ax.quiver(x_pos,y_pos,x_pos,x_direct,y_direct,z_direct,color='g',cmap="Reds")
plt.show()
