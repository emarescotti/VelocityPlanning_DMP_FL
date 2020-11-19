# SPDX-License-Identifier: AGPL-3.0-or-later

import numpy as np
import math
import matplotlib.pyplot as plt

def steeringAngle(path,body_length):
    
    # check that "path" is a column array
    if np.size(path,1) > np.size(path,0):
        path = path.T
    
    angle = np.zeros(int(body_length/2))
    steering = np.array([0])
    last = 0 # for unwrapping
    
    for i in range(int(body_length/2),int(np.size(path,0)-body_length/2)+1):
        a = path[i+int(body_length/2),1] - path[i-int(body_length/2),1]
        b = path[i+int(body_length/2),0] - path[i-int(body_length/2),0]
        
        c = np.arctan2(a,b)
        
        while c < last - np.pi: c += 2*math.pi
        while c > last + np.pi: c -= 2*math.pi
        last = c
        
        angle = np.append(angle,c)
    
    # first values are equal to the first computed one
    angle[0:int(body_length/2)] = np.ones(
        int(body_length/2))*angle[int(body_length/2)]
    
    # restore correct length of the 2 vectors
    angle = np.append(angle,np.ones(int(body_length/2))*angle[-1])
    
    for i in range(1,np.size(path,0)):
        steering = np.append(steering, angle[i] - angle[i-1])

    return angle, steering

"""
# EXAMPLE CODE: analyze a circumference
R = 5 # circumference radius
body_length = 5
x = []
y = []

for i in  range(500):
    th = 2*np.pi/500 * i
    x = np.append(x, R * np.cos(th))
    y = np.append(y, R * np.sin(th))
    
angle,steering = steeringAngle(np.array([x,y]),body_length)

# PLOT THE RESULTS
plt.figure(1)
plt.plot(angle)
plt.grid()
plt.get_current_fig_manager().window.showMaximized()
plt.show()
"""
