# SPDX-License-Identifier: AGPL-3.0-or-later

import numpy as np
import math
import matplotlib.pyplot as plt
from genStraight import funStraight
from genSawTooth import funSawtooth
from genCurve import funCurve 

# CODE TO GENERATE A PATH FOR OUR DMP FRAMEWORK

# line 1
X_pts = [0]
Y_pts = [0]

rad = 30      # initial radius
delta_r = 5  # decreasing radius value
y = 0         # abscissa
straight = 50 # straight line length

while rad > 0:
    
    y -= rad
    
    # curve 1 
    [a,b] = funCurve(straight,y,rad,np.pi/2,-np.pi)
    X_pts = np.append(X_pts, a)
    Y_pts = np.append(Y_pts, b)
    
    y -= rad
        
    if rad > 0:
        rad -= delta_r
    else:
        break
    
    y -= rad
        
    # curve 2 
    [a,b] = funCurve(0,y,rad,np.pi/2,np.pi)
    X_pts = np.append(X_pts, a)
    Y_pts = np.append(Y_pts, b)
    
    y -= rad
    rad -= delta_r

X_pts = np.append(X_pts, X_pts[-1] + 50)
Y_pts = np.append(Y_pts, Y_pts[-1])

Xrotate = np.append(0,-Y_pts)
Yrotate = np.append(0,-X_pts - 30)

# final plotting
plt.figure(1)
plt.grid()
plt.title('Generic Path [dim: mm]')
plt.plot(Xrotate,Yrotate,'o-')
plt.get_current_fig_manager().window.showMaximized()
plt.show()

np.savez('test1',[Xrotate,Yrotate])
