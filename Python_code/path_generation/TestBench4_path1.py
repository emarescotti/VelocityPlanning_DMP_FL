# SPDX-License-Identifier: AGPL-3.0-or-later

import numpy as np
import math
import matplotlib.pyplot as plt
from genStraight import funStraight
from genSawTooth import funSawtooth
from genCurve import funCurve 

# straight edges
X_pts = np.array([0,40,40,80,80,120.4])
Y_pts = np.array([0,0,-40,-40,0.2,0])

# smooth curves
[a,b] = funCurve(120,-30,30,np.pi/2,-np.pi)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)

X_pts = np.append(X_pts, 40)
Y_pts = np.append(Y_pts, -60)

[a,b] = funCurve(40,-100,40,np.pi/2,np.pi)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)

[a,b] = funCurve(70,-130,10,-np.pi/2,np.pi/2)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)

# curve up-down
updown_rep = 3
straight = 10
radius = 10
for i in range(updown_rep):
    
    [a,b] = funStraight(X_pts[-1],Y_pts[-1],X_pts[-1],Y_pts[-1]+straight)
    X_pts = np.append(X_pts, a)
    Y_pts = np.append(Y_pts, b)
    
    [a,b] = funCurve(X_pts[-1]+radius,Y_pts[-1],radius,np.pi,-np.pi)
    X_pts = np.append(X_pts, a)
    Y_pts = np.append(Y_pts, b)
    
    [a,b] = funStraight(X_pts[-1],Y_pts[-1],X_pts[-1],Y_pts[-1]-straight)
    X_pts = np.append(X_pts, a)
    Y_pts = np.append(Y_pts, b)
    
    [a,b] = funCurve(X_pts[-1]+radius,Y_pts[-1],radius,np.pi,np.pi)
    X_pts = np.append(X_pts, a)
    Y_pts = np.append(Y_pts, b)

X_pts = np.append(X_pts, 200)
Y_pts = np.append(Y_pts, -40)

# final plotting
plt.figure(1)
plt.grid()
plt.title('Generic Path [dim: mm]')
plt.plot(X_pts,Y_pts,'o-')
plt.get_current_fig_manager().window.showMaximized()
plt.show()

np.savez('test1',[X_pts,Y_pts])