# SPDX-License-Identifier: AGPL-3.0-or-later

import numpy as np
import math
import matplotlib.pyplot as plt
from genStraight import funStraight
from genSawTooth import funSawtooth
from genCurve import funCurve 

# straight edges
X_pts = np.array([0,50])
Y_pts = np.array([0,0])

[a,b] = funCurve(50,-5,5,np.pi/2,-np.pi/2)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)

# curve up-down
updown_rep = 4
straight = 20
radius = 5
for i in range(updown_rep):
    
    [a,b] = funStraight(X_pts[-1],Y_pts[-1],X_pts[-1],Y_pts[-1]-straight)
    X_pts = np.append(X_pts, a)
    Y_pts = np.append(Y_pts, b)
    
    [a,b] = funCurve(X_pts[-1]+radius,Y_pts[-1],radius,-np.pi,np.pi)
    X_pts = np.append(X_pts, a)
    Y_pts = np.append(Y_pts, b)
    
    [a,b] = funStraight(X_pts[-1],Y_pts[-1],X_pts[-1],Y_pts[-1]+straight)
    X_pts = np.append(X_pts, a)
    Y_pts = np.append(Y_pts, b)
    
    [a,b] = funCurve(X_pts[-1]+radius,Y_pts[-1],radius,-np.pi,-np.pi)
    X_pts = np.append(X_pts, a)
    Y_pts = np.append(Y_pts, b)

# smooth curves
[a,b] = funCurve(150,-5,15,-np.pi,np.pi/2)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)

[a,b] = funCurve(150,-60,40,np.pi/2,-np.pi)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)

X_pts = np.append(X_pts, 100)
Y_pts = np.append(Y_pts, -100)

[a,b] = funCurve(100,-90,10,-np.pi/2,-np.pi)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)
    
[a,b] = funCurve(100,-70,10,-np.pi/2,np.pi)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)

X_pts = np.append(X_pts, 75)
Y_pts = np.append(Y_pts, -60)

# Sawtooth characteristics
base = 20.0
height = 40.0
repetition = 3
# 0 for Horizontal, 1 for Vertical
direction = 1
# 0 for positive verso, 1 for negative verso
verso = 1

(x,y) = funSawtooth(base,height,repetition,direction,verso,X_pts[-1],Y_pts[-1])
X_pts = np.append(X_pts,x)
Y_pts = np.append(Y_pts,y)

X_pts = np.append(X_pts, 75)
Y_pts = np.append(Y_pts, -150)

X_pts = np.append(X_pts, 10)
Y_pts = np.append(Y_pts, -150)

[a,b] = funCurve(10,-140,10,-np.pi/2,-np.pi/2)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)

X_pts = np.append(X_pts, 0)
Y_pts = np.append(Y_pts, 0)

# final plotting
plt.figure(1)
plt.grid()
plt.title('Generic Path [dim: mm]')
plt.plot(X_pts,Y_pts,'o-')
plt.get_current_fig_manager().window.showMaximized()
plt.show()

np.savez('test1',[X_pts,Y_pts])