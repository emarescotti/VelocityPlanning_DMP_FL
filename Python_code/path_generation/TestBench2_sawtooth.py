# SPDX-License-Identifier: AGPL-3.0-or-later

import numpy as np
import math
import matplotlib.pyplot as plt
from genStraight import funStraight
from genSawTooth import funSawtooth
from genCurve import funCurve 

# CODE TO GENERATE A PATH FOR OUR DMP FRAMEWORK

# line 1
X_pts = [0,50]
Y_pts = [0,0]

# Sawtooth characteristics
base = 30.0
height = 50.0
repetition = 4
# 0 for Horizontal, 1 for Vertical
direction = 1
# 0 for positive verso, 1 for negative verso
verso = 1

(x,y) = funSawtooth(base,height,repetition,direction,verso,X_pts[-1],Y_pts[-1])
X_pts = np.append(X_pts,x)
Y_pts = np.append(Y_pts,y)

# final plotting
plt.figure(1)
plt.grid()
plt.title('Generic Path [dim: mm]')
plt.plot(X_pts,Y_pts,'o-')
plt.get_current_fig_manager().window.showMaximized()
plt.show()

np.savez('test1',[X_pts,Y_pts])