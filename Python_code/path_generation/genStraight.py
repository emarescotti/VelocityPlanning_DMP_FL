# SPDX-License-Identifier: AGPL-3.0-or-later

import numpy as np
import math
import matplotlib.pyplot as plt

def funStraight(x_in,y_in,x_fin,y_fin):
    
    length = np.sqrt((x_fin-x_in)**2 + (y_in-y_fin)**2)
    N = int(round(length*100)) # n points to generate for this segment
    
    t = np.linspace(0,length,N)
    x = np.interp(t,[0,length],[x_in,x_fin])
    y = np.interp(t,[0,length],[y_in,y_fin])

    return x,y

"""
# example code
[x,y] = funStraight(3,5,9,4)

plt.figure(1)
plt.axis('equal')
plt.grid()
plt.plot(x,y,'o-')
plt.title("Test Straight line")
"""