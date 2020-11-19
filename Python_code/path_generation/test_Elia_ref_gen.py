# SPDX-License-Identifier: AGPL-3.0-or-later

import numpy as np
import math
import matplotlib.pyplot as plt
from genSawTooth import funSawtooth

# CODE TO GENERATE THE REFERENCE
code = int(input("PATH GENERATOR: type a number between 1 and 8: "))

if code == 1:
    # CODE TO CREATE A FLAT LINE AND THEN A COSINE
    x = np.linspace(0,100*math.pi,num=1000)
    y = np.zeros(len(x))
    
    gain = 100
    N = 250
    for i in range(N):
        y[i] = gain
    for i in range(len(x)-N):
        y[i+N] = gain * np.cos((x[i]/20))
    
elif code == 2:
    # CODE TO CREATE A CIRCLE
    N = 1000;
    radius = 200;
    a = np.arange(N) / N * 2 * math.pi
    
    x = np.cos(a) * radius
    y = np.sin(a) * radius
    
elif code == 3:
    # CODE TO CREATE AN EIGHT
    N = 1000;
    gain = 75;
    a = np.arange(N) / N * 4 * math.pi
    
    x = np.zeros(N)
    for i in range(int(N/4)):
        x[i] = (1 + np.cos(a[i])) * gain
    for i in range(int(N/2)):
        x[i+int(N/4)] = (-1 + np.cos(a[i])) * gain
    for i in range(int(N/4)):
        x[i+int(N*3/4)] = (1 - np.cos(a[i])) * gain
    
    y = np.sin(a) * gain
    """
    x = x[:N-125]
    y = y[:N-125]
    """
    
elif code == 4:
    # CODE TO CREATE A STRAIGHT LINE
    x = np.linspace(0,350,num=1000)
    y = np.linspace(0,-150,num=1000)
    
elif code == 5:
    # CODE TO CREATE A SPIRAL
    turns = 4
    N = 1000
    R = 0
    R_fin = 400
    th = 0
    th_fin = turns * 2 * np.pi
    
    x = np.zeros(N)
    y = np.zeros(N)
    for i in range(N):
        th += th_fin / N
        R  += R_fin / N 
        x[i] = R * np.cos(th)
        y[i] = R * np.sin(th)
        
elif code == 6:
    # CODE TO CREATE A SAWTOOTH LINE
    base = 30
    height = 40
    repetition = 5
    a,b = funSawtooth(base,height,repetition)
    x = y = [0]
    x = np.append(x,a+10)
    y = np.append(y,b)
    x = np.append(x,x[-1]+10)
    y = np.append(y,y[-1])
    
elif code == 7:
    # VARIABLE DENSITY OF POINTS
    x = np.zeros(10)
    y = x
    
    x = np.append(x,np.linspace(10,60,30))
    y = np.append(y,np.zeros(30))
    
    x = np.append(x,np.linspace(60,120,1000))
    y = np.append(y,np.linspace(0,80,1000))
    
    x = np.append(x,np.linspace(120,190,10))
    y = np.append(y,np.linspace(80,30,10))
    
    x = np.append(x,np.ones(10)*x[-1])
    y = np.append(y,np.ones(10)*y[-1])
    
elif code == 8: # GOING BACK AND FORTH ON THE SAME LINE
    
    rep = 5 # number of repetitions
    
    x = np.zeros(5)
    
    for i in range(rep):
        x = np.append(x,np.linspace(0,100,20))
        x = np.append(x,np.linspace(100,0,20))
    
    x = np.append(x,np.ones(5)*x[-1])
    y = np.zeros(len(x))
    
plt.figure(1)
plt.plot(x,y,'o-')
plt.grid()
plt.get_current_fig_manager().window.showMaximized()
plt.show()
    
np.savez('test1',[x,y])