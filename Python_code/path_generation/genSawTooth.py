# SPDX-License-Identifier: AGPL-3.0-or-later

# Python Sawtooth Code
import matplotlib.pyplot as plt
import numpy as np

def funSawtooth(base,height,repetition,direction=0,verso=0,x_pos=0,y_pos=0):
    arr = np.array
       
    sawtoothx = [x_pos]
    sawtoothy = [y_pos]
    t = [0]
    
    for i in range(repetition):     
   
        t = np.append(t, t[-1] + np.sqrt(height**2 + base**2))
        t = np.append(t, t[-1] + height)
        
        if direction == 0: #horizontal sawtooth
            if verso == 0: #positive verso
        
                spike = arr([x_pos+base,y_pos+height])
                B = arr([x_pos+base,y_pos])
                x_pos = x_pos + base
            
            elif verso == 1: #negative verso
                
                spike = arr([x_pos-base,y_pos+height])
                B = arr([x_pos-base,y_pos])
                x_pos = x_pos - base
       
        elif direction == 1: #vertical sawtooth
              if verso == 0: #positive verso
                   
                spike = arr([x_pos-height,y_pos+base])  #to change spike direction modify x_pos+height
                B = arr([x_pos,y_pos+base])
                y_pos = y_pos + base
                
              elif verso == 1: #negative verso
        
                spike = arr([x_pos-height,y_pos-base])
                B = arr([x_pos,y_pos-base])
                y_pos = y_pos - base
               
        saw  = arr([spike,B])
        sawx = saw[:,0]
        sawy = saw[:,1]
    
        sawtoothx = np.append(sawtoothx, sawx)
        sawtoothy = np.append(sawtoothy, sawy)
        
    length = repetition * (height +np.sqrt(height**2 + base**2)) 
    N = round(length*10) # number of points of all the teeth
    q = np.linspace(0,length,int(N)+1)
    x = np.interp(q,t,sawtoothx)
    y = np.interp(q,t,sawtoothy)
 
    return x,y

"""
# Example

# Reference Starting Position
x_pos = 50 
y_pos = 50

# Sawtooth characteristics
base = 5
height = 1
repetition = 5
# 0 for Horizontal, 1 for Vertical
direction = 1
# 0 for positive verso, 1 for negative verso
verso = 1

(x,y) = funSawtooth(base,height,repetition,direction,verso,x_pos,y_pos)

plt.figure(2)
plt.plot(x,y,'-')
plt.grid()
plt.title('Sawtooth plot')
"""