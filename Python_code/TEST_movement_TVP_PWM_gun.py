#! /usr/bin/env python
# SPDX-License-Identifier: AGPL-3.0-or-later

import numpy as np

def euler_to_quaternion(yaw, pitch, roll): # Z Y X
    qx = round(np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - 
               np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2),6)
    qy = round(np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + 
               np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2),6)
    qz = round(np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - 
               np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2),6)
    qw = round(np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + 
               np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2),6)
    return [qx, qy, qz, qw]

dt = 0.001  # [s] - time step of the robot controller

# definition of the offset values (txt file generation)
x_off =  410.11 # 400.0 # [mm]
y_off = -479.90 # 50.0  # [mm]
z_off =  419.0 # 300.0 # [mm]

# gun inclination
inclination = 0 * np.pi / 180 # [rad] - inclination angle of the gun

########
# PATH #
########

v_target = 20.0 # [mm/s]
acc = 150.0      # [mm/s^2]
space = 240.0   # [mm] - total distance

t_acc = v_target / acc
t_tot = t_acc + space / v_target
t = np.linspace(0,t_tot,int(t_tot/dt))

path = np.zeros(len(t)) # straight movement
flow_path = np.zeros(len(t)) # flow for the caulking gun [values: 0-9]

for i in range(1,len(t)):
    
    if t[i] < v_target / acc:
        path[i] = path[i-1] + acc* t[i] * dt
    elif t[i] > t_tot - v_target / acc:
        path[i] = path[i-1] + acc* (t_tot-t[i]) * dt
    else:
        path[i] = path[i-1] + v_target * dt	
        
# 1st line
x = (-path + x_off) # [mm]
y = (np.zeros(len(x)) + y_off) # [mm]

x /= 1000 # [m]
y /= 1000 # [m]
z = (np.zeros(len(x)) + z_off) / 1000 # [m]

####################
# ANGLE DEFINITION #
####################

# Euler angles ZYX to quaternions
qx = qy = qz = qw = []

for i in range(len(x)-1):
    z_rot = np.arctan2((y[i+1]-y[i]),(x[i+1]-x[i])) + np.pi/4 # [rad]
    y_rot = np.pi + inclination # [rad]

    [qxi,qyi,qzi,qwi] = euler_to_quaternion(z_rot, y_rot, 0) # Z Y X
    
    qx = np.append(qx,qxi)
    qy = np.append(qy,qyi)
    qz = np.append(qz,qzi)
    qw = np.append(qw,qwi)

qx = np.append(qx,qxi)
qy = np.append(qy,qyi)
qz = np.append(qz,qzi)
qw = np.append(qw,qwi)

############
# ROS NODE #
############

import rospy
import serial
import time
from geometry_msgs.msg import PoseStamped

ard = serial.Serial('/dev/ttyUSB0',115200,timeout=0)
time.sleep(2)
ard.write('99\n')
time.sleep(1.0)

countH = 0 # we don't give serial values each time step
countL = 0
pwm = 1

try:
    pub = rospy.Publisher('/DMP_pose', PoseStamped, queue_size=1)
    rospy.init_node('DMP_planner', anonymous=True)
    rate = rospy.Rate(1/dt) # Hz
    
    p = PoseStamped()

    task_ended = False
    
    while not rospy.is_shutdown():
        
        if not task_ended:
            for i in range(len(x)):
                p.pose.position.x = round(x[i],5);
                p.pose.position.y = round(y[i],5);
                p.pose.position.z = round(z[i],5);
                p.pose.orientation.x = round(qx[i],6);
                p.pose.orientation.y = round(qy[i],6);
                p.pose.orientation.z = round(qz[i],6);
                p.pose.orientation.w = round(qw[i],6);
        	
                if pwm is 1:
		           if countH == 150:
		               ard.write(str(40))
		               ard.write('\n')
		               pwm = 0
		               countH = 0  
		           countH += 1            
                else:
		           if countL == 350:
		               ard.write(str(28))
		               ard.write('\n')
		               pwm = 1
		               countL = 0  
		           countL += 1
                   
                pub.publish(p)
                rate.sleep()

            task_ended=True
            ard.write('0\n')
            ard.close()

        pub.publish(p)
        rate.sleep()

except rospy.ROSInterruptException:
    pass
