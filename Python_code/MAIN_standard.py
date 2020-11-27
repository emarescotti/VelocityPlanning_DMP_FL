#! /usr/bin/env python
# SPDX-License-Identifier: AGPL-3.0-or-later

import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate

from steeringAngle import steeringAngle

from fuzzy_system.fuzzy_variable_output import FuzzyOutputVariable
from fuzzy_system.fuzzy_variable_input import FuzzyInputVariable
from fuzzy_system.fuzzy_system import FuzzySystem

import dmp_standard
import dmp_coupled3

from txtsave import txtSave

def euler_to_quaternion(yaw, pitch, roll): # Z Y X
    qx = round(np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - 
               np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2),4)
    qy = round(np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + 
               np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2),4)
    qz = round(np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - 
               np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2),4)
    qw = round(np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + 
               np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2),4)
    return [qx, qy, qz, qw]

plt.close("all")

# ##################################################################### #
#                            ALGORITHM STARTS                           #
# ##################################################################### #

# ###################### #
# 1. PARAMETERS SETTING  #
# ###################### #

nozzle_dia = 8.0 # [mm]         - diameter of the caulking gun nozzle

body = 5.0       # [ODD NUM]    - length of the "car" to compute the steering
clip = 0.49      #              - clipping of the steering parameter (to avoid too hard directional changes)
width_win = 7    # [ODD NUM]    - width of the moving window mean (to smooth the steering)

t_start = 0.2    # [s]          - time for let DMP start and end (to avoid a starting/ending non-null velocity)
t_extra = 10.0   # [s]          - extra simulation time ( Set 0.0 if not necessary)

nbfs = 2000      #              - number of DMP basis functions (the longer the path, the higher the number)

acc_lim = 30.0   # [mm/s^2]     - limit acceleration of the robot

dt = 1e-2        # [s]          - time step for the DMP execution
dt_panda = 1e-3  # [s]          - time step of the robot controller

# definition of the offset position (Each reference path starts at (0,0,0) coordinates, which have to 
# be translated according to the effective starting position in the workspace)
# SET THESE ACCORDING TO THE ACTUAL WORKSPACE
x_off =  410.11 # 400.0 # [mm]
y_off = -479.90 # 50.0  # [mm]
z_off =  419.0 # 300.0  # [mm]

# gun inclination: here it is selected a vertical orientation
inclination = 0 * np.pi / 180 # [rad] - inclination angle of the gun

# desired trajectory (from .npz file, set the directory if needed) - RUN BEFORE THE PATH GENERATOR CODE
y_teach = np.transpose(np.load("/test1.npz",'r+b')['arr_0']) # mm

# ###################################################### #
# 2. INITIAL REFITTING + ANGLE AND STEERING COMPUTATION  #
# ###################################################### #

# check that initial and final points are different [OTHERWISE DMP MIGHT FAIL]
for i in range(len(y_teach[0,:])):
    if y_teach[0,i] == y_teach[-1,i]: y_teach[-1,i] += 0.05 # [mm]

# curvilinea abscissa of the teached path
absc_teach = np.array([0]) 
for i in range(len(y_teach[:,0])-1):
        absc_teach = np.append(absc_teach,absc_teach[-1] + np.sqrt(
        (y_teach[i+1,0]-y_teach[i,0])**2 + (y_teach[i+1,1]-y_teach[i,1])**2))
    
    # check that 2 consecutive abscissa evaluations are not equal:
    # (if that appens, it doesn't allow to properly use the interpolation tool)
    if absc_teach[-1] == absc_teach[-2]:    absc_teach[-1] += 1e-5
    
# the final point of the curvilinea abscissa is also the path length
len_teach = absc_teach[-1]
print("\n The teached path length is: ",np.round(len_teach,2)," mm") 

pt_density = 4/nozzle_dia # [pts/mm] - initial refitting density (selected 4 pts per diameter)
ds = nozzle_dia/4         # [mm/pts] - spatial-step of the v_ref points

n_new = int(round(len_teach*pt_density)) # total number of points wished after refitting

# points are refitted to ensure EQUALLY spacing along the curvilinea abscissa 
# (x1 = cartesian X, x2 = cartesian Y)

# EQUAL SPACING ALONG ABSCISSA, no time characterization
x1 = np.interp(np.linspace(0,len_teach,n_new) , absc_teach,y_teach[:,0])
x2 = np.interp(np.linspace(0,len_teach,n_new) , absc_teach,y_teach[:,1])

y_eq_space = np.column_stack((x1,x2))

# computation of the steering
angle, steer = steeringAngle(y_eq_space,body)

# rectified value of steering (we don't care if we have a left of right curve)
rect_steer = abs(steer)

# clipping of the steering parameter
for i in range(len(rect_steer)):
    if rect_steer[i] > clip:
        rect_steer[i] = clip

# mean filter (moving window)
mean_steer = np.zeros(len(rect_steer))

hw = int(width_win/2) # half-width of the moving window

for i in range(len(rect_steer)):
    if i <= hw:
        mean_steer[i] = np.mean(rect_steer[0:width_win])
    elif i >= len(rect_steer)-hw:
        mean_steer[i] = np.mean(rect_steer[-width_win-1:-1])
    else:
        mean_steer[i] = np.mean(rect_steer[i-hw:i+hw+1])

# ########################################## #
# 3. FUZZY LOGIC + PATH REFERENCE GENERATION #
# ########################################## #

t_ref = v_ref = acc_ref = np.array([0])

for i in range(1,len(mean_steer)):
    
    # INPUT
    in_dict = dict({'Steering':mean_steer[i]}) # mean steering value
    
    # FUZZY SYSTEM SET UP : defintion of the shape functions
    steering = FuzzyInputVariable('Steering',0,0.50,400)
    steering.add_trapezoidal('Very small curve',0.21,0.35,0.5,0.5)
    steering.add_trapezoidal('Small curve',0.1,0.18,0.24,0.45)
    steering.add_triangular('Large curve',0.01,0.11,0.21)
    steering.add_triangular('Straight',0,0,0.1)
    
    speed = FuzzyOutputVariable('Speed', 0, 80, 400) # mm/s
    speed.add_triangular('Very slow', 0, 0, 30)
    speed.add_triangular('Slow', 15, 32, 49)
    speed.add_triangular('Moderate', 30, 52, 74)
    speed.add_triangular('Fast', 65, 80, 80)
    
    system = FuzzySystem()
    system.add_input_variable(steering)
    system.add_output_variable(speed)
    
    system.add_rule(
    		{ 'Steering':'Very small curve'},{'Speed':'Very slow'})
    system.add_rule(
    		{ 'Steering':'Small curve'},{'Speed':'Slow'})
    system.add_rule(
    		{ 'Steering':'Large curve'},{'Speed':'Moderate'})
    system.add_rule(
    		{ 'Steering':'Straight'},{'Speed':'Fast'})

    # OUTPUT EVALUATION
    out_dict = system.evaluate_output(in_dict)
    v_out = np.array(list(out_dict.values())) # - Fuzzy Output Velocity Vector
    
    # force the last point to have null reference velocity
    if i == len(mean_steer)-1:
        v_out = 0 # np.array([0])
    
    # evaluation of the instantaneous acceleration: dt = ds/dv
    t_inst = 2 * ds / abs(v_ref[-1] + v_out) 
    acc_inst = (v_out - v_ref[-1]) / t_inst
    
    # acceleration check
    if abs(acc_inst) > acc_lim: 
        
        if acc_inst > 0: # acceleration
            while acc_inst > acc_lim:       # convergence procedure on acceleration limit
                v_out -= 0.1 # reduction step                    
                t_inst = 2 * ds / abs(v_ref[-1] + v_out)  
                acc_inst = (v_out - v_ref[-1]) / t_inst   
            
            v_ref = np.append(v_ref,v_out)
            t_ref = np.append(t_ref,t_ref[-1] + t_inst)
            acc_ref = np.append(acc_ref,acc_inst)
            
        else: # deceleration
            
            # the last velocity is fixed (maximum velocity admitted by fuzzy logic)
            # previous velocities are decreased to match the decelleration limit
            v_ref = np.append(v_ref,v_out)
            t_ref = np.append(t_ref,t_ref[-1] + t_inst)
            acc_ref = np.append(acc_ref,acc_inst)
            
            back = 0 # counter and exit value of the while cycle
            
            while back >= 0:
                
                t_inst = 2 * ds / abs(v_ref[-2-back] + v_ref[-1-back])
                acc_inst = (v_ref[-1-back] - v_ref[-2-back]) / t_inst
                
                while abs(acc_inst) > acc_lim: # decrease the prev speed to meet the acc value
                    v_ref[-2-back] -= 0.1
                    t_inst = 2 * ds / abs(v_ref[-2-back] + v_ref[-1-back])
                    acc_inst = (v_ref[-1-back] - v_ref[-2-back]) / t_inst
                
                # changing the 2nd last point velocity and t_inst value,
                # update their values [t_inst is the new dt of last step]
                    
                t_back = 2 * ds / abs(v_ref[-3-back] + v_ref[2-back]) # new t_inst of 2nd last step
                
                # time shift due to change in velocity, of the previous time step
                shift_back = t_back - (t_ref[-2-back] - t_ref[-3-back])
                
                # time shift due to change in velocity, of the actual time step
                shift = t_inst - (t_ref[-1-back] - t_ref[-2-back]) + shift_back
                
                t_ref[-1] += shift
                t_ref[-1-back:-1] += shift
                t_ref[-2-back] += shift_back
                
                acc_ref[-1-back] = acc_inst
                acc_ref[-2-back] = (v_ref[-2-back]-v_ref[-3-back]) / t_back
                
                # check that the acc at previous time instant meets the limits 
                if abs(acc_ref[-2-back]) < acc_lim:
                    back = -1
                else:
                    back += 1

    else:
        v_ref = np.append(v_ref,v_out) # velocity vector function of space
        t_ref = np.append(t_ref,t_ref[-1] + t_inst) 
        acc_ref = np.append(acc_ref,acc_inst)

    if i <= len(mean_steer)-2:
        del system

t_tot = t_ref[-1]                   # total runtime of the path execution (DMP)
n_points = int(round(t_tot/dt)) + 1 

# ########################################## #
# 4. TRAJECTORY REFERENCE GENERATION         #
# ########################################## #

# fitting of velocity values with respect to time
f_des = interpolate.interp1d(t_ref,v_ref,kind="linear") 
t_fuzzy = np.linspace(0,t_tot,n_points)
v_des = f_des(t_fuzzy) 

# add some null points to the beginning and the end (useful for DMP framework)
points = np.zeros(int(round(t_start / dt)))
v_des = np.append(points,np.append(v_des,points)) # velocity vector function of time


# new curvilinea abscissa (with velocity reference embedded)
absc_des = np.cumsum(v_des) * dt

# definition of the x and y desired points to be fed into the DMP
x1_des = np.interp(absc_des,absc_teach,y_teach[:,0])
x2_des = np.interp(absc_des,absc_teach,y_teach[:,1])

y_des = np.column_stack((x1_des,x2_des))

# ################################### #
# 5. PATH APPROXIMATION THROUGH DMP   #
# ################################### #

# add the fake points time (beginning and end)
runtime = t_tot + 2*t_start

# total time instants of the simulation (without perturbations, 
# it should complete the path in a shorter time)
timesteps=int(round((runtime+t_extra)/dt))

# Selection of the DMP METHOD (0 is the uncoupled version, 1 is the coupled one)
# IF no perturbations are modelled, uncoupled and coupled work with the same results
dmp_method = 0
    
if dmp_method == 0:
    # Standard WORKS
    print("\n DMP method: standard \n")
    dmp_std = dmp_standard.DMPs(n_dmps=2,n_bfs=nbfs,dt=dt,run_time=runtime) # initialization
    dmp_std.imitate_path(np.transpose(y_des), plot=False)                   # trajectory learning
    y_track, dy_track, ddy_track = dmp_std.rollout(timesteps=timesteps)     # offline trajectory reproduction
elif dmp_method == 1:
    print("\n DMP method: coupled 3\n")
    dmp_coupled3 = dmp_coupled3.DMPs(n_dmps=2,n_bfs=nbfs,dt=dt,run_time=runtime)
    dmp_coupled3.imitate_path(np.transpose(y_des), plot=False)
    y_c, dy_c, ddy_c, y_track, dy_track, ddy_track, error, tau_save = dmp_coupled3.rollout(timesteps=timesteps) # online numerical trajectory reproduction

# #################################### #
# 6. GENERATION OF THE ROBOT INPUT [m] #
# #################################### #

# time vector (DMP time step)
time_DMP = np.linspace(0,runtime+t_extra,len(y_track[:,0]))

# definition of the coordinate values to be fed into the robot (with 1kHz frequency)
f_x = interpolate.interp1d(time_DMP,y_track[:,0],kind="linear")
f_y = interpolate.interp1d(time_DMP,y_track[:,1],kind="linear")
x = (-f_x(np.arange(0,runtime,dt_panda)) + x_off) / 1000 # [m]
y = (-f_y(np.arange(0,runtime,dt_panda)) + y_off) / 1000 # [m]

# definition of the vertical coordinate (imposed constant)
z = (np.zeros(len(x)) + z_off) / 1000 # [m]

x = np.append(x,x[-1] * np.ones(3000))
y = np.append(y,y[-1] * np.ones(3000))
z = np.append(z,np.linspace(z_off/1000,z_off/1000 + 0.08,3000))

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

# ################# #
# 7. SILICONE FLOW  #
# ################# #

v_x_fin = np.diff(x) / dt_panda
v_y_fin = np.diff(y) / dt_panda
v_x_fin = np.append(v_x_fin,v_x_fin[-1])
v_y_fin = np.append(v_y_fin,v_y_fin[-1])

v_final = np.sqrt(v_x_fin**2 + v_y_fin**2)

# EXPERIMENTAL DUTY CYCLE - VELOCITY RELATIONSHIP
duty_ref = np.array([0.02, 0.08, 0.22, 0.80, 0.9, 1.0]) # [%] 
ee_v_ref = np.array([0.0, 20.0, 35.0, 50.0, 65.0, 80.0]) / 1000 # [mm]

f_duty = interpolate.interp1d(ee_v_ref,duty_ref,kind="slinear")
duty = f_duty(v_final)

# ############# #
# 8. ROS NODE   #
# ############# #

import rospy
import serial
import time
from geometry_msgs.msg import PoseStamped

ard = serial.Serial('/dev/ttyUSB0',115200,timeout=0)
time.sleep(2)
ard.write('40\n') #ard.write('99\n')
time.sleep(0.8)
count = 0 # we don't give serial values each time step
pwm = 1

try:
    pub = rospy.Publisher('/DMP_pose', PoseStamped, queue_size=1)
    rospy.init_node('DMP_planner', anonymous=True)
    rate = rospy.Rate(1/dt_panda) # 1kHz
    
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
        	
            
                if i < (len(x)-4000):
                    if pwm is 1:
    		              ard.write(str(40))
    		              ard.write('\n')
    		              if count > duty[i] * 200:
    		                  pwm = 0
    		                  count = 0
                    else:
    		              ard.write(str(28))
    		              ard.write('\n')
    		              if count > (1-duty[i]) * 200:
    		                  pwm = 1
    		                  count = 0    
                    count += 1
                else:
                    ard.write('0\n')
                    
                pub.publish(p)
                rate.sleep()

            task_ended=True
            ard.write('0\n')
            ard.close()

        pub.publish(p)
        rate.sleep()

except rospy.ROSInterruptException:
    pass
