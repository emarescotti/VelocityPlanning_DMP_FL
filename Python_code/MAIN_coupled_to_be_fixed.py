#! /usr/bin/env python
# SPDX-License-Identifier: AGPL-3.0-or-later

import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate

from steeringAngle import steeringAngle

from fuzzy_system.fuzzy_variable_output import FuzzyOutputVariable
from fuzzy_system.fuzzy_variable_input import FuzzyInputVariable
from fuzzy_system.fuzzy_system import FuzzySystem

import dmp_coupled3

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

# ###################
# PARAMETERS TO SET #
# ###################

nozzle_dia = 8.0 # [mm] - diameter of the silicon nozzle

body = 5.0       # [ODD NUM] - length of the "car"
clip = 0.49      # clipping of the steering parameter (avoid too hard directional changes)
width_win = 7    # [ODD NUM] - width of the moving window mean

t_start = 0.2    # [s]       - time for let DMP start and end
t_extra = 15.0   # [s]       - extra simulation time

nbfs = 6000      #           - number of basis function for DMP

acc_lim = 30.0   # [mm/s^2]  - limit acceleration of the robot

dt = 1e-3        # [s] - time step for the DMP

# definition of the offset values (txt file generation)
x_off =  410.11 # 400.0 # [mm]
y_off = -479.90 # 50.0  # [mm]
z_off =  419.0  # 300.0 # [mm]

# gun inclination
inclination = 0 * np.pi / 180 # [rad] - inclination angle of the gun

# desired trajectory (from file) - RUN BEFORE THE OTHER CODE TO GENERATE THE FILE
y_teach = np.transpose(np.load("/home/franka/elia_ws/src/bpkg/scripts/test1.npz",'r+b')['arr_0']) # mm

# ####################################################
# INITIAL REFITTING + ANGLE AND STEERING COMPUTATION #
# ####################################################

# check that initial and final points are different [OTHERWISE DMP MIGHT FAIL]
for i in range(len(y_teach[0,:])):
    if y_teach[0,i] == y_teach[-1,i]: y_teach[-1,i] += 0.05 # [mm]

absc_teach = np.array([0]) # curvilinea abscissa of the teached path
for i in range(len(y_teach[:,0])-1):
    
    absc_teach = np.append(absc_teach,absc_teach[-1] + np.sqrt(
        (y_teach[i+1,0]-y_teach[i,0])**2 + (y_teach[i+1,1]-y_teach[i,1])**2))
    
    # check that 2 consecutive abscissa evaluations are not equal:
    # this don't allow to properly use the interpolation tool
    if absc_teach[-1] == absc_teach[-2]: absc_teach[-1] += 1e-5
    
# notice that the final point of the curvilinea abscissa is also the path length
len_teach = absc_teach[-1]
print("\n The teached path length is: ",np.round(len_teach,2)," mm") 

pt_density = 4/nozzle_dia # [pts/mm] - initial refitting density (4 pts per diameter)
ds = nozzle_dia/4         # [mm/pts] - spatial-step of the v_ref points

n_new = int(round(len_teach*pt_density)) # total number of points wished after refitting

# we want to refit the points in order to ensure that they are EQUALLY spaced 
# along the curvilinea abscissa (x1 = cartesian X, x2 = cartesian Y)

# EQUAL SPACING ALONG ABSCISSA, no time
x1 = np.interp(np.linspace(0,len_teach,n_new),absc_teach,y_teach[:,0])
x2 = np.interp(np.linspace(0,len_teach,n_new),absc_teach,y_teach[:,1])

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

# #########################################
# FUZZY LOGIC + PATH REFERENCE GENERATION #
###########################################

t_ref = v_ref = acc_ref = np.array([0])

for i in range(1,len(mean_steer)):
    
    # INPUT
    in_dict = dict({'Steering':mean_steer[i]}) # mean steering value
    
    # FUZZY SYSTEM
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
    v_out = np.array(list(out_dict.values()))
    
    # we force the last point to have null reference velocity
    if i == len(mean_steer)-1:
        v_out = 0 # np.array([0])
    
    # evaluation of the instantaneous acceleration
    t_inst = 2 * ds / abs(v_ref[-1] + v_out) # 2 is for the mean at the denominator
    acc_inst = (v_out - v_ref[-1]) / t_inst

    if abs(acc_inst) > acc_lim: # we are not able to perform sudden changes in velocity
        
        if acc_inst > 0: # we can't accelerate so fast
            while acc_inst > acc_lim:
                v_out -= 0.1                              # we can speed up the 
                t_inst = 2 * ds / abs(v_ref[-1] + v_out)  # code decreasing the
                acc_inst = (v_out - v_ref[-1]) / t_inst   # reduction step
            
            v_ref = np.append(v_ref,v_out)
            t_ref = np.append(t_ref,t_ref[-1] + t_inst)
            acc_ref = np.append(acc_ref,acc_inst)
            
        else: # we can't decelerate so fast
            
            # firstly the last velocity is fixed (we impose the lowest one)
            # then we decrease accordingly the previous velocities
            v_ref = np.append(v_ref,v_out)
            t_ref = np.append(t_ref,t_ref[-1] + t_inst)
            acc_ref = np.append(acc_ref,acc_inst)
            
            back = 0 # both counter and exit of the while cycle
            
            while back >= 0:
                
                t_inst = 2 * ds / abs(v_ref[-2-back] + v_ref[-1-back])
                acc_inst = (v_ref[-1-back] - v_ref[-2-back]) / t_inst
                
                while abs(acc_inst) > acc_lim: # decrease the prev speed to meet the acc value
                    v_ref[-2-back] -= 0.1
                    t_inst = 2 * ds / abs(v_ref[-2-back] + v_ref[-1-back])
                    acc_inst = (v_ref[-1-back] - v_ref[-2-back]) / t_inst
                
                # now we have to consider that changing the 2nd last velocity value,
                # the lasts 2 time instants will be different (longer), so we have
                # to update their values. [t_inst is the new dt of last step]
                    
                t_back = 2 * ds / abs(v_ref[-3-back] + v_ref[2-back]) # new dt of 2nd last step
                
                # time shift due to change in velocity, of the previous time step
                shift_back = t_back - (t_ref[-2-back] - t_ref[-3-back])
                
                # time shift due to change in velocity, of the actual time step
                shift = t_inst - (t_ref[-1-back] - t_ref[-2-back]) + shift_back
                
                t_ref[-1] += shift
                t_ref[-1-back:-1] += shift
                t_ref[-2-back] += shift_back
                
                acc_ref[-1-back] = acc_inst
                acc_ref[-2-back] = (v_ref[-2-back]-v_ref[-3-back]) / t_back
                
                # check that the acc at preeStamped' object has no attribute 'position'vious time instant meets the limits 
                if abs(acc_ref[-2-back]) < acc_lim:
                    back = -1
                else:
                    back += 1

    else:
        v_ref = np.append(v_ref,v_out)
        t_ref = np.append(t_ref,t_ref[-1] + t_inst) 
        acc_ref = np.append(acc_ref,acc_inst)

    if i <= len(mean_steer)-2:
        del system

# we want to create the desired curvilinea abscissa to be given to the DMP
t_tot = t_ref[-1]               # total runtime of the path execution (DMP)
n_points = int(round(t_tot/dt)) + 1 

# fitting of velocity values with respect to time
f_des = interpolate.interp1d(t_ref,v_ref,kind="linear") # THAT'S A FUNCTION
t_fuzzy = np.linspace(0,t_tot,n_points)
v_des = f_des(t_fuzzy)

# add some points to the beginning and the end (useful for DMP framework)
points = np.zeros(int(round(t_start / dt)))
v_des = np.append(points,np.append(v_des,points))


# new curvilinea abscissa (with velocity reference embeddded)
absc_des = np.cumsum(v_des) * dt

# define the x and y desired points to be fed into the DMP
x1_des = np.interp(absc_des,absc_teach,y_teach[:,0])
x2_des = np.interp(absc_des,absc_teach,y_teach[:,1])

y_des = np.column_stack((x1_des,x2_des))

# #########################
# PATH APPROX THROUGH DMP #
###########################

# add the time relative to the fake points (beginning and end)
runtime = t_tot + 2*t_start

# total time instants of the simulation (without perturbation, it should complete 
# the path in a shorter time)
timesteps=int(round((runtime+t_extra)/dt))

dmp_coupled3 = dmp_coupled3.DMPs(n_dmps=2,n_bfs=nbfs,dt=dt,run_time=runtime)
dmp_coupled3.imitate_path(np.transpose(y_des), plot=False)
dmp_coupled3.reset_state()

# set up tracking vectors
ytrack   = np.zeros(dmp_coupled3.n_dmps)

yr   = np.zeros(dmp_coupled3.n_dmps)
dyr  = np.zeros(dmp_coupled3.n_dmps)
ddyr = np.zeros(dmp_coupled3.n_dmps)

y_r = np.zeros((timesteps, dmp_coupled3.n_dmps))
y_a = np.zeros((timesteps, dmp_coupled3.n_dmps))
y_c = np.zeros((timesteps, dmp_coupled3.n_dmps))

yc   = np.zeros(dmp_coupled3.n_dmps)
dyc  = np.zeros(dmp_coupled3.n_dmps)
ddyc = np.zeros(dmp_coupled3.n_dmps)

error = np.array([0])
error_now = 0.0
tau_adapt = 0.0
tau_save = np.zeros(timesteps)

# ###################
# ROTATIONAL MATRIX #
# ###################

# Euler angles ZYX to quaternions
z_rot = 0.5880026 # [rad]
y_rot = np.pi + inclination # [rad]

# definition of the vertical coordinate
z = z_off / 1000 # [m]

[qxi,qyi,qzi,qwi] = euler_to_quaternion(z_rot, y_rot, 0) # Z Y X

# ###############
# SILICONE FLOW #
# ###############
"""
v_x_fin = np.diff(x) / dt
v_y_fin = np.diff(y) / dt
v_x_fin = np.append(v_x_fin,v_x_fin[-1])
v_y_fin = np.append(v_y_fin,v_y_fin[-1])

v_final = np.sqrt(v_x_fin**2 + v_y_fin**2)

duty_ref = np.array([0.02, 0.08, 0.22, 0.80, 0.9, 1.0]) # [%] - EXPERIMENTAL
ee_v_ref = np.array([0.0, 20.0, 35.0, 50.0, 65.0, 80.0]) / 1000 # [mm]

f_duty = interpolate.interp1d(ee_v_ref,duty_ref,kind="slinear") # THAT'S A FUNCTION
duty = f_duty(v_final)
"""
# ##########
# ROS NODE #
# ##########

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
"""
import serial
import time
ard = serial.Serial('/dev/ttyUSB0',115200,timeout=0)
time.sleep(2)
ard.write('40\n') #ard.write('99\n')
time.sleep(0.8)
count = 0 # we don't give serial values each time step
pwm = 1
"""
try:
    p = PoseStamped()
    
    def callback(Pose):
        global s
        s = Pose
        
    pub = rospy.Publisher('/DMP_pose', PoseStamped, queue_size=1)
    sub = rospy.Subscriber('/franka_ee_pose', Pose, callback)
    
    rospy.init_node('DMP_planner', anonymous=True)
    rate = rospy.Rate(1/dt) # 100 Hz
    
    task_ended = False

    while not rospy.is_shutdown():
        
        if not task_ended:
            for t in range(timesteps):
                
                ytrack[0] = x_off - s.position.x * 1000.0 # [mm]
                ytrack[1] = y_off - s.position.y * 1000.0 # [mm]
                #y_track[t,:] = y_r
                
                # run and record timestep
                yc, dyc, ddyc, ddyr, tau_adapt = dmp_coupled3.step(ya=ytrack,error=error_now)
                
                for i in range(dmp_coupled3.n_dmps):
                    dyr[i] += ddyr[i] * dt
                    yr[i] += dyr[i] * dt
                    
                error_now += dmp_coupled3.ae * (np.sqrt((ytrack[0]-yc[0])**2 + (ytrack[1]-yc[1])**2) - error_now) * dt
                error = np.append(error,error_now)
                
                x = (-yr[0] + x_off) / 1000.0 # [m]
                y = (-yr[1] + y_off) / 1000.0 # [m]
                
                y_r[t,:] = yr
                y_a[t,:] = ytrack
                y_c[t,:] = yc
                tau_save[t] = tau_adapt

                p.pose.position.x = round(x,5);
                p.pose.position.y = round(y,5);
                p.pose.position.z = round(z,5);
                p.pose.orientation.x = round(qxi,6);
                p.pose.orientation.y = round(qyi,6);
                p.pose.orientation.z = round(qzi,6);
                p.pose.orientation.w = round(qwi,6);
                """
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
                """
                pub.publish(p)
                rate.sleep()

            task_ended=True
            
            np.savez('/home/franka/elia_ws/src/bpkg/scripts/COUPLED',np.column_stack((y_a,y_r,y_c,tau_save)))
            
            """
            ard.write('0\n')
            ard.close()
            """
        pub.publish(p)
        rate.sleep()

except rospy.ROSInterruptException:
    pass
