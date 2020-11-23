#! /usr/bin/env python
# SPDX-License-Identifier: AGPL-3.0-or-later

import numpy as np
from cs import CanonicalSystem

# EDITED  version specific for Discrete DMP

class DMPs(object):

    def __init__(self, n_dmps, n_bfs, dt=0.001, run_time=6, y0=0, goal=1, 
                 w=None, ay=None, by=None, **kwargs):
        
        self.n_dmps = n_dmps
        self.n_bfs = n_bfs
        self.dt = dt
        self.run_time = run_time
        
        if isinstance(y0, (int, float)):
            y0 = np.ones(self.n_dmps) * y0
        self.y0 = y0
        
        if isinstance(goal, (int, float)):
            goal = np.ones(self.n_dmps) * goal
        self.goal = goal
        
        if w is None:
            # default is f = 0 so all the weights are null
            w = np.zeros((self.n_dmps, self.n_bfs))
        self.w = w 
        
        self.ay = np.ones(n_dmps) * 25.0 if ay is None else ay  # Schaal 2012 (*25)
        self.by = self.ay / 4.0 if by is None else by   # Schaal 2012
                
        # set up the CS
        self.cs = CanonicalSystem(dt=self.dt, run_time=self.run_time, **kwargs)
        self.timesteps = self.cs.timesteps

        # set up the DMP system
        self.reset_state() # this is a different reset with respect to CS (it's defined below)

        self.gen_centers()

        # set variance of Gaussian basis functions
        self.h = np.ones(self.n_bfs) * self.n_bfs ** 1.5 / self.c**1.8 / self.cs.ax

        self.check_offset()
        
    def gen_centers(self):
        
	# desired activations throughout time
        des_c = np.linspace(0, self.cs.run_time, self.n_bfs)

        self.c = np.ones(len(des_c))
        for n in range(len(des_c)):
            # finding x for desired times t
            self.c[n] = np.exp(-self.cs.ax * des_c[n]) #it should be /tau

    def gen_psi(self, x):

        if isinstance(x, np.ndarray):
            x = x[:, None]
        return np.exp(-self.h * (x - self.c) ** 2)

    def gen_weights(self, f_target):

        # calculate x and psi
        x_track = self.cs.rollout()
        psi_track = self.gen_psi(x_track)

        # efficiently calculate BF weights using weighted linear regression
        self.w = np.zeros((self.n_dmps, self.n_bfs))
        for d in range(self.n_dmps):
            # spatial scaling term
            k = self.goal[d] - self.y0[d]
            for b in range(self.n_bfs):
                numer = np.sum(x_track * psi_track[:, b] * f_target[:, d])
                denom = np.sum(x_track ** 2 * psi_track[:, b])
                self.w[d, b] = numer / denom
              # if abs(k) > 1e-5: THIS HAS BEEN REMOVED 
                self.w[d, b] /= k

        self.w = np.nan_to_num(self.w)

    def imitate_path(self, y_des, plot=False):

        # set initial state and goal
        if y_des.ndim == 1:
            y_des = y_des.reshape(1, len(y_des)) # to be sure it's a row
        self.y0 = y_des[:, 0].copy() # dynamic link between a vector and its copy
        self.y_des = y_des.copy()
        self.goal = np.copy(y_des[:, -1])
        
        # self.check_offset()

        # generate function to interpolate the desired trajectory
        import scipy.interpolate

        path = np.zeros((self.n_dmps, self.timesteps))
        x = np.linspace(0, self.cs.run_time, y_des.shape[1]) #number of columns of Y
        for d in range(self.n_dmps):
            path_gen = scipy.interpolate.interp1d(x, y_des[d]) #path_gen is a function (not an array)
            for t in range(self.timesteps):
                path[d, t] = path_gen(t * self.dt)
        y_des = path #this changes also the other linked array with function "copy"

        # calculate velocity of y_des with central differences
        dy_des = np.gradient(y_des, axis=1) / self.dt

        # calculate acceleration of y_des with central differences
        ddy_des = np.gradient(dy_des, axis=1) / self.dt

        f_target = np.zeros((y_des.shape[1], self.n_dmps)) #row: columns of y_des. col = n_dmps
        # find the force required to move along this trajectory
        for d in range(self.n_dmps):
            f_target[:, d] = ddy_des[d] - self.ay[d] * (
                self.by[d] * (self.goal[d] - y_des[d]) - dy_des[d]
            )

        # efficiently generate weights to realize f_target
        self.gen_weights(f_target)
        
        if plot is True:
            # plot the basis function activations
            import matplotlib.pyplot as plt

            plt.figure()
            plt.subplot(211)
            psi_track = self.gen_psi(self.cs.rollout())
            plt.plot(psi_track)
            plt.title("basis functions")

            # plot the desired forcing function vs approx
            for ii in range(self.n_dmps):
                plt.subplot(2, self.n_dmps, self.n_dmps + 1 + ii)
                plt.plot(f_target[:, ii], "o--", label="computed f_target of DoF %i" % ii)
            for ii in range(self.n_dmps):
                plt.subplot(2, self.n_dmps, self.n_dmps + 1 + ii)
                print("w shape: ", self.w.shape)
                plt.plot(
                    np.sum(psi_track * self.w[ii], axis=1) * self.dt,
                    "o-",label="sum(weight * psi %i)" % ii)
                plt.legend()
            plt.title("DMP forcing function")
            plt.tight_layout()
            plt.get_current_fig_manager().window.showMaximized()
            plt.show()

        self.reset_state()
        return y_des

    def step(self,ya,error=0.0,tau=1.0):

        self.ae = 5.0
        kp = 35.0 # 35.0
        kv = 8.0 # 10.0
        kc = 0.5 # #(original value:10000)
        tau = 1.0
        
        tau_adapt = tau*(1+(kc*error**2))
        
        # run canonical system

        # generate basis function activation
        psi = self.gen_psi(self.cs.x)
        
        for d in range(self.n_dmps):

            self.dya[d] = (ya[d] - self.ya_old[d]) / self.dt
            self.ya_old[d] = ya[d]
            
            # Feed Forward Control on ACTUAL ROBOT MOVEMENT
            self.ddyr[d] = kp*(self.yc[d]-ya[d]) + kv*(self.dyc[d]-self.dya[d]) + self.ddyc[d];
            
            # generate the forcing term
            f = self.cs.x*(self.goal[d]-self.y0[d])*(np.dot(psi,self.w[d]))/np.sum(psi)
            #f = np.nan_to_num(f)
            
            # z coupling
            self.dz[d] = 1/tau_adapt * (self.ay[d] * (self.by[d] * (self.goal[d]-self.yc[d])- self.z[d]) + f)

            self.z[d] += self.dz[d] * self.dt
            
            self.dyc[d] = self.z[d] #/ tau_adapt # BISOGNA CAPIRE SE GIUSTO O NO
        
            self.ddyc[d]= (self.dz[d] * tau_adapt - tau*self.z[d]*2*kc*error*(self.ae*(ya[d]-self.yc[d]-error)))/tau_adapt**2
            
            self.yc[d] += self.dyc[d] * self.dt
            
        _ = self.cs.step(tau=tau_adapt) 
        
        return self.yc, self.dyc, self.ddyc, self.ddyr, tau_adapt
    
    def check_offset(self):

        for d in range(self.n_dmps):
            if abs(self.y0[d] - self.goal[d]) < 1e-2: 
                self.goal[d] += 1e-2
                
    def reset_state(self):

        self.ya_old = self.y0.copy()
        self.dya = np.zeros(self.n_dmps)
        
        self.ddyr = np.zeros(self.n_dmps)
        
        self.yc = self.y0.copy() #this fix the value of y (if y changes, y0 doesn't change)
        self.dyc = np.zeros(self.n_dmps)
        self.ddyc = np.zeros(self.n_dmps)
        
        self.cs.reset_state()
        
        self.dz = np.zeros(self.n_dmps)
        self.z = np.zeros(self.n_dmps)
