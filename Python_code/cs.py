#!/usr/bin/env python
# SPDX-License-Identifier: AGPL-3.0-or-later

import numpy as np

class CanonicalSystem:
    def __init__(self, dt, ax=1.0, run_time = 6):
        
        self.ax = ax
        self.run_time = run_time
        self.dt = dt
        self.timesteps = int(self.run_time / self.dt) + 1 # +1 inserted by us 

        self.reset_state()

    def rollout(self, **kwargs):

        if "tau" in kwargs: # tau is the time flexibility constant 
            timesteps = int(self.timesteps / kwargs["tau"])
        else:
            timesteps = self.timesteps
            
        self.x_track = np.zeros(timesteps)
        self.reset_state()
        
        for t in range(timesteps): #for t=1:timestep
            self.x_track[t] = self.x
            self.step(**kwargs)

        return self.x_track

    def reset_state(self):
        self.x = 1.0

    def step(self, tau=1.0, error_coupling=1.0):

        self.x += (-self.ax * self.x * error_coupling) / tau * self.dt
        return self.x
