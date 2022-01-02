#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jun 13 00:46:05 2020

@author: McGillDocs
"""

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
plt.style.use('seaborn-pastel')

class L01Animate:
    
    # Constructor
    def __init__(self, LdaObj):
        
        # Animation Parameters: 
        self.frames = 500
        
        # Animation initializations
        self.fig = plt.figure()
        self.ax  = plt.axes(xlim=(-0.2, 0.2), ylim=(-0.2, 0.2))
        
        # Animated Objects
        self.mass   = Circle((0, 0), 0.005, color='r', alpha=0.3)
        self.mass_e = Circle((0, 0), 0.005, color='g', alpha=0.3)
        
        
        # Resize variables for correct animation scaling: 
        self.rho     = LdaObj.rho
        self.rho_des = LdaObj.rho_des 
        self.m       = LdaObj.M_rho_act/100
        self.m_e     = LdaObj.M_rho_e/100
        
    
    # Function to Initialize animation: 
    def FrameInit(self):
        mp1      = self.ax.add_artist(self.mass)
        mp2      = self.ax.add_artist(self.mass_e)
        return mp1,mp2
    
    # Function to update each frame in animation: 
    def FrameUpdate(self,i):
        
        k = i * int(np.size(self.rho)/self.frames)
        
        # Update Mass 1:
        self.mass.center   = self.rho[k], 0
        self.mass.set_radius(self.m[k])
        
        # Update Mass 2: 
        self.mass_e.center = self.rho_des[k], 0
        self.mass_e.set_radius(self.m_e[k])
        
        # Circle objects returned to animation function:
        mp1 = self.ax.add_artist(self.mass)
        mp2 = self.ax.add_artist(self.mass_e)
        
        return mp1,mp2
    
    def AnimateCable1D(LdaObj):
        
        ani = L01Animate(LdaObj)
        
        anim = FuncAnimation(ani.fig, ani.FrameUpdate, init_func=ani.FrameInit,
                              frames=ani.frames, interval=120, blit=True)

        anim.save('L01AdaptiveControllerStepWave.gif', writer='imagemagick')