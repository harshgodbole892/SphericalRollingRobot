#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun  7 19:56:14 2020

@author: Harsh Godbole
"""
import os
import subprocess as sp 
import config.L01Env as Env

class L01CppRunner :
    
    # Constructor:
    def __init__(self):
        self.rebuild = 1
    
    # Build solution: 
    def Build(self):
        
        # Change operating dir to the Project home dir:
        os.chdir(Env.ProjectHomeDir) 

        # Check Rebuild: 
        if (self.rebuild == 1):
            sp.call(["make", "clean"])
        sp.call(["make"])
   
    
    # Run generated CPP file
    def RunTest(self): 
        sp.call("./L01Runfile.sh")
