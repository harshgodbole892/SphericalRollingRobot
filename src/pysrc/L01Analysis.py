#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun  7 19:56:14 2020

@author: Harsh Godbole
"""
import os
os.chdir("../../")


from src.pysrc.L01CppRunner import L01CppRunner as CppR
from src.pysrc.L01LoadData  import L01DataArray as DatArr
from src.pysrc.L01Animate   import L01Animate

Build   = 1
Run     = 1
Load    = 1
Plot    = 1
Animate = 0

# Create Runner Obj
runner = CppR()
    
# Created an instance of CPP runner to build CPP solution: 
if (Build == 1) : 
    print("C++ : Build Started")
    runner.Build()
    print("C++ : Build Complete")

if (Run == 1) :     
    print("C++ : Executing Test Started")
    runner.RunTest()
    print("C++ : Data generation complete")

# Plot script here: 
if (Load == 1):
    print("Python: Loading results")
    op = DatArr()
    print("Python: Loading complete!")

    if (Plot == 1) :
        print("Python: Plotting results")
        op.PlotCustom()
        #op.PlotAll()
        print("Python: Plotting complete!")

    if (Animate == 1): 
        # Animation script
        print("Python: Generating animation gif")
        L01Animate.AnimateCable1D(op)
        print("Python: Animation Ready!")
