#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun  7 19:56:14 2020

@author: Harsh Godbole
"""
import os
import numpy             as np
import matplotlib.pyplot as plt 
import config.L01Env     as Env


# Plot script here: 
class L01DataArray: 
    
    data = {} 
    
    def populateResults(self):
        
        # initialize
        resultList = []
        
        # Get names of all result files: 
        for file in os.listdir(Env.GenDir):
            if file.endswith(".txt"): 
                resultList.append(file[:-4])
            
        # Populate results
            
        for result in resultList:
            self.data[result] = np.loadtxt(Env.GenDir + result +".txt")
                           
        
        
    def namestr(obj, namespace):
        return [name for name in namespace if namespace[name] is obj]

    def Plot2D(self,var1,var2,name1,name2):
        f = plt.figure()
        plt.plot(var1,var2)
        plt.xlabel(name1)
        plt.ylabel(name2)
        plt.grid(True)
        plt.show()
        return f
        
    # Constructor loads all data from the generated folder
    def __init__(self):

        # populate results
        self.populateResults();
        
    # Generic function to plot all time histories of variables: 
    def PlotAll(self):
                
        for  variable in self.data:
            if (len(self.data[variable].shape) < 2):
                self.Plot2D(self.data["t"], self.data[variable] , "time", variable)
                       
    #Specific plots: 
    def PlotCustom(self):
        
       # pltWin = plotWindow()
          
        plt.figure(1)
        plt.grid(True)
        plt.plot(self.data["t"], self.data["H"], color='k')
        plt.ylabel(" Hameltonian")
        plt.xlabel("time")
        plt.title("Hameltonian")
        
        plt.figure(2)
        plt.grid(True)
        plt.plot(self.data["t"], self.data["del_E"], color='k')
        plt.ylabel(" Del Energy with external force")
        plt.xlabel("time")
        plt.title("Del E")
        
        plt.figure(3)
        plt.grid(True)
        plt.plot(self.data["t"], self.data["det_C_si"], color='k')
        plt.plot(self.data["t"], self.data["det_C_p1i"], color='r')
        plt.plot(self.data["t"], self.data["det_C_p2i"], color='c')
        plt.ylabel(" Determinant of DCMs")
        plt.xlabel("time")
        plt.title("DCMs")
        
        plt.figure(4)
        plt.grid(True)
        plt.plot(self.data["t"], self.data["v_i_si_i"][0], color='k')
        plt.plot(self.data["t"], self.data["v_i_si_i"][1], color='r')
        plt.plot(self.data["t"], self.data["v_i_si_i"][2], color='c')
        plt.ylabel(" V_i_si_i")
        plt.xlabel("time")
        plt.title("V_i_si_i 1")
        
        
        plt.figure(5)
        plt.grid(True)
        plt.plot(self.data["t"], self.data["omega_i_si"][0], color='k')
        plt.plot(self.data["t"], self.data["omega_i_p1i"][0], color='r')
        plt.plot(self.data["t"], self.data["omega_i_p2i"][0], color='c')
        plt.ylabel(" Omega_i_xi [0]")
        plt.xlabel("time")   
        plt.title("Omega_i_xi 0")
                
        
        plt.figure(6)
        plt.grid(True)
        plt.plot(self.data["t"], self.data["omega_i_si"][1], color='k')
        plt.plot(self.data["t"], self.data["omega_i_p1i"][1], color='r')
        plt.plot(self.data["t"], self.data["omega_i_p2i"][1], color='c')
        plt.ylabel(" Omega_i_xi [1]")
        plt.xlabel("time")  
        plt.title("Omega_i_xi 1")
        
        plt.figure(7)
        plt.grid(True)
        plt.plot(self.data["t"], self.data["omega_i_si"][2], color='k')
        plt.plot(self.data["t"], self.data["omega_i_p1i"][2], color='r')
        plt.plot(self.data["t"], self.data["omega_i_p2i"][2], color='c')
        plt.ylabel(" Omega_i_xi [2]")
        plt.xlabel("time")  
        plt.title("Omega_i_xi 2")
        
        plt.show()