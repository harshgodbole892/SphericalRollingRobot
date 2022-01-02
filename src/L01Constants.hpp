/*
 <<<<<<<<<<<<<<< Multi-body Dynamics Simualtion, Estimation and Control Lab - McGill University >>>>>>>>>>>>>>
 
 
 Hierarchy     : Cable Driven Parallel Robots
 Model         : 1 DOF Flexible Cable
 Function Name : Example12a
 Function Type : Dynamic Simulation
 
 Description   : The following function contains dynamic sumulation of a
                 single DOF constrained CDPR.
 
 Example-L-0-1 is named using the following convention:
 L - Lumped-mass model type
 0 - Dynamics simulation
 1 - Variable mass/stiffness lumped-mass method.
 
 Revisions: 
 
 2017-09-06: Template creation
 
 Function Details:
 
 Information on addtional functionality can be added here.
 
 References:
 
 [1] Dynamic Modelling and Control of Cable-actuated systems, Harsh Godbole, Master's Thesis, McGill University, 2017.
 
 Armadillo documentation is available at:
 http://arma.sourceforge.net/docs.html
 
 Tempelate by Harsh Godbole.
 Reference credits to Dr. James Richard Forbes and Ryan Caverly.
 
 */

#include <iostream>
#include <armadillo>

//#include <conio.h>

#include <vector>
#include <fstream>
#include <stdio.h>

using namespace std;
using namespace arma;

#ifndef LO1CONSTANTS_H
#define LO1CONSTANTS_H

//1. Structure of Constants
class constants
{
public:
    
    // Exe Globals:
    string GenDir;
    string ParametersDir;
    
    // Gravitational Constants:
    vec    g_i; //gravitational accel, m/s/s
    double   g;

    // Structural constants for sphere:

    // Pendulum constants:
    double m_p1 ;
    double m_p2 ;

    vec r_s_c1s;
    vec r_s_c2s;

    vec c_p1;
    vec c_p2;

    mat J_p1;
    mat J_p2;

    //Sphere Constants:
    double m_s ;
    mat    J_s;
    double   r ;
    
    // External Forces:
    
    // Wind force
    vec    v_i_wi_i;
    double br ;
    double bs ;

    double c_f;

    double b_p1;
    double b_p2;
    
    // Generic Constants:
    vec unit_col_1;
    vec unit_col_2;
    vec unit_col_3;
    
    // Constructor:
    constants(std::string EnvGenDir, std::string EnvParametersDir);
    
    // Internal functions:
    void initializeGlobalConstants(std::string EnvGenDir, std::string EnvParametersDir);
    void initializeModelConstants();
    void saveConstants();
    void loadConstants();

};

#endif
