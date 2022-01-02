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

#ifndef LO1MODEL_H
#define LO1MODEL_H

// Forward Declarations:

class constants;

//1. Structure of Constants
class SphericalRollingRobot
{
public:
    // Variables:
    vec c_bar_si_1
    ,   c_bar_si_2
    ,   c_bar_si_3;

    mat C_si;
                 
    vec c_bar_p1i_1
    ,   c_bar_p1i_2
    ,   c_bar_p1i_3_1
    ,   c_bar_p1i_3_2
    ,   c_bar_p1i_3 ;

    mat C_p1i;

    vec c_bar_p2i_1
    ,   c_bar_p2i_2
    ,   c_bar_p2i_3_1
    ,   c_bar_p2i_3_2
    ,   c_bar_p2i_3 ;

    mat C_p2i;

    vec omega_s_si ;
    double   omega_p1_p1s_2;
    double   omega_p2_p2s_2;

    vec r_i_si ;

    double H;

    vec omega_p1_p1s
    ,   omega_p2_p2s;

    vec omega_p1_p1i
    ,   omega_p2_p2i;

    vec omega_cap ;

    // Kinematics Variables:
    vec c_bar_dot_si_1
    ,   c_bar_dot_si_2
    ,   c_bar_dot_p1i_1
    ,   c_bar_dot_p1i_2
    ,   c_bar_dot_p2i_1
    ,   c_bar_dot_p2i_2 ;

    // Velocities of wind and sphere
    vec v_i_si_i
    ,   v_i_ws_i  ;

    // Translational and attitude dynamics
    mat M ;

    // Kinematic transformation matrix
    mat pi;
    mat pi_dot;
    mat delta_bar_2_T;
    mat M_dot ;
    mat delta_3_T ;

    // external eneralized force
    mat F_ext ;
    mat F_ext_total ;

    mat M_eq ;

    // Omega Cap state vector
    vec omega_cap_dot ;

    // Energy rate:
    double H_dot;
    vec x_dot ;
    
    // Constructor:
    SphericalRollingRobot();
    
    // Populate inputs:
    void PopulateInputs(vec x, constants cst);
    void SolveKinematics(constants cst);
    void CalculateXDot();

};

#endif
