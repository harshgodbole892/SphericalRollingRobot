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


#include "L01Constants.hpp"

//1. Structure of Constants
constants::constants(std::string EnvGenDir)
{
    // Global Home Path for generated files:
    GenDir = EnvGenDir;

    // Gravitational Constants:
    g_i << 0.0 << 0.0 << -9.81; //gravitational accel, m/s/s
    g = 9.81;

    // Structural constants for sphere:

    // Pendulum constants:
    m_p1 = 5.5;
    m_p2 = 5.5;

    r_s_c1s << 0  <<  0.5 << 0;
    r_s_c2s << 0  << -0.5 << 0;
     
    //r_s_c1s << 0 <<  0.17 << 0;
    //r_s_c2s << 0 << -0.17 << 0;

    //r_s_c1s << 0 << 0 << 0;
    //r_s_c2s << 0 << 0 << 0;

    c_p1 << 0 << 0 << -3.675 ;
    c_p2 << 0 << 0 << -3.675 ;

    // c_p1 << 0 << 0 << -4.725;
    // c_p2 << 0 << 0 << -4.725;

    J_p1 <<    2.5337 << 0       << 0        << endr
            << 0      << 2.5337  << 0        << endr
            << 0      << 0       << 0.002135 << endr;
    
    J_p2 <<    2.5337 << 0       << 0        << endr
            << 0      << 2.5337  << 0        << endr
            << 0      << 0       << 0.002135 << endr;
    
    /*
     J_p1 <<   4.19   << 0       << 0        << endr
            << 0      << 4.19    << 0        << endr
            << 0      << 0       << 0.005335 << endr;
     J_p2 <<   4.19   << 0       << 0        << endr
            << 0      << 4.19    << 0        << endr
            << 0      << 0       << 0.005335 << endr;
    */
    
    //Sphere Constants:
    m_s = 15;
    
    J_s  << 7.567  << 0       << 0        << endr
         << 0      << 8.334   << 0        << endr
         << 0      << 0       << 7.567    << endr;
    
    r   = 1;

    // Wind force

    
    v_i_wi_i << 5 << 0 << 0;
    br       = 3.5;
    bs       = 3.5;
   
    b_p1     = 10.5;
    b_p2     = 1.5;
    
    c_f = 0.5;
    
    /*
     v_i_wi_i << 0 << 0 << 0;
     br       = 0;
     bs       = 0;
     
     b_p1     = 0;
     b_p2     = 0;
     
     c_f      = 0.3768;
    */
    
    // Generic vectors:
    unit_col_1 << 1  << 0  << 0;
    unit_col_2 << 0  << 1  << 0;
    unit_col_3 << 0  << 0  << 1;
    
}

