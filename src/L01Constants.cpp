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
constants::constants(std::string EnvGenDir, std::string EnvParametersDir)
{
    initializeGlobalConstants(EnvGenDir,EnvParametersDir);
    initializeModelConstants();
#ifdef DYNAMIC_PARAMETER_LOAD
    loadConstants();
#endif
    saveConstants();
}


void constants::initializeGlobalConstants(std::string EnvGenDir, std::string EnvParametersDir)
{
    // Global Home Path for generated files:
    GenDir = EnvGenDir;
    ParametersDir = EnvParametersDir;
    
    // Gravitational Constants:
    g_i << 0.0 << 0.0 << -9.81; //gravitational accel, m/s/s
    g = 9.81;
    
    // Generic vectors:
    unit_col_1 << 1  << 0  << 0;
    unit_col_2 << 0  << 1  << 0;
    unit_col_3 << 0  << 0  << 1;
}

void constants::initializeModelConstants()
{
    // Structural Constants:

    // 01: Pendulum constants:
    
    // Pendulum masses:
    m_p1 = 5.5;
    m_p2 = 5.5;

    // Pendulum locations
    r_s_c1s << 0  <<  0.5 << 0;
    r_s_c2s << 0  << -0.5 << 0;
    
    // Pendulum c_p
    c_p1 << 0 << 0 << -3.675 ;
    c_p2 << 0 << 0 << -3.675 ;

    
    /*
    //r_s_c1s << 0 <<  0.17 << 0;
    //r_s_c2s << 0 << -0.17 << 0;

    //r_s_c1s << 0 << 0 << 0;
    //r_s_c2s << 0 << 0 << 0;
    
    // c_p1 << 0 << 0 << -4.725;
    // c_p2 << 0 << 0 << -4.725;
    */
     
    // Pendulum inertia matrix:
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
    
    // 02: Sphere Constants:
    // Sphere mass
    m_s = 15;
    
    // Sphere inertia matrix
    J_s  << 7.567  << 0       << 0        << endr
         << 0      << 8.334   << 0        << endr
         << 0      << 0       << 7.567    << endr;
    // Sphere radius
    r   = 1;

    // 03: External Forces:
    // Wind force
    
    // Wind velocity vector
    v_i_wi_i << 5 << 0 << 0;
    
    
    // Coefficient of skin friction of sphere
    c_f = 0.5;
    
    // Damping - roll damping, sphere movement damping
    br       = 3.5;
    bs       = 3.5;
   
    // Pendulum damping:
    b_p1     = 10.5;
    b_p2     = 1.5;
    
    
    /*
     v_i_wi_i << 0 << 0 << 0;
     br       = 0;
     bs       = 0;
     
     b_p1     = 0;
     b_p2     = 0;
     
     c_f      = 0.3768;
    */
}

void constants::saveConstants()
{
    // Vectors for loading constants:
    vec m_p_vec(2);
    vec m_s_vec(1);
    vec r_vec(1);
    vec c_f_vec(1);
    vec b_p_vec(2);
    vec b_s_vec(2);
    
    // Structural Constants:

    // 01: Pendulum constants:
    
    // Pendulum masses:
    
    m_p_vec << m_p1 << m_p2;
    m_p_vec.save(GenDir  + "m_p_vec.csv"    , csv_ascii);
    
    // Pendulum locations
    r_s_c1s.save   (GenDir  + "r_s_c1s.csv"    , csv_ascii);
    r_s_c2s.save   (GenDir  + "r_s_c2s.csv"    , csv_ascii);
    
    // Pendulum c_p
    c_p1.save   (GenDir  + "c_p1.csv"    , csv_ascii);
    c_p2.save   (GenDir  + "c_p2.csv"    , csv_ascii);

     
    // Pendulum inertia matrix:
    J_p1.save   (GenDir  + "J_p1.csv"    , csv_ascii);
    J_p2.save   (GenDir  + "J_p2.csv"    , csv_ascii);
    
    
    // 02: Sphere Constants:
    // Sphere mass
    m_s_vec << m_s;
    m_s_vec.save(GenDir  + "m_s_vec.csv"    , csv_ascii);
    
    // Sphere inertia matrix
    J_s.save   (GenDir  + "J_s.csv"    , csv_ascii);
    // Sphere radius
    r_vec << r;
    r_vec.save(GenDir  + "r_vec.csv"    , csv_ascii);
    
    // 03: External Forces:
    // Wind force
    
    // Wind velocity vector
    v_i_wi_i.save   (GenDir  + "v_i_wi_i.csv"    , csv_ascii);
    
    
    // Coefficient of skin friction of sphere
    c_f_vec << c_f;
    c_f_vec.save(GenDir  + "c_f_vec.csv"    , csv_ascii);
    
    // Damping - roll damping, sphere movement damping
    b_s_vec << br << bs;
    b_s_vec.save(GenDir  + "b_s_vec.csv"    , csv_ascii);
    
    // Pendulum damping:
    b_p_vec << b_p1 << b_p2;
    b_p_vec.save(GenDir  + "b_p_vec.csv"    , csv_ascii);
}

void constants::loadConstants()
{
    // Vectors for loading constants:
    vec m_p_vec(2);
    vec m_s_vec(1);
    vec r_vec(1);
    vec c_f_vec(1);
    vec b_p_vec(2);
    vec b_s_vec(2);
    
    // Structural Constants:

    // 01: Pendulum constants:
    
    // Pendulum masses:
    
    m_p_vec.load(ParametersDir  + "m_p_vec.csv"    , csv_ascii);
    m_p1 = m_p_vec(0);
    m_p2 = m_p_vec(1);
    
    // Pendulum locations
    r_s_c1s.load(ParametersDir  + "r_s_c1s.csv"    , csv_ascii);
    r_s_c2s.load(ParametersDir  + "r_s_c2s.csv"    , csv_ascii);
    
    // Pendulum c_p
    c_p1.load   (ParametersDir  + "c_p1.csv"    , csv_ascii);
    c_p2.load   (ParametersDir  + "c_p2.csv"    , csv_ascii);

     
    // Pendulum inertia matrix:
    J_p1.load   (ParametersDir  + "J_p1.csv"    , csv_ascii);
    J_p2.load   (ParametersDir  + "J_p2.csv"    , csv_ascii);
    
    
    // 02: Sphere Constants:
    // Sphere mass
    m_s_vec.load(ParametersDir  + "m_s_vec.csv"    , csv_ascii);
    m_s = m_s_vec(0);
    
    // Sphere inertia matrix
    J_s.load   (ParametersDir  + "J_s.csv"    , csv_ascii);
    // Sphere radius
    r_vec.load(ParametersDir  + "r_vec.csv"    , csv_ascii);
    r = r_vec(0);
    
    // 03: External Forces:
    // Wind force
    
    // Wind velocity vector
    v_i_wi_i.load   (ParametersDir  + "v_i_wi_i.csv"    , csv_ascii);
    
    
    // Coefficient of skin friction of sphere
    c_f_vec.load(ParametersDir  + "c_f_vec.csv"    , csv_ascii);
    c_f = c_f_vec(0);
    
    // Damping - roll damping, sphere movement damping
    b_s_vec.load(ParametersDir  + "b_s_vec.csv"    , csv_ascii);
    br = b_s_vec(0);
    bs = b_s_vec(1);
    
    // Pendulum damping:
    b_p_vec.load(ParametersDir  + "b_p_vec.csv"    , csv_ascii);
    b_p1 = b_p_vec(0);
    b_p2 = b_p_vec(1);
}
