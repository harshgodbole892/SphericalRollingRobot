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
#include "VectrixLaws.hpp"
#include "L01Model.hpp"

//1. Structure of Constants
SphericalRollingRobot::SphericalRollingRobot()
{
    // State Variables:
    c_bar_si_1 = zeros<vec>(3);
    c_bar_si_2 = zeros<vec>(3);
    c_bar_si_3 = zeros<vec>(3);

    C_si = zeros<mat>(3,3);
             
    c_bar_p1i_1 = zeros<vec>(3);
    c_bar_p1i_2 = zeros<vec>(3);
    c_bar_p1i_3_1 = zeros<vec>(3);
    c_bar_p1i_3_2 = zeros<vec>(3);
    c_bar_p1i_3 = zeros<vec>(3);

    C_p1i = zeros<mat>(3,3);

    c_bar_p2i_1 = zeros<vec>(3);
    c_bar_p2i_2 = zeros<vec>(3);
    c_bar_p2i_3_1 = zeros<vec>(3);
    c_bar_p2i_3_2 = zeros<vec>(3);
    c_bar_p2i_3 = zeros<vec>(3);

    C_p2i = zeros<mat>(3,3);

    omega_s_si = zeros<vec>(3);
    omega_p1_p1s_2 = 0.0;
    omega_p2_p2s_2 = 0.0;

    r_i_si = zeros<vec>(3);

    H = 0.0;

    omega_p1_p1s = zeros<vec>(3);
    omega_p2_p2s = zeros<vec>(3);

    omega_p1_p1i = zeros<vec>(3);
    omega_p2_p2i = zeros<vec>(3);

    omega_cap = zeros<vec>(5);

    // Kinematics Variables:
    c_bar_dot_si_1 = zeros<vec>(3);
    c_bar_dot_si_2 = zeros<vec>(3);
    c_bar_dot_p1i_1 = zeros<vec>(3);
    c_bar_dot_p1i_2 = zeros<vec>(3);
    c_bar_dot_p2i_1 = zeros<vec>(3);
    c_bar_dot_p2i_2 = zeros<vec>(3);

    // Velocities of wind and sphere
    v_i_si_i = zeros<vec>(3);
    v_i_ws_i = zeros<vec>(3);

    // Translational and attitude dynamics
    M = zeros<mat>(12,12);

    // Kinematic transformation matrix
    pi  = zeros<mat> (12,5);
    pi_dot = zeros<mat>(12,5);
    delta_bar_2_T = zeros<mat>(5,12);
    M_dot = zeros<mat>(12,12);
    delta_3_T = zeros<mat>(5,1);

    // External eneralized force
    F_ext = zeros<mat>(12,1);
    F_ext_total = zeros<mat>(5,1);

    M_eq = zeros<mat>(5,5);

    // Omega Cap state vector
    omega_cap_dot = zeros<vec>(5,1);

    // Energy rate:
    H_dot = 0.0;
    x_dot = zeros<vec>(27);
}

void SphericalRollingRobot::PopulateInputs(vec x, constants cst)
{
#ifdef DEBUG_ODE
    cout<<endl <<"Execution Started" << endl;
#endif
    c_bar_si_1 = x.subvec(0,2);
    c_bar_si_2 = x.subvec(3,5);
    c_bar_si_3 = cross(c_bar_si_1) * c_bar_si_2;
    
    // Reconstruct Matrix
    C_si = reconstruct_dcm(c_bar_si_1, c_bar_si_2, c_bar_si_3);
    
    c_bar_p1i_1 = x.subvec(6,8);
    c_bar_p1i_2 = x.subvec(9,11);
    c_bar_p1i_3_1 =  cross(c_bar_p1i_1)*c_bar_p1i_2;
    c_bar_p1i_3_2 = -cross(c_bar_p1i_2)*c_bar_p1i_1;
    c_bar_p1i_3=(0.5)*(c_bar_p1i_3_1+c_bar_p1i_3_2);

    // Reconstruct Matrix
    C_p1i= reconstruct_dcm(c_bar_p1i_1, c_bar_p1i_2, c_bar_p1i_3);
    

    c_bar_p2i_1 = x.subvec(12,14);
    c_bar_p2i_2 = x.subvec(15,17);
    c_bar_p2i_3_1 =  cross(c_bar_p2i_1) * c_bar_p2i_2;
    c_bar_p2i_3_2 = -cross(c_bar_p2i_2) * c_bar_p2i_1;
    c_bar_p2i_3   = (0.5) * (c_bar_p2i_3_1 + c_bar_p2i_3_2);

    // Reconstruct Matrix
    C_p2i = reconstruct_dcm(c_bar_p2i_1, c_bar_p2i_2, c_bar_p2i_3);

    omega_s_si     = x.subvec(18,20);
    omega_p1_p1s_2 = x(21);
    omega_p2_p2s_2 = x(22);

    r_i_si = x.subvec(23,25);

    H = x(26);

    omega_p1_p1s = cst.unit_col_2 * omega_p1_p1s_2;
    omega_p2_p2s = cst.unit_col_2 * omega_p2_p2s_2;

    omega_p1_p1i = omega_p1_p1s + (C_p1i * C_si.t() * omega_s_si);
    omega_p2_p2i = omega_p2_p2s + (C_p2i * C_si.t() * omega_s_si);
    
    omega_cap.subvec(0,2) = omega_s_si;
    omega_cap(3)   = omega_p1_p1s_2;
    omega_cap(4)   = omega_p2_p2s_2;
}

void SphericalRollingRobot::SolveKinematics(constants cst)
{
    //% Solve kinematics
#ifdef DEBUG_ODE
    cout<<"Kinematics" << endl;
#endif
    
    c_bar_dot_si_1 = -(omega_s_si(1)) * (c_bar_si_3) + (omega_s_si(2)) * (c_bar_si_2);
    c_bar_dot_si_2 = -(omega_s_si(2)) * (c_bar_si_1) + (omega_s_si(0)) * (c_bar_si_3);
              
    c_bar_dot_p1i_1 = -(omega_p1_p1i(1)) * (c_bar_p1i_3)+(omega_p1_p1i(2)) * (c_bar_p1i_2);
    c_bar_dot_p1i_2 = -(omega_p1_p1i(2)) * (c_bar_p1i_1)+(omega_p1_p1i(0)) * (c_bar_p1i_3);

    c_bar_dot_p2i_1 = -(omega_p2_p2i(1)) * (c_bar_p2i_3)+(omega_p2_p2i(2)) * (c_bar_p2i_2);
    c_bar_dot_p2i_2 = -(omega_p2_p2i(2)) * (c_bar_p2i_1)+(omega_p2_p2i(0)) * (c_bar_p2i_3);

    // velocities of wind and sphere
#ifdef DEBUG_ODE
    cout<<"Sphere Velocities" << endl;
#endif
    
    v_i_si_i = -cst.r * cross(cst.unit_col_3) * C_si.t() * omega_s_si;

    v_i_ws_i = (cst.v_i_wi_i) - (v_i_si_i);
}

void SphericalRollingRobot::CalculateXDot()
{
    
    // DCM:
    x_dot.subvec(0,2) = c_bar_dot_si_1;
    x_dot.subvec(3,5) = c_bar_dot_si_2;
    
    x_dot.subvec(6,8)   = c_bar_dot_p1i_1;
    x_dot.subvec(9,11) = c_bar_dot_p1i_2;
    
    x_dot.subvec(12,14) = c_bar_dot_p2i_1;
    x_dot.subvec(15,17) = c_bar_dot_p2i_2;
    
    // Omega
    x_dot.subvec(18,22) = omega_cap_dot;
   
    // Position
    x_dot.subvec(23,25) = v_i_si_i;

    // Energy rate
    x_dot(26) = H_dot;
}
