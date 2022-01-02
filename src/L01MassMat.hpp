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
#include "L01Constants.hpp"

using namespace std;
using namespace arma;

//Mass Matrix Functions Prototype:
#ifndef L01MASSMAT_H
#define L01MASSMAT_H

//2. Common mass matrix function for equaly divided variable mass cable segments.
mat M_i_common(int n, constants cst);

//3. Mass Matrix function
mat mass_matrix(mat C_si, mat C_p1i, mat C_p2i, constants cst);

//4. Rate of change of Mass Matrix
mat M_dot_matrix(mat C_si, mat C_p1i, mat C_p2i, vec omega_s_si, vec omega_p1_p1s, vec omega_p2_p2s, vec omega_p1_p1i, vec omega_p2_p2i, constants cst);

//5. Mass matrix dependency on state variables
vec Partial_M(int n, constants cst);

//6. pi_matrix:
mat pi_matrix(mat C_si, mat C_p1i, mat C_p2i, constants cst);

// 7. pi_dot_matrix:
mat pi_dot_matrix(mat C_si, mat C_p1i, mat C_p2i, vec omega_s_si, vec omega_p1_p1s, vec omega_p2_p2s, constants cst);

// 8. delta_bar_dot_T:
mat delta_bar_2_T_matrix(mat C_si, mat C_p1i, mat C_p2i, vec omega_s_si, vec omega_p1_p1s, vec omega_p2_p2s, constants cst);

// 9. delta_3_t:
mat delta_3_T_matrix(mat C_si, mat C_p1i, mat C_p2i, vec omega_s_si, vec omega_p1_p1i, vec omega_p2_p2i, vec v_i_si_i, constants cst);

// 10. External Force matrix
mat F_ext_matrix(mat C_si, mat C_p1i, mat C_p2i, vec omega_s_si, double omega_p1_p1s_2, double omega_p2_p2s_2, vec v_i_ws_i, vec v_i_si_i, constants cst);

// 11. Energy:
double H_dot_matrix(mat C_si, mat C_p1i, mat C_p2i, vec omega_s_si, vec omega_p1_p1i, vec omega_p2_p2i, vec v_i_si_i, mat F_ext, mat M, constants cst);

#endif
