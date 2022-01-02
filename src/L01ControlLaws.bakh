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

//Control Laws Functions Prototype:
#ifndef L01CONTROLLAWS_H
#define L01CONTROLLAWS_H

//9. Desired trajectory
double rho_d(double t_act, constants cst);

//10. Velocity of desired trajectory
double rho_d_dot(double t_act, constants cst);

//11. Acceleration of desired trajectory
double rho_d_ddot(double t_act, constants cst);

//12. Main PD Feedback and Feed Forward control law
vec tau_c_theta(double t,double rho, double rho_dot, double theta_1, double theta_2, double theta_1_dot, double theta_2_dot,vec a_cap, int post_processing_switch, constants cst);

#endif
