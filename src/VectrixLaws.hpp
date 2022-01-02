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
#include <math.h>
#include "L01Constants.hpp"

using namespace std;
using namespace arma;

//Vectrix Functions Prototype:
#ifndef VECTRIXLAWS_H
#define VECTRIXLAWS_H

//1. Cross matrix function
mat cross(vec A);

//2. C1 Matrix
mat C1 (double theta);

//3. C2 Matrix
mat C2 (double theta);

//4. C3 Matrix
mat C3 (double theta);

//5. vex function
vec vex(mat A);

//6. Reconstruct DCM:
mat reconstruct_dcm(vec c1, vec c2, vec c3);

#endif
