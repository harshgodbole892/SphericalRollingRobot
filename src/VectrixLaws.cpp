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

#include "VectrixLaws.hpp"

//Function definations for Vectrix Laws start here: 

//1. Cross matrix function
mat cross(vec x)
{
    // Cross matrix: 3 x 1 column matrix input, 3 x 3 skew-sym matrix output.
    mat x_cross = eye<mat>(3,3);
    
    if (x.n_rows == 3)
    {
        x_cross <<   0   << -x(2)  <<   x(1) << endr
                <<  x(2) <<    0   <<  -x(0) << endr
                << -x(1) <<  x(0)  <<     0  << endr;
        return x_cross;
    }
    else
    {
        cout<<"Passed Vector with rows not 3 to Cross function "<<endl;
        return x_cross;
    }
}

//2. C1 Matrix
mat C1 (double th)
{
    //This is a ``1'' direction cosine matrix.
    mat C1_matrix= eye<mat>(3,3);
    
    C1_matrix <<   1  <<    0     <<     0    << endr
              <<   0  <<  cos(th) << sin(th)  << endr
              <<   0  << -sin(th) << cos(th)  << endr;
    
    return C1_matrix;
}
//3. C2 Matrix
mat C2 (double th)
{
    //This is a ``1'' direction cosine matrix.
    mat C2_matrix= eye<mat>(3,3);
    
    C2_matrix << cos(th)  <<    0     << -sin(th)  << endr
              <<   0      <<    1     <<    0      << endr
              << sin(th)  <<    0     <<  cos(th)  << endr;
    
    return C2_matrix;
}

//4. C3 Matrix
mat C3 (double th)
{
    //This is a ``1'' direction cosine matrix.
    mat C3_matrix= eye<mat>(3,3);
    
    C3_matrix <<  cos(th)  <<  sin(th) <<    0  << endr
              << -sin(th)  <<  cos(th) <<    0  << endr
              <<    0      <<    0     <<    1  << endr;
    
    return C3_matrix;
}

//5. vex function
vec vex(mat A)
{
    // Uncross operation, or ``vectorize". Input is 3 x 3 skew-sym matrix,
    // output is 3 x 1 column matrix.

    vec x_v = zeros<vec>(3);
    
    x_v << -A(1,2) <<  A(0,2) << -A(0,1);
    
    return x_v;
}

mat reconstruct_dcm(vec c1, vec c2, vec c3)
{
    mat C_dcm = zeros<mat>(3,3);
    
    // Reconstruct Matrix
    C_dcm.submat(0,0,2,0) = c1;
    C_dcm.submat(0,1,2,1) = c2;
    C_dcm.submat(0,2,2,2) = c3;
    
    return(C_dcm.t());
}
    
