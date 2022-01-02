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

//#define DEBUG_ODE 1

#include "L01ODE.hpp"
#include "L01Model.hpp"

// Initialize Model class:

SphericalRollingRobot Model;

//13. ODE function: f'(x) = ODE(x)
vec ODE(vec x, double t, constants cst)
{
    // Initialze Model Vectors:
    Model.PopulateInputs(x,cst);

    //% Solve kinematics
    Model.SolveKinematics(cst);

    //  Mass Matrix
#ifdef DEBUG_ODE
    cout<<"Mass Matrix" << endl;
#endif
    
    Model.M = mass_matrix(Model.C_si,
                          Model.C_p1i,
                          Model.C_p2i,
                          cst);

    // Kinematic transformation matrix
#ifdef DEBUG_ODE
    cout<<"Pi Matrix" << endl;
#endif

    Model.pi = pi_matrix(Model.C_si,
                         Model.C_p1i,
                         Model.C_p2i,
                         cst);
    
#ifdef DEBUG_ODE
    cout<<"Pi Dot" << endl;
#endif
    
    Model.pi_dot =  pi_dot_matrix(Model.C_si,
                                  Model.C_p1i,
                                  Model.C_p2i,
                                  Model.omega_s_si,
                                  Model.omega_p1_p1s,
                                  Model.omega_p2_p2s,
                                  cst);

    // Delta_bar_2 terms
#ifdef DEBUG_ODE
    cout<<"Delta Bar 2" << endl;
#endif
    
    Model.delta_bar_2_T = delta_bar_2_T_matrix( Model.C_si,
                                                Model.C_p1i,
                                                Model.C_p2i,
                                                Model.omega_s_si,
                                                Model.omega_p1_p1i,
                                                Model.omega_p2_p2i,
                                                cst );
                       
    // Delta_4 terms (M_dot):
#ifdef DEBUG_ODE
    cout<<"Delta 4 Mdot" << endl;
#endif
    
    Model.M_dot = M_dot_matrix(Model.C_si,
                               Model.C_p1i,
                               Model.C_p2i,
                               Model.omega_s_si,
                               Model.omega_p1_p1s,
                               Model.omega_p2_p2s,
                               Model.omega_p1_p1i,
                               Model.omega_p2_p2i,
                               cst);

    // Delta 3 terms
#ifdef DEBUG_ODE
    cout<<"Delta 3" << endl;
#endif
    
    Model.delta_3_T= delta_3_T_matrix( Model.C_si,
                                     Model.C_p1i,
                                     Model.C_p2i,
                                     Model.omega_s_si,
                                     Model.omega_p1_p1i,
                                     Model.omega_p2_p2i,
                                     Model.v_i_si_i,
                                     cst);


    // External eneralized force
    
#ifdef DEBUG_ODE
    cout<<"Fext Matrix" << endl;
#endif
    
    Model.F_ext  = F_ext_matrix( Model.C_si,
                                 Model.C_p1i,
                                 Model.C_p2i,
                                 Model.omega_s_si,
                                 Model.omega_p1_p1s_2,
                                 Model.omega_p2_p2s_2,
                                 Model.v_i_ws_i,
                                 Model.v_i_si_i,
                                 cst);
    
    Model.F_ext_total= Model.pi.t() * Model.F_ext;
    
    
    // Equivalent mass matrix:
#ifdef DEBUG_ODE
    cout<<"Equivalent Mass Matrix" << endl;
#endif
    
    Model.M_eq = Model.pi.t() * Model.M * Model.pi;
    
    // The Differencial Equation
#ifdef DEBUG_ODE
    cout<<"Omega Cap Dot" << endl;
#endif
    
    Model.omega_cap_dot =  solve(   Model.M_eq,
                                    (   Model.F_ext_total
                                     +  Model.delta_3_T
                                     - ( (    Model.pi.t()        * Model.M     * Model.pi_dot
                                            + Model.delta_bar_2_T * Model.M     * Model.pi
                                            + Model.pi.t()        * Model.M_dot * Model.pi
                                         ) * (Model.omega_cap)
                                       )
                                    )
                          );

    // Other -  numerically integrate H_dot - Rate of energy loss
#ifdef DEBUG_ODE
    cout<<"H Dot" << endl;
#endif

    Model.H_dot = H_dot_matrix(Model.C_si,
                               Model.C_p1i,
                               Model.C_p2i,
                               Model.omega_s_si,
                               Model.omega_p1_p1i,
                               Model.omega_p2_p2i,
                               Model.v_i_si_i,
                               Model.F_ext,
                               Model.M,
                               cst);

    // Retrun State Vector Output

    Model.CalculateXDot();
    
	return Model.x_dot;

}

//14. Test ODEs function
vec ODES01(vec x, double t, constants cst)
{
	x = 0.5*x;
	return x;
}

