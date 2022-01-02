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
 
 Tempelate by Harsh Godbole. 
 Reference credits to Dr. James Richard Forbes and Ryan Caverly.
 */
//Tempelate by Harsh Godbole. Reference credits Dr. James Richard Forbes

// Generic includes:
#include <iostream>
#include <armadillo>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>

#define DYNAMIC_PARAMETER_LOAD
// Project Specific includes:
#include "L01Constants.hpp"
#include "L01ODE.hpp"
#include "RK4.hpp"
#include "L01PostProcessing.hpp"


using namespace std;
using namespace arma;


//17. Main Function
int main(int argc, char** argv) {
    
    /*
     ------------------------------
     Step 01 : Initialize ODE
     ------------------------------
     */
    // Initializing constants parameters class
    cout<<"started operation "<<endl;
    
    // Get generated directory path and dump files in correct location
    std::string RelGenPath("/Generated/");
    std::string RelParametersPath("/Parameters/");
    std::string ProjectHomeDir("");
    
    // Check that getenv has not returned a NULL pointer:
    const char *ProjectHomeDirPtr = getenv("PROJECT_HOME_DIR");
    if (!ProjectHomeDirPtr)
    {
         ProjectHomeDir.assign("//");
         cout<<"Env Variable PROJECT_HOME_DIR is not set, Use L01Runfile to execute code"<<endl;
    }
    else
    {
        ProjectHomeDir.assign(getenv("PROJECT_HOME_DIR"));
    
        //std::string ProjectHomeDir = getenv("PROJECT_HOME_DIR");
        std::string GenDir(ProjectHomeDir + RelGenPath);
        std::string ParametersDir(ProjectHomeDir + RelParametersPath);
    
        // Initializing constants parameters class
        cout<<"Loading Constants "<<endl;
        constants cst(GenDir, ParametersDir);
        cout<<"Constants Loaded "<<endl;
        
        
        // ODE: Initial conditions
        int x_length_1;
        x_length_1 = 27;
        
        vec x_IC=zeros<vec>(x_length_1);

        //(a) Generalized co-ordinates
    
        // DCM matrices
        x_IC.subvec(0,2)   =  cst.unit_col_1; //c_bar_si_1_IC =[1;0;0];
        x_IC.subvec(3,5)   =  cst.unit_col_2; //c_bar_si_2_IC =[0;1;0];
        x_IC.subvec(6,8)   =  cst.unit_col_1; //c_bar_p1i_1_IC =[1;0;0];
        x_IC.subvec(9,11)  =  cst.unit_col_2; //c_bar_p1i_2_IC =[0;1;0];
        x_IC.subvec(12,14) =  cst.unit_col_1; //c_bar_p2i_1_IC =[1;0;0];
        x_IC.subvec(15,17) =  cst.unit_col_2; //c_bar_p2i_2_IC=[0;1;0];
        
        // Angular Velocities:
        x_IC.subvec(18,20) = arma::zeros(3,1);  //omega_s_si_IC=[0;0;0];
        x_IC(21) = 0.0;                         //omega_p1_p1i_2_IC=0;
        x_IC(22) = 0.0;                         //omega_p2_p2i_2_IC=0;
       
        // Linear Position:
        x_IC.subvec(23,25) = cst.unit_col_3;       //r_s_si_IC=[0;0;1];
        
        // Energy:
        x_IC(26) = 147.15;               // H_IC = 147.5
        
        
        // Load from constants file
#ifdef DYNAMIC_PARAMETER_LOAD
        x_IC.load(ParametersDir + "x_IC.csv" , csv_ascii);
#endif
        x_IC.save(GenDir + "x_IC.csv" , csv_ascii);
        
        //
        // -------------------------------------
        // Step 02 : Set Simulation Parameters:
        // -------------------------------------
        //
        
        // Initialize Integrator
        vec IntegratorParams(3);
        
        IntegratorParams(0) = 0,          // Start time in seconds
        IntegratorParams(1) = 20,         // End time in seconds
        IntegratorParams(2) = 0.001;      // Step Size in seconds
        
        // Load from constants file and save used constants
#ifdef DYNAMIC_PARAMETER_LOAD
        IntegratorParams.load(ParametersDir + "IntegratorParams.csv" , csv_ascii);
#endif
        IntegratorParams.save(GenDir + "IntegratorParams.csv" , csv_ascii);
        
        double t_start  = IntegratorParams(0);  // Start time in seconds
        double t_end    = IntegratorParams(1);  // End time in seconds
        double h        = IntegratorParams(2);  // Step Size in seconds
            
        // IntegratorParams.save(GenDir + "IntegratorParams.csv" , csv_ascii);
        
        // Assign output vector:
        
	    long long int a = (t_end - t_start) / h; //Need better counting Subroutine
	    cout << "C++ : Number of Iterations Estimated "<<a<<endl;
	    mat x_out=zeros<mat>(x_length_1, a);
	                                                    //vec x_out(x_length_1);//test
    
        //
        // -------------------------------------
        // Step 03 : Solve ODE
        // -------------------------------------
        //
    
	    cout<< "C++ : RK4 process started"<<endl;
    
	    x_out = RK4(h, t_start, t_end, x_IC, cst, ODE);
	                                                    //x_out=ODE(x_IC,1,cst);//test
    
	    cout << "C++ : RK4 process complete"<<endl;
	    
    
        //
        // -------------------------------------
        // Step 04 : Postprocessing Results
        // -------------------------------------
     
    
	    // Lagrange multipluer calculation option:
    
         int calc_lagrange_multiplier = 1; //  1 -> on
                                           //  0 -> off
      
    
	    // Post processing and file export
        
        cout<< "C++ : Post processing started"<<endl;
    
        post_processing(h,t_start,t_end,x_out,calc_lagrange_multiplier,cst);
    
        cout<< "C++ : Post processing completed"<<endl;
     
        
        // **** Plot-scripts in matlab:
        // **** 1. system() Directly execute test_script.m and plot script commands
        
        //system("/Applications/MATLAB_R2016a.app/bin/matlab -nodesktop -r \"run test_script.m\"");
        
    } // end if (!ProjectHomeDirPtr)

	return 0;
}
// End of Main
