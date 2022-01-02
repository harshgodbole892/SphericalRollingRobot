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

#include "RK4.hpp"

//15. Custom RK4 integrator
mat  RK4(double h,
         double t_start,
         double t_end,
         vec x_IC,
         constants cst,
         vec (*ODE)(vec, double, constants))
{
	int s;
	s = x_IC.size();
	vec k_1=zeros<vec>(s);
	vec k_2 = zeros<vec>(s);
	vec k_3 = zeros<vec>(s);
	vec k_4 = zeros<vec>(s);

	vec k_t = zeros<vec>(s);
	double t_i = 0;
	
	/*deciding the number of iterations required*/

	long long int a = (t_end - t_start) / h;
	cout << "C++ : Number of Iterations Estimated for current RK4 "<< a << endl; //counter FLAG
	mat x = zeros<mat>(s, a);
	vec t_s = zeros<vec>(a);

	/*integration starts here*/

	for (int i = 0; i < a; i++)
	{   
		                                            //cout<<"loop executed"<<i<<"times"<<endl;//test
		if (i == 0)
		{
            x.col(i) = x_IC;
            t_s(i) = t_start;
            cout<<"\r"<<i<<"/"<<a;
		}
		else
		{	/*calculating 4 constants*/

			k_1 = (*ODE)(x.col(i - 1), t_s(i - 1), cst);
			                             //k_1.print("k_1");//test
			for (int j = 0; j<k_1.size(); j++)
			{
				k_t(j) = x(j,i - 1) + 0.5*k_1(j)*h;
			}
			t_i = t_s(i - 1) + 0.5*h;
			                            //k_t.print("k_t_1:");//test
			                            //cout << "t_i_1" << t_i;//test

			k_2 = (*ODE)(k_t, t_i, cst);
			                            //k_2.print("k_2");//test

			for (int j = 0; j<k_2.size(); j++)
			{
				k_t(j) = x(j,i - 1) + 0.5*k_2(j)*h;
			}
			t_i = t_s(i - 1) + 0.5*h;
			                            //k_t.print("k_t_1:");//test
			                            //cout << "t_i_1" << t_i;//test

			k_3 = (*ODE)(k_t, t_i, cst);
			                            //k_3.print("k_3");//test

			for (int j = 0; j<k_3.size(); j++)
			{
				k_t(j) = x(j, i - 1) + k_3(j)*h;
			}
			t_i = t_s(i - 1) + h;
			                            //k_t.print("k_t_1:");// test
			                            //cout << "t_i_1"<<t_i;//test

			k_4 = (*ODE)(k_t, t_i, cst);
			                            //k_4.print("k_4");// test

			/*propogating x*/
			
			for (int j = 0; j<x_IC.size(); j++)
			{
				x(j,i) = x(j,i-1) + ((h*(k_1(j) + (2 * k_2(j)) + (2 * k_3(j)) + k_4(j))) / (6.0));
			}

			
			                            //x.col(i).print("xcol=");//test

			/*propogating time*/
			t_s(i) = t_s(i - 1) + h;
		}
        if (i%1000 == 0) cout<<"\r"<<i<<"/"<<a;
        fflush(stdout);

	}
    cout<<"\r"<<a<<"/"<<a<<endl;
	return x;
	
}
