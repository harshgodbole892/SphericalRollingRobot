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

#include "L01MassMat.hpp"
#include "VectrixLaws.hpp"

//2. Common mass matrix function for equaly divided variable mass cable segments.
mat M_i_common(int n, constants cst)
{    
	// Creates the mass matrix tempelate used M and M_dot. We can obtain the mass matrix by multiplying by m_i or m_i_dot 
	//This method will also assume that m_i is same for all of the fragments
	mat M_i = zeros<mat>(n + 2, n + 2);
	
	return M_i;
}

//3. Mass Matrix function
mat mass_matrix(mat C_si, mat C_p1i, mat C_p2i, constants cst)
{
    arma::mat M22_1(3,3);
    arma::mat M1(12,12);
    
    M22_1 = cst.J_s - (cross(cst.r_s_c1s) * cross(cst.r_s_c1s) * cst.m_p1)
                    - (cross(cst.r_s_c2s) * cross(cst.r_s_c2s) * cst.m_p2);
    
    M1.submat(0,0,2,2)   =   (cst.m_p1 + cst.m_p2 + cst.m_s) * arma::eye(3,3);  // M11
    M1.submat(0,3,2,5)   =  -((C_si).t())  * (cross(cst.r_s_c1s)*cst.m_p1 + cross(cst.r_s_c2s)*cst.m_p2); // M12
    M1.submat(0,6,2,8)   =  -((C_p1i).t()) * (cross(cst.c_p1));  // M13
    M1.submat(0,9,2,11)  =  -((C_p2i).t()) * (cross(cst.c_p2));  // M14
    
    M1.submat(3,0,5,2)   =  M1.submat(0,3,2,5).t() ;  // M21
    M1.submat(3,3,5,5)   =  0.5 * (M22_1 + M22_1.t()); // M22
    M1.submat(3,6,5,8)   =  -cross(cst.r_s_c1s) * C_si * ((C_p1i).t()) * cross(cst.c_p1); // M23
    M1.submat(3,9,5,11)  =  -cross(cst.r_s_c2s) * C_si * ((C_p2i).t()) * cross(cst.c_p2); // M24
    
    M1.submat(6,0,8,2)   =  M1.submat(0,6,2,8).t(); // M31
    M1.submat(6,3,8,5)   =  M1.submat(3,6,5,8).t(); // M32
    M1.submat(6,6,8,8)   = cst.J_p1; // M33
    M1.submat(6,9,8,11)  = arma::zeros(3,3); // M34
    
    M1.submat(9,0,11,2)   = M1.submat(0,9,2,11).t(); // M41
    M1.submat(9,3,11,5)   = M1.submat(3,9,5,11).t(); // M42
    M1.submat(9,6,11,8)   = M1.submat(6,9,8,11).t(); // M43
    M1.submat(9,9,11,11)  = cst.J_p2; // M44
    
    return( 0.5 * (M1 + M1.t()));
}

//4. Rate of change of Mass Matrix
mat M_dot_matrix(mat C_si, mat C_p1i, mat C_p2i, vec omega_s_si, vec omega_p1_p1s, vec omega_p2_p2s, vec omega_p1_p1i, vec omega_p2_p2i, constants cst)
{
    arma::mat M_dot = arma::zeros<mat>(12,12);
    
    // Assigned from bottom right side:
    
    M_dot.submat(9,0,11,2)   = -cross(cst.c_p2) * cross(omega_p2_p2i) * C_p2i; // M41 B1
    M_dot.submat(9,3,11,5)   =  cross(cst.c_p2) * cross(omega_p2_p2s) * C_p2i * ((C_si).t()) * cross(cst.r_s_c2s); // M42 D1
    // M_dot.submat(9,6,11,8)   = zeros; // M43
    // M_dot.submat(9,9,11,11)  = zeros; // M44
    
    M_dot.submat(6,0,8,2)   =  -cross(cst.c_p1) * cross(omega_p1_p1i) * C_p1i; // M31 A1
    M_dot.submat(6,3,8,5)   =   cross(cst.c_p1) * cross(omega_p1_p1s) * C_p1i * ((C_si).t()) * cross(cst.r_s_c1s); // M32 C1
    //M_dot.submat(6,6,8,8)   = zeros; // M33
    //M_dot.submat(6,9,8,11)  = zeros; // M34
    
    M_dot.submat(3,0,5,2)   =  -((cross(cst.r_s_c1s)*cst.m_p1)+(cross(cst.r_s_c2s)*cst.m_p2))*cross(omega_s_si)*C_si; ;  // M21 E1
    // M_dot.submat(3,3,5,5)   =  zeros // M22
    M_dot.submat(3,6,5,8)   =  M_dot.submat(6,3,8,5).t() ; // M23 C1T
    M_dot.submat(3,9,5,11)  =  M_dot.submat(9,3,11,5).t(); // M24 D1T
    
    //M1.submat(0,0,2,2)   =   zeros  // M11
    M_dot.submat(0,3,2,5)   =  M_dot.submat(3,0,5,2).t();  // M12 E1T
    M_dot.submat(0,6,2,8)   =  M_dot.submat(6,0,8,2).t();  // M13 A1T
    M_dot.submat(0,9,2,11)  =  M_dot.submat(9,0,11,2).t(); // M14 B1T
    
    /*
    mat A1 = -cross(cst.c_p1)*cross(omega_p1_p1i)*C_p1i;
    mat B1 = -cross(cst.c_p2)*cross(omega_p2_p2i)*C_p2i;
    mat C1 =  cross(cst.c_p1)*cross(omega_p1_p1s)*C_p1i*((C_si).T)*cross(cst.r_s_c1s);
    mat D1 =  cross(cst.c_p2)*cross(omega_p2_p2s)*C_p2i*((C_si).T)*cross(cst.r_s_c2s);
    mat E1 = -((cross(cst.r_s_c1s)*cst.m_p1)+(cross(cst.r_s_c2s)*cst.m_p2))*cross(omega_s_si)*C_si;

    mat M_dot_1=[zeros(3,3) E1.T A1.T B1.T; E1 zeros(3,3) C1.T D1.T; A1 C1 zeros(3,3) zeros(3,3); B1 D1 zeros(3,3) zeros(3,3)];
    
    */
    
    return ( (0.5) * ( M_dot + (M_dot).t() ));
    
}

//5. Mass matrix dependency on state variables
vec Partial_M(int n, constants cst)
{
	vec  M = zeros<vec>(n + 2);
	
	return M;
}

//6. Pi Matrix: Kinematic transformation matrix
mat pi_matrix(mat C_si, mat C_p1i, mat C_p2i, constants cst)
{
    // Define
    arma::mat pi_matrix = arma::zeros<mat>(12,5);
    
    pi_matrix.submat(0,0,2,2)  = -cst.r * cross(cst.unit_col_3) * (C_si).t();
    //pi_matrix.submat(0,3,2,3)  = zeros
    //pi_matrix.submat(0,4,2,4)  = zeros
     
    pi_matrix.submat(3,0,5,2)  = arma::eye(3,3);
    //pi_matrix.submat(3,3,5,3)  = zeros
    //pi_matrix.submat(3,4,5,4)  = zeros
     
    pi_matrix.submat(6,0,8,2)  = (C_p1i * (C_si).t());
    pi_matrix.submat(6,3,8,3)  = cst.unit_col_2;
    //pi_matrix.submat(6,4,8,4)  = zeros
    
    pi_matrix.submat(9,0,11,2) = (C_p2i * (C_si).t());
    //pi_matrix.submat(9,3,11,3) = zeros
    pi_matrix.submat(9,4,11,4) = cst.unit_col_2;
    
    //pi_matrix = [ -cst.r * cross([0;0;1]) * (C_si).T  zeros(3,1) zeros(3,1);
    //               eye(3)                             zeros(3,1) zeros(3,1);
    //               (C_p1i*(C_si).T)                   [0;1;0]    zeros(3,1);
    //               (C_p2i*(C_si).T)                   zeros(3,1) [0;1;0]];
    
    return pi_matrix;
}

//7. Pi dot Matrix: Kinematic transformation matrix
mat pi_dot_matrix(mat C_si, mat C_p1i, mat C_p2i, vec omega_s_si, vec omega_p1_p1s, vec omega_p2_p2s, constants cst)
{
    // Define
    arma::mat pi_matrix = arma::zeros<mat>(12,5);
    
    pi_matrix.submat(0,0,2,2)  = -cst.r * cross(cst.unit_col_3)*((C_si).t())*cross(omega_s_si);
    //pi_matrix.submat(0,3,2,3)  = zeros
    //pi_matrix.submat(0,4,2,4)  = zeros
     
    //pi_matrix.submat(3,0,5,2)  = zeros;
    //pi_matrix.submat(3,3,5,3)  = zeros
    //pi_matrix.submat(3,4,5,4)  = zeros
     
    pi_matrix.submat(6,0,8,2)  = -cross(omega_p1_p1s) * C_p1i * ((C_si).t());
    //pi_matrix.submat(6,3,8,3)  = zeros
    //pi_matrix.submat(6,4,8,4)  = zeros
    
    pi_matrix.submat(9,0,11,2) = -cross(omega_p2_p2s) * C_p2i * ((C_si).t());
    //pi_matrix.submat(9,3,11,3) = zeros
    //pi_matrix.submat(9,4,11,4) = zeros
    
 
    //mat pi_dot=[-cst.r*cross([0;0;1])*((C_si).T)*cross(omega_s_si)  zeros(3,1) zeros(3,1);
    //             zeros(3,3)                                         zeros(3,1) zeros(3,1);
    //            -cross(omega_p1_p1s)*C_p1i*((C_si).T)               zeros(3,1) zeros(3,1);
    //            -cross(omega_p2_p2s)*C_p2i*((C_si).T)               zeros(3,1) zeros(3,1)];
    
    return pi_matrix;
}

// 8. Delta_bar_T
 mat delta_bar_2_T_matrix(mat C_si, mat C_p1i, mat C_p2i, vec omega_s_si, vec omega_p1_p1i, vec omega_p2_p2i, constants cst)
{
    // Define
    arma::mat delta_bar_2_T = arma::zeros<mat>(5,12);
    
    //delta_bar_2_T.submat(0,0,2,2)  = zeros
    delta_bar_2_T.submat(0,3,2,5)   = cross(omega_s_si);
    delta_bar_2_T.submat(0,6,2,8)   = ( (C_si * (C_p1i.t() )) ) * cross(omega_p1_p1i);
    delta_bar_2_T.submat(0,9,2,11)  = ( (C_si * (C_p2i.t() )) ) * cross(omega_p2_p2i);
    
    //delta_bar_2_T.submat(3,0,3,2)  = zeros
    //delta_bar_2_T.submat(3,3,3,5)  = zeros
     delta_bar_2_T.submat(3,6,3,8)  = cst.unit_col_2.t() * cross(omega_p1_p1i);
    //delta_bar_2_T.submat(3,9,3,11)  = zeros
     
    //delta_bar_2_T.submat(4,0,4,2)  = zeros
    //delta_bar_2_T.submat(4,3,4,5)  = zeros
    //delta_bar_2_T.submat(4,6,4,8)  = zeros
     delta_bar_2_T.submat(4,9,4,11)  = cst.unit_col_2.t() * cross(omega_p2_p2i);
    
    
 
    //mat delta_bar_2_T=[zeros(3,3) cross(omega_s_si) ((C_si*((C_p1i).T)))*cross(omega_p1_p1i) ((C_si*((C_p2i).T)))*cross(omega_p2_p2i);
    //                   zeros(1,3) zeros(1,3) [0 1 0]*cross(omega_p1_p1i) zeros(1,3);
    //                   zeros(1,3) zeros(1,3) zeros(1,3) [0 1 0]*cross(omega_p2_p2i)];
    
    return delta_bar_2_T;
}

// 9. Delta_3:
 mat delta_3_T_matrix(mat C_si, mat C_p1i, mat C_p2i, vec omega_s_si, vec omega_p1_p1i, vec omega_p2_p2i, vec v_i_si_i, constants cst)
{
     // Define
     arma::mat delta_3 = arma::zeros<mat>(1,5);
     arma::mat A2 = arma::zeros<mat>(1,3);
     arma::mat B2 = arma::zeros<mat>(1,1);
     arma::mat C2 = arma::zeros<mat>(1,1);
     
     // Compute A2:
     A2 = A2 + ((omega_s_si).t())   * ((cross(cst.r_s_c1s) * cst.m_p1) + (cross(cst.r_s_c2s) * cst.m_p2)) * cross(C_si * v_i_si_i);
     A2 = A2 - ((omega_s_si).t())   *   cross(cst.r_s_c1s) * cross(C_si * ((C_p1i).t()) * cross(cst.c_p1) * omega_p1_p1i);
     A2 = A2 - ((omega_s_si).t())   *   cross(cst.r_s_c2s) * cross(C_si * ((C_p2i).t()) * cross(cst.c_p2) * omega_p2_p2i);
     A2 = A2 + ((omega_p1_p1i).t()) *   cross(cst.c_p1) * cross(C_p1i * v_i_si_i) * C_p1i * ((C_si).t());
     A2 = A2 - ((omega_p1_p1i).t()) *   cross(cst.c_p1) * cross(C_p1i * ((C_si).t()) * cross(cst.r_s_c1s) * omega_s_si) * C_p1i * ((C_si).t());
     A2 = A2 + ((omega_p2_p2i).t()) *   cross(cst.c_p2) * cross(C_p2i * v_i_si_i) * C_p2i * ((C_si).t());
     A2 = A2 - ((omega_p2_p2i).t()) *   cross(cst.c_p2) * cross(C_p2i * ((C_si).t()) * cross(cst.r_s_c2s) * omega_s_si) * C_p2i * ((C_si).t());
     A2 = A2 + cst.g * cst.m_p1 * cst.unit_col_3.t() * ((C_si).t()) * cross(cst.r_s_c1s);
     A2 = A2 + cst.g * cst.m_p2 * cst.unit_col_3.t() * ((C_si).t()) * cross(cst.r_s_c2s);
     A2 = A2 + cst.g * cst.unit_col_3.t() * (C_p1i).t() * cross(cst.c_p1) * C_p1i * ((C_si).t());
     A2 = A2 + cst.g * cst.unit_col_3.t() * (C_p2i).t() * cross(cst.c_p2) * C_p2i * ((C_si).t());
     
     // Compute B2:
     B2 = B2 + (((omega_p1_p1i).t()) * cross(cst.c_p1) * cross(C_p1i * v_i_si_i)) * cst.unit_col_2;
     B2 = B2 - (((omega_p1_p1i).t()) * cross(cst.c_p1) * cross(C_p1i * ((C_si).t()) * cross(cst.r_s_c1s) * omega_s_si)) * cst.unit_col_2;
     B2 = B2 + (cst.g * cst.unit_col_3.t() * C_p1i.t() * cross(cst.c_p1)) * cst.unit_col_2;
     
     // Compute C2:
     
     C2 = C2 + (((omega_p2_p2i).t()) * cross(cst.c_p2) * cross(C_p2i * v_i_si_i)) * cst.unit_col_2;
     C2 = C2 - (((omega_p2_p2i).t()) * cross(cst.c_p2) * cross(C_p2i * ((C_si).t()) * cross(cst.r_s_c2s) * omega_s_si)) * cst.unit_col_2;
     C2 = C2 + (cst.g * cst.unit_col_3.t() * C_p2i.t() * cross(cst.c_p2)) * cst.unit_col_2;
     
     // Assemble Delta_3:
     delta_3.submat(0,0,0,2)  = A2;
     delta_3(0,3)  = B2(0,0);
     delta_3(0,4)  = C2(0,0);
     
     return delta_3.t();
     
     /*
     mat A21= ((omega_s_si).T)*((cross(cst.r_s_c1s)*cst.m_p1)+(cross(cst.r_s_c2s)*cst.m_p2))*cross(C_si*v_i_si_i);
     mat A22=-((omega_s_si).T)*cross(cst.r_s_c1s)*cross(C_si*((C_p1i).T)*cross(cst.c_p1)*omega_p1_p1i);
     mat A23=-((omega_s_si).T)*cross(cst.r_s_c2s)*cross(C_si*((C_p2i).T)*cross(cst.c_p2)*omega_p2_p2i);
     mat A24= ((omega_p1_p1i).T)*cross(cst.c_p1)*cross(C_p1i*v_i_si_i)*C_p1i*((C_si).T);
     mat A25=-((omega_p1_p1i).T)*cross(cst.c_p1)*cross(C_p1i*((C_si).T)*cross(cst.r_s_c1s)*omega_s_si)*C_p1i*((C_si).T);
     mat A26= ((omega_p2_p2i).T)*cross(cst.c_p2)*cross(C_p2i*v_i_si_i)*C_p2i*((C_si).T);
     mat A27=-((omega_p2_p2i).T)*cross(cst.c_p2)*cross(C_p2i*((C_si).T)*cross(cst.r_s_c2s)*omega_s_si)*C_p2i*((C_si).T);
     
     mat A28=  cst.g*cst.m_p1*[0 0 1]*((C_si).T)*cross(cst.r_s_c1s);
     mat A29=  cst.g*cst.m_p2*[0 0 1]*((C_si).T)*cross(cst.r_s_c2s);
     mat A210= cst.g*[0 0 1]*(C_p1i).T * cross(cst.c_p1)*C_p1i*((C_si).T);
     mat A211= cst.g*[0 0 1]*(C_p2i).T * cross(cst.c_p2)*C_p2i*((C_si).T);

     mat A2=A21+A22+A23+A24+A25+A26+A27+A28+A29+A210+A211;
     mat B211=(((omega_p1_p1i)')*cross(cst.c_p1)*cross(C_p1i*v_i_si_i))*[0;1;0];
     mat B222=-(((omega_p1_p1i)')*cross(cst.c_p1)*cross(C_p1i*((C_si)')*cross(cst.r_s_c1s)*omega_s_si))*[0;1;0];
     mat B233=(cst.g*[0 0 1]*(C_p1i)'*cross(cst.c_p1))*[0;1;0];

     mat B2=B211+B222+B233;

     mat C211=(((omega_p2_p2i).T)*cross(cst.c_p2)*cross(C_p2i*v_i_si_i))*[0;1;0];
     mat C222=-(((omega_p2_p2i).T)*cross(cst.c_p2)*cross(C_p2i*((C_si).T)*cross(cst.r_s_c2s)*omega_s_si))*[0;1;0];
     mat C233=(cst.g*[0 0 1]*(C_p2i).T * cross(cst.c_p2))*[0;1;0];


     mat C2=C211+C222+C233;

     mat delta_3_T=([A2 B2 C2]).T;
     */
}

// 10. F_ext Matrix
 mat F_ext_matrix(mat C_si, mat C_p1i, mat C_p2i, vec omega_s_si, double omega_p1_p1s_2, double omega_p2_p2s_2, vec v_i_ws_i, vec v_i_si_i, constants cst)
{
     // Define
     arma::mat f_ext  = arma::zeros<mat>(12,1);
     arma::mat f_damp = arma::zeros<mat>(12,1);
     
     // tau_p1:
     f_damp.submat(6,0,8,0)  = - cst.b_p1 * omega_p1_p1s_2 * cst.unit_col_2;
     
     // tau_p2:
     f_damp.submat(9,0,11,0) = - cst.b_p2 * omega_p2_p2s_2 * cst.unit_col_2;
     
     // f_damp.subvec(0,0,2,0)  = zeros
     f_damp.submat(3,0,5,0)  =    - ( C_si * C_p1i.t()   * f_damp.submat(6,0,8,0) )
                                  - ( C_si * C_p2i.t()   * f_damp.submat(9,0,11,0)) ;
     
     // f_ext:
     f_ext.submat(0,0,2,0) = -  (  cst.br * v_i_si_i) + (cst.c_f * arma::norm(v_i_ws_i) * v_i_ws_i);
     f_ext.submat(3,0,5,0) = -  (  cst.bs * C_si * cst.unit_col_3 * cst.unit_col_3.t() * C_si.t()) * omega_s_si;
     //f_ext_1.submat(6,0,8,0) = zeros
     //f_ext_1.submat(9,0,11,0) = zeros
    
     return (f_ext  + f_damp );
     
     /*
     mat tau_p1 = -cst.b_p1 * omega_p1_p1s_2 * [0;1;0];
     mat tau_p2 = -cst.b_p2 * omega_p2_p2s_2 * [0;1;0];

     f_damp=[zeros(3,1); ((-((C_si)*((C_p1i).T))*tau_p1)-((C_si)*((C_p2i).T)*tau_p2)); tau_p1; tau_p2];

     F_ext_1=(([-((cst.br*v_i_si_i))+cst.c_f*norm_v_i_ws_i*(v_i_ws_i);-(cst.bs*C_si*[0;0;1]*(([0;0;1]).T)*((C_si).T))*omega_s_si; zeros(3,1); zeros(3,1)])+f_damp) ;
     F_ext=(pi.T)*F_ext_1;
     */
}

// 10. Energy Matrix
 double H_dot_matrix(mat C_si, mat C_p1i, mat C_p2i, vec omega_s_si, vec omega_p1_p1i, vec omega_p2_p2i, vec v_i_si_i, mat F_ext, mat M, constants cst)
{
     mat nu = zeros<mat>(12,1);
     mat H_dot = zeros<mat>(1,1);
     
     nu.submat(0,0,2,0)  = v_i_si_i;
     nu.submat(3,0,5,0)  = omega_s_si;
     nu.submat(6,0,8,0)  = omega_p1_p1i;
     nu.submat(9,0,11,0) = omega_p2_p2i;
     
     H_dot = H_dot + 0.5 * nu.t() * F_ext;
     H_dot = H_dot + 0.5 * F_ext.t() * nu;
     H_dot = H_dot + 0.5 * nu.t() * M * nu;

     H_dot = H_dot + cst.g * cst.unit_col_3.t() * C_si.t()  * cross(omega_s_si)   * cst.r_s_c1s * cst.m_p1;
     H_dot = H_dot + cst.g * cst.unit_col_3.t() * C_p1i.t() * cross(omega_p1_p1i) * cst.c_p1;
     H_dot = H_dot + cst.g * cst.unit_col_3.t() * C_si.t()  * cross(omega_s_si)   * cst.r_s_c2s * cst.m_p2;
     H_dot = H_dot + cst.g * cst.unit_col_3.t() * C_p2i.t() * cross(omega_p2_p2i) * cst.c_p2;

     return H_dot(0);
     
     /*
      mat nu=[v_i_si_i;omega_s_si;omega_p1_p1i;omega_p2_p2i];

      mat H_dot11 = 0.5*(nu.T)*F_ext_1;
      mat H_dot12 = 0.5*((F_ext_1).T)*nu;
      mat H_dot2  = 0.5*(nu.T)*M*nu;

      mat H_dot3 = cst.g * [0 0 1] * ((C_si).T)  * cross(omega_s_si)   * cst.r_s_c1s * cst.m_p1;
      mat H_dot4 = cst.g * [0 0 1] * ((C_p1i).T) * cross(omega_p1_p1i) * cst.c_p1;
      mat H_dot5 = cst.g * [0 0 1] * ((C_si).T)  * cross(omega_s_si)   * cst.r_s_c2s*cst.m_p2;
      mat H_dot6 = cst.g * [0 0 1] * ((C_p2i).T) * cross(omega_p2_p2i) * cst.c_p2;

      mat H_dot= H_dot11+H_dot12+H_dot2+H_dot3+H_dot4+H_dot5+H_dot6;
     */
}





          
          




