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

#include "L01PostProcessing.hpp"
#include "L01Model.hpp"

//#define DEBUG_PP 1

//16. Post Processing

void post_processing(double h, double t_start, double t_end, mat x_out, int calc_lagrange_multiplier , constants cst)
{
	//Initializing variables

	long long int a = (t_end - t_start) / h; //Need better counting Subroutine
	vec t_out(a);
    vec t(a);
    
    // Model Variables to save:
    vec det_C_si(a);
    vec det_C_p1i(a);
    vec det_C_p2i(a);
    vec omega_p1_p1s_2(a);
    vec omega_p2_p2s_2(a);
    
    // Angular Velocities:
    mat omega_p1_p1s(3,a);
    mat omega_p2_p2s(3,a);

    mat omega_p1_p1i(3,a);
    mat omega_p2_p2i(3,a);
    mat omega_i_p1i(3,a);
    mat omega_i_p2i(3,a);
    mat omega_cap(5,a);
    mat omega_s_si(3,a);
    mat omega_i_si(3,a);
    
    // Sphere variables:
    mat r_i_si(3,a);
    mat v_i_si_i(3,a);
    mat a_i_si_i(3,a);
    
    // State variables:
    mat nu(12,a);
    
    // Energy:
    vec H_ext(a);
    vec H_calc(a);
    vec U(a);
    vec K(a);
    vec del_omega(a);
    vec P_ext(a);
    vec W_ext(a);
    vec del_E(a);
    vec H_dot(a);
    
    double U_1 = 0.0;
    double U_2 = 0.0;
    
    // Model Container:
    SphericalRollingRobot Model;
    
	// Above Matrices being ocnstant through post processing are defined before the loop


	//Post processing:
	    	
	for(int i=0; i<a; i++)
	
	{
		//Generating time step
		if(i==0)
			t(i)= t_start;
		else
			t(i)=t(i-1)+h;

		//Extracting required variables:
        Model.PopulateInputs(x_out.col(i), cst);
        Model.SolveKinematics(cst);
        
#ifdef DEBUG_PP
        cout<<"model Populated" << endl;
#endif
        
        det_C_si(i)  = abs(det(Model.C_si));
        det_C_p1i(i) = abs(det(Model.C_p1i));
        det_C_p2i(i) = abs(det(Model.C_p2i));

#ifdef DEBUG_PP
        cout<<"DCM Populated" << endl;
#endif
        // Calculate Sphere Angular Velocities
        omega_s_si.col(i)  = Model.omega_s_si;
        omega_p1_p1s_2(i)  = Model.omega_p1_p1s_2;
        omega_p2_p2s_2(i)  = Model.omega_p1_p1s_2;
        
        omega_i_si.col(i)  = Model.C_si.t() * Model.omega_s_si;
        
        if (i>1)
        {
            del_omega(i)=(abs(Model.omega_p1_p1s_2  - Model.omega_p2_p2s_2))/((t(i)-t(i-1)));
        }
        else
        {    del_omega(i)=0;
        }
        

        omega_p1_p1s.col(i)  = Model.omega_p1_p1s;
        omega_p2_p2s.col(i)  = Model.omega_p2_p2s;

        omega_p1_p1i.col(i)  = Model.omega_p1_p1i;
        omega_p2_p2i.col(i)  = Model.omega_p2_p2i;

        omega_i_p1i.col(i) = ( Model.C_p1i.t() * Model.omega_p1_p1s + Model.C_si.t() * Model.omega_s_si );
        omega_i_p2i.col(i) = ( Model.C_p2i.t() * Model.omega_p2_p2s + Model.C_si.t() * Model.omega_s_si );
        
        omega_cap.col(i) = Model.omega_cap;

#ifdef DEBUG_PP
        cout<<"Omegas done" << endl;
#endif
        H_ext(i)  = Model.H;
        
        // Calculate sphere Positions
        r_i_si.col(i)  = Model.r_i_si;

#ifdef DEBUG_PP
        cout<<"Positions done" << endl;
#endif
        
        v_i_si_i.col(i) =  Model.v_i_si_i;

        if (i > 1)
        {
            a_i_si_i.col(i) =  (v_i_si_i.col(i)  -  v_i_si_i.col(i - 1) ) / ( t(i) - t(i-1) ) ;
        }
        else
        {
            a_i_si_i.col(i) = zeros(3,1);
        }
        
        //
        // Energy check
        //

        // Work done by external Forces:
        Model.F_ext  = F_ext_matrix( Model.C_si,
                                     Model.C_p1i,
                                     Model.C_p2i,
                                     Model.omega_s_si,
                                     Model.omega_p1_p1s_2,
                                     Model.omega_p2_p2s_2,
                                     Model.v_i_ws_i,
                                     Model.v_i_si_i,
                                     cst);
        
        
        nu.col(i).submat(0,0,2,0)  = Model.v_i_si_i;
        nu.col(i).submat(3,0,5,0)  = Model.omega_s_si;
        nu.col(i).submat(6,0,8,0)  = Model.omega_p1_p1i;
        nu.col(i).submat(9,0,11,0) = Model.omega_p2_p2i;
        
        // Hameltonian:
        Model.M = mass_matrix(Model.C_si,
                              Model.C_p1i,
                              Model.C_p2i,
                              cst);

        // Kinematic transformation matrix
        Model.pi = pi_matrix(Model.C_si,
                             Model.C_p1i,
                             Model.C_p2i,
                             cst);

        U_1 = as_scalar(
                         cst.unit_col_3.t() *  (   (cst.g *  Model.r_i_si *  cst.m_p1)
                                                 + (cst.g *  Model.C_si.t()  * cst.r_s_c1s * cst.m_p1)
                                                 + (cst.g *  Model.C_p1i.t() * cst.c_p1))
                        );
        U_2 = as_scalar(
                         cst.unit_col_3.t() * (    (cst.g *  Model.r_i_si    * cst.m_p2)
                                                 + (cst.g *  Model.C_si.t()  * cst.r_s_c2s * cst.m_p2)
                                                 + (cst.g *  Model.C_p2i.t() * cst.c_p2 ))
                        );
        
        U(i) = U_2 + U_1 + 147.15;
        
        K(i) = as_scalar(0.5 * nu.col(i).t() * Model.M * nu.col(i));
        
        H_calc(i)= U(i) + K(i);
        
        if (i>1)
        {
            // Calculate power supplied:
            P_ext(i)   = as_scalar((nu.col(i) - nu.col(i-1)).t() * Model.F_ext);
            
            // Integrate Power:
            W_ext(i) = W_ext(i-1) + P_ext(i) * ( (t(i) - t(i-1) ) );
            
            // Calculate H_dot:
            H_dot(i) = (H_calc(i) - H_calc(i-1)) / (t(i) - t(i-1));
        }
        else
        {
            P_ext(i) = 0;
            W_ext(i) = 0;
            H_dot(i) = 0;
        }
        del_E(i) = W_ext(i)-H_dot(i) ;
        
        //(a) Mass Matrix
        //(b) Stiffness Matrix
	    //(c) Hameltonian
	   
	    //H(i)=as_scalar((0.5*((trans(z_dot))*M_zz*z_dot)+(0.5*((trans(z))*K_zz*z))));
	    
        //Lagrange multiplier calculation starts here:
	    if(calc_lagrange_multiplier==1)
	    {
	    // Non Linear Components
	    // External forces and Control Laws
        }
     // Calculating Estimated Quantities:
	}



	//Saving relevant data:

    t.save(cst.GenDir + "t.txt", raw_ascii);
    x_out.save      (cst.GenDir  + "x_out.txt"       , raw_ascii);
    H_calc.save     (cst.GenDir  + "H.txt"           , raw_ascii);
    del_E.save      (cst.GenDir  + "del_E.txt"       , raw_ascii);
    det_C_si.save   (cst.GenDir  + "det_C_si.txt"    , raw_ascii);
    det_C_p1i.save  (cst.GenDir  + "det_C_p1i.txt"   , raw_ascii);
    det_C_p2i.save  (cst.GenDir  + "det_C_p2i.txt"   , raw_ascii);
    omega_i_si.save (cst.GenDir  + "omega_i_si.txt"  , raw_ascii);
    omega_i_p1i.save(cst.GenDir  + "omega_i_p1i.txt" , raw_ascii);
    omega_i_p2i.save(cst.GenDir  + "omega_i_p2i.txt" , raw_ascii);
    v_i_si_i.save   (cst.GenDir  + "v_i_si_i.txt"    , raw_ascii);
    a_i_si_i.save   (cst.GenDir  + "a_i_si_i.txt"    , raw_ascii);

    if(calc_lagrange_multiplier==1)
	{
	}
    cout << "C++ : Text saving process complete" << endl;
}
/*
 
 % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % Post Processing
 tic

 %%H = x_out(:,16); % This is the integral of H_dot.

 cst.output_flag = 1;

 for lv1 = 1:length(t)


     
     %energy check
     v_i_si_i(lv1,:)=(-cst.r*cross([0;0;1])*((C_si(:,:,lv1))')*((omega_s_si(lv1,:))'))';

     v_i_ws_i(lv1,:)=((cst.v_i_wi_i)')-(v_i_si_i(lv1,:));

     norm_v_i_ws_i(lv1,:)=norm(v_i_ws_i(lv1,:));

     tau_p1=-cst.b_p1*omega_p1_p1s_2(lv1)*[0;1;0];
     tau_p2=-cst.b_p2*omega_p1_p1s_2(lv1)*[0;1;0];

     f_damp=[zeros(3,1); ((-((C_si(:,:,lv1))*((C_p1i(:,:,lv1))'))*tau_p1)-((C_si(:,:,lv1))*((C_p2i(:,:,lv1))')*tau_p2)); tau_p1; tau_p2];


     F_ext(:,lv1)=(([-((cst.br*((v_i_si_i(lv1,:))')))+cst.c_f*((norm_v_i_ws_i(lv1,:))')*(((v_i_ws_i(lv1,:))'));-(cst.bs*C_si(:,:,lv1)*[0;0;1]*(([0;0;1])')*((C_si(:,:,lv1))'))*((omega_s_si(lv1,:))'); zeros(3,1); zeros(3,1)])+f_damp) ;

     if lv1>1
     
         %W_ext(lv1)=[(v_i_si_i(lv1,:)) (omega_s_si(lv1,:)) (omega_p1_p1i(lv1,:)) (omega_p2_p2i(lv1,:))]*F_ext(:,lv1);
     
         P_ext(lv1)=[(v_i_si_i(lv1,:)-v_i_si_i(lv1-1,:)) (omega_s_si(lv1,:)-omega_s_si(lv1-1,:)) (omega_p1_p1i(lv1,:)-omega_p1_p1i(lv1-1,:)) (omega_p2_p2i(lv1,:)-omega_p2_p2i(lv1,:))]*F_ext(:,lv1);
     else
         P_ext(lv1)=0;
     end
     
     if lv1>1
     
         W_ext(lv1)=W_ext(lv1-1)+P_ext(lv1)*((t(lv1)-t(lv1-1)));
     else
         W_ext(lv1)=0;
     end
     
     
     %hameltonian:
      pi=[-cst.r*cross([0;0;1])*(C_si(:,:,lv1))' zeros(3,1) zeros(3,1); eye(3) zeros(3,1) zeros(3,1); (C_p1i(:,:,lv1)*(C_si(:,:,lv1))') [0;1;0] zeros(3,1);(C_p2i(:,:,lv1)*(C_si(:,:,lv1))') zeros(3,1) [0;1;0]];
      
      M = mass_matrix(C_si(:,:,lv1),C_p1i(:,:,lv1),C_p2i(:,:,lv1),cst);
      U_1=[0 0 1]*(((cst.g*(r_i_si(lv1,:))')*cst.m_p1)+(cst.g*((C_si(:,:,lv1))')*cst.r_s_c1s*cst.m_p1)+(cst.g*((C_p1i(:,:,lv1))')*cst.c_p1));
      U_2=[0 0 1]*(((cst.g*(r_i_si(lv1,:))')*cst.m_p2)+(cst.g*((C_si(:,:,lv1))')*cst.r_s_c2s*cst.m_p2)+(cst.g*((C_p2i(:,:,lv1))')*cst.c_p2));
      U_3(lv1)=U_2+U_1+147.15;
      
      evalues=eig(M);
      flag1(lv1)=1;
      for i=1:rank(M)
          
          if flag1(lv1)==1
            if evalues(i)<=0
              flag1(lv1)=0;
            end
          end
          
      end
      
      nu=[v_i_si_i(lv1,:) omega_s_si(lv1,:) omega_p1_p1i(lv1,:) omega_p2_p2i(lv1,:)]';

      H_calc(lv1)=(0.5*(nu')*M*nu)+U_1+U_2+147.15;
      
      %H_calc(lv1)=H_calc*(t(lv1)-t(lv1-1));
      %H_calc(lv1)=(0.5*((omega_cap)')*((pi)')*M*pi*omega_cap)
       
      if lv1>1
       H_calc1(lv1)=H_calc(lv1)*(t(lv1)-t(lv1-1));
       H_dot(lv1)=((H_calc(lv1))-H_calc((lv1-1)))/((t(lv1)-t(lv1-1)));
       else
            H_dot(lv1)=0;
            H_calc(lv1)=0;
       end


     
     del_E(lv1)=abs(W_ext(lv1)-H_dot(lv1));
 %     v_a_zw_a(lv1,:) = x_out_post(lv1,10:12);
 %     omega_b_ba(lv1,:) = x_out_post(lv1,13:15);
 %
 %     phi_ba(lv1) = acos( ( trace(C_si(:,:,lv1)) - 1 )/2 );
 %     omega_b_ba_norm(lv1) = sqrt( omega_b_ba(lv1,:)*omega_b_ba(lv1,:).' );
 %
 %     det_C_ba_neg_1(lv1) = abs(det(C_si(:,:,lv1)) - 1);
 %
 %     E(lv1) = energy(r_a_zw(lv1,:).',C_si(:,:,lv1),v_a_zw_a(lv1,:).',omega_b_ba(lv1,:).',cst);
 %     Delta_E(lv1) = abs(E(lv1)-H(lv1)); % You fill in!
 %
 end


 font_size = 15;
 line_size = 15;
 line_width = 2;

 figure
 plot(t,W_ext,'Linewidth',line_width);
 hold on
 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$W_{ext}$ ','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on


 figure
 plot(t,H_calc,'Linewidth',line_width);
 hold on
 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$H$ Hameltonian','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on



 figure
 plot(t,del_E,'Linewidth',line_width);
 hold on
 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$Delta$ E','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on


 figure
 stairs(t,flag1,'Linewidth',line_width);
 hold on
 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('eig check','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on


    
 % figure
 % subplot(3,1,1)
 % plot(t,v_i_ws_i(:,1),'Linewidth',line_width);
 % hold on
 % xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 % ylabel('$v_{i1}^{si/i}$ (m)','fontsize',font_size,'Interpreter','latex');
 % set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 % grid on
 % subplot(3,1,2)
 % plot(t,v_i_ws_i(:,2),'Linewidth',line_width);
 % hold on
 % xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 % ylabel('$v_{i2}^{si/i}$ (m)','fontsize',font_size,'Interpreter','latex');
 % set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 % grid on
 % subplot(3,1,3)
 % plot(t,v_i_ws_i(:,3),'Linewidth',line_width);
 % hold on
 % xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 % ylabel('$v_{i3}^{si/i}$ (m)','fontsize',font_size,'Interpreter','latex');
 % set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 % grid on


 %
 % figure
 % subplot(3,1,1)
 % plot(t,F_ext(1,:),'Linewidth',line_width);
 % hold on
 % xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 % ylabel('$Fext$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 % set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 % grid on
 % subplot(3,1,2)
 % plot(t,F_ext(2,:),'Linewidth',line_width);
 % hold on
 % xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 % ylabel('$F_ext$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 % set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 % grid on
 % subplot(3,1,3)
 % plot(t,F_ext(3,:),'Linewidth',line_width);
 % hold on
 % xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 % ylabel('$F_ext$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 % set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 % grid on
 %
 %
 % figure
 % subplot(3,1,1)
 % plot(t,F_ext(4,:),'Linewidth',line_width);
 % hold on
 % xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 % ylabel('$Fext$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 % set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 % grid on
 % subplot(3,1,2)
 % plot(t,F_ext(5,:),'Linewidth',line_width);
 % hold on
 % xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 % ylabel('$F_ext$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 % set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 % grid on
 % subplot(3,1,3)
 % plot(t,F_ext(6,:),'Linewidth',line_width);
 % hold on
 % xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 % ylabel('$F_ext$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 % set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 % grid on
 %
 %
 % figure
 % subplot(3,1,1)
 % plot(t,F_ext(7,:),'Linewidth',line_width);
 % hold on
 % xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 % ylabel('$Fext$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 % set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 % grid on
 % subplot(3,1,2)
 % plot(t,F_ext(8,:),'Linewidth',line_width);
 % hold on
 % xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 % ylabel('$F_ext$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 % set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 % grid on
 % subplot(3,1,3)
 % plot(t,F_ext(9,:),'Linewidth',line_width);
 % hold on
 % xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 % ylabel('$F_ext$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 % set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 % grid on
 %
 %
 %
 % figure
 % subplot(3,1,1)
 % plot(t,F_ext(10,:),'Linewidth',line_width);
 % hold on
 % xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 % ylabel('$Fext$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 % set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 % grid on
 % subplot(3,1,2)
 % plot(t,F_ext(11,:),'Linewidth',line_width);
 % hold on
 % xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 % ylabel('$F_ext$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 % set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 % grid on
 % subplot(3,1,3)
 % plot(t,F_ext(12,:),'Linewidth',line_width);
 % hold on
 % xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 % ylabel('$F_ext$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 % set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 % grid on


 figure
 subplot(3,1,1)
 plot(t,omega_i_si(:,1),'Linewidth',line_width);
 hold on
 axislims=axis;
 axis([axislims(1) axislims(2) axislims(3)-0.1 axislims(4)+0.1]);
 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$\omega_{i1}^{si}$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
 subplot(3,1,2)
 plot(t,omega_i_si(:,2),'Linewidth',line_width);
 hold on
 axislims=axis;
 axis([axislims(1) axislims(2) axislims(3)-0.1 axislims(4)+0.1]);
 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$\omega_{i2}^{si}$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
 subplot(3,1,3)
 plot(t,omega_i_si(:,3),'Linewidth',line_width);
 hold on
 axislims=axis;
 axis([axislims(1) axislims(2) axislims(3)-0.1 axislims(4)+0.1]);
 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$\omega_{i3}^{si}$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
     
 time_stamp_interp = toc


 figure
 subplot(3,1,1)
 plot(t,omega_i_p1i(:,1),'Linewidth',line_width);
 hold on
 axislims=axis;
 axis([axislims(1) axislims(2) axislims(3)-0.1 axislims(4)+0.1]);
 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$\omega_{i1}^{p1i}$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
 subplot(3,1,2)
 plot(t,omega_i_p1i(:,2),'Linewidth',line_width);
 hold on
 axislims=axis;
 axis([axislims(1) axislims(2) axislims(3)-0.1 axislims(4)+0.1]);
 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$\omega_{i2}^{p2i}$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
 subplot(3,1,3)
 plot(t,omega_i_p1i(:,3),'Linewidth',line_width);
 hold on
 axislims=axis;
 axis([axislims(1) axislims(2) axislims(3)-0.1 axislims(4)+0.1]);

 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$\omega_{i3}^{p2i}$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
 %


 figure
 subplot(3,1,1)
 plot(t,omega_i_p2i(:,1),'Linewidth',line_width);
 hold on
 axislims=axis;
 axis([axislims(1) axislims(2) axislims(3)-0.1 axislims(4)+0.1]);

 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$\omega_{i1}^{p2i}$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
 subplot(3,1,2)
 plot(t,omega_i_p2i(:,2),'Linewidth',line_width);
 hold on
 axislims=axis;
 axis([axislims(1) axislims(2) axislims(3)-0.1 axislims(4)+0.1]);

 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$\omega_{i2}^{p2i}$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
 subplot(3,1,3)
 plot(t,omega_i_p2i(:,3),'Linewidth',line_width);
 hold on
 axislims=axis;
 axis([axislims(1) axislims(2) axislims(3)-0.1 axislims(4)+0.1]);

 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$\omega_{i3}^{p2i}$ (rad/s)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on

 figure
 subplot(3,1,1)
 plot(t,v_i_si_i(:,1),'Linewidth',line_width);
 hold on
 axislims=axis;
 axis([axislims(1) axislims(2) axislims(3)-0.1 axislims(4)+0.1]);

 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$v_{i1}^{si/i}$ (m)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
 subplot(3,1,2)
 plot(t,v_i_si_i(:,2),'Linewidth',line_width);
 hold on
 axislims=axis;
 axis([axislims(1) axislims(2) axislims(3)-0.1 axislims(4)+0.1]);

 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$v_{i2}^{si/i}$ (m)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
 subplot(3,1,3)
 plot(t,v_i_si_i(:,3),'Linewidth',line_width);
 hold on
 axislims=axis;
 axis([axislims(1) axislims(2) axislims(3)-0.1 axislims(4)+0.1]);

 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$v_{i3}^{si/i}$ (m)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on


 figure
 subplot(3,1,1)
 plot(t,a_i_si_i(:,1),'Linewidth',line_width);
 hold on
 axislims=axis;
 axis([axislims(1) axislims(2) axislims(3)-0.1 axislims(4)+0.1]);

 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$a_{i1}^{si/i}$ (m)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
 subplot(3,1,2)
 plot(t,a_i_si_i(:,2),'Linewidth',line_width);
 hold on
 axislims=axis;
 axis([axislims(1) axislims(2) axislims(3)-0.1 axislims(4)+0.1]);

 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$a_{i2}^{si/i}$ (m)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
 subplot(3,1,3)
 plot(t,a_i_si_i(:,3),'Linewidth',line_width);
 hold on
 axislims=axis;
 axis([axislims(1) axislims(2) axislims(3)-0.1 axislims(4)+0.1]);

 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$a_{i3}^{si/i}$ (m)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on


 figure
 plot(t,omega_p1_p1s_2,'Linewidth',line_width);
 hold on
 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$omega_(p1)^(p1s)$ (rad)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
 % print -depsc -r720 plots/?
 figure
 plot(t,omega_p2_p2s_2,'Linewidth',line_width);
 hold on
 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$omega_(p2)^(p2s)$ (rad)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
 % print -depsc -r720 plots/?
 figure
 semilogy(t,del_omega,'Linewidth',line_width);
 hold on
 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$delta$$omega$','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
 % print -depsc -r720 plots/?



 figure
 subplot(3,1,1)
 semilogy(t,det_C_si,'Linewidth',line_width);
 hold on
 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('det${\bf C}_{si}$ - 1','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
 subplot(3,1,2)
 semilogy(t,det_C_p1i,'Linewidth',line_width);
 hold on
 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$det(C_p1i-1)$ (J)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on
 subplot(3,1,3)
 semilogy(t,det_C_p2i,'Linewidth',line_width);
 hold on
 xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
 ylabel('$det c_p2i$ (J)','fontsize',font_size,'Interpreter','latex');
 set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
 grid on

 
 */
