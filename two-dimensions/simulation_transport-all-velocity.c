#include <stdio.h>
#include <math.h>

#define	T	25 //seconds
#define ST	0.0002 //second
#define GDIS	2.0
#define K_p	10.0
#define K_v	50.0
#define M_c	1.0
#define K_f	0.5
#define K_o	1000.0 //N/m
#define D_o	1.0
#define M_o	3.0 //kg
#define J_o	2.0
#define L_o	1.000 //meter
#define R_w	0.033 //meter
#define R_r	0.08 //meter
#define K_tn	0.4   //Nm
#define J_n	0.01 //Ns^2/m
#define D	0.06
#define Pi	3.1415
#define D_f	50.0
#define D_f_rotation	100
#define K_phi	5000.0
#define K_dphi	50.0
#define K_intphi	0.2
#define M_r	1.0 //kg
#define J_r	0.0032

#define K_pa	0.5
#define K_ia	50.0
#define K_da	0.0
#define K_pl	0.5	
#define K_il	50.0
#define K_dl	0.0

int main(void)
{
	double t = 0.0;

	double v_t = 0.0;
	double w_t = 0.0;
	double v_left_t = 0.0;
	double v_right_t = 0.0;
	double v_left_inverse_t = 0.0;
	double v_right_inverse_t = 0.0;

	int firstRound = 1;

/*LEFT1*/
/*Reference Commands*/

	double f_left1_command = 0.0;

/*Mobile Robot Outputs*/

	double x_res_left1 = -1.1;
	double dx_res_left1 = 0.0;
	double ddx_res_left1 = 0.0;
	double y_res_left1 = 0.6;
	double dy_res_left1 = 0.0;
	double ddy_res_left1 = 0.0;
	double phi_res_left1 = 0.0;
	double dphi_res_left1 = 0.0;
	double ddphi_res_left1 = 0.0;

	double x_c_left1 = 0.0;
	double y_c_left1 = 0.0;
	
	double l_left1 = 0.0;
	double dl_left1 = 0.0;
	double l_left1_prev = 0.0;

	double err_v_left_left1 = 0.0;
	double err_int_v_left_left1 = 0.0;
	double err_der_v_left_left1 = 0.0;
	double pre_err_v_left_left1 = 0.0;
	double err_v_right_left1 = 0.0;
	double err_int_v_right_left1 = 0.0;
	double err_der_v_right_left1 = 0.0;
	double pre_err_v_right_left1 = 0.0;

	double f_res_left1 = 0.0;
	double tau_res_left1 = 0.0;

/*Mobile Robot Inputs*/
	
	double cmd_acc_left_left1 = 0.0;
	double cmd_acc_right_left1 = 0.0;

	double f_dis_left1 = 0.0;


/*left1 Motor*/

	double ddtheta_ref_left1_l = 0.0;
	double ddtheta_ref_left1_r = 0.0;
	
	double ia_ref_left1_l = 0.0;
	double ia_ref_left1_r = 0.0;

	double ia_left1_l = 0.0;
	double ia_left1_r = 0.0;

	double tau_dis_left1_l = 0.0;
	double tau_dis_left1_r = 0.0;

	double tau_dob_left1_l = 0.0;
	double tau_dob_left1_r = 0.0;

	double integral_tau_dob_left1_l = 0.0;
	double integral_tau_dob_left1_r = 0.0;

	double tau_rtob_left1_l = 0.0;
	double tau_rtob_left1_r = 0.0;

	double integral_tau_rtob_left1_l = 0.0;
	double integral_tau_rtob_left1_r = 0.0;

	double theta_res_left1_l = 0.0;
	double theta_res_left1_r = 0.0;

	double dtheta_res_left1_l = 0.0;
	double dtheta_res_left1_r = 0.0;

	double ddtheta_res_left1_l = 0.0;
	double ddtheta_res_left1_r = 0.0;

/*LEFT2*/
/*Reference Commands*/

	double f_left2_command = 0.0;

/*Mobile Robot Outputs*/

	double x_res_left2 = -1.1;
	double dx_res_left2 = 0.0;
	double ddx_res_left2 = 0.0;
	double y_res_left2 = -0.6;
	double dy_res_left2 = 0.0;
	double ddy_res_left2 = 0.0;
	double phi_res_left2 = 0.0;
	double dphi_res_left2 = 0.0;
	double ddphi_res_left2 = 0.0;

	double x_c_left2 = 0.0;
	double y_c_left2 = 0.0;
	
	double l_left2 = 0.0;
	double dl_left2 = 0.0;
	double l_left2_prev = 0.0;

	double err_v_left_left2 = 0.0;
	double err_int_v_left_left2 = 0.0;
	double err_der_v_left_left2 = 0.0;
	double pre_err_v_left_left2 = 0.0;
	double err_v_right_left2 = 0.0;
	double err_int_v_right_left2 = 0.0;
	double err_der_v_right_left2 = 0.0;
	double pre_err_v_right_left2 = 0.0;

	double f_res_left2 = 0.0;
	double tau_res_left2 = 0.0;

/*Mobile Robot Inputs*/
	
	double cmd_acc_left_left2 = 0.0;
	double cmd_acc_right_left2 = 0.0;

	double f_dis_left2 = 0.0;

/*left2 Motor*/

	double ddtheta_ref_left2_l = 0.0;
	double ddtheta_ref_left2_r = 0.0;
	
	double ia_ref_left2_l = 0.0;
	double ia_ref_left2_r = 0.0;

	double ia_left2_l = 0.0;
	double ia_left2_r = 0.0;

	double tau_dis_left2_l = 0.0;
	double tau_dis_left2_r = 0.0;

	double tau_dob_left2_l = 0.0;
	double tau_dob_left2_r = 0.0;

	double integral_tau_dob_left2_l = 0.0;
	double integral_tau_dob_left2_r = 0.0;

	double tau_rtob_left2_l = 0.0;
	double tau_rtob_left2_r = 0.0;

	double integral_tau_rtob_left2_l = 0.0;
	double integral_tau_rtob_left2_r = 0.0;

	double theta_res_left2_l = 0.0;
	double theta_res_left2_r = 0.0;

	double dtheta_res_left2_l = 0.0;
	double dtheta_res_left2_r = 0.0;

	double ddtheta_res_left2_l = 0.0;
	double ddtheta_res_left2_r = 0.0;


/*RIGHT 1*/
/*Reference Commands*/

	double f_right1_command = 0.0;

/*Mobile Robot Outputs*/

	double x_res_right1 = +1.1;
	double dx_res_right1 = 0.0;
	double ddx_res_right1 = 0.0;
	double y_res_right1 = 0.0;
	double dy_res_right1 = 0.0;
	double ddy_res_right1 = 0.0;
	double phi_res_right1 = Pi;
	double dphi_res_right1 = 0.0;
	double ddphi_res_right1 = 0.0;

	double x_c_right1 = 0.0;
	double y_c_right1 = 0.0;
	
	double l_right1 = 0.0;
	double dl_right1 = 0.0;
	double l_right1_prev = 0.0;

	double err_v_left_right1 = 0.0;
	double err_int_v_left_right1 = 0.0;
	double err_der_v_left_right1 = 0.0;
	double pre_err_v_left_right1 = 0.0;
	double err_v_right_right1 = 0.0;
	double err_int_v_right_right1 = 0.0;
	double err_der_v_right_right1 = 0.0;
	double pre_err_v_right_right1 = 0.0;

	double f_res_right1 = 0.0;
	double tau_res_right1 = 0.0;

/*Mobile Robot Inputs*/
	
	double cmd_acc_left_right1 = 0.0;
	double cmd_acc_right_right1 = 0.0;

	double f_dis_right1 = 0.0;

/*right1 Motor*/

	double ddtheta_ref_right1_l = 0.0;
	double ddtheta_ref_right1_r = 0.0;
	
	double ia_ref_right1_l = 0.0;
	double ia_ref_right1_r = 0.0;

	double ia_right1_l = 0.0;
	double ia_right1_r = 0.0;

	double tau_dis_right1_l = 0.0;
	double tau_dis_right1_r = 0.0;

	double tau_dob_right1_l = 0.0;
	double tau_dob_right1_r = 0.0;

	double integral_tau_dob_right1_l = 0.0;
	double integral_tau_dob_right1_r = 0.0;

	double tau_rtob_right1_l = 0.0;
	double tau_rtob_right1_r = 0.0;

	double integral_tau_rtob_right1_l = 0.0;
	double integral_tau_rtob_right1_r = 0.0;

	double theta_res_right1_l = 0.0;
	double theta_res_right1_r = 0.0;

	double dtheta_res_right1_l = 0.0;
	double dtheta_res_right1_r = 0.0;

	double ddtheta_res_right1_l = 0.0;
	double ddtheta_res_right1_r = 0.0;


/*OBJECT*/
	double x_o = 0.0;
	double dx_o = 0.0;
	double ddx_o = 0.0;
	double y_o = 0.0;
	double dy_o = 0.0;
	double ddy_o = 0.0;
	double intphi_o = 0.0;
	double phi_o = 0.0;
	double dphi_o = 0.0;
	double ddphi_o = 0.0;


	FILE *fp;

	if((fp=fopen("simulation_transport_all_velocity.dat", "w"))==NULL){
		fprintf(stderr, "File open failed.\n");
		return 1;
	}

	while(t <= T){	

		if (t<10){
			v_t = 0.2;
			w_t = 0.0;
		}
		else if(t>=10 && t<15){
			v_t = 0.2;
			w_t = Pi/10;
		}
		else if(t>=15 && t<25){
			v_t = 0.2;
			w_t = 0.0;
		}
		else{
			v_t = 0.0;
			w_t = 0.0;
		}

		v_left_t = v_t/R_w-R_r*w_t/R_w;
		v_right_t = v_t/R_w+R_r*w_t/R_w;
		v_left_inverse_t = -v_t/R_w-R_r*w_t/R_w;
		v_right_inverse_t = -v_t/R_w+R_r*w_t/R_w;

		/*left1 commands*/

		f_left1_command = 3.0;
		
		x_c_left1 = x_res_left1 + cos(phi_o)*R_r-sin(phi_o)*(0.0);
		y_c_left1 = y_res_left1 + sin(phi_o)*R_r+cos(phi_o)*(0.0);

		if(cos(phi_o)*(x_c_left1-x_o+cos(phi_o)*L_o)+sin(phi_o)*(y_c_left1-y_o+sin(phi_o)*L_o)>0){
			//f_dis_left1 = K_o*l_left1 + D_o*dl_left1; 
			f_dis_left1 = 0;
		}
		else{
			f_dis_left1 = 0.0;
		}
		
		/*left1 Mobile Robot Controller*/
		
		

		err_v_left_left1 = v_left_t - dtheta_res_left1_l;
		err_v_right_left1 = v_right_t - dtheta_res_left1_r;
		err_int_v_left_left1 = err_int_v_left_left1 + err_v_left_left1*ST;
 		err_int_v_right_left1 = err_int_v_right_left1 + err_v_right_left1*ST;
		err_der_v_left_left1 = (err_v_left_left1 - pre_err_v_left_left1)/ST;
		err_der_v_right_left1 = (err_v_right_left1 - pre_err_v_right_left1)/ST;
		pre_err_v_left_left1 = err_v_left_left1;
		pre_err_v_right_left1 = err_v_right_left1;

		cmd_acc_left_left1 = K_pl*(err_v_left_left1)+K_il*(err_int_v_left_left1)+K_dl*(err_der_v_left_left1);
		cmd_acc_right_left1 = K_pa*(err_v_right_left1)+K_ia*(err_int_v_right_left1)+K_da*(err_der_v_right_left1);

		/*left1 Motor Controller*/

		ddtheta_ref_left1_l = cmd_acc_left_left1;
		ddtheta_ref_left1_r = cmd_acc_right_left1;

		ia_ref_left1_l = J_n*ddtheta_ref_left1_l/K_tn;
		ia_ref_left1_r = J_n*ddtheta_ref_left1_r/K_tn;

		tau_dis_left1_l = f_dis_left1*R_w*(1-sin(phi_res_left1-phi_o))/2 + (D*dtheta_res_left1_l) ;
		tau_dis_left1_r = f_dis_left1*R_w*(1+sin(phi_res_left1-phi_o))/2 + (D*dtheta_res_left1_r) ;

		ia_left1_l = ia_ref_left1_l + tau_dob_left1_l/K_tn;
		ia_left1_r = ia_ref_left1_r + tau_dob_left1_r/K_tn;

		ddtheta_res_left1_l = (K_tn*ia_left1_l - tau_dis_left1_l)/J_n; //ajouter masse et inertie du robot ici
		ddtheta_res_left1_r = (K_tn*ia_left1_r - tau_dis_left1_r)/J_n; //ajouter masse et inertie du robot ici

		dtheta_res_left1_l += ddtheta_res_left1_l * ST;
		dtheta_res_left1_r += ddtheta_res_left1_r * ST;

		theta_res_left1_l += dtheta_res_left1_l * ST;
		theta_res_left1_r += dtheta_res_left1_r * ST;

		/*left1 Robot Motion*/
		ddx_res_left1 = R_w*(ddtheta_res_left1_l+ddtheta_res_left1_r)*cos(phi_res_left1)/2 -D_f*dx_res_left1;
		dx_res_left1 += ddx_res_left1*ST;
		x_res_left1 += dx_res_left1*ST;

		ddy_res_left1 = R_w*(ddtheta_res_left1_l+ddtheta_res_left1_r)*sin(phi_res_left1)/2-D_f*dy_res_left1;
		dy_res_left1 += ddy_res_left1*ST;
		y_res_left1 += dy_res_left1*ST;

		ddphi_res_left1 = R_w*(ddtheta_res_left1_r-ddtheta_res_left1_l)/(2*R_r);
		dphi_res_left1 += ddphi_res_left1*ST; 
		phi_res_left1 += dphi_res_left1*ST;

		/*left2 commands*/

		f_left2_command = 3.0;
		
		x_c_left2 = x_res_left2 + cos(phi_o)*R_r-sin(phi_o)*(0.0);
		y_c_left2 = y_res_left2 + sin(phi_o)*R_r+cos(phi_o)*(0.0);

		if(cos(phi_o)*(x_c_left2-x_o+cos(phi_o)*L_o)+sin(phi_o)*(y_c_left2-y_o+sin(phi_o)*L_o)>0){
			f_dis_left2 = 0;
			//f_dis_left2 = K_o*l_left2 + D_o*dl_left2;
		}
		else{
			f_dis_left2 = 0.0;
		}
		
		/*left2 Mobile Robot Controller*/


		err_v_left_left2 = v_left_t - dtheta_res_left2_l;
		err_v_right_left2 = v_right_t - dtheta_res_left2_r;
		err_int_v_left_left2 = err_int_v_left_left2 + err_v_left_left2*ST;
 		err_int_v_right_left2 = err_int_v_right_left2 + err_v_right_left2*ST;
		err_der_v_left_left2 = (err_v_left_left2 - pre_err_v_left_left2)/ST;
		err_der_v_right_left2 = (err_v_right_left2 - pre_err_v_right_left2)/ST;
		pre_err_v_left_left2 = err_v_left_left2;
		pre_err_v_right_left2 = err_v_right_left2;

		cmd_acc_left_left2 = K_pl*(err_v_left_left2)+K_il*(err_int_v_left_left2)+K_dl*(err_der_v_left_left2);
		cmd_acc_right_left2 = K_pa*(err_v_right_left2)+K_ia*(err_int_v_right_left2)+K_da*(err_der_v_right_left2);

		/*left2 Motor Controller*/

		ddtheta_ref_left2_l = cmd_acc_left_left2;
		ddtheta_ref_left2_r = cmd_acc_right_left2;
		ia_ref_left2_l = J_n*ddtheta_ref_left2_l/K_tn;
		ia_ref_left2_r = J_n*ddtheta_ref_left2_r/K_tn;

		tau_dis_left2_l = f_dis_left2*R_w*(1-sin(phi_res_left2-phi_o))/2 + D*dtheta_res_left2_l;
		tau_dis_left2_r = f_dis_left2*R_w*(1+sin(phi_res_left2-phi_o))/2 + D*dtheta_res_left2_r;

		ia_left2_l = ia_ref_left2_l + tau_dob_left2_l/K_tn;
		ia_left2_r = ia_ref_left2_r + tau_dob_left2_r/K_tn;

		ddtheta_res_left2_l = (K_tn*ia_left2_l - tau_dis_left2_l)/J_n; //ajouter masse et inertie du robot ici
		ddtheta_res_left2_r = (K_tn*ia_left2_r - tau_dis_left2_r)/J_n; //ajouter masse et inertie du robot ici

		dtheta_res_left2_l += ddtheta_res_left2_l * ST;
		dtheta_res_left2_r += ddtheta_res_left2_r * ST;

		theta_res_left2_l += dtheta_res_left2_l * ST;
		theta_res_left2_r += dtheta_res_left2_r * ST;

		/*left2 Robot Motion*/
		ddx_res_left2 = R_w*(ddtheta_res_left2_l+ddtheta_res_left2_r)*cos(phi_res_left2)/2-D_f*dx_res_left2;
		dx_res_left2 += ddx_res_left2*ST;
		x_res_left2 += dx_res_left2*ST;

		ddy_res_left2 = R_w*(ddtheta_res_left2_l+ddtheta_res_left2_r)*sin(phi_res_left2)/2-D_f*dy_res_left2;
		dy_res_left2 += ddy_res_left2*ST;
		y_res_left2 += dy_res_left2*ST;

		ddphi_res_left2 = R_w*(ddtheta_res_left2_r-ddtheta_res_left2_l)/(2*R_r);
		dphi_res_left2 += ddphi_res_left2*ST; 
		phi_res_left2 += dphi_res_left2*ST;


		/*right1 commands*/


		f_right1_command = -3.0;
		
		x_c_right1 = x_res_right1 + (cos(phi_o+Pi)*(-R_r)-sin(phi_o+Pi)*(0.0));
		y_c_right1 = y_res_right1 + (sin(phi_o+Pi)*(-R_r)+cos(phi_o+Pi)*(0.0));

		if(cos(phi_o)*(x_c_right1-x_o-cos(phi_o)*L_o)+sin(phi_o)*(y_c_right1-y_o-sin(phi_o)*L_o)<0){
			f_dis_right1 = 0;
			//f_dis_right1 = (K_o*l_right1 + D_o*dl_right1); 
		}
		else{
			f_dis_right1 = 0.0;
		}
		
		/*right1 Mobile Robot Controller*/

		err_v_left_right1 = v_left_inverse_t - dtheta_res_right1_l;
		err_v_right_right1 = v_right_inverse_t - dtheta_res_right1_r;
		err_int_v_left_right1 = err_int_v_left_right1 + err_v_left_right1*ST;
 		err_int_v_right_right1 = err_int_v_right_right1 + err_v_right_right1*ST;
		err_der_v_left_right1 = (err_v_left_right1 - pre_err_v_left_right1)/ST;
		err_der_v_right_right1 = (err_v_right_right1 - pre_err_v_right_right1)/ST;
		pre_err_v_left_right1 = err_v_left_right1;
		pre_err_v_right_right1 = err_v_right_right1;

		cmd_acc_left_right1 = K_pl*(err_v_left_right1)+K_il*(err_int_v_left_right1)+K_dl*(err_der_v_left_right1);
		cmd_acc_right_right1 = K_pa*(err_v_right_right1)+K_ia*(err_int_v_right_right1)+K_da*(err_der_v_right_right1);

		/*right1 Motor Controller*/

		ddtheta_ref_right1_l = cmd_acc_left_right1;
		ddtheta_ref_right1_r = cmd_acc_right_right1;

		ia_ref_right1_l = J_n*ddtheta_ref_right1_l/K_tn;
		ia_ref_right1_r = J_n*ddtheta_ref_right1_r/K_tn;

		tau_dis_right1_l = f_dis_right1*R_w*(1-sin(phi_res_right1-phi_o))/2 + D*dtheta_res_right1_l;
		tau_dis_right1_r = f_dis_right1*R_w*(1+sin(phi_res_right1-phi_o))/2+ D*dtheta_res_right1_r;

		ia_right1_l = ia_ref_right1_l + tau_dob_right1_l/K_tn;
		ia_right1_r = ia_ref_right1_r + tau_dob_right1_r/K_tn;

		ddtheta_res_right1_l = (K_tn*ia_right1_l - tau_dis_right1_l)/J_n;
		ddtheta_res_right1_r = (K_tn*ia_right1_r - tau_dis_right1_r)/J_n;

		dtheta_res_right1_l += ddtheta_res_right1_l * ST;
		dtheta_res_right1_r += ddtheta_res_right1_r * ST;

		theta_res_right1_l += dtheta_res_right1_l * ST;
		theta_res_right1_r += dtheta_res_right1_r * ST;

		/*right1 Robot Motion*/
		ddx_res_right1 = R_w*(ddtheta_res_right1_l+ddtheta_res_right1_r)*cos(phi_res_right1)/2-D_f*dx_res_right1;
		dx_res_right1 += ddx_res_right1*ST;
		x_res_right1 += dx_res_right1*ST;

		ddy_res_right1 = R_w*(ddtheta_res_right1_l+ddtheta_res_right1_r)*sin(phi_res_right1)/2-D_f*dy_res_right1;
		dy_res_right1 += ddy_res_right1*ST;
		y_res_right1 += dy_res_right1*ST;

		ddphi_res_right1 = R_w*(ddtheta_res_right1_r-ddtheta_res_right1_l)/(2*R_r);
		dphi_res_right1 += ddphi_res_right1*ST; 
		phi_res_right1 += dphi_res_right1*ST;


		/*Object Motion*/

		ddx_o = (f_dis_left1*cos(phi_res_left1)+f_dis_left2*cos(phi_res_left2)+f_dis_right1*cos(phi_res_right1))/M_o - D_f*dx_o;
		dx_o += ddx_o*ST;
		x_o += dx_o*ST;
		ddy_o = (f_dis_left1*sin(phi_res_left1)+f_dis_left2*sin(phi_res_left2)+f_dis_right1*sin(phi_res_right1))/M_o - D_f*dy_o;
		dy_o += ddy_o*ST;
		y_o += dy_o*ST;
		ddphi_o = (f_dis_left1*(-sin(phi_res_left1)*(x_o-x_c_left1)+cos(phi_res_left1)*(y_o-y_c_left1))
			+ f_dis_left2*(-sin(phi_res_left2)*(x_o-x_c_left2)+cos(phi_res_left2)*(y_o-y_c_left2))
			+ f_dis_right1*(sin(phi_res_right1)*(x_o-x_c_right1)-cos(phi_res_right1)*(y_o-y_c_right1))
			)/J_o - D_f_rotation*dphi_o;
		dphi_o += ddphi_o*ST;
		phi_o += dphi_o*ST;
		intphi_o += phi_o;

		l_left1 = fabs(cos(phi_o)*(x_c_left1-x_o+cos(phi_o)*L_o)+sin(phi_o)*(y_c_left1-y_o+sin(phi_o)*L_o));
		if (firstRound==1){
			dl_left1 = 0.0;
		}
		else{
			dl_left1 = (l_left1 - l_left1_prev)/ST;
		}
		l_left1_prev = l_left1;

		l_left2 = fabs(cos(phi_o)*(x_c_left2-x_o+cos(phi_o)*L_o)+sin(phi_o)*(y_c_left2-y_o+sin(phi_o)*L_o));
		if (firstRound==1){
			dl_left2 = 0.0;
		}
		else{
			dl_left2 = (l_left2 - l_left2_prev)/ST;
		}
		l_left2_prev = l_left2;

		l_right1 = fabs(cos(phi_o)*(x_c_right1-x_o-cos(phi_o)*L_o)+sin(phi_o)*(y_c_right1-y_o-sin(phi_o)*L_o));
		if (firstRound==1){
			dl_right1 = 0.0;
		}
		else{
			dl_right1 = (l_right1 - l_right1_prev)/ST;
		}
		l_right1_prev = l_right1;

		/*left1 disturbance observer*/	
		tau_dob_left1_l = integral_tau_dob_left1_l - dtheta_res_left1_l*J_n*GDIS;
		integral_tau_dob_left1_l 
			+= ((K_tn*ia_left1_l + dtheta_res_left1_l*J_n*GDIS) - integral_tau_dob_left1_l)*GDIS*ST;
		
		tau_dob_left1_r = integral_tau_dob_left1_r - dtheta_res_left1_r*J_n*GDIS;
		integral_tau_dob_left1_r 
			+= ((K_tn*ia_left1_r + dtheta_res_left1_r*J_n*GDIS) - integral_tau_dob_left1_r)*GDIS*ST;

		/*left1 Mobile Robot Reaction Force*/

		f_res_left1 = (tau_dob_left1_l+tau_dob_left1_r)/R_w;
		tau_res_left1 = (tau_dob_left1_l-tau_dob_left1_r)*R_r/(R_w);


		/*left2 disturbance observer*/	
		tau_dob_left2_l = integral_tau_dob_left2_l - dtheta_res_left2_l*J_n*GDIS;
		integral_tau_dob_left2_l 
			+= ((K_tn*ia_left2_l + dtheta_res_left2_l*J_n*GDIS) - integral_tau_dob_left2_l)*GDIS*ST;
		
		tau_dob_left2_r = integral_tau_dob_left2_r - dtheta_res_left2_r*J_n*GDIS;
		integral_tau_dob_left2_r 
			+= ((K_tn*ia_left2_r + dtheta_res_left2_r*J_n*GDIS) - integral_tau_dob_left2_r)*GDIS*ST;

		/*left2 Mobile Robot Reaction Force*/

		f_res_left2 = (tau_dob_left2_l+tau_dob_left2_r)/R_w;
		tau_res_left2 = (tau_dob_left2_l-tau_dob_left2_r)*R_r/(R_w);

		/*right1 disturbance observer*/	
		tau_dob_right1_l = integral_tau_dob_right1_l - dtheta_res_right1_l*J_n*GDIS;
		integral_tau_dob_right1_l 
			+= ((K_tn*ia_right1_l + dtheta_res_right1_l*J_n*GDIS) - integral_tau_dob_right1_l)*GDIS*ST;
		
		tau_dob_right1_r = integral_tau_dob_right1_r - dtheta_res_right1_r*J_n*GDIS;
		integral_tau_dob_right1_r 
			+= ((K_tn*ia_right1_r + dtheta_res_right1_r*J_n*GDIS) - integral_tau_dob_right1_r)*GDIS*ST;

		/*right1 Mobile Robot Reaction Force*/

		f_res_right1 = (tau_dob_right1_l+tau_dob_right1_r)/R_w;
		tau_res_right1 = (tau_dob_right1_l-tau_dob_right1_r)*R_r/(R_w);

		fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", t, x_o, y_o, phi_o, x_res_left1, y_res_left1, phi_res_left1, f_dis_left1, x_res_right1, y_res_right1, phi_res_right1, f_dis_right1, x_res_left2, y_res_left2, phi_res_left2, f_dis_left2);
		
		t += ST;
		firstRound = 0;
      
	}

	fclose(fp);




	return 0;
}

