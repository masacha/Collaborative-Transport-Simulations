#include <stdio.h>
#include <math.h>

#define	T	60 //seconds
#define ST	0.0001 //second
#define Pi	3.1415

#define nrobots	3
#define mu 	1.0

//Wheel parameters
#define K_tn	0.43   //Nm
#define J_n	0.01 //Ns^2/m
#define D_r	0.06
#define I_limit 2.0 //A

//Robot parameters
#define R_w	0.033 //meter
#define R_r	0.08 //meter
#define M_r	1.0 //kg
#define J_r	0.0032

//Object parameters
#define K_o	1000.0 //N/m
#define D_o	1.0
#define M_o	3.0 //kg
#define J_o	2.0
#define L_o	1.000 //meter

#define K_m	10000.0 //N/m
#define D_m	1.0

//Friction from floor
#define D_f	50.0
#define D_f_rotation	100

//Gains
#define GDIS	500.0
#define GROBOT  500.0
#define K_p	0.0
#define K_v	50.0
#define K_fc	50.0
#define M_c	1.0
#define K_f	0.5
#define K_phi	1.0
#define K_dphi	0.0
#define K_intphi	1.0
#define K_pv	15.0
#define K_iv	0.0
#define K_dv	0.0
#define K_pw 	1.0
#define K_ro	0.1
#define K_alpha	0.15

#define F_LIMIT	15.0

#define GOTOOBJECT	0
#define GRASP		1
#define GOTOGOAL	2
#define AVOIDOBSTACLE	3

#define COMPLIANT	0
#define VELOCITY	1
#define FORCE		2

int main(void)
{
	double t = 0.0;

	int firstRound = 1;

	int behaviour = 0;
	double total_cmd_force = 0.0;

	double v_o_max = 0.05;
	double w_o_max = Pi/(2*30);

	double err_phi = 0.0;
	double pre_err_phi = 0.0;
	double err_int_phi = 0.0;
	double err_der_phi = 0.0;

	double t_start = 0.0;
	double t_avoid = 0.0;


/*LEFT1*/
/*Reference Commands*/

	int control_mode_left1 = VELOCITY;
	int pre_control_mode_left1 = VELOCITY;
	double f_cmd_left1 = 0.0;
	double x_cmd_left1 = 0.0;
	double v_cmd_left1 = 0.0;
	double w_cmd_left1 = 0.0;
	double phi_cmd_left1 = 0.0;

	double x_goal_left1 = 0.0;
	double y_goal_left1 = 0.0;

/*Mobile Robot Outputs*/

	double dv_res_left1 = 0.0;
	double v_res_left1 = 0.0;
	double dw_res_left1 = 0.0;
	double w_res_left1 = 0.0;

	double x_res_left1 = -1.28;
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

	double err_x_left1 = 0.0;
	double derr_x_left1 = 0.0;
	double err_y_left1 = 0.0;
	double derr_y_left1 = 0.0;

	double f_res_left1 = 0.0;
	double integral_f_res_left1 = 0.0;

	double l_tan_left1 = 0.0;
	double dl_tan_left1 = 0.0;
	double f_tan_left1 = 0.0;
	double l_tan_left1_prev = 0.0;

/*Mobile Robot Inputs*/

	double err_vel_left1 = 0.0;
	double pre_err_vel_left1 = 0.0;
	double err_int_vel_left1 = 0.0;
	double err_der_vel_left1 = 0.0;

	double err_phi_left1 = 0.0;
	double pre_err_phi_left1 = 0.0;
	double err_int_phi_left1 = 0.0;
	double err_der_phi_left1 = 0.0;

	double dv_ref_left1 = 0.0;
	double dw_ref_left1 = 0.0;

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

	int control_mode_left2 = VELOCITY;
	int pre_control_mode_left2 = VELOCITY;
	double f_cmd_left2 = 0.0;
	double x_cmd_left2 = 0.0;
	double v_cmd_left2 = 0.0;
	double w_cmd_left2 = 0.0;
	double phi_cmd_left2 = 0.0;

	double x_goal_left2 = 0.0;
	double y_goal_left2 = 0.0;

/*Mobile Robot Outputs*/

	double dv_res_left2 = 0.0;
	double v_res_left2 = 0.0;
	double dw_res_left2 = 0.0;
	double w_res_left2 = 0.0;

	double x_res_left2 = -1.28;
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

	double err_x_left2 = 0.0;
	double derr_x_left2 = 0.0;
	double err_y_left2 = 0.0;
	double derr_y_left2 = 0.0;

	double f_res_left2 = 0.0;
	double integral_f_res_left2 = 0.0;

	double l_tan_left2 = 0.0;
	double dl_tan_left2 = 0.0;
	double f_tan_left2 = 0.0;
	double l_tan_left2_prev = 0.0;

/*Mobile Robot Inputs*/
	
	double err_vel_left2 = 0.0;
	double pre_err_vel_left2 = 0.0;
	double err_int_vel_left2 = 0.0;
	double err_der_vel_left2 = 0.0;

	double err_phi_left2 = 0.0;
	double pre_err_phi_left2 = 0.0;
	double err_int_phi_left2 = 0.0;
	double err_der_phi_left2 = 0.0;

	double dv_ref_left2 = 0.0;
	double dw_ref_left2 = 0.0;

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

/*right1*/
/*Reference Commands*/

	int control_mode_right1 = VELOCITY;
	int pre_control_mode_right1 = VELOCITY;
	double f_cmd_right1 = 0.0;
	double x_cmd_right1 = 0.0;
	double v_cmd_right1 = 0.0;
	double w_cmd_right1 = 0.0;
	double phi_cmd_right1 = 0.0;

	double x_goal_right1 = 0.0;
	double y_goal_right1 = 0.0;
	
	double reaction_wall_right1 = 0.0;

/*Mobile Robot Outputs*/

	double dv_res_right1 = 0.0;
	double v_res_right1 = 0.0;
	double dw_res_right1 = 0.0;
	double w_res_right1 = 0.0;

	double x_res_right1 = +1.28;
	double dx_res_right1 = 0.0;
	double ddx_res_right1 = 0.0;
	double y_res_right1 = 0.0;
	double dy_res_right1 = 0.0;
	double ddy_res_right1 = 0.0;
	double phi_res_right1 = 0.0;
	double dphi_res_right1 = 0.0;
	double ddphi_res_right1 = 0.0;

	double x_c_right1 = 0.0;
	double y_c_right1 = 0.0;
	
	double l_right1 = 0.0;
	double dl_right1 = 0.0;
	double l_right1_prev = 0.0;

	double err_x_right1 = 0.0;
	double derr_x_right1 = 0.0;
	double err_y_right1 = 0.0;
	double derr_y_right1 = 0.0;

	double f_res_right1 = 0.0;
	double integral_f_res_right1 = 0.0;

	double l_tan_right1 = 0.0;
	double dl_tan_right1 = 0.0;
	double f_tan_right1 = 0.0;
	double l_tan_right1_prev = 0.0;

/*Mobile Robot Inputs*/
	
	double err_vel_right1 = 0.0;
	double pre_err_vel_right1 = 0.0;
	double err_int_vel_right1 = 0.0;
	double err_der_vel_right1 = 0.0;

	double err_phi_right1 = 0.0;
	double pre_err_phi_right1 = 0.0;
	double err_int_phi_right1 = 0.0;
	double err_der_phi_right1 = 0.0;

	double dv_ref_right1 = 0.0;
	double dw_ref_right1 = 0.0;

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
	double x_goal = 1.0;
	double y_goal = 1.0;

	double v_ref_o = 0.0;
	double w_ref_o = 0.0;
	double x_ref_o = 0.0;
	double y_ref_o = 0.0;
	double phi_ref_o = 0.0;
	double time_to_goal = 0.0;
	
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

	double x_wall = 2.5;
	double y_wall = -1.0;

	double reaction_wall = 0.0;
	double l_wall = 0.0;
	double dl_wall = 0.0;
	double l_wall_prev = 0.0;

	FILE *fp;

	if((fp=fopen("simulation_transport.dat", "w"))==NULL){
		fprintf(stderr, "File open failed.\n");
		return 1;
	}

	while(t <= T){			
		
		/*Cooperative Strategy*/

		if (behaviour==GOTOOBJECT && (f_dis_left1 != 0.0 || f_dis_left2 != 0.0 || f_dis_right1 != 0.0)){
			behaviour = GRASP;
		}
		else if (behaviour == GRASP && fabs(f_cmd_left1-f_dis_left1)<0.25 && fabs(f_cmd_left2-f_dis_left2)<0.25 && fabs(f_cmd_right1-f_dis_right1)<0.5 && fabs(v_res_left1)<0.001 && fabs(v_res_left2)<0.001 && fabs(v_res_right1)<0.001){
			behaviour = GOTOGOAL;
			v_ref_o = 0.05;
			w_ref_o = 0.00;
			t_start = t;
		}
		else if (behaviour == GOTOGOAL && f_res_right1+f_res_left1+f_res_left2>F_LIMIT){
			behaviour = AVOIDOBSTACLE;
			t_avoid = t;
		}

		if (behaviour==GOTOOBJECT){
			control_mode_left1 = VELOCITY;
			control_mode_left2 = VELOCITY;
			control_mode_right1 = VELOCITY;
			v_cmd_left1 = 0.1;
			v_cmd_left2 = 0.1;
			v_cmd_right1 = -0.1;
		}		
		else if (behaviour==GRASP){
			control_mode_left1 = FORCE;
			control_mode_left2 = FORCE;
			control_mode_right1 = FORCE;
			f_cmd_left1 = 2.5;
			f_cmd_left2 = 2.5;
			f_cmd_right1 = -5.0;
		}
		else if (behaviour==GOTOGOAL){
			control_mode_left1 = FORCE;
			control_mode_left2 = FORCE;
			control_mode_right1 = VELOCITY;
			if (f_dis_left1>0){
				f_cmd_left1 = 5.0;
			}else{
				f_cmd_left1 = 5.0;
			}
			if (f_dis_left2>0){
				f_cmd_left2 = 5.0;
			}else{
				f_cmd_left2 = 5.0;
			}
			v_cmd_right1 = v_ref_o;
			w_cmd_left1 = w_ref_o;
			w_cmd_left2 = w_ref_o;
			w_cmd_right1 = w_ref_o;
		}
		else if (behaviour==AVOIDOBSTACLE && t-t_avoid<2.5){
			control_mode_left1 = VELOCITY;
			control_mode_left2 = VELOCITY;
			control_mode_right1 = VELOCITY;
			v_cmd_left1 = -0.1*(t-t_avoid)/2.5;
			v_cmd_left2 = -0.1*(t-t_avoid)/2.5;
			v_cmd_right1 = -0.1*(t-t_avoid)/2.5;
		}
		else if (behaviour==AVOIDOBSTACLE && t-t_avoid>=2.5 && t-t_avoid<=5){
			control_mode_left1 = VELOCITY;
			control_mode_left2 = VELOCITY;
			control_mode_right1 = VELOCITY;
			v_cmd_left1 = -0.1+0.1*(t-t_avoid-2.5)/2.5;
			v_cmd_left2 = -0.1+0.1*(t-t_avoid-2.5)/2.5;
			v_cmd_right1 = -0.1+0.1*(t-t_avoid-2.5)/2.5;
		}
		else if (behaviour==AVOIDOBSTACLE && t-t_avoid>5.0){			
			control_mode_left1 = FORCE;
			control_mode_left2 = FORCE;
			control_mode_right1 = VELOCITY;
			f_cmd_left1 = 2.5;
			f_cmd_left2 = 2.5;
			v_cmd_right1 = 0.05;
			w_cmd_left1 = 0.05;
			w_cmd_left2 = 0.05;
			w_cmd_right1 = 0.05;
		}
		
		

		/*left1*/
		
		x_c_left1 = x_res_left1 + cos(phi_o)*R_r-sin(phi_o)*(0.0);
		y_c_left1 = y_res_left1 + sin(phi_o)*R_r+cos(phi_o)*(0.0);

		if(cos(phi_o)*(x_c_left1-x_o+cos(phi_o)*L_o)+sin(phi_o)*(y_c_left1-y_o+sin(phi_o)*L_o)>0 && cos(phi_o)*(y_c_left1-y_o+cos(phi_o)*L_o)-sin(phi_o)*(x_c_left1-x_o-sin(phi_o)*L_o)>0 && cos(phi_o)*(y_c_left1-y_o-cos(phi_o)*L_o)-sin(phi_o)*(x_c_left1-x_o+sin(phi_o)*L_o)<0){
			f_dis_left1 = K_o*l_left1 + D_o*dl_left1; 
		}
		else{
			f_dis_left1 = 0.0;
		}

		tau_dis_left1_l = f_dis_left1*R_w/2 + D_r*dtheta_res_left1_l + ((R_w*R_w)/(4*R_r*R_r))*((M_r*R_r*R_r+J_r)*ddtheta_res_left1_l+(M_r*R_r*R_r-J_r)*ddtheta_res_left1_r);
		tau_dis_left1_r = f_dis_left1*R_w/2 + D_r*dtheta_res_left1_r + ((R_w*R_w)/(4*R_r*R_r))*((M_r*R_r*R_r-J_r)*ddtheta_res_left1_l+(M_r*R_r*R_r+J_r)*ddtheta_res_left1_r);
		
		/*left1 Mobile Robot Controller*/

		if (pre_control_mode_left1 != control_mode_left1){
			err_vel_left1 = 0.0;
			err_int_vel_left1 = 0.0;
			err_der_vel_left1 = 0.0;
			pre_err_vel_left1 = 0.0;
			err_int_vel_left1 = 0.0;
			pre_control_mode_left1 = control_mode_left1;
		}
		
		if (control_mode_left1 == COMPLIANT){
			dv_ref_left1 = (K_p*(x_cmd_left1 - x_res_left1) + K_v*(v_cmd_left1-v_res_left1) + K_fc*(f_cmd_left1-f_dis_left1))/M_c;
			dw_ref_left1 = 0.0;
		}
		else if (control_mode_left1 == VELOCITY){
			err_vel_left1 = v_cmd_left1 - v_res_left1;
			err_int_vel_left1 += err_vel_left1*ST;
			err_der_vel_left1 = (err_vel_left1 - pre_err_vel_left1)/ST;
			pre_err_vel_left1 = err_vel_left1;
			dv_ref_left1 = K_pv*err_vel_left1 + K_iv*err_int_vel_left1 + K_dv*err_der_vel_left1;
			dw_ref_left1 = K_pw*(w_cmd_left1-dphi_res_left1);
		}
		else if (control_mode_left1 == FORCE){
			dv_ref_left1 = K_f*(f_cmd_left1-f_res_left1);
			dw_ref_left1 = K_pw*(w_cmd_left1-dphi_res_left1);
		}
		else{
			dv_ref_left1 = 0.0;
			dw_ref_left1 = 0.0;
		}

		/*left1 Motor Controller*/

		ddtheta_ref_left1_l = dv_ref_left1/R_w - R_r*dw_ref_left1/R_w;
		ddtheta_ref_left1_r = dv_ref_left1/R_w + R_r*dw_ref_left1/R_w;

		ia_ref_left1_l = J_n*ddtheta_ref_left1_l/K_tn;
		ia_ref_left1_r = J_n*ddtheta_ref_left1_r/K_tn;

		ia_left1_l = ia_ref_left1_l + tau_dob_left1_l/K_tn;
		ia_left1_r = ia_ref_left1_r + tau_dob_left1_r/K_tn;

		ddtheta_res_left1_l = (K_tn*ia_left1_l - tau_dis_left1_l)/J_n;
		ddtheta_res_left1_r = (K_tn*ia_left1_r - tau_dis_left1_r)/J_n;

		dtheta_res_left1_l += ddtheta_res_left1_l * ST;
		dtheta_res_left1_r += ddtheta_res_left1_r * ST;

		theta_res_left1_l += dtheta_res_left1_l * ST;
		theta_res_left1_r += dtheta_res_left1_r * ST;

		/*left1 Robot Motion*/
		dv_res_left1 = R_w*(ddtheta_res_left1_l+ddtheta_res_left1_r)/2;
		v_res_left1 += dv_res_left1*ST;
		dw_res_left1 = R_w*(ddtheta_res_left1_r-ddtheta_res_left1_l)/(2*R_r);
		w_res_left1 += dw_res_left1*ST;

		ddx_res_left1 = dv_res_left1*cos(phi_res_left1);
		dx_res_left1 = v_res_left1*cos(phi_res_left1);
		x_res_left1 += dx_res_left1*ST;

		ddy_res_left1 = dv_res_left1*sin(phi_res_left1);
		dy_res_left1 = v_res_left1*sin(phi_res_left1);
		y_res_left1 += dy_res_left1*ST;

		ddphi_res_left1 = dw_res_left1;
		dphi_res_left1 += ddphi_res_left1*ST; 
		phi_res_left1 += dphi_res_left1*ST;

		/*left2 commands*/
		
		x_c_left2 = x_res_left2 + cos(phi_o)*R_r-sin(phi_o)*(0.0);
		y_c_left2 = y_res_left2 + sin(phi_o)*R_r+cos(phi_o)*(0.0);

		if(cos(phi_o)*(x_c_left2-x_o+cos(phi_o)*L_o)+sin(phi_o)*(y_c_left2-y_o+sin(phi_o)*L_o)>0 && cos(phi_o)*(y_c_left2-y_o+cos(phi_o)*L_o)-sin(phi_o)*(x_c_left2-x_o-sin(phi_o)*L_o)>0 && cos(phi_o)*(y_c_left2-y_o-cos(phi_o)*L_o)-sin(phi_o)*(x_c_left2-x_o+sin(phi_o)*L_o)<0){
			f_dis_left2 = K_o*l_left2 + D_o*dl_left2; 
		}
		else{
			f_dis_left2 = 0.0;
		}
		
		tau_dis_left2_l = f_dis_left2*R_w/2 + D_r*dtheta_res_left2_l + ((R_w*R_w)/(4*R_r*R_r))*((M_r*R_r*R_r+J_r)*ddtheta_res_left2_l+(M_r*R_r*R_r-J_r)*ddtheta_res_left2_r);
		tau_dis_left2_r = f_dis_left2*R_w/2 + D_r*dtheta_res_left2_r + ((R_w*R_w)/(4*R_r*R_r))*((M_r*R_r*R_r-J_r)*ddtheta_res_left2_l+(M_r*R_r*R_r+J_r)*ddtheta_res_left2_r);

		/*left2 Mobile Robot Controller*/

		if (pre_control_mode_left2 != control_mode_left2){
			err_vel_left2 = 0.0;
			err_int_vel_left2 = 0.0;
			err_der_vel_left2 = 0.0;
			pre_err_vel_left2 = 0.0;
			err_int_phi_left2 = 0.0;
			pre_control_mode_left2 = control_mode_left2;
		}

		if (control_mode_left2 == COMPLIANT){
			dv_ref_left2 = (K_p*(x_cmd_left2 - x_res_left2) + K_v*(v_cmd_left2-v_res_left2) + K_fc*(f_cmd_left2-f_dis_left2))/M_c;
			dw_ref_left2 = 0.0;
		}
		else if (control_mode_left2 == VELOCITY){
			err_vel_left2 = v_cmd_left2 - v_res_left2;
			err_int_vel_left2 += err_vel_left2*ST;
			err_der_vel_left2 = (err_vel_left2 - pre_err_vel_left2)/ST;
			pre_err_vel_left2 = err_vel_left2;
			dv_ref_left2 = K_pv*err_vel_left2 + K_iv*err_int_vel_left2 + K_dv*err_der_vel_left2;
			dw_ref_left2 = K_pw*(w_cmd_left2-dphi_res_left2);
		}
		else if (control_mode_left2 == FORCE){
			dv_ref_left2 = K_f*(f_cmd_left2-f_dis_left2);
			dw_ref_left2 = K_pw*(w_cmd_left2-dphi_res_left2);
		}
		else{
			dv_ref_left2 = 0.0;
			dw_ref_left2 = 0.0;
		}

		/*left2 Motor Controller*/

		ddtheta_ref_left2_l = dv_ref_left2/R_w - R_r*dw_ref_left2/R_w;
		ddtheta_ref_left2_r = dv_ref_left2/R_w + R_r*dw_ref_left2/R_w;
		ia_ref_left2_l = J_n*ddtheta_ref_left2_l/K_tn;
		ia_ref_left2_r = J_n*ddtheta_ref_left2_r/K_tn;

		ia_left2_l = ia_ref_left2_l + tau_dob_left2_l/K_tn;
		ia_left2_r = ia_ref_left2_r + tau_dob_left2_r/K_tn;

		ddtheta_res_left2_l = (K_tn*ia_left2_l - tau_dis_left2_l)/J_n;
		ddtheta_res_left2_r = (K_tn*ia_left2_r - tau_dis_left2_r)/J_n;

		dtheta_res_left2_l += ddtheta_res_left2_l * ST;
		dtheta_res_left2_r += ddtheta_res_left2_r * ST;

		theta_res_left2_l += dtheta_res_left2_l * ST;
		theta_res_left2_r += dtheta_res_left2_r * ST;

		/*left2 Robot Motion*/
		dv_res_left2 = R_w*(ddtheta_res_left2_l+ddtheta_res_left2_r)/2;
		v_res_left2 += dv_res_left2*ST;
		dw_res_left2 = R_w*(ddtheta_res_left2_r-ddtheta_res_left2_l)/(2*R_r);
		w_res_left2 += dw_res_left2*ST;

		ddx_res_left2 = dv_res_left2*cos(phi_res_left2);
		dx_res_left2  = v_res_left2*cos(phi_res_left2);
		x_res_left2 += dx_res_left2*ST;

		ddy_res_left2 = dv_res_left2*sin(phi_res_left2);
		dy_res_left2 = v_res_left2*sin(phi_res_left2);
		y_res_left2 += dy_res_left2*ST;

		ddphi_res_left2 = dw_res_left2;
		dphi_res_left2 += ddphi_res_left2*ST; 
		phi_res_left2 += dphi_res_left2*ST;

		/*right1 commands*/

		x_c_right1 = x_res_right1 - cos(phi_o)*R_r-sin(phi_o)*(0.0);
		y_c_right1 = y_res_right1 - sin(phi_o)*R_r+cos(phi_o)*(0.0);

		if(cos(phi_o)*(x_c_right1-x_o-cos(phi_o)*L_o)+sin(phi_o)*(y_c_right1-y_o-sin(phi_o)*L_o)<0 && cos(phi_o)*(y_c_right1-y_o+cos(phi_o)*L_o)-sin(phi_o)*(x_c_right1-x_o-sin(phi_o)*L_o)>0 && cos(phi_o)*(y_c_right1-y_o-cos(phi_o)*L_o)-sin(phi_o)*(x_c_right1-x_o+sin(phi_o)*L_o)<0){
			f_dis_right1 = -(K_o*l_right1 + D_o*dl_right1); 
		}
		else{
			f_dis_right1 = 0.0;
		}
		
		if(x_res_right1>x_wall-R_r && y_res_right1 - R_r> y_wall){
			reaction_wall_right1 = K_m*(x_res_right1+R_r-x_wall) + D_m*dx_res_right1; 
		}
		else{
			reaction_wall_right1 = 0.0;
		}

		tau_dis_right1_l = f_dis_right1*R_w/2 + reaction_wall_right1*R_w/2 + D_r*dtheta_res_right1_l + ((R_w*R_w)/(4*R_r*R_r))*((M_r*R_r*R_r+J_r)*ddtheta_res_right1_l+(M_r*R_r*R_r-J_r)*ddtheta_res_right1_r);
		tau_dis_right1_r = f_dis_right1*R_w/2 + reaction_wall_right1*R_w/2 + D_r*dtheta_res_right1_r + ((R_w*R_w)/(4*R_r*R_r))*((M_r*R_r*R_r-J_r)*ddtheta_res_right1_l+(M_r*R_r*R_r+J_r)*ddtheta_res_right1_r);

		/*right1 Mobile Robot Controller*/

		if (pre_control_mode_right1 != control_mode_right1){
			err_vel_right1 = 0.0;
			err_int_vel_right1 = 0.0;
			err_der_vel_right1 = 0.0;
			pre_err_vel_right1 = 0.0;
			err_int_phi_right1 = 0.0;
			pre_control_mode_right1 = control_mode_right1;
		}

		if (control_mode_right1 == COMPLIANT){
			dv_ref_right1 = (K_p*(x_cmd_right1 - x_res_right1) + K_v*(v_cmd_right1-v_res_right1) + K_fc*(f_cmd_right1-f_dis_right1))/M_c;
			dw_ref_right1 = 0.0;
		}
		else if (control_mode_right1 == VELOCITY){
			err_vel_right1 = v_cmd_right1 - v_res_right1;
			err_int_vel_right1 += err_vel_right1*ST;
			err_der_vel_right1 = (err_vel_right1 - pre_err_vel_right1)/ST;
			pre_err_vel_right1 = err_vel_right1;
			dv_ref_right1 = K_pv*err_vel_right1 + K_iv*err_int_vel_right1 + K_dv*err_der_vel_right1;
			dw_ref_right1 = K_pw*(w_cmd_right1-dphi_res_right1);
		}
		else if (control_mode_right1 == FORCE){
			dv_ref_right1 = K_f*(f_cmd_right1-f_dis_right1);
			dw_ref_right1 = K_pw*(w_cmd_right1-dphi_res_right1);
		}
		else{
			dv_ref_right1 = 0.0;
			dw_ref_right1 = 0.0;
		}

		/*right1 Motor Controller*/

		ddtheta_ref_right1_l = dv_ref_right1/R_w - R_r*dw_ref_right1/R_w;
		ddtheta_ref_right1_r = dv_ref_right1/R_w + R_r*dw_ref_right1/R_w;
		ia_ref_right1_l = J_n*ddtheta_ref_right1_l/K_tn;
		ia_ref_right1_r = J_n*ddtheta_ref_right1_r/K_tn;

		ia_right1_l = ia_ref_right1_l + tau_dob_right1_l/K_tn;
		ia_right1_r = ia_ref_right1_r + tau_dob_right1_r/K_tn;

		ddtheta_res_right1_l = (K_tn*ia_right1_l - tau_dis_right1_l)/J_n;
		ddtheta_res_right1_r = (K_tn*ia_right1_r - tau_dis_right1_r)/J_n;

		dtheta_res_right1_l += ddtheta_res_right1_l * ST;
		dtheta_res_right1_r += ddtheta_res_right1_r * ST;

		theta_res_right1_l += dtheta_res_right1_l * ST;
		theta_res_right1_r += dtheta_res_right1_r * ST;

		/*right1 Robot Motion*/
		dv_res_right1 = R_w*(ddtheta_res_right1_l+ddtheta_res_right1_r)/2;
		v_res_right1 += dv_res_right1*ST;
		dw_res_right1 = R_w*(ddtheta_res_right1_r-ddtheta_res_right1_l)/(2*R_r);
		w_res_right1 += dw_res_right1*ST;

		ddx_res_right1 = dv_res_right1*cos(phi_res_right1);
		dx_res_right1 = v_res_right1*cos(phi_res_right1);
		x_res_right1 += dx_res_right1*ST;

		ddy_res_right1 = dv_res_right1*sin(phi_res_right1);
		dy_res_right1  = v_res_right1*sin(phi_res_right1);
		y_res_right1 += dy_res_right1*ST;

		ddphi_res_right1 = dw_res_right1;
		dphi_res_right1 += ddphi_res_right1*ST; 
		phi_res_right1 += dphi_res_right1*ST;

		/*Object Motion*/

		/*if(cos(phi_o)*(x_wall-x_o-cos(phi_o)*L_o)+sin(phi_o)*(y_wall-y_o-sin(phi_o)*L_o)<0 && cos(phi_o)*(y_wall-y_o+cos(phi_o)*L_o)-sin(phi_o)*(x_wall-x_o-sin(phi_o)*L_o)>0 && cos(phi_o)*(y_wall-y_o-cos(phi_o)*L_o)-sin(phi_o)*(x_wall-x_o+sin(phi_o)*L_o)<0){
			reaction_wall_object = K_o*l_wall + D_o*dl_wall; 
		}
		else{
			reaction_wall_object = 0.0;
		}*/

		if (f_dis_left1 != 0){
			f_tan_left1 = fabs(mu*f_dis_left1);
		}else{
			f_tan_left1 = 0.0;	
		}
		if (f_dis_left2 != 0){
			f_tan_left2 = fabs(mu*f_dis_left2);
		}else{
			f_tan_left2 = 0.0;	
		}
		if (f_dis_right1 != 0){
			f_tan_right1 = fabs(mu*f_dis_right1);
		}else{
			f_tan_right1 = 0.0;	
		}


		ddx_o = (f_dis_left1*cos(phi_o)+f_dis_left2*cos(phi_o)+f_dis_right1*cos(phi_o))/M_o - D_f*dx_o;
			ddy_o = (f_dis_left1*sin(phi_o)+f_dis_left2*sin(phi_o)+f_dis_right1*sin(phi_o))/M_o - D_f*dy_o;
			ddphi_o = (f_dis_left1*(-sin(phi_o)*(x_o-x_c_left1)+cos(phi_o)*(y_o-y_c_left1)) 
				+ f_dis_left2*(-sin(phi_o)*(x_o-x_c_left2)+cos(phi_o)*(y_o-y_c_left2)) 
				+ f_dis_right1*(-sin(phi_o)*(x_o-x_c_right1)+cos(phi_o)*(y_o-y_c_right1)) 
				)/J_o - D_f_rotation*dphi_o;

		if (dl_tan_left1 > 0 ) {
			ddx_o += (f_tan_left1*(-sin(phi_o)))/M_o;
			ddy_o = (f_tan_left1+f_tan_left2 + f_tan_right1)*(cos(phi_o)))/M_o - D_f*dy_o;
			ddphi_o = (f_dis_left1*(-sin(phi_o)*(x_o-x_c_left1)+cos(phi_o)*(y_o-y_c_left1)) + fabs((f_tan_left1)*(-cos(phi_o)*(x_o-x_c_left1)+sin(phi_o)*(y_o-y_c_left1)))
				+ f_dis_left2*(-sin(phi_o)*(x_o-x_c_left2)+cos(phi_o)*(y_o-y_c_left2)) + fabs((f_tan_left2)*(-cos(phi_o)*(x_o-x_c_left2)+sin(phi_o)*(y_o-y_c_left2)))
				+ f_dis_right1*(-sin(phi_o)*(x_o-x_c_right1)+cos(phi_o)*(y_o-y_c_right1)) - fabs((f_tan_right1)*(-cos(phi_o)*(x_o-x_c_right1)+sin(phi_o)*(y_o-y_c_right1)))
				)/J_o - D_f_rotation*dphi_o;
		}
		else if (dl_tan_left1 < 0 ){
			ddx_o = (f_dis_left1*cos(phi_o)+f_dis_left2*cos(phi_o)+f_dis_right1*cos(phi_o) - (f_tan_left1+f_tan_left2 + f_tan_right1)*(-sin(phi_o)))/M_o - D_f*dx_o;
			ddy_o = (f_dis_left1*sin(phi_o)+f_dis_left2*sin(phi_o)+f_dis_right1*sin(phi_o) - (f_tan_left1+f_tan_left2 + f_tan_right1)*(cos(phi_o)))/M_o - D_f*dy_o;
			ddphi_o = (f_dis_left1*(-sin(phi_o)*(x_o-x_c_left1)+cos(phi_o)*(y_o-y_c_left1)) - fabs((f_tan_left1)*(-cos(phi_o)*(x_o-x_c_left1)+sin(phi_o)*(y_o-y_c_left1)))
				+ f_dis_left2*(-sin(phi_o)*(x_o-x_c_left2)+cos(phi_o)*(y_o-y_c_left2)) - fabs((f_tan_left2)*(-cos(phi_o)*(x_o-x_c_left2)+sin(phi_o)*(y_o-y_c_left2)))
				+ f_dis_right1*(-sin(phi_o)*(x_o-x_c_right1)+cos(phi_o)*(y_o-y_c_right1)) + fabs((f_tan_right1)*(-cos(phi_o)*(x_o-x_c_right1)+sin(phi_o)*(y_o-y_c_right1)))
				)/J_o - D_f_rotation*dphi_o;
		}
		else {


		if (dl_tan_left1 > 0 ) {
			ddx_o = (f_dis_left1*cos(phi_o)+f_dis_left2*cos(phi_o)+f_dis_right1*cos(phi_o) + (f_tan_left1+f_tan_left2 + f_tan_right1)*(-sin(phi_o)))/M_o - D_f*dx_o;
			ddy_o = (f_dis_left1*sin(phi_o)+f_dis_left2*sin(phi_o)+f_dis_right1*sin(phi_o) + (f_tan_left1+f_tan_left2 + f_tan_right1)*(cos(phi_o)))/M_o - D_f*dy_o;
			ddphi_o = (f_dis_left1*(-sin(phi_o)*(x_o-x_c_left1)+cos(phi_o)*(y_o-y_c_left1)) + fabs((f_tan_left1)*(-cos(phi_o)*(x_o-x_c_left1)+sin(phi_o)*(y_o-y_c_left1)))
				+ f_dis_left2*(-sin(phi_o)*(x_o-x_c_left2)+cos(phi_o)*(y_o-y_c_left2)) + fabs((f_tan_left2)*(-cos(phi_o)*(x_o-x_c_left2)+sin(phi_o)*(y_o-y_c_left2)))
				+ f_dis_right1*(-sin(phi_o)*(x_o-x_c_right1)+cos(phi_o)*(y_o-y_c_right1)) - fabs((f_tan_right1)*(-cos(phi_o)*(x_o-x_c_right1)+sin(phi_o)*(y_o-y_c_right1)))
				)/J_o - D_f_rotation*dphi_o;
		}
		else if (dl_tan_left1 < 0 ){
			ddx_o = (f_dis_left1*cos(phi_o)+f_dis_left2*cos(phi_o)+f_dis_right1*cos(phi_o) - (f_tan_left1+f_tan_left2 + f_tan_right1)*(-sin(phi_o)))/M_o - D_f*dx_o;
			ddy_o = (f_dis_left1*sin(phi_o)+f_dis_left2*sin(phi_o)+f_dis_right1*sin(phi_o) - (f_tan_left1+f_tan_left2 + f_tan_right1)*(cos(phi_o)))/M_o - D_f*dy_o;
			ddphi_o = (f_dis_left1*(-sin(phi_o)*(x_o-x_c_left1)+cos(phi_o)*(y_o-y_c_left1)) - fabs((f_tan_left1)*(-cos(phi_o)*(x_o-x_c_left1)+sin(phi_o)*(y_o-y_c_left1)))
				+ f_dis_left2*(-sin(phi_o)*(x_o-x_c_left2)+cos(phi_o)*(y_o-y_c_left2)) - fabs((f_tan_left2)*(-cos(phi_o)*(x_o-x_c_left2)+sin(phi_o)*(y_o-y_c_left2)))
				+ f_dis_right1*(-sin(phi_o)*(x_o-x_c_right1)+cos(phi_o)*(y_o-y_c_right1)) + fabs((f_tan_right1)*(-cos(phi_o)*(x_o-x_c_right1)+sin(phi_o)*(y_o-y_c_right1)))
				)/J_o - D_f_rotation*dphi_o;
		}
		else {
		}
		//ddx_o = (f_dis_left1*cos(phi_res_left1)+f_dis_left2*cos(phi_res_left2)+f_dis_right1*cos(phi_res_right1))/M_o - D_f*dx_o;
		//ddy_o = (f_dis_left1*sin(phi_res_left1)+f_dis_left2*sin(phi_res_left2)+f_dis_right1*sin(phi_res_right1))/M_o - D_f*dy_o;
		//ddphi_o = (f_dis_left1*(-sin(phi_res_left1)*(x_o-x_c_left1)+cos(phi_res_left1)*(y_o-y_c_left1)) 
		//	+ f_dis_left2*(-sin(phi_res_left2)*(x_o-x_c_left2)+cos(phi_res_left2)*(y_o-y_c_left2)) 
		//	+ f_dis_right1*(-sin(phi_res_right1)*(x_o-x_c_right1)+cos(phi_res_right1)*(y_o-y_c_right1)) 
		//	)/J_o - D_f_rotation*dphi_o;

		

		dx_o += ddx_o*ST;
		x_o += dx_o*ST;
		dy_o += ddy_o*ST;
		y_o += dy_o*ST;
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

		l_tan_left1 = cos(phi_o)*(y_c_left1-y_o-sin(phi_o)*(-L_o))-sin(phi_o)*(x_c_left2-x_o-cos(phi_o)*(-L_o));
		if (firstRound==1){
			dl_tan_left1 = 0.0;
		}
		else{
			dl_tan_left1 = (l_tan_left1 - l_tan_left1_prev)/ST;
		}
		l_tan_left1_prev = l_tan_left1;
		
		l_tan_left2 = cos(phi_o)*(y_c_left2-y_o-sin(phi_o)*(-L_o))-sin(phi_o)*(x_c_left2-x_o-cos(phi_o)*(-L_o));
		if (firstRound==1){
			dl_tan_left2 = 0.0;
		}
		else{
			dl_tan_left2 = (l_tan_left2 - l_tan_left2_prev)/ST;
		}
		l_tan_left2_prev = l_tan_left2;

		l_tan_right1 = cos(phi_o)*(y_c_right1-y_o-sin(phi_o)*(-L_o))-sin(phi_o)*(x_c_right1-x_o-cos(phi_o)*(L_o));
		if (firstRound==1){
			dl_tan_right1 = 0.0;
		}
		else{
			dl_tan_right1 = (l_tan_right1 - l_tan_right1_prev)/ST;
		}
		l_tan_right1_prev = l_tan_right1;


		/*l_wall = fabs(cos(phi_o)*(x_wall-x_o-cos(phi_o)*L_o)+sin(phi_o)*(y_wall-y_o-sin(phi_o)*L_o));
		if (firstRound==1){
			dl_wall = 0.0;
		}
		else{
			dl_wall = (l_wall - l_wall_prev)/ST;
		}
		l_wall_prev = l_wall;*/

		/*left1 disturbance observer*/	
		tau_dob_left1_l = integral_tau_dob_left1_l - dtheta_res_left1_l*J_n*GDIS;
		integral_tau_dob_left1_l 
			+= ((K_tn*ia_left1_l + dtheta_res_left1_l*J_n*GDIS) - integral_tau_dob_left1_l)*GDIS*ST;
		
		tau_dob_left1_r = integral_tau_dob_left1_r - dtheta_res_left1_r*J_n*GDIS;
		integral_tau_dob_left1_r 
			+= ((K_tn*ia_left1_r + dtheta_res_left1_r*J_n*GDIS) - integral_tau_dob_left1_r)*GDIS*ST;

		/*left1 reaction torque observer*/
		tau_rtob_left1_l = integral_tau_rtob_left1_l - dtheta_res_left1_l*J_n*GDIS;
		integral_tau_rtob_left1_l 
			+= (K_tn*ia_left1_l - D_r*dtheta_res_left1_l + dtheta_res_left1_l*J_n*GDIS - integral_tau_rtob_left1_l)*GDIS*ST;
		
		tau_rtob_left1_r = integral_tau_rtob_left1_r - dtheta_res_left1_r*J_n*GDIS;
		integral_tau_rtob_left1_r 
			+= (K_tn*ia_left1_r - D_r*dtheta_res_left1_r + dtheta_res_left1_l*J_n*GDIS - integral_tau_rtob_left1_r)*GDIS*ST;

		/*left1 Mobile Robot Reaction Force*/

		f_res_left1 = integral_f_res_left1 - GROBOT*M_r*R_w*(dtheta_res_left1_l+dtheta_res_left1_r)/2;
		integral_f_res_left1 += ((tau_rtob_left1_l+tau_rtob_left1_r)/R_w + GROBOT*M_r*R_w*(dtheta_res_left1_l+dtheta_res_left1_r)/2 - integral_f_res_left1)*GROBOT*ST;

		/*left2 disturbance observer*/	
		tau_dob_left2_l = integral_tau_dob_left2_l - dtheta_res_left2_l*J_n*GDIS;
		integral_tau_dob_left2_l 
			+= ((K_tn*ia_left2_l + dtheta_res_left2_l*J_n*GDIS) - integral_tau_dob_left2_l)*GDIS*ST;
		
		tau_dob_left2_r = integral_tau_dob_left2_r - dtheta_res_left2_r*J_n*GDIS;
		integral_tau_dob_left2_r 
			+= ((K_tn*ia_left2_r + dtheta_res_left2_r*J_n*GDIS) - integral_tau_dob_left2_r)*GDIS*ST;

		/*left2 reaction torque observer*/
		tau_rtob_left2_l = integral_tau_rtob_left2_l - dtheta_res_left2_l*J_n*GDIS;
		integral_tau_rtob_left2_l 
			+= (K_tn*ia_left2_l - D_r*dtheta_res_left2_l + dtheta_res_left2_l*J_n*GDIS - integral_tau_rtob_left2_l)*GDIS*ST;
		
		tau_rtob_left2_r = integral_tau_rtob_left2_r - dtheta_res_left2_r*J_n*GDIS;
		integral_tau_rtob_left2_r 
			+= (K_tn*ia_left2_r - D_r*dtheta_res_left2_r + dtheta_res_left2_l*J_n*GDIS - integral_tau_rtob_left2_r)*GDIS*ST;

		/*left2 Mobile Robot Reaction Force*/

		f_res_left2 = integral_f_res_left2 - GROBOT*M_r*R_w*(dtheta_res_left2_l+dtheta_res_left2_r)/2;
		integral_f_res_left2 += ((tau_rtob_left2_l+tau_rtob_left2_r)/R_w + GROBOT*M_r*R_w*(dtheta_res_left2_l+dtheta_res_left2_r)/2 - integral_f_res_left2)*GROBOT*ST;

		/*right1 disturbance observer*/	
		tau_dob_right1_l = integral_tau_dob_right1_l - dtheta_res_right1_l*J_n*GDIS;
		integral_tau_dob_right1_l 
			+= ((K_tn*ia_right1_l + dtheta_res_right1_l*J_n*GDIS) - integral_tau_dob_right1_l)*GDIS*ST;
		
		tau_dob_right1_r = integral_tau_dob_right1_r - dtheta_res_right1_r*J_n*GDIS;
		integral_tau_dob_right1_r 
			+= ((K_tn*ia_right1_r + dtheta_res_right1_r*J_n*GDIS) - integral_tau_dob_right1_r)*GDIS*ST;

		/*right1 reaction torque observer*/
		tau_rtob_right1_l = integral_tau_rtob_right1_l - dtheta_res_right1_l*J_n*GDIS;
		integral_tau_rtob_right1_l 
			+= (K_tn*ia_right1_l - D_r*dtheta_res_right1_l + dtheta_res_right1_l*J_n*GDIS - integral_tau_rtob_right1_l)*GDIS*ST;
		
		tau_rtob_right1_r = integral_tau_rtob_right1_r - dtheta_res_right1_r*J_n*GDIS;
		integral_tau_rtob_right1_r 
			+= (K_tn*ia_right1_r - D_r*dtheta_res_right1_r + dtheta_res_right1_l*J_n*GDIS - integral_tau_rtob_right1_r)*GDIS*ST;

		/*right1 Mobile Robot Reaction Force*/

		f_res_right1 = integral_f_res_right1 - GROBOT*M_r*R_w*(dtheta_res_right1_l+dtheta_res_right1_r)/2;
		integral_f_res_right1 += ((tau_rtob_right1_l+tau_rtob_right1_r)/R_w + GROBOT*M_r*R_w*(dtheta_res_right1_l+dtheta_res_right1_r)/2 - integral_f_res_right1)*GROBOT*ST;

		fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %i %i %i %i\n", t, x_o, y_o, phi_o, x_res_left1, y_res_left1, phi_res_left1, f_dis_left1, x_res_left2, y_res_left2, phi_res_left2, f_dis_left2, v_res_left1, f_res_left1, f_res_left2, reaction_wall, x_wall, y_wall, x_res_right1, y_res_right1, phi_res_right1, f_dis_right1, f_res_right1, behaviour, control_mode_left1, control_mode_left2, control_mode_right1);
		
		t += ST;
		firstRound = 0;
      
	}

	fclose(fp);




	return 0;
}

