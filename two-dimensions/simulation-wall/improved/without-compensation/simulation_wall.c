#include <stdio.h>
#include <math.h>

#define	T	10 //seconds
#define ST	0.0001 //second
#define Pi	3.1415

//Wheel parameters
#define K_tn	0.43   //Nm
#define J_n	0.01 //Ns^2/m
#define D_r	0.06

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

//Friction from floor
#define D_f	50.0
#define D_f_rotation	100

//Gains
#define GDIS	500.0
#define K_p	10.0
#define K_v	50.0
#define M_c	1.0
#define K_f	0.6
#define K_f_int	3.75
#define K_f_der 0.024
#define K_phi	0.0
#define K_dphi	0.0
#define K_intphi	0.0

int main(void)
{
	double t = 0.0;

	int firstRound = 1;

/*bottom1*/
/*Reference Commands*/

	double f_bottom1_command = 0.0;
	double err_force_bottom1 = 0.0;
	double err_int_force_bottom1 = 0.0;
	double err_der_force_bottom1 = 0.0;
	double pre_err_force_bottom1 = 0.0;

/*Mobile Robot Outputs*/

	double x_res_bottom1 = -0.8;
	double dx_res_bottom1 = 0.0;
	double ddx_res_bottom1 = 0.0;
	double y_res_bottom1 = -1.08;
	double dy_res_bottom1 = 0.0;
	double ddy_res_bottom1 = 0.0;
	double phi_res_bottom1 = Pi/2;
	double dphi_res_bottom1 = 0.0;
	double ddphi_res_bottom1 = 0.0;

	double x_c_bottom1 = 0.0;
	double y_c_bottom1 = 0.0;
	
	double l_bottom1 = 0.0;
	double dl_bottom1 = 0.0;
	double l_bottom1_prev = 0.0;

	double err_x_bottom1 = 0.0;
	double derr_x_bottom1 = 0.0;
	double err_y_bottom1 = 0.0;
	double derr_y_bottom1 = 0.0;

	double f_res_bottom1 = 0.0;
	double tau_res_bottom1 = 0.0;

/*Mobile Robot Inputs*/
	
	double dv_ref_bottom1 = 0.0;
	double dw_ref_bottom1 = 0.0;

	double f_dis_bottom1 = 0.0;


/*bottom1 Motor*/

	double ddtheta_ref_bottom1_l = 0.0;
	double ddtheta_ref_bottom1_r = 0.0;
	
	double ia_ref_bottom1_l = 0.0;
	double ia_ref_bottom1_r = 0.0;

	double ia_bottom1_l = 0.0;
	double ia_bottom1_r = 0.0;

	double tau_dis_bottom1_l = 0.0;
	double tau_dis_bottom1_r = 0.0;

	double tau_dob_bottom1_l = 0.0;
	double tau_dob_bottom1_r = 0.0;

	double integral_tau_dob_bottom1_l = 0.0;
	double integral_tau_dob_bottom1_r = 0.0;

	double tau_rtob_bottom1_l = 0.0;
	double tau_rtob_bottom1_r = 0.0;

	double integral_tau_rtob_bottom1_l = 0.0;
	double integral_tau_rtob_bottom1_r = 0.0;

	double theta_res_bottom1_l = 0.0;
	double theta_res_bottom1_r = 0.0;

	double dtheta_res_bottom1_l = 0.0;
	double dtheta_res_bottom1_r = 0.0;

	double ddtheta_res_bottom1_l = 0.0;
	double ddtheta_res_bottom1_r = 0.0;

/*bottom2*/
/*Reference Commands*/

	double err_force_bottom2 = 0.0;
	double err_int_force_bottom2 = 0.0;
	double err_der_force_bottom2 = 0.0;
	double pre_err_force_bottom2 = 0.0;

	double f_bottom2_command = 0.0;

/*Mobile Robot Outputs*/

	double x_res_bottom2 = 0.8;
	double dx_res_bottom2 = 0.0;
	double ddx_res_bottom2 = 0.0;
	double y_res_bottom2 = -1.08;
	double dy_res_bottom2 = 0.0;
	double ddy_res_bottom2 = 0.0;
	double phi_res_bottom2 = Pi/2;
	double dphi_res_bottom2 = 0.0;
	double ddphi_res_bottom2 = 0.0;

	double x_c_bottom2 = 0.0;
	double y_c_bottom2 = 0.0;
	
	double l_bottom2 = 0.0;
	double dl_bottom2 = 0.0;
	double l_bottom2_prev = 0.0;

	double err_x_bottom2 = 0.0;
	double derr_x_bottom2 = 0.0;
	double err_y_bottom2 = 0.0;
	double derr_y_bottom2 = 0.0;

	double f_res_bottom2 = 0.0;
	double tau_res_bottom2 = 0.0;

/*Mobile Robot Inputs*/
	
	double dv_ref_bottom2 = 0.0;
	double dw_ref_bottom2 = 0.0;

	double f_dis_bottom2 = 0.0;

/*bottom2 Motor*/

	double ddtheta_ref_bottom2_l = 0.0;
	double ddtheta_ref_bottom2_r = 0.0;
	
	double ia_ref_bottom2_l = 0.0;
	double ia_ref_bottom2_r = 0.0;

	double ia_bottom2_l = 0.0;
	double ia_bottom2_r = 0.0;

	double tau_dis_bottom2_l = 0.0;
	double tau_dis_bottom2_r = 0.0;

	double tau_dob_bottom2_l = 0.0;
	double tau_dob_bottom2_r = 0.0;

	double integral_tau_dob_bottom2_l = 0.0;
	double integral_tau_dob_bottom2_r = 0.0;

	double tau_rtob_bottom2_l = 0.0;
	double tau_rtob_bottom2_r = 0.0;

	double integral_tau_rtob_bottom2_l = 0.0;
	double integral_tau_rtob_bottom2_r = 0.0;

	double theta_res_bottom2_l = 0.0;
	double theta_res_bottom2_r = 0.0;

	double dtheta_res_bottom2_l = 0.0;
	double dtheta_res_bottom2_r = 0.0;

	double ddtheta_res_bottom2_l = 0.0;
	double ddtheta_res_bottom2_r = 0.0;


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
	double tau_res_right1 = 0.0;

/*Mobile Robot Inputs*/
	
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

	double x_wall = 0.1;
	double y_wall = 1.0;

	double reaction_wall = 0.0;
	double l_wall = 0.0;
	double dl_wall = 0.0;
	double l_wall_prev = 0.0;

	FILE *fp;

	if((fp=fopen("simulation_wall.dat", "w"))==NULL){
		fprintf(stderr, "File open failed.\n");
		return 1;
	}

	while(t <= T){	


		/*bottom1 commands*/

		f_bottom1_command = 2.5;
		
		x_c_bottom1 = x_res_bottom1 + cos(phi_o)*R_r-sin(phi_o)*(0.0);
		y_c_bottom1 = y_res_bottom1 + sin(phi_o)*R_r+cos(phi_o)*(0.0);

		if(cos(phi_o)*(y_c_bottom1-y_o+cos(phi_o)*L_o)-sin(phi_o)*(x_c_bottom1-x_o-sin(phi_o)*L_o)>0){
			f_dis_bottom1 = K_o*l_bottom1 + D_o*dl_bottom1; 
		}
		else{
			f_dis_bottom1 = 0.0;
		}

		tau_dis_bottom1_l = f_dis_bottom1*R_w/2 + D_r*dtheta_res_bottom1_l + ((R_w*R_w)/(4*R_r*R_r))*((M_r*R_r*R_r+J_r)*ddtheta_res_bottom1_l+(M_r*R_r*R_r-J_r)*ddtheta_res_bottom1_r);
		tau_dis_bottom1_r = f_dis_bottom1*R_w/2 + D_r*dtheta_res_bottom1_r + ((R_w*R_w)/(4*R_r*R_r))*((M_r*R_r*R_r-J_r)*ddtheta_res_bottom1_l+(M_r*R_r*R_r+J_r)*ddtheta_res_bottom1_r);
		
		/*bottom1 Mobile Robot Controller*/
		
		if (phi_o>0){
			f_bottom1_command = 2.5+(K_phi*(phi_o)+K_dphi*(dphi_o)+K_intphi*intphi_o);
		}
		else{
			f_bottom1_command = 2.5;
		}

		err_force_bottom1 = f_bottom1_command-f_dis_bottom1;
		err_int_force_bottom1 += err_force_bottom1*ST;
		err_der_force_bottom1 = (err_force_bottom1-pre_err_force_bottom1)/ST;
		pre_err_force_bottom1 = err_force_bottom1;

		dv_ref_bottom1 = K_f*err_force_bottom1+K_f_int*err_int_force_bottom1+K_f_der*err_der_force_bottom1;

		dw_ref_bottom1 = 0.0;

		/*bottom1 Motor Controller*/

		ddtheta_ref_bottom1_l = dv_ref_bottom1/R_w - R_r*dw_ref_bottom1/R_w;
		ddtheta_ref_bottom1_r = dv_ref_bottom1/R_w + R_r*dw_ref_bottom1/R_w;

		ia_ref_bottom1_l = J_n*ddtheta_ref_bottom1_l/K_tn;
		ia_ref_bottom1_r = J_n*ddtheta_ref_bottom1_r/K_tn;

		ia_bottom1_l = ia_ref_bottom1_l + tau_dob_bottom1_l/K_tn;
		ia_bottom1_r = ia_ref_bottom1_r + tau_dob_bottom1_r/K_tn;

		ddtheta_res_bottom1_l = (K_tn*ia_bottom1_l - tau_dis_bottom1_l)/J_n;
		ddtheta_res_bottom1_r = (K_tn*ia_bottom1_r - tau_dis_bottom1_r)/J_n;

		dtheta_res_bottom1_l += ddtheta_res_bottom1_l * ST;
		dtheta_res_bottom1_r += ddtheta_res_bottom1_r * ST;

		theta_res_bottom1_l += dtheta_res_bottom1_l * ST;
		theta_res_bottom1_r += dtheta_res_bottom1_r * ST;

		/*bottom1 Robot Motion*/
		ddx_res_bottom1 = R_w*(ddtheta_res_bottom1_l+ddtheta_res_bottom1_r)*cos(phi_res_bottom1)/2;
		dx_res_bottom1 += ddx_res_bottom1*ST;
		x_res_bottom1 += dx_res_bottom1*ST;

		ddy_res_bottom1 = R_w*(ddtheta_res_bottom1_l+ddtheta_res_bottom1_r)*sin(phi_res_bottom1)/2;
		dy_res_bottom1 += ddy_res_bottom1*ST;
		y_res_bottom1 += dy_res_bottom1*ST;

		ddphi_res_bottom1 = R_w*(ddtheta_res_bottom1_r-ddtheta_res_bottom1_l)/(2*R_r);
		dphi_res_bottom1 += ddphi_res_bottom1*ST; 
		phi_res_bottom1 += dphi_res_bottom1*ST;

		/*bottom2 commands*/

		f_bottom2_command = 2.5;
		
		x_c_bottom2 = x_res_bottom2 + cos(phi_o)*R_r-sin(phi_o)*(0.0);
		y_c_bottom2 = y_res_bottom2 + sin(phi_o)*R_r+cos(phi_o)*(0.0);

		if(cos(phi_o)*(y_c_bottom2-y_o+cos(phi_o)*L_o)-sin(phi_o)*(x_c_bottom2-x_o-sin(phi_o)*L_o)>0){
			f_dis_bottom2 = K_o*l_bottom2 + D_o*dl_bottom2; 
		}
		else{
			f_dis_bottom2 = 0.0;
		}

		
		tau_dis_bottom2_l = f_dis_bottom2*R_w/2 + D_r*dtheta_res_bottom2_l + ((R_w*R_w)/(4*R_r*R_r))*((M_r*R_r*R_r+J_r)*ddtheta_res_bottom2_l+(M_r*R_r*R_r-J_r)*ddtheta_res_bottom2_r);
		tau_dis_bottom2_r = f_dis_bottom2*R_w/2 + D_r*dtheta_res_bottom2_r + ((R_w*R_w)/(4*R_r*R_r))*((M_r*R_r*R_r-J_r)*ddtheta_res_bottom2_l+(M_r*R_r*R_r+J_r)*ddtheta_res_bottom2_r);

		/*bottom2 Mobile Robot Controller*/

		if (phi_o<0){
			f_bottom2_command = 2.5-(K_phi*(phi_o)+K_dphi*(dphi_o)+K_intphi*intphi_o);
		}
		else{
			f_bottom2_command = 2.5;
		}

		err_force_bottom2 = f_bottom2_command-f_dis_bottom2;
		err_int_force_bottom2 += err_force_bottom2*ST;
		err_der_force_bottom2 = (err_force_bottom2-pre_err_force_bottom2)/ST;
		pre_err_force_bottom2 = err_force_bottom2;

		dv_ref_bottom2 = K_f*err_force_bottom2+K_f_int*err_int_force_bottom2+K_f_der*err_der_force_bottom2;
		dw_ref_bottom2 = 0.0;

		/*bottom2 Motor Controller*/

		ddtheta_ref_bottom2_l = dv_ref_bottom2/R_w - R_r*dw_ref_bottom2/R_w;
		ddtheta_ref_bottom2_r = dv_ref_bottom2/R_w + R_r*dw_ref_bottom2/R_w;
		ia_ref_bottom2_l = J_n*ddtheta_ref_bottom2_l/K_tn;
		ia_ref_bottom2_r = J_n*ddtheta_ref_bottom2_r/K_tn;

		ia_bottom2_l = ia_ref_bottom2_l + tau_dob_bottom2_l/K_tn;
		ia_bottom2_r = ia_ref_bottom2_r + tau_dob_bottom2_r/K_tn;

		ddtheta_res_bottom2_l = (K_tn*ia_bottom2_l - tau_dis_bottom2_l)/J_n; //ajouter masse et inertie du robot ici
		ddtheta_res_bottom2_r = (K_tn*ia_bottom2_r - tau_dis_bottom2_r)/J_n; //ajouter masse et inertie du robot ici

		dtheta_res_bottom2_l += ddtheta_res_bottom2_l * ST;
		dtheta_res_bottom2_r += ddtheta_res_bottom2_r * ST;

		theta_res_bottom2_l += dtheta_res_bottom2_l * ST;
		theta_res_bottom2_r += dtheta_res_bottom2_r * ST;

		/*bottom2 Robot Motion*/
		ddx_res_bottom2 = R_w*(ddtheta_res_bottom2_l+ddtheta_res_bottom2_r)*cos(phi_res_bottom2)/2;
		dx_res_bottom2 += ddx_res_bottom2*ST;
		x_res_bottom2 += dx_res_bottom2*ST;

		ddy_res_bottom2 = R_w*(ddtheta_res_bottom2_l+ddtheta_res_bottom2_r)*sin(phi_res_bottom2)/2;
		dy_res_bottom2 += ddy_res_bottom2*ST;
		y_res_bottom2 += dy_res_bottom2*ST;

		ddphi_res_bottom2 = R_w*(ddtheta_res_bottom2_r-ddtheta_res_bottom2_l)/(2*R_r);
		dphi_res_bottom2 += ddphi_res_bottom2*ST; 
		phi_res_bottom2 += dphi_res_bottom2*ST;

		/*Object Motion*/

		if (cos(phi_o)*1.0-sin(phi_o)*0.0<y_o+L_o){
			reaction_wall = K_o*l_wall + D_o*dl_wall; 
		}
		else{
			reaction_wall = 0.0;
		}

		ddx_o = (f_dis_bottom1*sin(phi_o)+f_dis_bottom2*sin(phi_o)+sin(phi_o)*reaction_wall)/M_o - D_f*dx_o;
		dx_o += ddx_o*ST;
		x_o += dx_o*ST;
		ddy_o = (f_dis_bottom1*cos(phi_o)+f_dis_bottom2*cos(phi_o)-cos(phi_o)*reaction_wall)/M_o - D_f*dy_o;
		dy_o += ddy_o*ST;
		y_o += dy_o*ST;
		ddphi_o = -(f_dis_bottom1*fabs((cos(phi_o)*(x_o-x_c_bottom1)+sin(phi_o)*(y_o-y_c_bottom1)))
			+ f_dis_bottom2*fabs((cos(phi_o)*(x_o-x_c_bottom2)+sin(phi_o)*(y_o-y_c_bottom2)))
			- reaction_wall*fabs((cos(phi_o)*(x_o-x_wall)+sin(phi_o)*(y_o-y_wall)))
			)/J_o - D_f_rotation*dphi_o;
		dphi_o += ddphi_o*ST;
		phi_o += dphi_o*ST;
		intphi_o += phi_o;

		l_bottom1 = fabs(cos(phi_o)*(y_c_bottom1-y_o+cos(phi_o)*L_o)-sin(phi_o)*(x_c_bottom1-x_o-sin(phi_o)*L_o));
		if (firstRound==1){
			dl_bottom1 = 0.0;
		}
		else{
			dl_bottom1 = (l_bottom1 - l_bottom1_prev)/ST;
		}
		l_bottom1_prev = l_bottom1;

		l_bottom2 = fabs(cos(phi_o)*(y_c_bottom2-y_o+cos(phi_o)*L_o)-sin(phi_o)*(x_c_bottom2-x_o-sin(phi_o)*L_o));
		if (firstRound==1){
			dl_bottom2 = 0.0;
		}
		else{
			dl_bottom2 = (l_bottom2 - l_bottom2_prev)/ST;
		}
		l_bottom2_prev = l_bottom2;

		l_wall = fabs(cos(phi_o)*(y_wall-y_o-cos(phi_o)*L_o)-sin(phi_o)*(x_wall-x_o+sin(phi_o)*L_o));
		if (firstRound==1){
			dl_wall = 0.0;
		}
		else{
			dl_wall = (l_wall - l_wall_prev)/ST;
		}
		l_wall_prev = l_wall;

		/*bottom1 disturbance observer*/	
		tau_dob_bottom1_l = integral_tau_dob_bottom1_l - dtheta_res_bottom1_l*J_n*GDIS;
		integral_tau_dob_bottom1_l 
			+= ((K_tn*ia_bottom1_l + dtheta_res_bottom1_l*J_n*GDIS) - integral_tau_dob_bottom1_l)*GDIS*ST;
		
		tau_dob_bottom1_r = integral_tau_dob_bottom1_r - dtheta_res_bottom1_r*J_n*GDIS;
		integral_tau_dob_bottom1_r 
			+= ((K_tn*ia_bottom1_r + dtheta_res_bottom1_r*J_n*GDIS) - integral_tau_dob_bottom1_r)*GDIS*ST;

		/*bottom1 reaction torque observer*/
		tau_rtob_bottom1_l = integral_tau_rtob_bottom1_l - dtheta_res_bottom1_l*J_n*GDIS;
		integral_tau_rtob_bottom1_l 
			+= ((K_tn*ia_bottom1_l - D_r*dtheta_res_bottom1_l - ((R_w*R_w)/(4*R_r*R_r))*((M_r*R_r*R_r+J_r)*ddtheta_res_bottom1_l+(M_r*R_r*R_r-J_r)*ddtheta_res_bottom1_r) + dtheta_res_bottom1_l*J_n*GDIS) - integral_tau_rtob_bottom1_l)*GDIS*ST;
		
		tau_rtob_bottom1_r = integral_tau_rtob_bottom1_r - dtheta_res_bottom1_r*J_n*GDIS;
		integral_tau_rtob_bottom1_r 
			+= ((K_tn*ia_bottom1_r - D_r*dtheta_res_bottom1_r - ((R_w*R_w)/(4*R_r*R_r))*((M_r*R_r*R_r-J_r)*ddtheta_res_bottom1_l+(M_r*R_r*R_r+J_r)*ddtheta_res_bottom1_r) + dtheta_res_bottom1_r*J_n*GDIS) - integral_tau_rtob_bottom1_r)*GDIS*ST;

		/*bottom1 Mobile Robot Reaction Force*/

		f_res_bottom1 = (tau_rtob_bottom1_l+tau_rtob_bottom1_r)/R_w;
		tau_res_bottom1 = (tau_rtob_bottom1_l-tau_rtob_bottom1_r)*R_r/(R_w);


		/*bottom2 disturbance observer*/	
		tau_dob_bottom2_l = integral_tau_dob_bottom2_l - dtheta_res_bottom2_l*J_n*GDIS;
		integral_tau_dob_bottom2_l 
			+= ((K_tn*ia_bottom2_l + dtheta_res_bottom2_l*J_n*GDIS) - integral_tau_dob_bottom2_l)*GDIS*ST;
		
		tau_dob_bottom2_r = integral_tau_dob_bottom2_r - dtheta_res_bottom2_r*J_n*GDIS;
		integral_tau_dob_bottom2_r 
			+= ((K_tn*ia_bottom2_r + dtheta_res_bottom2_r*J_n*GDIS) - integral_tau_dob_bottom2_r)*GDIS*ST;

		/*bottom2 reaction torque observer*/
		tau_rtob_bottom2_l = integral_tau_rtob_bottom2_l - dtheta_res_bottom2_l*J_n*GDIS;
		integral_tau_rtob_bottom2_l 
			+= ((K_tn*ia_bottom2_l - D_r*dtheta_res_bottom2_l - ((R_w*R_w)/(4*R_r*R_r))*((M_r*R_r*R_r+J_r)*ddtheta_res_bottom2_l+(M_r*R_r*R_r-J_r)*ddtheta_res_bottom2_r) + dtheta_res_bottom2_l*J_n*GDIS) - integral_tau_rtob_bottom2_l)*GDIS*ST;
		
		tau_rtob_bottom2_r = integral_tau_rtob_bottom2_r - dtheta_res_bottom2_r*J_n*GDIS;
		integral_tau_rtob_bottom2_r 
			+= ((K_tn*ia_bottom2_r - D_r*dtheta_res_bottom2_r - ((R_w*R_w)/(4*R_r*R_r))*((M_r*R_r*R_r-J_r)*ddtheta_res_bottom2_l+(M_r*R_r*R_r+J_r)*ddtheta_res_bottom2_r) + dtheta_res_bottom2_r*J_n*GDIS) - integral_tau_rtob_bottom2_r)*GDIS*ST;

		/*bottom2 Mobile Robot Reaction Force*/

		f_res_bottom2 = (tau_dob_bottom2_l+tau_dob_bottom2_r)/R_w;
		tau_res_bottom2 = (tau_dob_bottom2_l-tau_dob_bottom2_r)*R_r/(R_w);

		fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", t, x_o, y_o, phi_o, x_res_bottom1, y_res_bottom1, phi_res_bottom1, f_dis_bottom1, x_res_bottom2, y_res_bottom2, phi_res_bottom2, f_dis_bottom2, reaction_wall, x_wall, y_wall);
		
		t += ST;
		firstRound = 0;
      
	}

	fclose(fp);




	return 0;
}

