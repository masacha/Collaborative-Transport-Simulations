#include <stdio.h>
#include <math.h>

#define	T	10 //seconds
#define ST	0.0001 //second
#define GDIS	500.0
#define K_p	10.0
#define K_v	10.0
#define M_c	1.0
#define K_f	1.0
#define K_o	1000.0 //N/m
#define D_o	1.0
#define M_o	0.5 //kg
#define J_o	1.0
#define L_o	1.000 //meter
#define R_w	0.030 //meter
#define R_r	0.10 //meter
#define K_tn	1.7   //Nm
#define J_n	0.1 //Ns^2/m
#define Pi	3.1415
#define D_f	50.0
#define D_f_rotation	100

int main(void)
{
	double t = 0.0;

/*Reference Commands*/

	double x_bottom_command = 0.0; //Ne pas modifier
	double y_bottom_command = 0.0;
	double dx_bottom_command = 0.0; //Ne pas modifier
	double dy_bottom_command = 0.0;
	double phi_bottom_command = 0.0;
	double f_bottom_command = 0.0;

/*Mobile Robot Outputs*/

	double x_res_bottom = -0.2;
	double dx_res_bottom = 0.0;
	double ddx_res_bottom = 0.0;
	double y_res_bottom = -1.5;
	double dy_res_bottom = 0.0;
	double ddy_res_bottom = 0.0;
	double phi_res_bottom = Pi/2;
	double dphi_res_bottom = 0.0;
	double ddphi_res_bottom = 0.0;

	double x_c_bottom = 0.0;
	double y_c_bottom = 0.0;
	
	double l_bottom = 0.0;
	double dl_bottom = 0.0;
	double l_bottom_prev = 0.0;

	double err_x_bottom = 0.0;
	double derr_x_bottom = 0.0;
	double err_y_bottom = 0.0;
	double derr_y_bottom = 0.0;

	double f_res_bottom = 0.0;
	double tau_res_bottom = 0.0;

/*Mobile Robot Inputs*/
	
	double ddx_comp_bottom = 0.0;
	double ddy_comp_bottom = 0.0;

	double ddx_ref_bottom = 0.0;
	double ddy_ref_bottom = 0.0;

	double ddphi_ref_bottom = 0.0;

	double f_dis_bottom = 0.0;

/*Reference Commands*/

	double x_left_command = 0.0; //Ne pas modifier
	double y_left_command = 0.0;
	double dx_left_command = 0.0; //Ne pas modifier
	double dy_left_command = 0.0;
	double phi_left_command = 0.0;
	double f_left_command = 0.0;

/*Mobile Robot Outputs*/

	double x_res_left = -1.5;
	double dx_res_left = 0.0;
	double ddx_res_left = 0.0;
	double y_res_left = 0.0;
	double dy_res_left = 0.0;
	double ddy_res_left = 0.0;
	double phi_res_left = 0.0;
	double dphi_res_left = 0.0;
	double ddphi_res_left = 0.0;

	double x_c_left = 0.0;
	double y_c_left = 0.0;
	
	double l_left = 0.0;
	double dl_left = 0.0;
	double l_left_prev = 0.0;

	double err_x_left = 0.0;
	double derr_x_left = 0.0;
	double err_y_left = 0.0;
	double derr_y_left = 0.0;

	double f_res_left = 0.0;
	double tau_res_left = 0.0;

/*Mobile Robot Inputs*/
	
	double ddx_comp_left = 0.0;
	double ddy_comp_left = 0.0;

	double ddx_ref_left = 0.0;
	double ddy_ref_left = 0.0;

	double ddphi_ref_left = 0.0;

	double f_dis_left = 0.0;

/*Reference Commands*/

	double x_right_command = 0.0; //Ne pas modifier
	double y_right_command = 0.0;
	double dx_right_command = 0.0; //Ne pas modifier
	double dy_right_command = 0.0;
	double phi_right_command = 0.0;
	double f_right_command = 0.0;

/*Mobile Robot Outputs*/

	double x_res_right = +1.5;
	double dx_res_right = 0.0;
	double ddx_res_right = 0.0;
	double y_res_right = 0.8;
	double dy_res_right = 0.0;
	double ddy_res_right = 0.0;
	double phi_res_right = Pi;
	double dphi_res_right = 0.0;
	double ddphi_res_right = 0.0;

	double x_c_right = 0.0;
	double y_c_right = 0.0;
	
	double l_right = 0.0;
	double dl_right = 0.0;
	double l_right_prev = 0.0;

	double err_x_right = 0.0;
	double derr_x_right = 0.0;
	double err_y_right = 0.0;
	double derr_y_right = 0.0;

	double f_res_right = 0.0;
	double tau_res_right = 0.0;

/*Mobile Robot Inputs*/
	
	double ddx_comp_right = 0.0;
	double ddy_comp_right = 0.0;

	double ddx_ref_right = 0.0;
	double ddy_ref_right = 0.0;

	double ddphi_ref_right = 0.0;

	double f_dis_right = 0.0;

/*Object*/
	double x_o = 0.0;
	double dx_o = 0.0;
	double ddx_o = 0.0;
	double y_o = 0.0;
	double dy_o = 0.0;
	double ddy_o = 0.0;
	double phi_o = 0.0;
	double dphi_o = 0.0;
	double ddphi_o = 0.0;

	double reaction_wall = 0.0;
	double l_wall = 0.0;
	double dl_wall = 0.0;
	double l_wall_prev = 0.0;

	l_bottom_prev = fabs(cos(phi_o)*(y_c_bottom-y_o+cos(phi_o)*L_o)-sin(phi_o)*(x_c_bottom-x_o-sin(phi_o)*L_o));
	l_left_prev = fabs(cos(phi_o)*(x_c_left-x_o+cos(phi_o)*L_o)+sin(phi_o)*(y_c_left-y_o+sin(phi_o)*L_o));
	l_right_prev = fabs(cos(phi_o)*(x_c_right-x_o-cos(phi_o)*L_o)+sin(phi_o)*(y_c_right-y_o-sin(phi_o)*L_o));
	l_wall_prev = fabs(cos(phi_o)*(1.0-y_o-cos(phi_o)*L_o)-sin(phi_o)*(0.0-x_o+sin(phi_o)*L_o));

/*Bottom Motor*/

	double ddtheta_ref_bottom_l = 0.0;
	double ddtheta_ref_bottom_r = 0.0;
	
	double ia_ref_bottom_l = 0.0;
	double ia_ref_bottom_r = 0.0;

	double ia_bottom_l = 0.0;
	double ia_bottom_r = 0.0;

	double tau_dis_bottom_l = 0.0;
	double tau_dis_bottom_r = 0.0;

	double tau_dob_bottom_l = 0.0;
	double tau_dob_bottom_r = 0.0;

	double integral_tau_dob_bottom_l = 0.0;
	double integral_tau_dob_bottom_r = 0.0;

	double tau_rtob_bottom_l = 0.0;
	double tau_rtob_bottom_r = 0.0;

	double integral_tau_rtob_bottom_l = 0.0;
	double integral_tau_rtob_bottom_r = 0.0;

	double theta_res_bottom_l = 0.0;
	double theta_res_bottom_r = 0.0;

	double dtheta_res_bottom_l = 0.0;
	double dtheta_res_bottom_r = 0.0;

	double ddtheta_res_bottom_l = 0.0;
	double ddtheta_res_bottom_r = 0.0;

/*Left Motor*/

	double ddtheta_ref_left_l = 0.0;
	double ddtheta_ref_left_r = 0.0;
	
	double ia_ref_left_l = 0.0;
	double ia_ref_left_r = 0.0;

	double ia_left_l = 0.0;
	double ia_left_r = 0.0;

	double tau_dis_left_l = 0.0;
	double tau_dis_left_r = 0.0;

	double tau_dob_left_l = 0.0;
	double tau_dob_left_r = 0.0;

	double integral_tau_dob_left_l = 0.0;
	double integral_tau_dob_left_r = 0.0;

	double tau_rtob_left_l = 0.0;
	double tau_rtob_left_r = 0.0;

	double integral_tau_rtob_left_l = 0.0;
	double integral_tau_rtob_left_r = 0.0;

	double theta_res_left_l = 0.0;
	double theta_res_left_r = 0.0;

	double dtheta_res_left_l = 0.0;
	double dtheta_res_left_r = 0.0;

	double ddtheta_res_left_l = 0.0;
	double ddtheta_res_left_r = 0.0;

/*Right Motor*/

	double ddtheta_ref_right_l = 0.0;
	double ddtheta_ref_right_r = 0.0;
	
	double ia_ref_right_l = 0.0;
	double ia_ref_right_r = 0.0;

	double ia_right_l = 0.0;
	double ia_right_r = 0.0;

	double tau_dis_right_l = 0.0;
	double tau_dis_right_r = 0.0;

	double tau_dob_right_l = 0.0;
	double tau_dob_right_r = 0.0;

	double integral_tau_dob_right_l = 0.0;
	double integral_tau_dob_right_r = 0.0;

	double tau_rtob_right_l = 0.0;
	double tau_rtob_right_r = 0.0;

	double integral_tau_rtob_right_l = 0.0;
	double integral_tau_rtob_right_r = 0.0;

	double theta_res_right_l = 0.0;
	double theta_res_right_r = 0.0;

	double dtheta_res_right_l = 0.0;
	double dtheta_res_right_r = 0.0;

	double ddtheta_res_right_l = 0.0;
	double ddtheta_res_right_r = 0.0;

	FILE *fp;

	if((fp=fopen("simulation_wall.dat", "w"))==NULL){
		fprintf(stderr, "File open failed.\n");
		return 1;
	}

	while(t <= T){	

		/*Bottom commands*/

		x_bottom_command = -0.2;
		dx_bottom_command = 0.0;
		y_bottom_command = 0.0;
		dy_bottom_command = 0.0;
		phi_bottom_command = 0.0;

		f_bottom_command = 10;
		
		x_c_bottom = x_res_bottom + cos(phi_o)*0.0-sin(phi_o)*(R_r);
		y_c_bottom = y_res_bottom+ sin(phi_o)*0.0+cos(phi_o)*(R_r);

		if(cos(phi_o)*(y_c_bottom-y_o+cos(phi_o)*L_o)-sin(phi_o)*(x_c_bottom-x_o-sin(phi_o)*L_o)>0){
			f_dis_bottom = K_o*l_bottom + D_o*dl_bottom; 
			tau_dis_bottom_l = f_dis_bottom*R_w*(1-sin(phi_res_bottom-phi_o))/2;
			tau_dis_bottom_r = f_dis_bottom*R_w*(1+sin(phi_res_bottom-phi_o))/2;
		}
		else{
			f_dis_bottom = 0.0;
			tau_dis_bottom_l = 0.0;
			tau_dis_bottom_r = 0.0;
		}
		
		/*Bottom Mobile Robot Controller*/

		err_x_bottom = x_bottom_command - x_res_bottom;
		derr_x_bottom = dx_bottom_command - dx_res_bottom;

		err_y_bottom = y_bottom_command - y_res_bottom;
		derr_y_bottom = dy_bottom_command - dy_res_bottom;

		ddx_comp_bottom = (K_p*err_x_bottom+K_v*derr_x_bottom)/M_c;
		ddy_comp_bottom = (K_p*err_y_bottom+K_v*derr_y_bottom)/M_c;

		ddx_ref_bottom = 0;
		ddy_ref_bottom = 0;//K_f*(f_bottom_command - f_dis_bottom);

		ddphi_ref_bottom = 0.0;

		/*Bottom Motor Controller*/

		ddtheta_ref_bottom_l = (ddx_ref_bottom*cos(phi_res_bottom)+ddy_ref_bottom*sin(phi_res_bottom)-R_r*ddphi_ref_bottom)/R_w;
		ddtheta_ref_bottom_r = (ddx_ref_bottom*cos(phi_res_bottom)+ddy_ref_bottom*sin(phi_res_bottom)+R_r*ddphi_ref_bottom)/R_w;

		ia_ref_bottom_l = J_n*ddtheta_ref_bottom_l/K_tn;
		ia_ref_bottom_r = J_n*ddtheta_ref_bottom_r/K_tn;

		ia_bottom_l = ia_ref_bottom_l + tau_dob_bottom_l/K_tn;
		ia_bottom_r = ia_ref_bottom_r + tau_dob_bottom_r/K_tn;

		ddtheta_res_bottom_l = (K_tn*ia_bottom_l - tau_dis_bottom_l)/J_n;
		ddtheta_res_bottom_r = (K_tn*ia_bottom_r - tau_dis_bottom_r)/J_n;

		dtheta_res_bottom_l += ddtheta_res_bottom_l * ST;
		dtheta_res_bottom_r += ddtheta_res_bottom_r * ST;

		theta_res_bottom_l += dtheta_res_bottom_l * ST;
		theta_res_bottom_r += dtheta_res_bottom_r * ST;

		/*Bottom Robot Motion*/
		ddx_res_bottom = R_w*(ddtheta_res_bottom_l+ddtheta_res_bottom_r)*cos(phi_res_bottom)/2;
		dx_res_bottom += ddx_res_bottom*ST;
		x_res_bottom += dx_res_bottom*ST;

		ddy_res_bottom = R_w*(ddtheta_res_bottom_l+ddtheta_res_bottom_r)*sin(phi_res_bottom)/2;
		dy_res_bottom += ddy_res_bottom*ST;
		y_res_bottom += dy_res_bottom*ST;

		ddphi_res_bottom = R_w*(ddtheta_res_bottom_r-ddtheta_res_bottom_l)/(2*R_r);
		dphi_res_bottom += ddphi_res_bottom*ST; 
		phi_res_bottom += dphi_res_bottom*ST;

		/*Left commands*/

		x_left_command = 0.0;
		dx_left_command = 0.0;
		y_left_command = 0.0;
		dy_left_command = 0.0;
		phi_left_command = 0.0;

		f_left_command = 10;
		
		x_c_left = x_res_left + cos(phi_o)*R_r-sin(phi_o)*(0.0);
		y_c_left = y_res_left + sin(phi_o)*R_r+cos(phi_o)*(0.0);

		if(cos(phi_o)*(x_c_left-x_o+cos(phi_o)*L_o)+sin(phi_o)*(y_c_left-y_o+sin(phi_o)*L_o)>0){
			f_dis_left = K_o*l_left + D_o*dl_left; 
			tau_dis_left_l = f_dis_left*R_w*(1-sin(phi_res_left-phi_o))/2;
			tau_dis_left_r = f_dis_left*R_w*(1+sin(phi_res_left-phi_o))/2;
		}
		else{
			f_dis_left = 0.0;
			tau_dis_left_l = 0.0;
			tau_dis_left_r = 0.0;
		}
		
		/*left Mobile Robot Controller*/

		err_x_left = x_left_command - x_res_left;
		derr_x_left = dx_left_command - dx_res_left;

		err_y_left = y_left_command - y_res_left;
		derr_y_left = dy_left_command - dy_res_left;

		ddx_comp_left = (K_p*err_x_left+K_v*derr_x_left)/M_c;
		ddy_comp_left = (K_p*err_y_left+K_v*derr_y_left)/M_c;

		ddx_ref_left = 0.0;//(K_p*err_x_left+K_v*derr_x_left)/M_c;
		ddy_ref_left = 0.0;

		ddphi_ref_left = 0.0;

		/*left Motor Controller*/

		ddtheta_ref_left_l = (ddx_ref_left*cos(phi_res_left)+ddy_ref_left*sin(phi_res_left)-R_r*ddphi_ref_left)/R_w;
		ddtheta_ref_left_r = (ddx_ref_left*cos(phi_res_left)+ddy_ref_left*sin(phi_res_left)+R_r*ddphi_ref_left)/R_w;

		ia_ref_left_l = J_n*ddtheta_ref_left_l/K_tn;
		ia_ref_left_r = J_n*ddtheta_ref_left_r/K_tn;

		ia_left_l = ia_ref_left_l + tau_dob_left_l/K_tn;
		ia_left_r = ia_ref_left_r + tau_dob_left_r/K_tn;

		ddtheta_res_left_l = (K_tn*ia_left_l - tau_dis_left_l)/J_n;
		ddtheta_res_left_r = (K_tn*ia_left_r - tau_dis_left_r)/J_n;

		dtheta_res_left_l += ddtheta_res_left_l * ST;
		dtheta_res_left_r += ddtheta_res_left_r * ST;

		theta_res_left_l += dtheta_res_left_l * ST;
		theta_res_left_r += dtheta_res_left_r * ST;

		/*left Robot Motion*/
		ddx_res_left = R_w*(ddtheta_res_left_l+ddtheta_res_left_r)*cos(phi_res_left)/2;
		dx_res_left += ddx_res_left*ST;
		x_res_left += dx_res_left*ST;

		ddy_res_left = R_w*(ddtheta_res_left_l+ddtheta_res_left_r)*sin(phi_res_left)/2;
		dy_res_left += ddy_res_left*ST;
		y_res_left += dy_res_left*ST;

		ddphi_res_left = R_w*(ddtheta_res_left_r-ddtheta_res_left_l)/(2*R_r);
		dphi_res_left += ddphi_res_left*ST; 
		phi_res_left += dphi_res_left*ST;

		/*right commands*/

		x_right_command = 0.0;
		dx_right_command = 0.0;
		y_right_command = 0.0;
		dy_right_command = 0.0;
		phi_right_command = 0.0;

		f_right_command = 10;
		
		x_c_right = x_res_right + cos(phi_o)*(-R_r)-sin(phi_o)*(0.0);
		y_c_right = y_res_right+ sin(phi_o)*(-R_r)+cos(phi_o)*(0.0);

		if(cos(phi_o)*(x_c_right-x_o-cos(phi_o)*L_o)+sin(phi_o)*(y_c_right-y_o-sin(phi_o)*L_o)<0){
			f_dis_right = K_o*l_right + D_o*dl_right; 
			tau_dis_right_l = f_dis_right*R_w*(1-sin(phi_res_right-phi_o))/2;
			tau_dis_right_r = f_dis_right*R_w*(1+sin(phi_res_right-phi_o))/2;
		}
		else{
			f_dis_right = 0.0;
			tau_dis_right_l = 0.0;
			tau_dis_right_r = 0.0;
		}
		
		/*right Mobile Robot Controller*/

		err_x_right = x_right_command - x_res_right;
		derr_x_right = dx_right_command - dx_res_right;

		err_y_right = y_right_command - y_res_right;
		derr_y_right = dy_right_command - dy_res_right;

		ddx_comp_right = (K_p*err_x_right+K_v*derr_x_right)/M_c;
		ddy_comp_right = (K_p*err_y_right+K_v*derr_y_right)/M_c;

		ddx_ref_right = ddx_comp_right;
		ddy_ref_right = 0;//K_f*(f_right_command - f_dis_right);

		ddphi_ref_right = 0.0;

		/*right Motor Controller*/

		ddtheta_ref_right_l = (ddx_ref_right*cos(phi_res_right)+ddy_ref_right*sin(phi_res_right)-R_r*ddphi_ref_right)/R_w;
		ddtheta_ref_right_r = (ddx_ref_right*cos(phi_res_right)+ddy_ref_right*sin(phi_res_right)+R_r*ddphi_ref_right)/R_w;

		ia_ref_right_l = J_n*ddtheta_ref_right_l/K_tn;
		ia_ref_right_r = J_n*ddtheta_ref_right_r/K_tn;

		ia_right_l = ia_ref_right_l + tau_dob_right_l/K_tn;
		ia_right_r = ia_ref_right_r + tau_dob_right_r/K_tn;

		ddtheta_res_right_l = (K_tn*ia_right_l - tau_dis_right_l)/J_n;
		ddtheta_res_right_r = (K_tn*ia_right_r - tau_dis_right_r)/J_n;

		dtheta_res_right_l += ddtheta_res_right_l * ST;
		dtheta_res_right_r += ddtheta_res_right_r * ST;

		theta_res_right_l += dtheta_res_right_l * ST;
		theta_res_right_r += dtheta_res_right_r * ST;

		/*right Robot Motion*/
		ddx_res_right = R_w*(ddtheta_res_right_l+ddtheta_res_right_r)*cos(phi_res_right)/2;
		dx_res_right += ddx_res_right*ST;
		x_res_right += dx_res_right*ST;

		ddy_res_right = R_w*(ddtheta_res_right_l+ddtheta_res_right_r)*sin(phi_res_right)/2;
		dy_res_right += ddy_res_right*ST;
		y_res_right += dy_res_right*ST;

		ddphi_res_right = R_w*(ddtheta_res_right_r-ddtheta_res_right_l)/(2*R_r);
		dphi_res_right += ddphi_res_right*ST; 
		phi_res_right += dphi_res_right*ST;

		/*Object Motion*/

		if (cos(phi_o)*1.0-sin(phi_o)*0.0<y_o+L_o){
			reaction_wall = K_o*l_wall + D_o*dl_wall; 
		}
		else{
			reaction_wall = 0.0;
		}
		ddx_o = (f_dis_bottom*cos(phi_res_bottom)+f_dis_left*cos(phi_res_left)+f_dis_right*cos(phi_res_right))/M_o - D_f*dx_o + sin(phi_o)*reaction_wall;
		dx_o += ddx_o*ST;
		x_o += dx_o*ST;
		ddy_o = (f_dis_bottom*sin(phi_res_bottom)+f_dis_left*sin(phi_res_left)+f_dis_right*sin(phi_res_right))/M_o - D_f*dy_o - cos(phi_o)*reaction_wall;
		dy_o += ddy_o*ST;
		y_o += dy_o*ST;
		ddphi_o = (f_dis_bottom*(-sin(phi_res_bottom)*(x_o-x_c_bottom)+cos(phi_res_bottom)*(y_o-y_c_bottom))
			+ f_dis_left*(-sin(phi_res_left)*(x_o-x_c_left)+cos(phi_res_left)*(y_o-y_c_left))
			+ f_dis_right*(-sin(phi_res_right)*(x_o-x_c_right)+cos(phi_res_right)*(y_o-y_c_right))
			)/J_o - D_f_rotation*dphi_o
			+ reaction_wall*(cos(phi_o)*(x_o-0.0)+sin(phi_o)*(y_o-1.0));
		dphi_o += ddphi_o*ST;
		phi_o += dphi_o*ST;

		l_bottom = fabs(cos(phi_o)*(y_c_bottom-y_o+cos(phi_o)*L_o)-sin(phi_o)*(x_c_bottom-x_o-sin(phi_o)*L_o));
		dl_bottom = (l_bottom - l_bottom_prev)/ST;
		l_bottom_prev = l_bottom;

		l_left = fabs(cos(phi_o)*(x_c_left-x_o+cos(phi_o)*L_o)+sin(phi_o)*(y_c_left-y_o+sin(phi_o)*L_o));
		dl_left = (l_left - l_left_prev)/ST;
		l_left_prev = l_left;

		l_right = fabs(cos(phi_o)*(x_c_right-x_o-cos(phi_o)*L_o)+sin(phi_o)*(y_c_right-y_o-sin(phi_o)*L_o));
		dl_right = (l_right - l_right_prev)/ST;
		l_right_prev = l_right;

		l_wall = fabs(cos(phi_o)*(1.0-y_o-cos(phi_o)*L_o)-sin(phi_o)*(0.0-x_o+sin(phi_o)*L_o));
		dl_wall = (l_wall-l_wall_prev)/ST;
		l_wall_prev = l_wall;

		/*bottom disturbance observer*/	
		tau_dob_bottom_l = integral_tau_dob_bottom_l - dtheta_res_bottom_l*J_n*GDIS;
		integral_tau_dob_bottom_l 
			+= ((K_tn*ia_bottom_l + dtheta_res_bottom_l*J_n*GDIS) - integral_tau_dob_bottom_l)*GDIS*ST;
		
		tau_dob_bottom_r = integral_tau_dob_bottom_r - dtheta_res_bottom_r*J_n*GDIS;
		integral_tau_dob_bottom_r 
			+= ((K_tn*ia_bottom_r + dtheta_res_bottom_r*J_n*GDIS) - integral_tau_dob_bottom_r)*GDIS*ST;

		/*Bottom Mobile Robot Reaction Force*/

		f_res_bottom = (tau_dob_bottom_l+tau_dob_bottom_r)/R_w;
		tau_res_bottom = (tau_dob_bottom_l-tau_dob_bottom_r)*R_r/(R_w);

		/*left disturbance observer*/	
		tau_dob_left_l = integral_tau_dob_left_l - dtheta_res_left_l*J_n*GDIS;
		integral_tau_dob_left_l 
			+= ((K_tn*ia_left_l + dtheta_res_left_l*J_n*GDIS) - integral_tau_dob_left_l)*GDIS*ST;
		
		tau_dob_left_r = integral_tau_dob_left_r - dtheta_res_left_r*J_n*GDIS;
		integral_tau_dob_left_r 
			+= ((K_tn*ia_left_r + dtheta_res_left_r*J_n*GDIS) - integral_tau_dob_left_r)*GDIS*ST;

		/*left Mobile Robot Reaction Force*/

		f_res_left = (tau_dob_left_l+tau_dob_left_r)/R_w;
		tau_res_left = (tau_dob_left_l-tau_dob_left_r)*R_r/(R_w);

		/*right disturbance observer*/	
		tau_dob_right_l = integral_tau_dob_right_l - dtheta_res_right_l*J_n*GDIS;
		integral_tau_dob_right_l 
			+= ((K_tn*ia_right_l + dtheta_res_right_l*J_n*GDIS) - integral_tau_dob_right_l)*GDIS*ST;
		
		tau_dob_right_r = integral_tau_dob_right_r - dtheta_res_right_r*J_n*GDIS;
		integral_tau_dob_right_r 
			+= ((K_tn*ia_right_r + dtheta_res_right_r*J_n*GDIS) - integral_tau_dob_right_r)*GDIS*ST;

		/*right Mobile Robot Reaction Force*/

		f_res_right = (tau_dob_right_l+tau_dob_right_r)/R_w;
		tau_res_right = (tau_dob_right_l-tau_dob_right_r)*R_r/(R_w);

		fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \n", t, x_res_bottom, y_res_bottom, phi_res_bottom, x_o, y_o, phi_o, f_dis_bottom, f_res_bottom, tau_res_bottom, x_res_left, y_res_left, phi_res_left, f_dis_left, x_res_right, y_res_right, phi_res_right, f_dis_right);
		
		t += ST;
      
	}

	fclose(fp);




	return 0;
}

