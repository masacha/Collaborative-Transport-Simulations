#include <stdio.h>
#include <math.h>

#define	T	10 //seconds
#define ST	0.0001 //second
#define GDIS	500.0
#define K_p	10.0
#define K_v	10.0
#define M_c	1.0
#define K_f	10.0
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

int main(void)
{
	double t = 0.0;

/*Reference Commands*/

	double x_command = 0.0; //Ne pas modifier
	double dx_command = 0.0; //Ne pas modifier
	double f_command = 0.0; //Ne pas modifier

/*Mobile Robot Outputs*/

	double x_res_push = -0.2;
	double dx_res_push = 0.0;
	double ddx_res_push = 0.0;
	double phi_res_push = 0.0;
	double dphi_res_push = 0.0;
	double ddphi_res_push = 0.0;

	double x_c_push = 0.0;
	double y_c_push = 0.0;
	
	double l_push = 0.0;
	double dl_push = 0.0;
	double l_push_prev = 0.0;

	double err_x_push = 0.0;
	double derr_x_push = 0.0;

	double f_res_push = 0.0;
	double tau_res_push = 0.0;

/*Mobile Robot Inputs*/
	
	double ddx_comp_push = 0.0;
	double ddy_comp_push = 0.0;

	double ddx_ref_push = 0.0;
	double ddy_ref_push = 0.0;

	double f_dis_push = 0.0;

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

/*Motor*/

	double ddtheta_ref_push_l = 0.0;
	double ddtheta_ref_push_r = 0.0;
	
	double ia_ref_push_l = 0.0;
	double ia_ref_push_r = 0.0;

	double ia_push_l = 0.0;
	double ia_push_r = 0.0;

	double tau_dis_push_l = 0.0;
	double tau_dis_push_r = 0.0;

	double tau_dob_push_l = 0.0;
	double tau_dob_push_r = 0.0;

	double integral_tau_dob_push_l = 0.0;
	double integral_tau_dob_push_r = 0.0;

	double tau_rtob_push_l = 0.0;
	double tau_rtob_push_r = 0.0;

	double integral_tau_rtob_push_l = 0.0;
	double integral_tau_rtob_push_r = 0.0;

	double theta_res_push_l = 0.0;
	double theta_res_push_r = 0.0;

	double dtheta_res_push_l = 0.0;
	double dtheta_res_push_r = 0.0;

	double ddtheta_res_push_l = 0.0;
	double ddtheta_res_push_r = 0.0;

	l_push_prev = abs(-sin(phi_o)*x_c_push+cos(phi_o)*y_c_push - (y_o-L_o));

	FILE *fp;

	if((fp=fopen("collaborative_transport_simulation.dat", "w"))==NULL){
		fprintf(stderr, "File open failed.\n");
		return 1;
	}

	while(t <= T){	

		x_command = 2.0;
		dx_command = 0;
		f_command = 5.0;
		
		x_c_push = cos(phi_o)*x_res_push-sin(phi_o)*(y_res_push+R_r);
		y_c_push = sin(phi_o)*x_res_push+cos(phi_o)*(y_res_push+R_r);

		if(cos(phi_o)*y_c_push-sin(phi_o)*x_c_push>y_o-L_o){
			f_dis_push = K_o*l_push + D_o*dl_push; 
			tau_dis_push_l = f_dis_push*R_w*(1-sin(phi_res_push-phi_o))/2;
			tau_dis_push_r = f_dis_push*R_w*(1+sin(phi_res_push-phi_o))/2;
		}
		else{
			f_dis_push = 0.0;
			tau_dis_push_l = 0.0;
			tau_dis_push_r = 0.0;
		}
		
		/*Mobile Robot Controller*/

		err_x_push = x_command - L_o - x_res_push;
		derr_x_push = dx_command - dx_res_push;

		err_y_push = y_command - L_o - y_res_push;
		derr_y_push = dy_command - dy_res_push;

		ddx_comp_push = (K_p*err_x_push+K_v*derr_x_push)/M_c;
		ddy_comp_push = (K_p*err_y_push+K_v*derr_y_push)/M_c;

		ddx_ref_push = ddx_comp_push;
		ddy_ref_push = ddy_comp_push;

		ddphi_ref_push = 0.0;

		/*Motor Controller*/

		ddtheta_ref_push_l = (ddx_ref_push*cos(phi_res_push)+ddy_ref_push*sin(phi_res_push)-R_r*ddphi_ref_push)/R_w;
		ddtheta_ref_push_r = (ddx_ref_push*cos(phi_res_push)+ddy_ref_push*sin(phi_res_push)+R_r*ddphi_ref_push)/R_w;

		ia_ref_push_l = J_n*ddtheta_ref_push_l/K_tn;
		ia_ref_push_r = J_n*ddtheta_ref_push_r/K_tn;

		ia_push_l = ia_ref_push_l + tau_dob_push_l/K_tn;
		ia_push_r = ia_ref_push_r + tau_dob_push_r/K_tn;

		ddtheta_res_push_l = (K_tn*ia_push_l - tau_dis_push_l)/J_n;
		ddtheta_res_push_r = (K_tn*ia_push_r - tau_dis_push_r)/J_n;

		dtheta_res_push_l += ddtheta_res_push_l * ST;
		dtheta_res_push_r += ddtheta_res_push_r * ST;

		theta_res_push_l += dtheta_res_push_l * ST;
		theta_res_push_r += dtheta_res_push_r * ST;

		/*Robot Motion*/
		ddx_res_push = -R_w*(ddtheta_res_push_l+ddtheta_res_push_r)*sin(phi_res_push)/2;
		dx_res_push += ddx_res_push*ST;
		x_res_push += dx_res_push*ST;

		ddy_res_push = R_w*(ddtheta_res_push_l+ddtheta_res_push_r)*cos(phi_res_push)/2;
		dy_res_push += ddy_res_push*ST;
		y_res_push += dy_res_push*ST;

		ddphi_res_push = R_w*(ddtheta_res_push_r-ddtheta_res_push_l)/(2*R_r);
		dphi_res_push += ddphi_res_push*ST; 
		phi_res_push += dphi_res_push*ST;

		/*Object Motion*/

		ddx_o = -(f_dis_push)*sin(phi_r)/M_o;
		dx_o += ddx_o*ST;
		x_o += dx_o*ST;
		ddy_o = (f_dis_push)*cos(phi_r)/M_o;
		dy_o += ddy_o*ST;
		y_o += dy_o*ST;
		ddphi_o = -F_dis_push*(cos(phi_res_push)*(x_o-x_c_push)+sin(phi_res_push)*(y_o-y_c_push))/J_o;
		dphi_o += ddphi_o*ST;
		phi_o += dphi_o*ST;

		l_push = abs(-sin(phi_o)*x_c_push+cos(phi_o)*y_c_push - (y_o-L_o));
		dl_push = (l_push - l_push_prev)/ST;
		l_push_prev = l_push;

		/*disturbance observer*/		
		tau_dob_push_l = integral_tau_dob_push_l - dtheta_res_push_l*J_n*GDIS;
		integral_tau_dob_push_l 
			+= ((K_tn*ia_push_l + dtheta_res_push_l*J_n*GDIS) - integral_tau_dob_push_l)*GDIS*ST;
		
		tau_dob_push_r = integral_tau_dob_push_r - dtheta_res_push_r*J_n*GDIS;
		integral_tau_dob_push_r 
			+= ((K_tn*ia_push_r + dtheta_res_push_r*J_n*GDIS) - integral_tau_dob_push_r)*GDIS*ST;

		/*Mobile Robot Reaction Force*/

		f_res_push = (tau_dob_push_l+tau_dob_push_r)/R_w;
		tau_res_push = (tau_dob_push_l-tau_dob_push_r)*R_r/(R_w);

		fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", t, x_res_push, y_res_push, phi_res_push, x_o, y_o, phi_o, f_dis_push, f_res_push, tau_res_push);
		
		t += ST;
      
	}

	fclose(fp);




	return 0;
}

