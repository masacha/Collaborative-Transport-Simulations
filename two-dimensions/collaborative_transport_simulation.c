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
#define L_o	0.100 //meter
#define R_w	0.030 //meter
#define W_r	0.15 //meter
#define K_tn	1.7   //Nm
#define J_n	0.1 //Ns^2/m
#define Pi	3.1415

int main(void)
{
	double t = 0.0;

/*Reference Commands*/

	double x_command = 0.0; //Modifier plus bas !
	double dx_command = 0.0; //Modifier plus bas !
	double f_command = 0.0; //Modifier plus bas !

/*Mobile Robot Outputs*/

	double x_res_push = -0.15;
	double x_res_pull = 0.15;
	double dx_res_push = 0.0;
	double dx_res_pull = 0.0;
	double ddx_res_push = 0.0;
	double ddx_res_pull = 0.0;

	double err_x_push = 0.0;
	double err_x_pull = 0.0;
	double derr_x_push = 0.0;
	double derr_x_pull = 0.0;

	double f_res_push = 0.0;
	double f_res_pull = 0.0;

/*Mobile Robot Inputs*/
	
	double ddx_comp_push = 0.0;
	double ddx_comp_pull = 0.0;

	double ddx_ref_push = 0.0;
	double ddx_ref_pull = 0.0;

	double f_dis_push = 0.0;
	double f_dis_pull = 0.0;


/*Object*/
	double x_o = 0.0;
	double dx_o = 0.0;
	double ddx_o = 0.0;
	double y_o = 0.0;
	double dy_o = 0.0;
	double ddy_o = 0.0;
	double theta_o = 0.0;
	double dtheta_o = 0.0;
	double ddtheta_o = 0.0;

/*Motor*/

	double ddtheta_ref_push_l = 0.0;
	double ddtheta_ref_push_r = 0.0;
	double ddtheta_ref_pull_l = 0.0;
	double ddtheta_ref_pull_r = 0.0;
	
	double ia_ref_push_l = 0.0;
	double ia_ref_push_r = 0.0;
	double ia_ref_pull_l = 0.0;
	double ia_ref_pull_r = 0.0;

	double ia_push_l = 0.0;
	double ia_push_r = 0.0;
	double ia_pull_l = 0.0;
	double ia_pull_r = 0.0;

	double tau_dis_push_l = 0.0;
	double tau_dis_push_r = 0.0;
	double tau_dis_pull_l = 0.0;
	double tau_dis_pull_r = 0.0;

	double tau_dob_push_l = 0.0;
	double tau_dob_push_r = 0.0;
	double tau_dob_pull_l = 0.0;
	double tau_dob_pull_r = 0.0;

	double integral_tau_dob_push_l = 0.0;
	double integral_tau_dob_push_r = 0.0;
	double integral_tau_dob_pull_l = 0.0;
	double integral_tau_dob_pull_r = 0.0;

	double tau_rtob_push_l = 0.0;
	double tau_rtob_push_r = 0.0;
	double tau_rtob_pull_l = 0.0;
	double tau_rtob_pull_r = 0.0;

	double integral_tau_rtob_push_l = 0.0;
	double integral_tau_rtob_push_r = 0.0;
	double integral_tau_rtob_pull_l = 0.0;
	double integral_tau_rtob_pull_r = 0.0;

	double theta_res_push_l = 0.0;
	double theta_res_push_r = 0.0;
	double theta_res_pull_l = 0.0;
	double theta_res_pull_r = 0.0;

	double dtheta_res_push_l = 0.0;
	double dtheta_res_push_r = 0.0;
	double dtheta_res_pull_l = 0.0;
	double dtheta_res_pull_r = 0.0;

	double ddtheta_res_push_l = 0.0;
	double ddtheta_res_push_r = 0.0;
	double ddtheta_res_pull_l = 0.0;
	double ddtheta_res_pull_r = 0.0;

	FILE *fp;

	if((fp=fopen("collaborative_transport_simulation.dat", "w"))==NULL){
		fprintf(stderr, "File open failed.\n");
		return 1;
	}

	while(t <= T){	

		x_command = 2.0;
		dx_command = 0;
		f_command = 5.0;
		
		/*Changer f_dis pour ajouter l'orientation*/
		
		if(x_res_push>=x_o-L_o){
			f_dis_push = K_o*(x_res_push-(x_o-L_o)) + D_o*(dx_res_push-dx_o); //A modifier par une sorte d'integrale
			tau_dis_push_l = f_dis_push*R_w/2; //pas aussi simple
			tau_dis_push_r = f_dis_push*R_w/2;
		}
		else{
			f_dis_push = 0.0; //pas aussi simple malheureusement
			tau_dis_push_l = 0.0;
			tau_dis_push_r = 0.0;
		}
		if(x_res_pull<=x_o+L_o){
			f_dis_pull = K_o*(x_res_pull-(x_o+L_o)) + D_o*(dx_res_pull-dx_o); //a modifier par une sorte d'integrale
			tau_dis_pull_l = f_dis_pull*R_w/2;
			tau_dis_pull_r =  f_dis_pull*R_w/2;
		}
		else{
			f_dis_pull = 0.0;
			tau_dis_pull_l = 0.0;
			tau_dis_pull_r = 0.0;
		}
		
		/*Mobile Robot Controller*/

		//Quelle force ??
		err_x_push = x_command - L_o - x_res_push;
		err_x_pull = x_command + L_o - x_res_pull;
		derr_x_push = dx_command - dx_res_push;
		derr_x_pull = dx_command - dx_res_pull;

		ddx_comp_push = (K_p*err_x_push+K_v*derr_x_push)/M_c;
		ddx_comp_pull = (K_p*err_x_pull+K_v*derr_x_pull)/M_c;

		ddx_ref_push = K_f*(f_command-f_dis_push)/M_o + ddx_comp_push;
		ddx_ref_pull = ddx_comp_pull;

		/*Motor Controller*/

		ddtheta_ref_push_l = (ddx_ref_push*cos(phi_res_push)+ddy_ref_push*sin(phi_res_push)+W_r*ddphi_ref_push/2)/R_w;
		ddtheta_ref_push_r = (ddx_ref_push*cos(phi_res_push)+ddy_ref_push*sin(phi_res_push)-W_r*ddphi_ref_push/2)/R_w;
		ddtheta_ref_pull_l = (ddx_ref_pull*cos(phi_res_pull)+ddy_ref_pull*sin(phi_res_pull)+W_r*ddphi_ref_pull/2)/R_w;
		ddtheta_ref_pull_r = (ddx_ref_pull*cos(phi_res_pull)+ddy_ref_pull*sin(phi_res_pull)-W_r*ddphi_ref_pull/2)/R_w;

		ia_ref_push_l = J_n*ddtheta_ref_push_l/K_tn;
		ia_ref_push_r = J_n*ddtheta_ref_push_r/K_tn;
		ia_ref_pull_l = J_n*ddtheta_ref_pull_l/K_tn;
		ia_ref_pull_r = J_n*ddtheta_ref_pull_r/K_tn;

		ia_push_l = ia_ref_push_l + tau_dob_push_l/K_tn;
		ia_push_r = ia_ref_push_r + tau_dob_push_r/K_tn;
		ia_pull_l = ia_ref_pull_l + tau_dob_pull_l/K_tn;
		ia_pull_r = ia_ref_pull_r + tau_dob_pull_r/K_tn;

		ddtheta_res_push_l = (K_tn*ia_push_l - tau_dis_push_l)/J_n;
		ddtheta_res_push_r = (K_tn*ia_push_r - tau_dis_push_r)/J_n;
		ddtheta_res_pull_l = (K_tn*ia_pull_l - tau_dis_pull_l)/J_n;
		ddtheta_res_pull_r = (K_tn*ia_pull_r - tau_dis_pull_r)/J_n;

		dtheta_res_push_l += ddtheta_res_push_l * ST;
		dtheta_res_push_r += ddtheta_res_push_r * ST;
		dtheta_res_pull_l += ddtheta_res_pull_l * ST;
		dtheta_res_pull_r += ddtheta_res_pull_r * ST;

		theta_res_push_l += dtheta_res_push_l * ST;
		theta_res_push_r += dtheta_res_push_r * ST;
		theta_res_pull_l += dtheta_res_pull_l * ST;
		theta_res_pull_r += dtheta_res_pull_r * ST;

		/*Robot Motion*/
		ddx_res_push = R_w*(ddtheta_res_push_l+ddtheta_res_push_r)*cos(phi_res_push)/2;
		ddx_res_pull = R_w*(ddtheta_res_pull_l+ddtheta_res_pull_r)*cos(phi_res_pull)/2;
		dx_res_push += ddx_res_push*ST;
		dx_res_pull += ddx_res_pull*ST;
		x_res_push += dx_res_push*ST;
		x_res_pull += dx_res_pull*ST;

		ddy_res_push = R_w*(ddtheta_res_push_l+ddtheta_res_push_r)*sin(phi_res_push)/2;
		ddy_res_pull = R_w*(ddtheta_res_pull_l+ddtheta_res_pull_r)*sin(phi_res_pull)/2;
		dy_res_push += ddy_res_push*ST;
		dy_res_pull += ddy_res_pull*ST;
		y_res_push += dy_res_push*ST;
		y_res_pull += dy_res_pull*ST;

		ddphi_res_push = R_w*(ddtheta_res_push_l-ddtheta_res_push_r)/W_r;
		ddphi_res_pull = R_w*(ddtheta_res_pull_l-ddtheta_res_pull_r)/W_r;
		dphi_res_push += ddphi_res_push*ST; 
		dphi_res_pull += ddphi_res_pull*ST;
		phi_res_push += dphi_res_push*ST;
		phi_res_pull += dphi_res_pull*ST;

		/*Object Motion*/

		ddx_o = (f_dis_push+f_dis_pull)*cos(theta_o)/M_o; //A verifier!! (f_dis est la sorte d'integrale)
		dx_o += ddx_o*ST;
		x_o += dx_o*ST;
		ddy_o = (f_dis_push+f_dis_pull)*sin(theta_o)/M_o; //A verifier!!
		dy_o += ddy_o*ST;
		y_o += dy_o*ST;
		ddphi_o = /J_o; //moment des forces divise par inertie (il faut encore faire la sorte d'integrale)
		dphi_o += ddphi_o*ST;
		phi_o += dphi_o*ST;

		/*disturbance observer*/		
		tau_dob_push_l = integral_tau_dob_push_l - dtheta_res_push_l*J_n*GDIS;
		integral_tau_dob_push_l 
			+= ((K_tn*ia_push_l + dtheta_res_push_l*J_n*GDIS) - integral_tau_dob_push_l)*GDIS*ST;
		
		tau_dob_push_r = integral_tau_dob_push_r - dtheta_res_push_r*J_n*GDIS;
		integral_tau_dob_push_r 
			+= ((K_tn*ia_push_r + dtheta_res_push_r*J_n*GDIS) - integral_tau_dob_push_r)*GDIS*ST;

		tau_dob_pull_l = integral_tau_dob_pull_l - dtheta_res_pull_l*J_n*GDIS;
		integral_tau_dob_pull_l 
			+= ((K_tn*ia_pull_l + dtheta_res_pull_l*J_n*GDIS) - integral_tau_dob_pull_l)*GDIS*ST;
		
		tau_dob_pull_r = integral_tau_dob_pull_r - dtheta_res_pull_r*J_n*GDIS;
		integral_tau_dob_pull_r 
			+= ((K_tn*ia_pull_r + dtheta_res_pull_r*J_n*GDIS) - integral_tau_dob_pull_r)*GDIS*ST;

		/*Mobile Robot Reaction Force*/

		f_res_push = (tau_dob_push_l+tau_dob_push_r)/R_w;
		f_res_pull = (tau_dob_pull_l+tau_dob_pull_r)/R_w;
		tau_res_push = (tau_dob_push_l-tau_dob_push_r)*W_r/(2*R_w);
		tau_res_pull = (tau_dob_pull_l-tau_dob_pull_r)*W_r/(2*R_w);

		fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf\n", t, x_res_push, x_res_pull, x_o, f_dis_push, f_dis_pull, f_res_push, f_res_pull);
		
		t += ST;
      
	}

	fclose(fp);




	return 0;
}

