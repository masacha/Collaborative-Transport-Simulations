#include <stdio.h>
#include <math.h>



int main(void)
{
	double t = 0.0;

/*Reference Commands*/

	double x_command = 0.0;
	double dx_command = 0.0;
	double f_command = 0.0;

/*Mobile Robot Outputs*/

	double x_res_push = 0.0;
	double x_res_pull = 0.0;
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

	double tau_dis_push_l = 0.0;
	double tau_dis_push_r = 0.0;
	double tau_dis_pull_l = 0.0;
	double tau_dis_pull_r = 0.0;

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

		x_command = 0.0;
		dx_command = 0.0;
		f_command = 3.0;

		tau_dis_push_l = ;
		tau_dis_push_r = ;
		tau_dis_pull_l = ;
		tau_dis_pull_r = ;
		
		/*Mobile Robot Controller*/
		err_x_push = x_command - x_res_push;
		err_x_push = x_command - x_res_pull;
		derr_x_push = dx_command - dx_res_push;
		derr_x_pull = dx_command - dx_res_pull;

		ddx_comp_push = (K_c*err_x_push+D_c*derr_x_push)/M_c;
		ddx_comp_pull = (K_c*err_x_pull+D_c*derr_x_pull)/M_c;

		ddx_ref_push = K_f*(f_command-f_res_push)/M_n+ddx_comp_push;
		ddx_ref_pull = K_f*(-fcommand-f_res_pull)/M_n+ddx_comp_pull;

		/*Motor Controller*/

		ddtheta_ref_push_l = ddx_ref_push/R_w;
		ddtheta_ref_push_r = ddx_ref_push/R_w;
		ddtheta_ref_pull_l = ddx_ref_pull/R_w;
		ddtheta_ref_pull_r = ddx_ref_pull/R_w;

		ia_ref_push_l = Jn_push_l*ddtheta_ref_push_l/K_tn;
		ia_ref_push_r = Jn_push_r*ddtheta_ref_push_r/K_tn;
		ia_ref_pull_l = Jn_pull_l*ddtheta_ref_pull_l/K_tn;
		ia_ref_pull_r = Jn_pull_r*ddtheta_ref_pull_r/K_tn;

		ia_push_l = ia_ref_push_l + tau_dob_push_l/K_tn;
		ia_push_r = ia_ref_push_r + tau_dob_push_r/K_tn;
		ia_pull_l = ia_ref_pull_l + tau_dob_pull_l/K_tn;
		ia_pull_r = ia_ref_pull_r + tau_dob_pull_r/K_tn;

		ddtheta_res_push_l = (K_tn*ia_push_l - tau_dis_push_l)/Jn_push_l;
		ddtheta_res_push_r = (K_tn*ia_push_r - tau_dis_push_r)/Jn_push_r;
		ddtheta_res_pull_l = (K_tn*ia_pull_l - tau_dis_pull_l)/Jn_pull_l;
		ddtheta_res_pull_r = (K_tn*ia_pull_r - tau_dis_pull_r)/Jn_pull_r;

		dtheta_res_push_l += ddtheta_res_push_l * ST;
		dtheta_res_push_r += ddtheta_res_push_r * ST;
		dtheta_res_pull_l += ddtheta_res_pull_l * ST;
		dtheta_res_pull_r += ddtheta_res_pull_r * ST;

		theta_res_push_l += dtheta_res_push_l * ST;
		theta_res_push_r += dtheta_res_push_r * ST;
		theta_res_pull_l += dtheta_res_pull_l * ST;
		theta_res_pull_r += dtheta_res_pull_r * ST;

		/*disturbance observer*/		
		tau_dob_push_l = integral_dob_push_l - dtheta_res_push_l*Jn_push_l*GDIS;
		integral_dob_push_l 
			+= ((K_tn*ia_push_l + dtheta_res_push_l*Jn_push_l*GDIS) - integral_dob_push_l)*GDIS*ST;
		
		tau_dob_push_r = integral_dob_push_r - dtheta_res_push_r*Jn_push_r*GDIS;
		integral_dob_push_r 
			+= ((K_tn*ia_push_r + dtheta_res_push_r*Jn_push_r*GDIS) - integral_dob_push_r)*GDIS*ST;

		tau_dob_pull_l = integral_dob_pull_l - dtheta_res_pull_l*Jn_pull_l*GDIS;
		integral_dob_pull_l 
			+= ((K_tn*ia_pull_l + dtheta_res_pull_l*Jn_pull_l*GDIS) - integral_dob_pull_l)*GDIS*ST;
		
		tau_dob_pull_r = integral_dob_pull_r - dtheta_res_pull_r*Jn_pull_r*GDIS;
		integral_dob_pull_r 
			+= ((K_tn*ia_pull_r + dtheta_res_pull_r*Jn_pull_r*GDIS) - integral_dob_pull_r)*GDIS*ST;

		/*Mobile Robot Reaction Force*/

		f_res_push = R_w*(ddtheta_res_push_l+ddtheta_res_push_r)/2 + (tau_dob_push_l+tau_dob_push_r)/R_w; /*Ajouter des termes de friction (sol, air)*/
		f_res_pull = R_w*(ddtheta_res_pull_l+ddtheta_res_pull_r)/2 + (tau_dob_pull_l+tau_dob_pull_r)/R_w;

		fprintf(fp, "%lf %lf %lf %lf %lf\n", t, x_res_push, x_res_pull, f_res_push, f_res_pull);
		
		t += ST;
      
	}

	fclose(fp);




	return 0;
}

