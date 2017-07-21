/*
 * estimator.c
 *
 *  Created on: Oct 5, 2016
 *      Author: Nhat Tan
 */
#include "estimator.h"
model_type quad_model;


void model_init()
{
	quad_model.K_L_omega=0.000009;// 9*10^-6;
	quad_model.a_omega=0.8066;
	quad_model.b_omega=36.5053;
	quad_model.alpha_filter=0.9375;
	quad_model.M=M_QUADCOPTER;
	quad_model.g=11.2;
}

void update_omega(float output_1,float output_2,float output_3,float output_4){
	quad_model.omega_static[0]=quad_model.a_omega*output_1+quad_model.b_omega;
	quad_model.omega_static[1]=quad_model.a_omega*output_2+quad_model.b_omega;
	quad_model.omega_static[2]=quad_model.a_omega*output_3+quad_model.b_omega;
	quad_model.omega_static[3]=quad_model.a_omega*output_4+quad_model.b_omega;

	unsigned char i;
	for(i=0;i<4;i++)
	quad_model.omega_dynamic[i]=quad_model.omega_dynamic[i]*quad_model.alpha_filter+(1-quad_model.alpha_filter)*quad_model.omega_static[i];

	quad_model.Tz3=quad_model.K_L_omega*(quad_model.omega_dynamic[0]*quad_model.omega_dynamic[0]+quad_model.omega_dynamic[1]*quad_model.omega_dynamic[1]+quad_model.omega_dynamic[2]*quad_model.omega_dynamic[2]+quad_model.omega_dynamic[3]*quad_model.omega_dynamic[3]);
	quad_model.z1_dot_dot=quad_model.Tz3/quad_model.M-quad_model.g;
}



