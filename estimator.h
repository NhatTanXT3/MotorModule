/*
 * estimator.h
 *
 *  Created on: Oct 5, 2016
 *      Author: Nhat Tan
 */

#ifndef ESTIMATOR_H_
#define ESTIMATOR_H_

#define M_3300mAh_		1.374 //kg
#define M_4200mAh_		1.361 //kg
#define M_QUADCOPTER	M_3300mAh_

typedef struct{
	float K_L_omega;
	float a_omega;
	float b_omega;
	float alpha_filter;
	float M;

	float omega_static[4];
	float omega_dynamic[4];
	float Tz3;
	float z1_dot_dot;
	float z1_dot;
	float z1;
	float g;
}model_type;
extern model_type quad_model;
extern void model_init();
extern void update_omega(float output_1,float output_2,float output_3,float output_4);

#endif /* ESTIMATOR_H_ */
