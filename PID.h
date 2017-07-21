/*
 * PID.h
 *
 *  Created on: Jun 30, 2015
 *      Author: NhatTan
 */

#ifndef QUADROTOR_CURRENT_WORK_PID_H_
#define QUADROTOR_CURRENT_WORK_PID_H_


typedef struct{
	float set_point;
	float KP;
	float KD;
	float KI;
	float output;
	float er;
	float pre_er;
	float pre_pre_er;
	float p_er;
	float p_pre_er;
	float p_pre_pre_er;
	float sampling_time;

	float P_term;
	float I_term;
	float D_term;
	float D_limit;
	float I_limit;

	float fb;//feedback
	float fb_filter;
	float pre_fb_filter;
	float pre_fb;
	float pre_pre_fb;
}PID;

extern void PID_controller(float feeback, PID *ptr_pid);
extern void PD_controller(float feeback, PID *ptr_pid);

extern void PID_type_1(float feedback, PID *ptr_pid);
extern void PD_type_1(float feedback, PID *ptr_pid);

extern void PID_type_3(float feedback, PID *ptr_pid);
extern void PD_type_3(float feedback, PID *ptr_pid);

extern void PID_type_4(float feedback,PID *ptr_pid,float filter);
extern void PD_type_4(float feedback, PID *ptr_pid,float filter);



#endif /* QUADROTOR_CURRENT_WORK_PID_H_ */
