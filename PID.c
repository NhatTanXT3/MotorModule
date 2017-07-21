/*
 * PID.c
 *
 *  Created on: Jun 30, 2015
 *      Author: NhatTan
 */
#include <stdint.h>
#include "PID.h"
#include "numManipulate.h"
//PID_type PID;

void PID_controller(float feeback, PID *ptr_pid)
{
	ptr_pid->p_er=(ptr_pid->KP)+(ptr_pid->KI)*(ptr_pid->sampling_time)/(2.0)+(ptr_pid->KD)/(ptr_pid->sampling_time);
	ptr_pid->p_pre_er=-(ptr_pid->KP)+(ptr_pid->KI)*(ptr_pid->sampling_time)/(2.0)-(2.0)*(ptr_pid->KD)/(ptr_pid->sampling_time);
	ptr_pid->p_pre_pre_er=(ptr_pid->KD)/(ptr_pid->sampling_time);

	ptr_pid->er=ptr_pid->set_point-feeback;
	ptr_pid->output += ptr_pid->er*ptr_pid->p_er + ptr_pid->pre_er*ptr_pid->p_pre_er + ptr_pid->pre_pre_er*ptr_pid->p_pre_pre_er;

	ptr_pid->pre_pre_er=ptr_pid->pre_er;
	ptr_pid->pre_er=ptr_pid->er;
}
void PD_controller(float feeback, PID *ptr_pid)
{
	ptr_pid->p_er=(ptr_pid->KP)+(ptr_pid->KD)/(ptr_pid->sampling_time);
	ptr_pid->p_pre_er=-(ptr_pid->KP)-(2.0)*(ptr_pid->KD)/(ptr_pid->sampling_time);
	ptr_pid->p_pre_pre_er=(ptr_pid->KD)/(ptr_pid->sampling_time);

	ptr_pid->er=ptr_pid->set_point-feeback;
	ptr_pid->output += ptr_pid->er*ptr_pid->p_er + ptr_pid->pre_er*ptr_pid->p_pre_er + ptr_pid->pre_pre_er*ptr_pid->p_pre_pre_er;

	ptr_pid->pre_pre_er=ptr_pid->pre_er;
	ptr_pid->pre_er=ptr_pid->er;
}

void PID_type_1(float feedback, PID *ptr_pid)
{
	ptr_pid->er=ptr_pid->set_point-feedback;
	ptr_pid->P_term=ptr_pid->KP*ptr_pid->er;
	ptr_pid->I_term += ptr_pid->KI*ptr_pid->sampling_time*(ptr_pid->er+ptr_pid->pre_er)/2.0;
	ptr_pid->I_term=constrain(ptr_pid->I_term,-ptr_pid->I_limit,ptr_pid->I_limit);

	ptr_pid->D_term = ptr_pid->KD*(ptr_pid->er-ptr_pid->pre_er)/ptr_pid->sampling_time;
	ptr_pid->pre_er=ptr_pid->er;

	ptr_pid->output=ptr_pid->P_term+ptr_pid->I_term+ptr_pid->D_term;
}

void PD_type_1(float feedback, PID *ptr_pid)
{
	ptr_pid->er=ptr_pid->set_point-feedback;
	ptr_pid->P_term=ptr_pid->KP*ptr_pid->er;

	ptr_pid->D_term = ptr_pid->KD*(ptr_pid->er-ptr_pid->pre_er)/ptr_pid->sampling_time;
	ptr_pid->pre_er=ptr_pid->er;

	ptr_pid->output=ptr_pid->P_term+ptr_pid->D_term;
}

void PID_type_3(float feedback, PID *ptr_pid)
{
	ptr_pid->er=ptr_pid->set_point-feedback;
	ptr_pid->fb=feedback;

	ptr_pid->P_term=ptr_pid->KP*ptr_pid->er;
	ptr_pid->I_term += ptr_pid->KI*ptr_pid->sampling_time*(ptr_pid->er+ptr_pid->pre_er)/2.0;
	ptr_pid->I_term=constrain(ptr_pid->I_term,-ptr_pid->I_limit,ptr_pid->I_limit);

	ptr_pid->D_term = ptr_pid->KD*(-ptr_pid->fb + ptr_pid->pre_fb)/ptr_pid->sampling_time;
	ptr_pid->pre_er=ptr_pid->er;
	ptr_pid->pre_fb=ptr_pid->fb;

	ptr_pid->output=ptr_pid->P_term+ptr_pid->I_term+ptr_pid->D_term;
}

void PD_type_3(float feedback, PID *ptr_pid)
{
	ptr_pid->er=ptr_pid->set_point-feedback;
	ptr_pid->fb=feedback;

	ptr_pid->P_term=ptr_pid->KP*ptr_pid->er;

	ptr_pid->D_term = ptr_pid->KD*(-ptr_pid->fb +ptr_pid->pre_fb)/ptr_pid->sampling_time;
	ptr_pid->pre_er=ptr_pid->er;

	ptr_pid->pre_fb=ptr_pid->fb;

	ptr_pid->output=ptr_pid->P_term+ptr_pid->D_term;
}


void PID_type_4(float feedback,PID *ptr_pid,float filter)
{
	ptr_pid->er=ptr_pid->set_point-feedback;
	ptr_pid->fb=feedback;
	ptr_pid->fb_filter=ptr_pid->fb_filter*filter+ptr_pid->fb*(1-filter);

	ptr_pid->P_term=ptr_pid->KP*ptr_pid->er;
	ptr_pid->I_term += ptr_pid->KI*ptr_pid->sampling_time*(ptr_pid->er+ptr_pid->pre_er)/2.0;
	ptr_pid->I_term=constrain(ptr_pid->I_term,-ptr_pid->I_limit,ptr_pid->I_limit);

	ptr_pid->D_term = ptr_pid->KD*(-ptr_pid->fb_filter + ptr_pid->pre_fb_filter)/ptr_pid->sampling_time;
	ptr_pid->D_term=constrain(ptr_pid->D_term,-ptr_pid->D_limit,ptr_pid->D_limit);
	ptr_pid->pre_er=ptr_pid->er;
	ptr_pid->pre_fb_filter=ptr_pid->fb_filter;
	ptr_pid->pre_fb=ptr_pid->fb;

	ptr_pid->output=ptr_pid->P_term+ptr_pid->I_term+ptr_pid->D_term;
}

void PD_type_4(float feedback, PID *ptr_pid,float filter)
{
	ptr_pid->er=ptr_pid->set_point-feedback;
	ptr_pid->fb=feedback;
	ptr_pid->fb_filter=ptr_pid->fb_filter*filter+ptr_pid->fb*(1-filter);

	ptr_pid->P_term=ptr_pid->KP*ptr_pid->er;

	ptr_pid->D_term = ptr_pid->KD*(-ptr_pid->fb_filter +ptr_pid->pre_fb_filter)/ptr_pid->sampling_time;
	ptr_pid->D_term=constrain(ptr_pid->D_term,-ptr_pid->D_limit,ptr_pid->D_limit);
	ptr_pid->pre_er=ptr_pid->er;

	ptr_pid->pre_fb_filter=ptr_pid->fb_filter;

	ptr_pid->output=ptr_pid->P_term+ptr_pid->D_term;
}
