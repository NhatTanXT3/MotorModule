/*
 * Sonar.h
 *
 *  Created on: Sep 30, 2016
 *      Author: Nhat Tan
 */

#ifndef SONAR_H_
#define SONAR_H_

#define FRS_SONAR
#ifdef FRS_SONAR
#define SONAR_MAX_PULSE_WIDTH_ 10000//2000mm
#define SONAR_MIN_PULSE_WIDTH_ 200//
#define SONAR_FILTER_FACTOR 0.8
#define SONAR_SCALE_FACTOR 0.2
#define SONAR_PULSE_WIDTH_OFFSET_ 200
#define SONAR_NOISE_RANGE 25

#define SONAR_MAX_D_PULSE_WIDTH 600

#endif


#ifdef  USE_SONAR_MB1000_EZ0
#define SONAR_MAX_PULSE_WIDTH_ 11575//2000mm
#define SONAR_MIN_PULSE_WIDTH_ 800//
#define SONAR_PULSE_WIDTH_OFFSET_ 200//925//830
#define SONAR_FILTER_FACTOR 0.8 //low pass filter
#define SONAR_NOISE_RANGE 3

#ifdef QUADCOPTER_V_1
#define SONAR_PULSE_WIDTH_OFFSET_ 830//925//830
//#define SONAR_MAX_ATTITUDE_	2000//1000//691//1382 //8000*scale_factor
//#define SONAR_MIN_ATTITUDE_	143 //830*scale_factor

#define SONAR_FILTER_FACTOR 0.8 //low pass filter
#else
#ifdef QUADCOPTER_V_0
#define SONAR_PULSE_WIDTH_OFFSET_ 830
//#define SONAR_MAX_ATTITUDE_	2000//1000//691//1382 //8000*scale_factor
//#define SONAR_MIN_ATTITUDE_	143 //830*scale_factor

#define SONAR_FILTER_FACTOR 0.7 //low pass filter
#endif
#endif

#define SONAR_SCALE_FACTOR 0.172789//convert from microsecond to mimetter. read from datasheet

#endif

#ifdef USE_SONAR_MB1240_EZ4
#define SONAR_MAX_PULSE_WIDTH_ 11600//2000mm
#define SONAR_MIN_PULSE_WIDTH_ 900//172mm

//#define SONAR_MAX_ATTITUDE_	2000//1000//691//1382 //8000*scale_factor
//#define SONAR_MIN_ATTITUDE_	143 //830*scale_factor

#define SONAR_FILTER_FACTOR 0.8 //low pass filter
#define SONAR_SCALE_FACTOR 0.172414//convert from microsecond to mimetter. read from datasheet

#endif

typedef volatile struct{
	uint32_t rise_time;
	uint32_t pulse_width;
	uint32_t pre_pulse_width;
	int32_t d_pulse_width;

	unsigned char flag_update;
	unsigned char fail_signal_count;
	float distance;
	float distance_1;
	float distance_raw;
	float pre_distance_raw;
	float attitude;
	float pulse_width_offset;

	float z1;
	float z1_filter;
	float z1_dot;
	float z1_dot_measure;
	float z1_dot_estimate;
	float z1_dot_filter;
	float z1_dot_predict;
	float z1_dot_dot;
	float z1_error;

	unsigned char divergence_error_count;
	unsigned char flag_divergence_error;

}Sonar_type;
extern Sonar_type Sonar_module;


extern void Sonar_module_init();
extern unsigned char Calib_Sonar_module();
//float Get_MaxSonar_distance();
extern void Timer3_init();


#endif /* SONAR_H_ */
