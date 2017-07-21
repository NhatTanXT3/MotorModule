/*
 * PPM.h
 *
 *  Created on: May 21, 2015
 *      Author: NhatTan
 */

#ifndef PPM_H_
#define PPM_H_
//#include "config_hardware.h"

#define QUADCOPTER_V_1
//#define QUADCOPTER_V_0


#define ui32Period 100000

#ifdef QUADCOPTER_V_1
#define ESC_1_MIN  	39000//38570//34717//36600
#define ESC_1_MAX  	72781//55670//56018

#define ESC_2_MIN 	39000//38570//33076//33077
#define ESC_2_MAX 	72781//67997//68475

#define ESC_3_MIN	39000//38570//31300//33191
#define ESC_3_MAX	72781//70586//70228

#define ESC_4_MIN	39000//38570//33592//35014
#define ESC_4_MAX	72781//70258//71378

#else

#ifdef QUADCOPTER_V_0
#define ESC_1_MIN  	34717//36600
#define ESC_1_MAX  	55670//56018

#define ESC_2_MIN 	33076//33077
#define ESC_2_MAX 	67997//68475

#define ESC_3_MIN	31300//33191
#define ESC_3_MAX	70586//70228

#define ESC_4_MIN	33592//35014
#define ESC_4_MAX	70258//71378
#endif

#endif


#define ESC_STOP	35000

typedef struct {
	int32_t ppm_1,ppm_2,ppm_3,ppm_4;
} ESCtype;

extern ESCtype ESC;


extern void PPM_init();
extern void Motor_stop();
extern void ESC_ppm(unsigned char Esc_num, int32_t ppm);


#endif /* PPM_H_ */
