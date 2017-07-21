/*
 * RF.h
 *
 *  Created on: Jun 23, 2015
 *      Author: NhatTan
 */


#ifndef QUADROTOR_CURRENT_WORK_RF_H_
#define QUADROTOR_CURRENT_WORK_RF_H_

#define RF_FILTER_FACTOR_STICK 0.7
#define RF_FILTER_FACTOR_VOLUME 0.95

#define C1_MAX	1888
#define C1_MIN	1057
#define C1_ZERO 1478

#define C2_MAX	1868
#define C2_MIN	1046
#define C2_ZERO	1465

#define C3_MAX 1879//limit 1748
#define C3_MIN 1100
#define C3_LIMIT_STICK_TOP 		1700
#define C3_LIMIT_STICK_BOTTOM	1200


#define C4_MAX	1880
#define C4_MIN	1047
#define C4_ZERO	1460

#define C5_MAX	1874
#define C5_MIN	1068
#define C5_ZERO	1475

#define C6_MAX	1862
#define C6_MIN	1055
#define C6_ZERO	1475

#define C7_MAX	1862
#define C7_MIN	1054
#define C7_ZERO	1475

#define C8_MAX	1862
#define C8_MIN	1055
#define C8_ZERO	1475

#define MAX_PULSE_WIDTH		2200
#define MIN_PULSE_WIDTH		990

extern volatile uint32_t timer_rising_edge[8];
extern volatile uint32_t RF_pulse_width[8];


typedef volatile struct{
	int32_t Channel_1;
	int32_t Channel_2;
	int32_t Channel_3;
	int32_t Channel_4;
	int32_t Channel_5;
	int32_t Channel_6;
	int32_t Channel_7;
	int32_t Channel_8;
	unsigned char flag_update;
	uint16_t signal_count;
}RF_module_type;

extern RF_module_type RF_module;

void RF_init();
#endif /* QUADROTOR_CURRENT_WORK_RF_H_ */
