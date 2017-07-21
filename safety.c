/*
 * safety.c
 *
 *  Created on: Sep 29, 2016
 *      Author: Nhat Tan
 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "safety.h"
#include "serial.h"
#include "RF.h"
#include "myTimer.h"
#include "PPM.h"


Flag_Safe_type Flag_Safe;

void safe_check()
{
	ESC_ppm(1,ESC_1_MIN);
	ESC_ppm(2,ESC_2_MIN);
	ESC_ppm(3,ESC_3_MIN);
	ESC_ppm(4,ESC_4_MIN);


	while(RF_module.Channel_3<C3_LIMIT_STICK_TOP);
	while(RF_module.Channel_3>C3_LIMIT_STICK_BOTTOM);
	SerialPutStrLn(UART_COM_2_CONTROLLER_,"Drone_ready!");

}

