/*
 * interrupt_handler.c
 *
 *  Created on: Sep 29, 2016
 *      Author: Nhat Tan
 */
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "driverlib/interrupt.h"
#include "inc/tm4c123gh6pm.h"

#include "myIO.h"
#include "mpu6050.h"
#include "RF.h"
#include "serial.h"
#include "myTimer.h"
#include "Sonar.h"
/*
 * for Sonar module
 */
volatile unsigned char Flag_SonarHoldAttitudeMode;

/*
 * For IMU module
 */

void GPIOE_Interrupt_Handler(void)
{
	// return: true-the masked interrupt status | false -the current interrupt status
	uint32_t interrupt_status;
	interrupt_status=GPIOIntStatus(GPIO_PORTE_BASE,true);
	//	toggle_led[2]^=1;
	//	led(LED_RED,toggle_led[2]);

	switch(interrupt_status){
	case GPIO_INT_PIN_0:
		GPIOIntClear(GPIO_PORTE_BASE,GPIO_INT_PIN_0);
		if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0)==GPIO_PIN_0)
		{
			Sonar_module.rise_time=getMicroSecond();
		}
		else
		{
			Sonar_module.pulse_width=getMicroSecond()-Sonar_module.rise_time;
			Sonar_module.d_pulse_width=Sonar_module.pulse_width-Sonar_module.pre_pulse_width;
			Sonar_module.pre_pulse_width=Sonar_module.pulse_width;
//			if((Sonar_module.pulse_width>SONAR_MIN_PULSE_WIDTH_)&&(Sonar_module.pulse_width<SONAR_MAX_PULSE_WIDTH_))
//			{
//				Sonar_module.distance=Sonar_module.distance*SONAR_FILTER_FACTOR+(1-SONAR_FILTER_FACTOR)*(Sonar_module.pulse_width-Sonar_module.pulse_width_offset)*SONAR_SCALE_FACTOR;
//				Sonar_module.distance_1=Sonar_module.distance_1*0.5+(1-0.5)*(Sonar_module.pulse_width-Sonar_module.pulse_width_offset)*SONAR_SCALE_FACTOR;
//				Sonar_module.distance_raw=(Sonar_module.pulse_width-Sonar_module.pulse_width_offset)*SONAR_SCALE_FACTOR;
//				Sonar_module.z1_dot=Sonar_module.distance_raw-Sonar_module.pre_distance_raw;
//				Sonar_module.pre_distance_raw=Sonar_module.distance_raw;
//
//				Sonar_module.flag_update=1;
//			}
//			else
//			{
//				Sonar_module.fail_signal_count++;
//				SerialPutStrLn(UART_COM_2_CONTROLLER_,"error_sonar");
//			}
			Sonar_module.flag_update=1;

		}
		break;
	case GPIO_INT_PIN_1:
		GPIOIntClear(GPIO_PORTE_BASE,GPIO_INT_PIN_1);
		if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1)==GPIO_PIN_1)
			timer_rising_edge[6]=getMicroSecond();
		else{
			RF_pulse_width[6]=getMicroSecond()-timer_rising_edge[6];
			if((RF_pulse_width[6]>MIN_PULSE_WIDTH)&&(RF_pulse_width[6]<MAX_PULSE_WIDTH)){
				float buffer;
				buffer=(float)RF_module.Channel_7*RF_FILTER_FACTOR_VOLUME+(float)RF_pulse_width[6]*(1.0-RF_FILTER_FACTOR_VOLUME);
				RF_module.Channel_7=buffer;

				if(RF_module.Channel_7>C7_ZERO)
				{
					Flag_SonarHoldAttitudeMode=1;
					//					led(LED_RED,1);
				}
				else
				{
					Flag_SonarHoldAttitudeMode=0;
					//					led(LED_RED,0);
				}
			}

		}
		break;
	case GPIO_INT_PIN_2:
		GPIOIntClear(GPIO_PORTE_BASE,GPIO_INT_PIN_2);
		if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2)==GPIO_PIN_2)
			timer_rising_edge[7]=getMicroSecond();
		else{
			RF_pulse_width[7]=getMicroSecond()-timer_rising_edge[7];
			if((RF_pulse_width[7]>MIN_PULSE_WIDTH)&&(RF_pulse_width[7]<MAX_PULSE_WIDTH)){
				float buffer;
				buffer=(float)RF_module.Channel_8*RF_FILTER_FACTOR_VOLUME+(float)RF_pulse_width[7]*(1.0-RF_FILTER_FACTOR_VOLUME);
				RF_module.Channel_8=buffer;
			}
		}
		break;
	case GPIO_INT_PIN_3:
		GPIOIntClear(GPIO_PORTE_BASE,GPIO_INT_PIN_3);
		flag_MPU6050_INTpin=1;
		break;

	default:
		GPIOIntClear(GPIO_PORTE_BASE,0xFF);
		break;
	}
}

void GPIOD_Interrupt_Handler(void)
{
	// return: true-the masked interrupt status | false -the current interrupt status
	uint32_t interrupt_status;
	interrupt_status=GPIOIntStatus(GPIO_PORTD_BASE,true);

	switch(interrupt_status){
	case GPIO_INT_PIN_0:
		GPIOIntClear(GPIO_PORTD_BASE,GPIO_INT_PIN_0);
		if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0)==GPIO_PIN_0){
			timer_rising_edge[0]=getMicroSecond();
		}
		else
		{
			RF_pulse_width[0]=getMicroSecond()-timer_rising_edge[0];
			if((RF_pulse_width[0]>MIN_PULSE_WIDTH)&&(RF_pulse_width[0]<MAX_PULSE_WIDTH)){
				float buffer;
				buffer=(float)RF_module.Channel_1*RF_FILTER_FACTOR_STICK+(float)RF_pulse_width[0]*(1.0-RF_FILTER_FACTOR_STICK);
				RF_module.Channel_1=buffer;
			}
		}
		break;
	case GPIO_INT_PIN_1:
		GPIOIntClear(GPIO_PORTD_BASE,GPIO_INT_PIN_1);
		if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1)==GPIO_PIN_1)
			timer_rising_edge[1]=getMicroSecond();
		else{
			RF_pulse_width[1]=getMicroSecond()-timer_rising_edge[1];
			if((RF_pulse_width[1]>MIN_PULSE_WIDTH)&&(RF_pulse_width[1]<MAX_PULSE_WIDTH)){
				float buffer;
				buffer=(float)RF_module.Channel_2*RF_FILTER_FACTOR_STICK+(float)RF_pulse_width[1]*(1.0-RF_FILTER_FACTOR_STICK);
				RF_module.Channel_2=buffer;
			}
		}
		break;
	case GPIO_INT_PIN_2:
		GPIOIntClear(GPIO_PORTD_BASE,GPIO_INT_PIN_2);

		if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_2)==GPIO_PIN_2)
			timer_rising_edge[2]=getMicroSecond();
		else{
			RF_pulse_width[2]=getMicroSecond()-timer_rising_edge[2];
			if((RF_pulse_width[2]>MIN_PULSE_WIDTH)&&(RF_pulse_width[2]<MAX_PULSE_WIDTH)){
				float buffer;
				buffer=(float)RF_module.Channel_3*RF_FILTER_FACTOR_STICK+(float)RF_pulse_width[2]*(1.0-RF_FILTER_FACTOR_STICK);
				RF_module.Channel_3=buffer;
			}
		}

		break;
	case GPIO_INT_PIN_3:
		GPIOIntClear(GPIO_PORTD_BASE,GPIO_INT_PIN_3);

		if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_3)==GPIO_PIN_3)
			timer_rising_edge[3]=getMicroSecond();
		else{
			RF_pulse_width[3]=getMicroSecond()-timer_rising_edge[3];
			if((RF_pulse_width[3]>MIN_PULSE_WIDTH)&&(RF_pulse_width[3]<MAX_PULSE_WIDTH)){
				float buffer;
				buffer=(float)RF_module.Channel_4*RF_FILTER_FACTOR_STICK+(float)RF_pulse_width[3]*(1.0-RF_FILTER_FACTOR_STICK);
				RF_module.Channel_4=buffer;
			}
		}

		break;
	default:
		GPIOIntClear(GPIO_PORTD_BASE,GPIO_INT_PIN_0|GPIO_INT_PIN_1|GPIO_INT_PIN_2|GPIO_INT_PIN_3);
		break;
	}
}

void GPIOC_Interrupt_Handler(void)
{
	// return: true-the masked interrupt status | false -the current interrupt status
	uint32_t interrupt_status;
	interrupt_status=GPIOIntStatus(GPIO_PORTC_BASE,true);

	switch(interrupt_status){
	case GPIO_INT_PIN_4:
		GPIOIntClear(GPIO_PORTC_BASE,GPIO_INT_PIN_4);
		if(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)==GPIO_PIN_4)
			timer_rising_edge[4]=getMicroSecond();
		else{
			RF_pulse_width[4]=getMicroSecond()-timer_rising_edge[4];
			if((RF_pulse_width[4]>MIN_PULSE_WIDTH)&&(RF_pulse_width[4]<MAX_PULSE_WIDTH)){
				float buffer;
				buffer=(float)RF_module.Channel_5*RF_FILTER_FACTOR_VOLUME+(float)RF_pulse_width[4]*(1.0-RF_FILTER_FACTOR_VOLUME);
				RF_module.Channel_5=buffer;
			}
		}
		break;
	case GPIO_INT_PIN_5:
		GPIOIntClear(GPIO_PORTC_BASE,GPIO_INT_PIN_5);
		if(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)==GPIO_PIN_5)
			timer_rising_edge[5]=getMicroSecond();
		else{
			RF_pulse_width[5]=getMicroSecond()-timer_rising_edge[5];
			if((RF_pulse_width[5]>MIN_PULSE_WIDTH)&&(RF_pulse_width[5]<MAX_PULSE_WIDTH)){
				float buffer;
				buffer=(float)RF_module.Channel_6*RF_FILTER_FACTOR_VOLUME+(float)RF_pulse_width[5]*(1.0-RF_FILTER_FACTOR_VOLUME);
				RF_module.Channel_6=buffer;
				RF_module.flag_update=1;
				RF_module.signal_count++;
			}
			else
			{
				SerialPutStrLn(UART_COM_2_CONTROLLER_,"ErRF");
			}
		}
		break;
	default:
		GPIOIntClear(GPIO_PORTC_BASE,GPIO_INT_PIN_5|GPIO_INT_PIN_4);
		break;
	}
}
