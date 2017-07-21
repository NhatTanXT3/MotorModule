/*
 * PPM.c
 *
 *  Created on: May 21, 2015
 *      Author: NhatTan
 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"

#include "inc/hw_types.h"
#include "inc/hw_gpio.h"


#include "PPM.h"

ESCtype ESC;


//===========output pin=========
// PC4 - T0CCP0 - TimerA
//
//void WTimer0_init()
//{
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
//	//
//	// Enable pin PC4 for WTIMER0 WT0CCP0
//	//
//	GPIOPinConfigure(GPIO_PC4_WT0CCP0);
//	GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_4);
//
//	TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM);
//	TimerControlLevel(WTIMER0_BASE, TIMER_A,1);
//
//	//==note: pwm have the resolution is 16 bit, so the value load TimerLoadSet don't exceed 2^16.
//	//				(SysCtlClockGet())/400;
//	TimerLoadSet(WTIMER0_BASE, TIMER_A, 100000);
//	TimerMatchSet(WTIMER0_BASE, TIMER_A,31000);
//
//	TimerEnable(WTIMER0_BASE, TIMER_A);
//}


//===========output pin=========
// PC6 - T1CCP0 - TimerA
// PC7 - T1CCP1 - TimerB
void WTimer1_init()
{
	//
	// Enable Peripheral Clocks
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	//
	// Enable pin PC6 for WTIMER1 WT1CCP0
	//
	GPIOPinConfigure(GPIO_PC6_WT1CCP0);
	GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_6);

	//
	// Enable pin PC7 for WTIMER1 WT1CCP1
	//
	GPIOPinConfigure(GPIO_PC7_WT1CCP1);
	GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_7);

	//
	// Config timerB and timerA as PWM for WTIMER1
	//
	TimerConfigure(WTIMER1_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);
	TimerControlLevel(WTIMER1_BASE, TIMER_BOTH,1);


	//==note: pwm have the resolution is 32 bit, so the value load TimerLoadSet don't exceed 2^32.
	//				(SysCtlClockGet())/400;
	TimerLoadSet(WTIMER1_BASE, TIMER_BOTH, 100000);
	TimerMatchSet(WTIMER1_BASE, TIMER_BOTH,31000);


	TimerEnable(WTIMER1_BASE, TIMER_BOTH);
}


//===========output pin=========
// PD6 - T5CCP0 - TimerA
// PD7 - T5CCP1 - TimerB
void WTimer5_init()
{
	//
	// Enable Peripheral Clocks
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);


	//
	// Enable pin PD7 for WTIMER5 WT5CCP1
	// First open the lock and select the bits we want to modify in the GPIO commit register.
	//
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;

	//
	// Now modify the configuration of the pins that we unlocked.
	//
	GPIOPinConfigure(GPIO_PD7_WT5CCP1);
	GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_7);

	//
	// Enable pin PD6 for WTIMER5 WT5CCP0
	//
	GPIOPinConfigure(GPIO_PD6_WT5CCP0);
	GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_6);

	//
	// Config timerB and timerA as PWM for WTIMER1
	//
	TimerConfigure(WTIMER5_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);
	TimerControlLevel(WTIMER5_BASE, TIMER_BOTH,1);


	//==note: pwm have the resolution is 32 bit, so the value load TimerLoadSet don't exceed 2^32.
	//				(SysCtlClockGet())/400;
	TimerLoadSet(WTIMER5_BASE, TIMER_BOTH, 100000);

	TimerMatchSet(WTIMER5_BASE, TIMER_A,41000);
	TimerMatchSet(WTIMER5_BASE, TIMER_B,28000);


	TimerEnable(WTIMER5_BASE, TIMER_BOTH);
}


void PPM_init()
{
	WTimer1_init();
	WTimer5_init();
	ESC_ppm(1,ESC_1_MIN);
	ESC_ppm(2,ESC_2_MIN);
	ESC_ppm(3,ESC_3_MIN);
	ESC_ppm(4,ESC_4_MIN);
}

void Motor_stop()
{
	ESC_ppm(1,ESC_1_MIN);
	ESC_ppm(2,ESC_2_MIN);
	ESC_ppm(3,ESC_3_MIN);
	ESC_ppm(4,ESC_4_MIN);
}

extern volatile struct{
	unsigned char RF_module:1;
	unsigned char MaxSonar:1;
	unsigned char System:1;
} Flag_Safe;

void ESC_ppm(unsigned char Esc_num, int32_t ppm)
{
	if(Flag_Safe.System==1)
	{
		switch(Esc_num){
		case 1:
			if((ppm<ESC_1_MIN))
				TimerMatchSet(WTIMER1_BASE, TIMER_A,ESC_1_MIN);//PC6
			else if(ppm>ESC_1_MAX)
				TimerMatchSet(WTIMER1_BASE, TIMER_A,ESC_1_MAX);//PC6
			else
				TimerMatchSet(WTIMER1_BASE, TIMER_A,ppm);//PC6
			break;
		case 2:
			if(ppm<ESC_2_MIN)
				TimerMatchSet(WTIMER1_BASE, TIMER_B,ESC_2_MIN);//PC7
			else if(ppm>ESC_2_MAX)
				TimerMatchSet(WTIMER1_BASE, TIMER_B,ESC_2_MAX);//PC7
			else
				TimerMatchSet(WTIMER1_BASE, TIMER_B,ppm);//PC7
			break;
		case 3:
			if(ppm<ESC_3_MIN)
				TimerMatchSet(WTIMER5_BASE, TIMER_A,ESC_3_MIN);//PD6
			else if (ppm>ESC_3_MAX)
				TimerMatchSet(WTIMER5_BASE, TIMER_A,ESC_3_MAX);//PD6
			else
				TimerMatchSet(WTIMER5_BASE, TIMER_A,ppm);//PD6
			break;
		case 4:
			if(ppm<ESC_4_MIN)
				TimerMatchSet(WTIMER5_BASE, TIMER_B,ESC_4_MIN);//PD7
			else if(ppm>ESC_4_MAX)
				TimerMatchSet(WTIMER5_BASE, TIMER_B,ESC_4_MAX);//PD7
			else
				TimerMatchSet(WTIMER5_BASE, TIMER_B,ppm);//PD7
			break;
		default:
			TimerMatchSet(WTIMER1_BASE, TIMER_A,ESC_1_MIN);//PC6
			TimerMatchSet(WTIMER1_BASE, TIMER_B,ESC_2_MIN);//PC7
			TimerMatchSet(WTIMER5_BASE, TIMER_A,ESC_3_MIN);//PD6
			TimerMatchSet(WTIMER5_BASE, TIMER_B,ESC_4_MIN);//PD7
			break;
		}//end of switch
	}//end of if
	else
	{
		TimerMatchSet(WTIMER1_BASE, TIMER_A,ESC_1_MIN);//PC6
		TimerMatchSet(WTIMER1_BASE, TIMER_B,ESC_2_MIN);//PC7
		TimerMatchSet(WTIMER5_BASE, TIMER_A,ESC_3_MIN);//PD6
		TimerMatchSet(WTIMER5_BASE, TIMER_B,ESC_4_MIN);//PD7
	}
}



