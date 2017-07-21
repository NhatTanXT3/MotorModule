/*
 * Sonar_module.c
 *
 *  Created on: Sep 30, 2016
 *      Author: Nhat Tan
 */

/*
 * Use T2CPP0 - PB3 for sonar trigger
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "Sonar.h"
#include "serial.h"
#include "numManipulate.h"

Sonar_type Sonar_module;


void Timer3_init()
{
	//
	// Enable Peripheral Clocks
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//
	// Enable pin PB2 for TIMER3
	GPIOPinConfigure(GPIO_PB2_T3CCP0);
	GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_2);


	//
	// Config  timerA as PWM for TIMER3
	//
	TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM);
	TimerControlLevel(TIMER3_BASE, TIMER_A,1);


	//==note: pwm have the resolution is 32 bit, so the value load TimerLoadSet don't exceed 2^32.
	//				(SysCtlClockGet())/400;
	TimerLoadSet(TIMER3_BASE, TIMER_A, 33920);
	TimerPrescaleSet(TIMER3_BASE,TIMER_A,30);

	//	TimerMatchSet(TIMER3_BASE, TIMER_A,16960);//50% duty
	//	TimerPrescaleMatchSet(TIMER3_BASE,TIMER_A,15);


//	TimerMatchSet(TIMER3_BASE, TIMER_A,40000);// 1ms
	//	TimerPrescaleMatchSet(TIMER3_BASE,TIMER_A,0);

	TimerMatchSet(TIMER3_BASE, TIMER_A,3392);//5ms
	TimerPrescaleMatchSet(TIMER3_BASE,TIMER_A,3);


	TimerEnable(TIMER3_BASE, TIMER_A);
}

void Sonar_module_init(){
	//
	//initial value
	//
	Sonar_module.attitude=0;
	Sonar_module.distance=0;
	Sonar_module.fail_signal_count=0;
	Sonar_module.pulse_width_offset=0;
	//
	// Enable Peripheral Clocks
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	//
	// Enable pin PE0 for GPIOInput
	//
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);

	//
	// set interrupt when both edges happen
	//
	GPIOIntTypeSet(GPIO_PORTE_BASE,GPIO_PIN_0,GPIO_BOTH_EDGES);


	//enable processor interrupts
	IntMasterEnable();

	//enable the GPIOB interrupt
	IntEnable(INT_GPIOE);

	IntPrioritySet(INT_GPIOE,0xA0);

	//enable the GPIO interrupt for pinb3
	GPIOIntEnable(GPIO_PORTE_BASE,GPIO_PIN_0);

	//trigger init
	Timer3_init();

	/*
	 * init value
	 */
	 Sonar_module.z1_dot_estimate=0;

}

unsigned char Calib_Sonar_module()
{

	Sonar_module.pulse_width_offset=0;

	int32_t sum=0;
	uint8_t numOfsamples=50;
	int32_t buffer;
	int32_t max_value;
	int32_t min_value;
	uint8_t i;

	while(Sonar_module.flag_update==0);
	Sonar_module.flag_update=0;
	buffer=Sonar_module.pulse_width;
	max_value=Sonar_module.pulse_width;
	min_value=Sonar_module.pulse_width;

	for (i=1;i<=numOfsamples;i++)
	{
		while(Sonar_module.flag_update==0);
		Sonar_module.flag_update=0;
		buffer=	Sonar_module.pulse_width;

		if(buffer>max_value) max_value=buffer;
		else if(buffer<min_value) min_value=buffer;

		if(max_value-min_value>SONAR_NOISE_RANGE)
		{
			SerialPutStr_NonTer(UART_COM_2_CONTROLLER_," err sonar noise!");
			char buffer[20];
			float2num(max_value-min_value,buffer);
			SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);

			return  1;
		}
		sum+=buffer;
	}
	Sonar_module.pulse_width_offset=sum/numOfsamples;


	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_," sonar pulse_width offset: ");

	char str_buffer[15];
	float2num(Sonar_module.pulse_width_offset,str_buffer);
	SerialPutStrLn(UART_COM_2_CONTROLLER_,str_buffer);

//	if(abs(Sonar_module.pulse_width_offset-SONAR_PULSE_WIDTH_OFFSET_)>1000)
//	{
//		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_," Plug out the power of sonar module and reset!");
//		while(1);
//	}



	return 0;
}


