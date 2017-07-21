/*
 * RF.c
 *
 *  Created on: Jun 23, 2015
 *      Author: NhatTan
 */
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "driverlib/interrupt.h"
#include "inc/tm4c123gh6pm.h"

#include "RF.h"
#include "myTimer.h"
#include "myIO.h"
#include "serial.h"



volatile uint32_t timer_rising_edge[8];
volatile uint32_t RF_pulse_width[8];



RF_module_type RF_module;


void RF_init()
{
	/*
	 * software initialization
	 */
	uint8_t i;
	for(i=0;i<5;i++)
	{
		RF_module.Channel_1=C1_ZERO;
		RF_module.Channel_2=C2_ZERO;
		RF_module.Channel_3=C3_MIN;
		RF_module.Channel_4=C4_ZERO;
		RF_module.Channel_5=C5_ZERO	;
		RF_module.Channel_6=C6_ZERO;
		RF_module.Channel_7=C7_MIN;
		RF_module.Channel_8=C8_MIN;
	}
	RF_module.flag_update=0;
	RF_module.signal_count=0;
	/*
	 * hardware initialization
	 */

	//
	// Enable Peripheral Clocks
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//3.3.2016
	//
	// Enable pin PD0 to PD3, PC4, PC5 for GPIOInput
	//
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5);
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2);//3.3.2016
	//
	// set interrupt when both edges happen
	//
	GPIOIntTypeSet(GPIO_PORTD_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_BOTH_EDGES);
	GPIOIntTypeSet(GPIO_PORTC_BASE,GPIO_PIN_4|GPIO_PIN_5,GPIO_BOTH_EDGES);
	GPIOIntTypeSet(GPIO_PORTE_BASE,GPIO_PIN_1|GPIO_PIN_2,GPIO_BOTH_EDGES);//3.3.2016

	//enable processor interrupts
	IntMasterEnable();

	//enable the GPIOD interrupt
	IntEnable(INT_GPIOD);
	IntEnable(INT_GPIOC);
	IntEnable(INT_GPIOE);//3.3.2016
	//	IntPrioritySet(INT_GPIOD,4);
	//	IntPrioritySet(INT_GPIOC,5);
	IntPrioritySet(INT_GPIOD,0xC0);//configurated in RF_init()
	IntPrioritySet(INT_GPIOC,0xC0);
//	IntPrioritySet(INT_GPIOE,0xC0);

	//enable the GPIO interrupt for pind0 to pind3, pinc4, pinc5
	GPIOIntEnable(GPIO_PORTD_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	GPIOIntEnable(GPIO_PORTC_BASE,GPIO_PIN_4|GPIO_PIN_5);
	GPIOIntEnable(GPIO_PORTE_BASE,GPIO_PIN_1|GPIO_PIN_2);//3.3.2016
}

// start timing when detect a rising edge and stop timing when detect falling edge, return delta_t



