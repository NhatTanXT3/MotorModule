/*
 * safety.h
 *
 *  Created on: Sep 29, 2016
 *      Author: Nhat Tan
 */

#ifndef SAFETY_H_
#define SAFETY_H_


typedef volatile struct{
	unsigned char RF_module:1;
	unsigned char MaxSonar:1;
	unsigned char System:1;
} Flag_Safe_type;

extern Flag_Safe_type Flag_Safe;

extern void safe_check();

#endif /* SAFETY_H_ */
