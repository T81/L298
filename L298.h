/*************************************************************************
* L298 libary
* by Christodoulos P. Lekkos <tolis81@gmail.com> , September 19, 2014.
*
* This file is free software; you can redistribute it and/or modify
* it under the terms of either the GNU General Public License version 3
* published by the Free Software Foundation.
*************************************************************************/

#ifndef L298_H
#define L298_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define MOTOR_A				0x00
#define MOTOR_B				0x05
#define MOTOR_ACTION		0x03
#define MOTOR_SPEED			0x04
#define MOTOR_FAST_STOP		0
#define MOTOR_FREE_STOP		1
#define MOTOR_FORWARD		2
#define MOTOR_REVERSE		3


#define MOTOR_DISABLE		0xFF

#define L298_DEBUG 			0	// set to 1 for debugging

class L298
{
	public:
		L298();
		void Begin(unsigned char P1 = MOTOR_DISABLE, unsigned char M11 = MOTOR_DISABLE, unsigned char M12 = MOTOR_DISABLE, unsigned char P2 = MOTOR_DISABLE, unsigned char M21 = MOTOR_DISABLE, unsigned char M22 = MOTOR_DISABLE);
		void Motor(unsigned char motorSelect, unsigned char action, unsigned char speed);

	private:
		unsigned char _motorsArray[9];
};
#endif 