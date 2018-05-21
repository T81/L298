/*************************************************************************
* L298
* by Christodoulos P. Lekkos <tolis81@gmail.com> , September 03, 2014.
*
* http://www.microchip.com/wwwproducts/Devices.aspx?product=L298
* This file is free software; you can redistribute it and/or modify
* it under the terms of either the GNU General Public License version 3
* published by the Free Software Foundation.
*************************************************************************/

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <L298.h>

L298::L298() {
}

void L298::Begin(unsigned char P1, unsigned char M11, unsigned char M12, unsigned char P2, unsigned char M21, unsigned char M22) {
	_motorsArray[0] = P1;		// ENABLE A
	_motorsArray[1] = M11;		// INPUT 1
	_motorsArray[2] = M12;		// INPUT 2
	_motorsArray[3] = 0;		// MOTOR A ACTION
	_motorsArray[4] = 0;		// MOTOR A SPEED
	_motorsArray[5] = P2;		// ENABLE B
	_motorsArray[6] = M21;		// INPUT 3
	_motorsArray[7] = M22;		// INPUT 4
	_motorsArray[8] = 0;		// MOTOR B ACTION
	_motorsArray[9] = 0;		// MOTOR B SPEED

	if (_motorsArray[MOTOR_A] != MOTOR_DISABLE) {
		for (int i = 0; i < 3; i++) {
			pinMode(_motorsArray[MOTOR_A + i], OUTPUT);
			digitalWrite[_motorsArray[MOTOR_A + i], LOW);
		}
	}
	if (_motorsArray[MOTOR_B] != MOTOR_DISABLE) {
		for (int i = 0; i < 3; i++) {
			pinMode(_motorsArray[MOTOR_B + i], OUTPUT);
			digitalWrite[_motorsArray[MOTOR_B +i ], LOW);
		}	
	}
}

void L298::Motor(unsigned char motorSelect, unsigned char action, unsigned char speed) {
	if (motorSelect == MOTOR_A || motorSelect == MOTOR_B) {
		if (_motorsArray[motorSelect + MOTOR_ACTION] != action) {
			_motorsArray[motorSelect + MOTOR_ACTION] = action;
			if (action == MOTOR_FORWARD) {
				digitalWrite(_motorsArray[motorSelect + 1], true);
				digitalWrite(_motorsArray[motorSelect + 2], false);
			}
			else if (action == MOTOR_REVERSE) {
				digitalWrite(_motorsArray[motorSelect + 1], false);
				digitalWrite(_motorsArray[motorSelect + 2], true);
			}
			else if (action == MOTOR_FAST_STOP) {
				digitalWrite(_motorsArray[motorSelect + 1], false);
				digitalWrite(_motorsArray[motorSelect + 2], false);
				analogWrite(_motorsArray[motorSelect], 255);
			}
			else if (action == MOTOR_FREE_STOP) {
				analogWrite(_motorsArray[motorSelect], 0);
			}			
		}
		if (_motorsArray[motorSelect + MOTOR_SPEED] != speed) {
			_motorsArray[motorSelect + MOTOR_SPEED] = speed;
			analogWrite(_motorsArray[motorSelect], _motorsArray[motorSelect + MOTOR_SPEED]);
		}
	}
}