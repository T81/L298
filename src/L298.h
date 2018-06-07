/*************************************************************************
* L298 Dual Full-Bridge Driver library
* by Christodoulos P. Lekkos <tolis81@gmail.com>
*
* This file is free software; you can redistribute it and/or modify
* it under the terms of either the GNU General Public License version 3
* published by the Free Software Foundation.
*************************************************************************/

/*************************************************************************
* Status register Byte (MSB)
**************************************************************************
* 7 * 
* 6 * 
* 5 * 
* 4 * 
* 3 * 
* 2 * LMT * Indicates that the limits function is active
* 1 * IN2 *
* 0 * RUN *
*************************************************************************/


/*************************************************************************
* Status register Byte	(LSB)
**************************************************************************
* 7 * OCR * 1 if current exceeds user defined limit
* 6 * LM2 * Indicates that Limit 2 is reached
* 5 * LM1 * Indicates that Limit 1 is reached 
* 4 * ACC * 1 if motor is accelerating or decelerating
* 3 * DIR * 1 if direction is set to either CCW, UP, FORWARD, RIGHT
* 2 * CST * 1 if motor is coasting
* 1 * BRK * 1 if motor brake on
* 0 * RUN * 1 if motor is running
*************************************************************************/

#ifndef L298_H
#define L298_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif



// comment out the following line to disable "ACCELERATION" functions
#define ACCELERATION_FUNCTIONS

// comment out the following line to disable "CURRENT" functions
#define CURRENT_FUNCTIONS

// comment out the following line to disable "LIMITING" functions
#define LIMITING_FUNCTIONS

// comment out the following line to disable "POSITION" functions
#define POSITION_FUNCTIONS


// uncomment the following line to enable "DEBUG" messages
#define DEBUG


// direction definitions
#define CW 0x00
#define BACKWARDS 0x00
#define DOWN 0x00
#define LEFT 0x00

#define CCW	0x01
#define FORWARD 0x01
#define UP 0x01
#define RIGHT 0x01

// brake parameters definitions
#define OFF 0x00
#define ON 0x01

// limits mode definitions
#define NO_PULLUP 0x07
#define EXTERNAL_PULLUP 0x08
#define INTERNAL_PULLUP 0x09


#define DIGITAL_LIMITS 0x00
#define ANALOG_LIMITS 0x01
#define ANALOG_DIGITAL_LIMITS 0x02

// status register definitions
//LSB
#define RUNNING 0x01
#define DIRECTION 0x02
#define ACCELERATION 0x04
#define BRAKING 0x08
#define COASTING 0x10
#define BRAKE_ON 0x20
#define LIMITS 0x40
#define LIMIT_CW 0x80

//MSB
#define LIMIT_CCW 0x100
#define OVERCURRENT	0x200
#define AUTO_ON 0x400


class L298
{
	/* Public variables and functions */
	public:
		L298();
		void begin(unsigned char enable, unsigned char inputA, unsigned char inputB);
		
		void brake(bool state);
		bool isBrakeOn();
		
		void coast();
		bool isCoasting();

		void setSpeed(unsigned char speed);	
		unsigned char getSpeed();
		bool isRunning();
		
		void setDirection(bool direction);
		bool getDirection();

#if defined(ACCELERATION_FUNCTIONS) || defined(LIMITING_FUNCTIONS) || defined(CURRENT_FUNCTIONS)
		void update();
#endif

#ifdef ACCELERATION_FUNCTIONS
		void setAccelerationTime(unsigned long milliSeconds);
		bool isAccelerating();
		bool isBraking();
#endif

#ifdef CURRENT_FUNCTIONS
		void currentPin(unsigned char currentPin);
		void setCurrent(double amperes);
		double getCurrent();
		bool checkCurrent();
#endif

#ifdef LIMITING_FUNCTIONS
		void setLimitPins(unsigned char limitCWpin, unsigned char limitCCWpin);
		void limitsConfig(unsigned char pullup);
		bool checkCollision(bool limit);
#endif

#ifdef POSITION_FUNCTIONS
		void setPositionPin(int pin);
		void setPosition(int position);
		int getPosition();
#endif


/* Private variables and functions */
	private:
		void _setMotionFlags();
		bool _direction;
		unsigned char _enable, _inputA, _inputB, _currentSpeed, _targetSpeed;
		unsigned int _status;
		unsigned long _interval;

#ifdef LIMITING_FUNCTIONS
		bool _pullup;
		unsigned char _limitCWpin, _limitCCWpin;
		void _checkLimits();
#endif

#ifdef POSITION_FUNCTIONS
		int _positionPin;
		int _previousPosition, _currentPosition, _targetPosition;
#endif

#ifdef ACCELERATION_FUNCTIONS
		unsigned char _startingSpeed;
		unsigned long _currentMillis, _previousMillis, _millisTarget;
		double _rampRate;
#endif

#ifdef CURRENT_FUNCTIONS
		unsigned char _currentPin;
		int _voltageRead;
		double _ampsMax, _setAmps, _currentAmps;
		void configCurrent(double supplyVoltageMax, double senseResistor);
#endif

		inline void setStatusFlag(const unsigned int flag)    {_status |= flag;}
		inline void unsetStatusFlag(const unsigned int flag)  {_status &= ~flag;}
		inline void toggleStatusFlag(const unsigned int flag) {_status ^= flag;}
		inline bool getStatusFlag(const unsigned int flag)    {return(_status & flag);}
};
#endif
