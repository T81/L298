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


/************************************************************************
                    Enable or disable advanced functionality 
*************************************************************************/

// comment out the following line to disable "ACCELERATION" functions
#define ACCELERATION_FUNCTIONS

// comment out the following line to disable "CURRENT" functions
#define CURRENT_FUNCTIONS

// comment out the following line to disable "LIMITING" functions
#define DIGITAL_FUNCTIONS

// comment out the following line to disable "POSITION" functions
#define ANALOG_FUNCTIONS


// uncomment the following line to enable "DEBUG" messages
#define DEBUG

/*************************************************************************/



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
#define DECELERATION 0x08
#define BRAKING 0x10
#define COASTING 0x20
#define BRAKE_ON 0x40
#define ANALOG_LIMIT 0x80


//MSB
#define ANALOG_LIMIT_CW 0x100
#define ANALOG_LIMIT_CCW 0x200
#define DIGITAL_LIMIT 0x400
#define DIGITAL_LIMIT_CW 0x800
#define DIGITAL_LIMIT_CCW 0x100
#define OVERCURRENT	0x2000
// #define NOT_USED 0x4000
// #define NOT_USED 0x8000



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
		
		void safeDirectionChange(bool directionRestriction); 
		void setDirection(bool direction);
		bool getDirection();

		
/* These method is available if any advanced mode is enabled*/
#if defined(ACCELERATION_FUNCTIONS) || defined(CURRENT_FUNCTIONS) || defined(DIGITAL_FUNCTIONS) || defined(ANALOG_FUNCTIONS)
		void update();
#endif


/* These methods are available if ACCELERATION functionality is enabled*/
#ifdef ACCELERATION_FUNCTIONS
		void setAccelerationTime(unsigned long milliSeconds);
		bool isAccelerating();
		bool isDecelerating();
		bool isBraking();
#endif


/* These methods are available if CURRENT functionality is enabled*/
#ifdef CURRENT_FUNCTIONS
		void configCurrentSense(unsigned char sensePin, double supplyVoltage, double senseResistor);
		void setCurrent(double amperes);
		double getCurrent();
		bool checkOvercurrent();
		double maxCurrent();
		void resetCurrent();
#endif


/* These methods are available if LIMITS functionality is enabled*/
#ifdef DIGITAL_FUNCTIONS
		void setLimitPins(unsigned char limitCWpin, unsigned char limitCCWpin);
		void configLimitPins(unsigned char pullup);
		bool checkCollision(bool limit);
		void analogLimits(bool enable);
#endif


/* These methods are available if POSITION functionality is enabled*/
#ifdef ANALOG_FUNCTIONS
		void positionPin(unsigned char pin);
		void setPositionLimits(int lowerLimit, int upperLimit);
		int getPosition();
#endif



/* Private variables and functions */
	private:
		void _setMotionFlags();
		bool _direction, _directionRestriction;
		unsigned char _enable, _inputA, _inputB, _currentSpeed, _targetSpeed;
		unsigned int _status;
		// long _interval;


/* These methods are available if ACCELERATION functionality is enabled*/
#ifdef ACCELERATION_FUNCTIONS
		unsigned char _startingSpeed;
		unsigned long _currentMillis, _previousMillis, _millisTarget;
		double _rampRate;
#endif

/* These methods are available if CURRENT functionality is enabled*/
#ifdef CURRENT_FUNCTIONS
		unsigned char _sensePin;
		int _vSense;
		double _ampsMax, _setAmps, _currentAmps, _maxCurrent;
#endif

/* These methods are available if LIMITS functionality is enabled*/
#ifdef DIGITAL_FUNCTIONS
		bool _pullup;
		unsigned char _limitCWpin, _limitCCWpin;
		void _checkDigitalLimits();
#endif

/* These methods are available if POSITION functionality is enabled*/
#ifdef ANALOG_FUNCTIONS
		int _positionPin;
		int _currentPosition, _lowerLimit, _upperLimit;
		void _checkAnalogLimits();
#endif


		inline void setStatusFlag(const unsigned int flag)    {_status |= flag;}
		inline void unsetStatusFlag(const unsigned int flag)  {_status &= ~flag;}
		inline void toggleStatusFlag(const unsigned int flag) {_status ^= flag;}
		inline bool getStatusFlag(const unsigned int flag)    {return(_status & flag);}
};
#endif
