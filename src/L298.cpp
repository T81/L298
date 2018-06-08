/*************************************************************************
* L298 Dual Full-Bridge Driver library
* by Christodoulos P. Lekkos <tolis81@gmail.com>
*
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


void L298::begin(unsigned char enable, unsigned char inputA, unsigned char inputB) {
	_enable = enable;
	_inputA = inputA;
	_inputB = inputB;
	pinMode(_enable, OUTPUT);
	pinMode(_inputA, OUTPUT);
	pinMode(_inputB, OUTPUT);
	// _currentSpeed = 0;
	// analogWrite(_enable, _currentSpeed);
 	// _status = 0;
	// setSpeed(0);
	coast();
	setDirection(CW);
#ifdef CURRENT_FUNCTIONS
	setCurrent(2.0);
#endif
 	//digitalWrite(_inputA, false);
	//digitalWrite(_inputB, false);
}


void L298::brake(bool state) {
	if (state == ON) {
		if (!getStatusFlag(BRAKE_ON)) {
			setStatusFlag(BRAKE_ON);
			digitalWrite(_inputA, true);
			digitalWrite(_inputB, true);
			analogWrite(_enable, 255);
			_targetSpeed = 0;
			_currentSpeed = 0;
			_setMotionFlags();
		}
	}
	else if (state == OFF) {
		if (getStatusFlag(BRAKE_ON)) {
			unsetStatusFlag(BRAKE_ON);
			coast();
/* 			analogWrite(_enable, 255);
			_targetSpeed = 255;
			_currentSpeed = 255; */
			setDirection(_direction);	// revert to previously chosen direction
			
		}	
	}
}

void L298::coast() {
	if (!getStatusFlag(BRAKE_ON)) {
		if (!getStatusFlag(COASTING)) {
			//setStatusFlag(COASTING);
			analogWrite(_enable, 0);
			_targetSpeed = 0;
			_currentSpeed = 0;
			_setMotionFlags();
		}
	}
}

bool L298::getDirection(){
	return getStatusFlag(DIRECTION);
}

bool L298::isRunning() {
	return getStatusFlag(RUNNING);
}

bool L298::isCoasting() {
	return (getStatusFlag(COASTING));
}

bool L298::isBrakeOn() {
	return getStatusFlag(BRAKE_ON);
}

unsigned char L298::getSpeed() {
	return (_currentSpeed);
}


void L298::setDirection(bool direction) {
	_direction = direction;
	if (_direction) { // either CCW, FORWARD, UP, RIGHT
		setStatusFlag(DIRECTION);
		digitalWrite(_inputA, true);
		digitalWrite(_inputB, false);
	}
	else { // either CW, BACKWARDS, DOWN, LEFT
		unsetStatusFlag(DIRECTION);
		digitalWrite(_inputA, false);
		digitalWrite(_inputB, true);
	}
}

void L298::setSpeed(unsigned char speed) {
#ifdef LIMITING_FUNCTIONS
	if (!getStatusFlag(LIMITS)) {
#ifdef DEBUG
	Serial.println("Error @ setSpeed():  Use setLimitPins() first or disable LIMITING_FUNCTIONS");
#endif
	return;
	}
	else {
		
	}
#endif
	if (!getStatusFlag(BRAKE_ON)) { // brake should be released
		_targetSpeed = speed;
		if (_currentSpeed != _targetSpeed) { // if speed has changed
#ifdef ACCELERATION_FUNCTIONS
			if (_millisTarget) {
				_startingSpeed = _currentSpeed; // update the starting speed
				_rampRate = (double(_targetSpeed) - double(_startingSpeed)) / double(_millisTarget);
				_previousMillis = millis(); // start the clock
				// update();
			}
			else {
				_currentSpeed = _targetSpeed;
				analogWrite(_enable, _currentSpeed);
			}
#else
			_currentSpeed = _targetSpeed;
			analogWrite(_enable, _currentSpeed);
#endif
			_setMotionFlags();
		}
	}
}

void L298::_setMotionFlags() {
	if (getStatusFlag(BRAKE_ON)) {
		unsetStatusFlag(RUNNING | COASTING | ACCELERATION | BRAKING);
	}
	else {
		if (_currentSpeed || _targetSpeed) {
			unsetStatusFlag(COASTING);
			setStatusFlag(RUNNING);
#ifdef ACCELERATION_FUNCTIONS
			if (_currentSpeed < _targetSpeed) {
				unsetStatusFlag(BRAKING);
				setStatusFlag(ACCELERATION);
			}
			else if (_currentSpeed > _targetSpeed) {
				unsetStatusFlag(ACCELERATION);
				setStatusFlag(BRAKING);
			}
			else {
				unsetStatusFlag(ACCELERATION | BRAKING);
			}
#endif
		}
		else {
			unsetStatusFlag(RUNNING);
#ifdef ACCELERATION_FUNCTIONS
			unsetStatusFlag(ACCELERATION);
			unsetStatusFlag(BRAKING);
#endif
			setStatusFlag(COASTING);
		}
	}
}

#ifdef ACCELERATION_FUNCTIONS
void L298::setAccelerationTime(unsigned long milliSeconds) {
	_millisTarget = milliSeconds;
}

bool L298::isBraking() {
	return (getStatusFlag(BRAKING));
}


bool L298::isAccelerating() {
	return (getStatusFlag(ACCELERATION));
}
#endif

#if defined(ACCELERATION_FUNCTIONS) || defined(LIMITING_FUNCTIONS) || defined(CURRENT_FUNCTIONS)
void L298::update() {	// this function should run on the main loop
#ifdef CURRENT_FUNCTIONS
	_voltageRead = analogRead(_currentPin);
	_currentAmps = _voltageRead / 1023. * _currentSpeed / 255.* _ampsMax; // do not confuse _currentSpeed which is the PWM value applied on enable pin, with amperage (current)

	if (_currentAmps >= _setAmps) {
		setStatusFlag(OVERCURRENT);
	}
	else {
		unsetStatusFlag(OVERCURRENT);
	}
#endif

#ifdef LIMITING_FUNCTIONS
	_checkDigitalLimits();
#endif

#ifdef POSITION_FUNCTIONS
	_currentPosition = analogRead(_positionPin);
#endif

#ifdef ACCELERATION_FUNCTIONS
	if (_currentSpeed != _targetSpeed) {
		_currentMillis = millis();
		if (_currentMillis - _previousMillis < _millisTarget) {
			_currentSpeed = _startingSpeed + _rampRate * double(_currentMillis - _previousMillis);
		}
		else {
			_currentSpeed = _targetSpeed;
		}
		_setMotionFlags();
		analogWrite(_enable, _currentSpeed);
	}
#endif
}
#endif


#ifdef POSITION_FUNCTIONS
void L298::setPositionPin(int pin) {
	_positionPin = pin;
}

int L298::getPosition() {
	return _currentPosition;
}
#endif

#ifdef LIMITING_FUNCTIONS
bool L298::checkCollision(bool limit) {
	return (limit ? getStatusFlag(LIMIT_CCW) : getStatusFlag(LIMIT_CW));
}

void L298::setLimitPins(unsigned char limitCWpin, unsigned char limitCCWpin) {
	if (!getStatusFlag(LIMITS)) {
		_limitCWpin = limitCWpin;
		_limitCCWpin = limitCCWpin;
	}
}

void L298::configLimits(unsigned char pullup) {
	if (!getStatusFlag(LIMITS)) {
		_pullup = pullup;
		setStatusFlag(LIMITS);	// set the limits flag
		if (pullup == NO_PULLUP || _pullup == EXTERNAL_PULLUP) {
			pinMode(_limitCWpin, INPUT);
			pinMode(_limitCCWpin, INPUT);
		}
		else if (pullup == INTERNAL_PULLUP) {
			pinMode(_limitCWpin, INPUT_PULLUP);
			pinMode(_limitCCWpin, INPUT_PULLUP);
		}
		else {
#ifdef DEBUG
			Serial.println("configLimits() invalid parameter.");
			Serial.println("Choose between: NO_PULLUP, EXTERNAL_PULLUP, INTERNAL_PULLUP");
#endif
		}
	}
}

void L298::_checkDigitalLimits() {
	if ((_pullup == INTERNAL_PULLUP) || (_pullup == EXTERNAL_PULLUP)) {	// if (_pullup)
 		if (digitalRead(_limitCWpin)) {
			unsetStatusFlag(LIMIT_CW);
		}
		else {
			brake(ON);
			setStatusFlag(LIMIT_CW);
		}
		if (digitalRead(_limitCCWpin)) {
			unsetStatusFlag(LIMIT_CCW);
		}
		else {
			brake(ON);
			setStatusFlag(LIMIT_CCW);
		}
	}
	else if (_pullup == NO_PULLUP) { // else if (!_pullup)
 		if (digitalRead(_limitCWpin)) {
			brake(ON);
			setStatusFlag(LIMIT_CW);
		}
		else {
			brake(ON);
			unsetStatusFlag(LIMIT_CW);
		}
		if (digitalRead(_limitCCWpin)) {
			brake(ON);
			setStatusFlag(LIMIT_CCW);
		}
		else {
			unsetStatusFlag(LIMIT_CCW);
		}
	}
	
	if (getStatusFlag(LIMIT_CW) && (_direction == CW)) {
		return;
	}
	else if (getStatusFlag(LIMIT_CW) && !(_direction == CW) && getStatusFlag(BRAKE_ON) && _targetSpeed) {
		brake(OFF);
	}
	if (getStatusFlag(LIMIT_CCW) && (_direction == CCW)) {
		return;
	}
	else if (getStatusFlag(LIMIT_CCW) && !(_direction == CCW) && getStatusFlag(BRAKE_ON) && _targetSpeed) {
		brake(OFF);
	}
}

#endif


#ifdef CURRENT_FUNCTIONS
void L298::configCurrentSense(unsigned char currentPin, double supplyVoltage, double senseResistor) {
	_currentPin = currentPin;
	if (senseResistor > 0) {
		_ampsMax = supplyVoltage / senseResistor;
	}
	else {
#ifdef DEBUG
	Serial.println("Sense resistor value should be > 0");
#endif
	}
}

double L298::getCurrent() {
	return _currentAmps;
}

void L298::setCurrent(double amperes) {
	if (amperes >= 0) {
		_setAmps = amperes;
	}
	else {
#ifdef DEBUG
	Serial.println("Set current should be >= 0");
#endif
	}
}

bool L298::checkCurrent() {
	return getStatusFlag(OVERCURRENT);
}
#endif
