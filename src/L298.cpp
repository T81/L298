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
	_directionRestriction = true;
	coast();
	setDirection(CW);
#ifdef ANALOG_FUNCTIONS
	//setPositionLimits(0, 1023);
#endif	
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


void L298::safeDirectionChange(bool directionRestriction) {
		_directionRestriction = directionRestriction;
}


void L298::setDirection(bool direction) {
	if (_directionRestriction) {
		if (_direction != direction) {
			if (_currentSpeed) {
/* #ifdef DEBUG
				Serial.println("Motor should be stopped in order to change direction.");
#endif */
				return;
			}
		}
	
	}
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
#ifdef DIGITAL_FUNCTIONS
	if (!getStatusFlag(DIGITAL_LIMIT)) {
#ifdef DEBUG
	Serial.println("Error @ setSpeed():  Use setLimitPins() first or disable DIGITAL_FUNCTIONS");
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
	unsetStatusFlag(RUNNING | COASTING | ACCELERATION | DECELERATION | BRAKING);
	}
	else {
		if (_currentSpeed || _targetSpeed) {
			unsetStatusFlag(COASTING);
			setStatusFlag(RUNNING);
#ifdef ACCELERATION_FUNCTIONS
			if (_currentSpeed < _targetSpeed) {
				unsetStatusFlag(DECELERATION);
				setStatusFlag(ACCELERATION);
			}
			else if (_currentSpeed > _targetSpeed) {
				unsetStatusFlag(ACCELERATION);
				setStatusFlag(DECELERATION);
			}
			else {
				unsetStatusFlag(ACCELERATION | DECELERATION);
			}
#endif
		}
		else {
			unsetStatusFlag(RUNNING);
#ifdef ACCELERATION_FUNCTIONS
			unsetStatusFlag(ACCELERATION);
			unsetStatusFlag(DECELERATION);
#endif
			setStatusFlag(COASTING);
		}
	}
}


#ifdef ACCELERATION_FUNCTIONS
void L298::setAccelerationTime(unsigned long milliSeconds) {
	_millisTarget = milliSeconds;
}

bool L298::isAccelerating() {
	return (getStatusFlag(ACCELERATION));
}

bool L298::isDecelerating() {
	return (getStatusFlag(DECELERATION));
}
#endif


#if defined(ACCELERATION_FUNCTIONS) || defined(DIGITAL_FUNCTIONS) || defined(CURRENT_FUNCTIONS)
void L298::update() {	// this function should run on the main loop
#ifdef CURRENT_FUNCTIONS
	_vSense = analogRead(_sensePin);
	// _currentAmps = _vSense / 1023. * _currentSpeed / 255.* _ampsMax; // do not confuse _currentSpeed which is the PWM value applied on enable pin, with amperage (current)
	_currentAmps = _vSense / 1023. * _currentSpeed / 255.* _ampsMax; // do not confuse _currentSpeed which is the PWM value applied on enable pin, with amperage (current)

	if (_currentAmps >= _setAmps) {
		setStatusFlag(OVERCURRENT);
	}
	else {
		unsetStatusFlag(OVERCURRENT);
	}
	if (_currentAmps > _maxCurrent) {
		_maxCurrent = _currentAmps;
	}
#endif

#ifdef DIGITAL_FUNCTIONS
	_checkDigitalLimits();
	//_checkAnalogLimits();
#endif

#ifdef ANALOG_FUNCTIONS
	_currentPosition = analogRead(_positionPin);
	_checkAnalogLimits();
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


#ifdef ANALOG_FUNCTIONS
void L298::positionPin(unsigned char pin) {
	_positionPin = pin;
}

void L298::setPositionLimits(int lowerLimit, int upperLimit) {
	if ((lowerLimit >= 0) && (lowerLimit < 1023) && (upperLimit > 0) && (upperLimit <= 1023)) {
		if (lowerLimit < upperLimit) {
			setStatusFlag(ANALOG_LIMIT);
			_lowerLimit = lowerLimit;
			_upperLimit = upperLimit;
		}
		else {
#ifdef DEBUG
			Serial.println("Error: lowerLimit >= upperLimit.");
#endif
		}
	}
	else {
#ifdef DEBUG
		Serial.println("Error: lowerLimit and upperLimit must be in 0 - 1023 range.");
#endif
	}
}


int L298::getPosition() {
	return _currentPosition;
}

void L298::_checkAnalogLimits() {
	if (getStatusFlag(ANALOG_LIMIT)) {
		if (_currentPosition <= _lowerLimit ) {
			if (_direction == CW) {
				brake(ON);
			}
			setStatusFlag(ANALOG_LIMIT_CW);
		}
		else {
			unsetStatusFlag(ANALOG_LIMIT_CW);
		}
		if (_currentPosition >= _upperLimit ) {
			if (_direction == CCW) {
				brake(ON);
			}
			setStatusFlag(ANALOG_LIMIT_CCW);
		}
		else {
			unsetStatusFlag(ANALOG_LIMIT_CCW);
		}
	}
	else {
		unsetStatusFlag(ANALOG_LIMIT_CW | ANALOG_LIMIT_CCW);
	}
}

void L298::analogLimits(bool enable) {
	enable ? setStatusFlag(ANALOG_LIMIT) : unsetStatusFlag(ANALOG_LIMIT); 
}
#endif

#ifdef DIGITAL_FUNCTIONS
bool L298::checkCollision(bool limit) {
	return (limit ? getStatusFlag(DIGITAL_LIMIT_CCW) : getStatusFlag(DIGITAL_LIMIT_CW));
}

void L298::setLimitPins(unsigned char limitCWpin, unsigned char limitCCWpin) {
	if (!getStatusFlag(DIGITAL_LIMIT)) {
		_limitCWpin = limitCWpin;
		_limitCCWpin = limitCCWpin;
	}
}

void L298::configLimitPins(unsigned char pullup) {
	if (!getStatusFlag(DIGITAL_LIMIT)) {
		_pullup = pullup;
		setStatusFlag(DIGITAL_LIMIT);	// set the limits flag
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
			Serial.println("configLimitPins() invalid parameter.");
			Serial.println("Choose between: NO_PULLUP, EXTERNAL_PULLUP, INTERNAL_PULLUP");
#endif
		}
	}
}

void L298::_checkDigitalLimits() {
	if ((_pullup == INTERNAL_PULLUP) || (_pullup == EXTERNAL_PULLUP)) {	// if (_pullup)
 		if (digitalRead(_limitCWpin)) {
			unsetStatusFlag(DIGITAL_LIMIT_CW);
		}
		else {
			brake(ON);
			setStatusFlag(DIGITAL_LIMIT_CW);
		}
		if (digitalRead(_limitCCWpin)) {
			unsetStatusFlag(DIGITAL_LIMIT_CCW);
		}
		else {
			brake(ON);
			setStatusFlag(DIGITAL_LIMIT_CCW);
		}
	}
	else if (_pullup == NO_PULLUP) { // else if (!_pullup)
 		if (digitalRead(_limitCWpin)) {
			brake(ON);
			setStatusFlag(DIGITAL_LIMIT_CW);
		}
		else {
			brake(ON);
			unsetStatusFlag(DIGITAL_LIMIT_CW);
		}
		if (digitalRead(_limitCCWpin)) {
			brake(ON);
			setStatusFlag(DIGITAL_LIMIT_CCW);
		}
		else {
			unsetStatusFlag(DIGITAL_LIMIT_CCW);
		}
	}
	if (getStatusFlag(DIGITAL_LIMIT_CW) && (_direction == CW)) {
		return;
	}
	else if (getStatusFlag(DIGITAL_LIMIT_CW) && !(_direction == CW) && getStatusFlag(BRAKE_ON) && _targetSpeed) {
		brake(OFF);
	}
	if (getStatusFlag(DIGITAL_LIMIT_CCW) && (_direction == CCW)) {
		return;
	}
	else if (getStatusFlag(DIGITAL_LIMIT_CCW) && !(_direction == CCW) && getStatusFlag(BRAKE_ON) && _targetSpeed) {
		brake(OFF);
	}
}

#endif


#ifdef CURRENT_FUNCTIONS
void L298::configCurrentSense(unsigned char sensePin, double supplyVoltage, double senseResistor) {
	_sensePin = sensePin;
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

bool L298::checkOvercurrent() {
	return getStatusFlag(OVERCURRENT);
}

double L298::maxCurrent() {
	return _maxCurrent;
}

void L298::resetCurrent() {
	_maxCurrent = 0;
}
#endif
