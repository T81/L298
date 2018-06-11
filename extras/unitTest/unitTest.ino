/*************************************************************************
  L298 Dual Full-Bridge Driver library
  by Christodoulos P. Lekkos <tolis81@gmail.com>

  Unit test coverage

  This file is free software; you can redistribute it and/or modify
  it under the terms of either the GNU General Public License version 3
  published by the Free Software Foundation.
*************************************************************************/

#include <ArduinoUnit.h> // https://github.com/mmurdoch/arduinounit
//#include <ArduinoUnitMock.h>

  /******************************************************************
  Choose the functions you want to use before including the library
*******************************************************************/
//#define ACCELERATION_FUNCTIONS
//#define CURRENT_FUNCTIONS
//#define LIMITS_FUNCTIONS
//#define POSITION_FUNCTIONS
//
//// uncomment to enable debugging messages
//#define DEBUG
/*******************************************************************/


#include <L298.h>

L298 motor;


// constructor
test(1_0_constructor_should_begin_stopped) {
  Serial.println("");
  Serial.println("Initialization tests");
  Serial.println("====================");
  assertFalse(motor.isRunning());
}


test(1_1_constructor_should_begin_with_zero_speed) {
  assertFalse(motor.getSpeed());
}


test(1_2_constructor_should_begin_with_direction_set_to_cw) {
  assertFalse(motor.getDirection());
}


test(1_3_constructor_should_begin_coasting) {
  assertTrue(motor.isCoasting());
}

test(1_4_constructor_should_begin_with_brake_off) {
  assertFalse(motor.isBrakeOn());
}

test(1_5_constructor_should_begin_with_acceleration_flag_off) {
  assertFalse(motor.isAccelerating());
}


test(1_6_constructor_should_begin_with_deceleration_flag_off) {
  assertFalse(motor.isDecelerating());
}

// flags
test(2_0_0_running_flag_on) {
  Serial.println("");
  Serial.println("Flags tests");
  Serial.println("===========");

  motor.setAccelerationTime(0);
  motor.setSpeed(0);
  motor.setSpeed(127);
  assertTrue(motor.isRunning());

  motor.setSpeed(0);
  motor.setAccelerationTime(250);
  motor.setSpeed(127);
  assertTrue(motor.isRunning());

  delay(250);
  motor.update();
  assertTrue(motor.isRunning());
}


test(2_0_1_running_flag_off) {
  motor.setAccelerationTime(0);
  motor.setSpeed(127);
  motor.setSpeed(0);

  assertFalse(motor.isRunning());


  motor.setSpeed(127);
  motor.brake(ON);

  assertFalse(motor.isRunning());


  motor.brake(OFF);
  motor.setSpeed(127);
  motor.coast();

  assertFalse(motor.isRunning());
}


test(2_1_0_coast_flag_on) {
  motor.setSpeed(127);
  motor.setSpeed(0);

  assertTrue(motor.isCoasting());


  motor.setSpeed(127);
  motor.coast();

  assertTrue(motor.isCoasting());
}

test(2_1_1_coast_flag_off) {
  motor.setSpeed(127);
  assertFalse(motor.isCoasting());


  motor.brake(ON);
  assertFalse(motor.isCoasting());
}

test(2_2_0_brake_on_flag) {
  motor.brake(ON);
  assertTrue(motor.isBrakeOn());

  motor.brake(OFF);
  motor.setSpeed(127);
  motor.brake(ON);

  assertTrue(motor.isBrakeOn());


  motor.brake(OFF);
  motor.setAccelerationTime(250);
  motor.setSpeed(127);
  delay(50);
  motor.update();
  motor.brake(ON);

  assertTrue(motor.isBrakeOn());
}


test(2_2_1_brake_off_flag) {
  motor.brake(OFF);
  assertFalse(motor.isBrakeOn());
}

test(2_3_0_acceleration_flag_on) {
  motor.setAccelerationTime(0);
  motor.setSpeed(0);
  motor.setAccelerationTime(250);
  motor.setSpeed(127);
  assertTrue(motor.isAccelerating());
  delay(50);
  motor.update();
  assertTrue(motor.isAccelerating());
}

test(2_3_1_acceleration_flag_off) {
  motor.setAccelerationTime(0);
  motor.setSpeed(0);

  assertFalse(motor.getSpeed());
  assertFalse(motor.isAccelerating());

  motor.setSpeed(127);
  assertEqual(motor.getSpeed(), 127);
  assertFalse(motor.isAccelerating());

  motor.setAccelerationTime(250);
  motor.setSpeed(255);
  delay(251); // CHECKMEAGAIN
  motor.update();
  assertEqual(motor.getSpeed(), 255);
  assertFalse(motor.isAccelerating());

  motor.setSpeed(0);
  delay(250);
  motor.update();
  assertFalse(motor.isAccelerating());
}



test(2_4_0_deceleration_flag_off) {
  motor.setAccelerationTime(0);
  motor.setSpeed(0);
  assertFalse(motor.isDecelerating());

  motor.setSpeed(255);
  assertFalse(motor.isDecelerating());

  motor.setAccelerationTime(250);
  motor.setSpeed(127);
  delay(250);
  motor.update();
  assertFalse(motor.isDecelerating());

  motor.setSpeed(0);
  delay(250);
  motor.update();
  assertFalse(motor.isDecelerating());
}

test(2_4_1_acceleration_flag_on) {
  motor.setAccelerationTime(0);
  motor.setSpeed(127);
  motor.setAccelerationTime(250);
  motor.setSpeed(0);
  assertTrue(motor.isDecelerating());
  delay(50);
  motor.update();
  assertTrue(motor.isDecelerating());
}

test(2_5_0_direction_restriction_on) {
  motor.setAccelerationTime(0);
  motor.setSpeed(0);
  motor.setDirection(CW);
  assertFalse(motor.getDirection());
  motor.setSpeed(127);
  motor.setDirection(CCW);
  assertFalse(motor.getDirection());
}

test(2_5_1_direction_restriction_off) {
  motor.setAccelerationTime(0);
  motor.setSpeed(0);
  motor.setDirection(CW);
  assertFalse(motor.getDirection());
  motor.setSpeed(127);
  motor.safeDirectionChange(OFF);
  motor.setDirection(CCW);
  assertTrue(motor.getDirection());
  motor.safeDirectionChange(ON);
}

test(2_5_2_direction_flag) {
  motor.setAccelerationTime(0);
  motor.setSpeed(0);

  motor.setDirection(CCW);
  assertTrue(motor.getDirection());

  motor.setDirection(CW);
  assertFalse(motor.getDirection());

  motor.setDirection(FORWARD);
  assertTrue(motor.getDirection());

  motor.setDirection(BACKWARDS);
  assertFalse(motor.getDirection());


  motor.setDirection(UP);
  assertTrue(motor.getDirection());

  motor.setDirection(DOWN);
  assertFalse(motor.getDirection());

  motor.setDirection(RIGHT);
  assertTrue(motor.getDirection());

  motor.setDirection(LEFT);
  assertFalse(motor.getDirection());
}

test(7_7_0_limits_flags) {
  assertFalse(motor.checkCollision(CW));
  assertFalse(motor.checkCollision(CCW));
}


// motor motion
test(3_0_constant_speed) {
  Serial.println("");
  Serial.println("Motion tests");
  Serial.println("============");

  motor.setAccelerationTime(0);
  motor.setSpeed(0);

  assertFalse(motor.getSpeed());

  //  assertFalse(motor.isAccelerating());
  assertFalse(motor.isBrakeOn());
  assertFalse(motor.isDecelerating());
  assertTrue(motor.isCoasting()); // True
  assertFalse(motor.isRunning());


  motor.setSpeed(127);

  assertEqual(motor.getSpeed(), 127);

  assertFalse(motor.isAccelerating());
  assertFalse(motor.isBrakeOn());
  assertFalse(motor.isDecelerating());
  assertFalse(motor.isCoasting());
  assertTrue(motor.isRunning()); // True
}

test(3_1_acceleration_deceleration) {
  motor.setAccelerationTime(0);
  motor.setSpeed(0);
  motor.setAccelerationTime(250);
  motor.setSpeed(255);

  assertTrue(motor.isAccelerating());
  assertFalse(motor.isBrakeOn());
  assertFalse(motor.isDecelerating());
  assertFalse(motor.isCoasting());
  assertTrue(motor.isRunning());


  delay(125); // 125 : 250
  motor.update();

  assertLess(motor.getSpeed(), 255);
  assertTrue(motor.isAccelerating());
  assertFalse(motor.isBrakeOn());
  assertFalse(motor.isDecelerating());
  assertFalse(motor.isCoasting());
  assertTrue(motor.isRunning());


  delay(125); // 250 : 250
  motor.update();

  assertEqual(motor.getSpeed(), 255);
  assertFalse(motor.isAccelerating());
  assertFalse(motor.isBrakeOn());
  assertFalse(motor.isDecelerating());
  assertFalse(motor.isCoasting());
  assertTrue(motor.isRunning());


  motor.setSpeed(0); // 0 : 250

  assertFalse(motor.isAccelerating());
  assertFalse(motor.isBrakeOn());
  assertTrue(motor.isDecelerating());
  assertFalse(motor.isCoasting());
  assertTrue(motor.isRunning());

  delay(125); //  125 : 250
  motor.update();
  assertTrue(motor.isRunning());
  assertTrue(motor.isDecelerating());
  assertFalse(motor.isCoasting());
  assertFalse(motor.isAccelerating());
  assertFalse(motor.isBrakeOn());


  delay(126); // CHECKMEAGAIN  250 : 250
  motor.update();

  assertFalse(motor.getSpeed());

  assertFalse(motor.isAccelerating());
  assertFalse(motor.isBrakeOn());
  assertFalse(motor.isDecelerating());
  assertTrue(motor.isCoasting());
  assertFalse(motor.isRunning());
}

test(3_2_0_constant_speed_brake_on_off) {
  motor.setAccelerationTime(0);
  motor.setSpeed(127);
  assertEqual(motor.getSpeed(), 127);
  assertFalse(motor.isAccelerating());
  assertFalse(motor.isBrakeOn());
  assertFalse(motor.isDecelerating());
  assertFalse(motor.isCoasting());
  assertTrue(motor.isRunning());
  motor.brake(ON);

  assertFalse(motor.isAccelerating());
  assertTrue(motor.isBrakeOn());
  assertFalse(motor.isDecelerating());
  assertFalse(motor.isCoasting());
  assertFalse(motor.isRunning());


  motor.brake(OFF);

  assertFalse(motor.getSpeed());
  assertFalse(motor.isAccelerating());
  assertFalse(motor.isBrakeOn());
  assertFalse(motor.isDecelerating());
  assertTrue(motor.isCoasting());
  assertFalse(motor.isRunning());
}

test(3_2_1_acceleration_brake_on_off) {
  motor.setAccelerationTime(250);
  motor.setSpeed(127);
  delay(125);
  motor.update();
  motor.brake(ON);

  assertFalse(motor.getSpeed());
  assertFalse(motor.isAccelerating());
  assertTrue(motor.isBrakeOn());
  assertFalse(motor.isDecelerating());
  assertFalse(motor.isCoasting());
  assertFalse(motor.isRunning());


  motor.brake(OFF);
}

test(3_2_2_deceleration_brake_on_off) {

  motor.setAccelerationTime(0);
  motor.setSpeed(255);

  assertEqual(motor.getSpeed(), 255);
  assertFalse(motor.isAccelerating());
  assertFalse(motor.isBrakeOn());
  assertFalse(motor.isDecelerating());
  assertFalse(motor.isCoasting());
  assertTrue(motor.isRunning());

  motor.setAccelerationTime(250);
  motor.setSpeed(0);
  delay(125);
  motor.update();
  motor.brake(ON);

  assertFalse(motor.getSpeed());
  assertFalse(motor.isAccelerating());
  assertTrue(motor.isBrakeOn());
  assertFalse(motor.isDecelerating());
  assertFalse(motor.isCoasting());
  assertFalse(motor.isRunning());

  motor.brake(OFF);
}

test(4_0_check_timing_consistency) {
  Serial.println("");
  Serial.println("Timing test");
  Serial.println("============");

  motor.setAccelerationTime(0);
  motor.setSpeed(0);
  assertFalse(motor.getSpeed());
  motor.setAccelerationTime(1000);
  motor.setSpeed(255);
  delay(500);
  motor.update();

  assertEqual(motor.getSpeed(), 127);


  motor.coast();
  motor.setAccelerationTime(0);
  motor.setSpeed(255);
  motor.setAccelerationTime(1000);
  motor.setSpeed(0);
  delay(500);
  motor.update();

  assertEqual(motor.getSpeed(), 127);
}



void setup() {
  Serial.begin(9600);
  while (!Serial);  // Portability for Leonardo/Micro
  Serial.println("");

  motor.begin(2, 3, 4);

#ifdef ACCELERATION_FUNCTIONS
  Serial.println("ACCELERATING FUNCTIONS ENABLED");
#else
  Serial.println("ACCELERATING FUNCTIONS DISABLED");
  Test::exclude("*acceleration*");
  Test::exclude("*deceleration*");
#endif

#ifdef LIMITS_FUNCTIONS
  motor.setLimitPins(5, 6);
  motor.configLimits(INTERNAL_PULLUP);
  Serial.println("LIMITING FUNCTIONS ENABLED");
#else
  Serial.println("LIMITING FUNCTIONS DISABLED");
#endif

#ifdef CURRENT_FUNCTIONS
  Serial.println("CURRENT FUNCTIONS ENABLED");
#else
  Serial.println("LIMITING FUNCTIONS DISABLED");
#endif


}

void loop() {
  Test::run();
}