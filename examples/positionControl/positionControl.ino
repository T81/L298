/*************************************************************************
  L298 Dual Full-Bridge Driver library
  by Christodoulos P. Lekkos <tolis81@gmail.com>

  Position Control

  This file is free software; you can redistribute it and/or modify
  it under the terms of either the GNU General Public License version 3
  published by the Free Software Foundation.
*************************************************************************/

#include <PID_v1.h>
#include <L298.h>
// change PID output limits and deal with direction....
L298 motor;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID motorPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  Serial.begin(9600);
  motor.begin(enablePin[thisMotor], directionPin[thisMotor][0], directionPin[thisMotor][1]);
  motor.setLimitPins(limitPin[thisMotor][0], limitPin[thisMotor][1]);
  motor.configLimitPins(EXTERNAL_PULLUPS);
  motor.setAccelerationTime(5000);

  Input = motor.getPosition();
  Setpoint = 512;
  motorPID.SetMode(AUTOMATIC);
}

void loop() {
  motor.update();
  Input = motor.getPosition();
  motorPID.Compute();
  motor.setSpeed(OUTPUT);
}