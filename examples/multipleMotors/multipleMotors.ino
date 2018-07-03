/*************************************************************************
  L298 Dual Full-Bridge Driver library
  by Christodoulos P. Lekkos <tolis81@gmail.com>

  Multiple Motors

  This file is free software; you can redistribute it and/or modify
  it under the terms of either the GNU General Public License version 3
  published by the Free Software Foundation.
*************************************************************************/

#include <L298.h>

const byte numberOfMotors = 3;

const byte enablePin[numberOfMotors] {
  2,
  3,
  4
};

const byte directionPin[numberOfMotors][2] {
  {22, 24},
  {26, 28},
  {30, 32}
};

const byte limitPin[numberOfMotors][2] {
  {23, 25},
  {27, 29},
  {31, 33}
};

L298 motor[numberOfMotors] = {
  L298(),
  L298(),
  L298()
};

void setup() {
  Serial.begin(9600);
  for (int thisMotor = 0; thisMotor < numberOfMotors; thisMotor++) {
    motor[thisMotor].begin(enablePin[thisMotor], directionPin[thisMotor][0], directionPin[thisMotor][1]);
    motor[thisMotor].setLimitPins(limitPin[thisMotor][0], limitPin[thisMotor][1]);
    motor[thisMotor].configLimitPins(EXTERNAL_PULLUPS);
    motor[thisMotor].setAccelerationTime(5000);
  }
}

void loop() {
  for (int thisMotor = 0; thisMotor < numberOfMotors; thisMotor++) {
    if (motor[thisMotor].checkCollision(CW)) {
      delay(1000);
      motor[thisMotor].setDirection(CCW);
    }
    else if (motor[thisMotor].checkCollision(CCW)) {
      delay(1000);
      motor[thisMotor].setDirection(CW);
    }
    motor[thisMotor].setSpeed(255);
    motor[thisMotor].update();
  }
  delay(10);
}