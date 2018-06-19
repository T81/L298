/*************************************************************************
  L298 Dual Full-Bridge Driver library
  by Christodoulos P. Lekkos <tolis81@gmail.com>

  Single motor

  This file is free software; you can redistribute it and/or modify
  it under the terms of either the GNU General Public License version 3
  published by the Free Software Foundation.
*************************************************************************/

#include <L298.h>

L298 motor;

void setup() {
  Serial.begin(9600);

  motor.begin(2, 24, 22);
  motor.setLimitPins(23, 25);
  motor.configLimitPins(EXTERNAL_PULLUPS);

  motor.setAccelerationTime(5000);
}

void loop() {
  if (motor.checkCollision(CW)) {
    delay(1000);
    motor.setDirection(CCW);
  }
  else if (motor.checkCollision(CCW)) {
    delay(1000);

    motor.setDirection(CW);
  }
  motor.setSpeed(255);
  motor.update();
  delay(10);
}