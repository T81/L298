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
  //  motor.setLimitPins(23, 25);
  //  motor.positionPin(A0);
  //  motor.setPositionLimits(0, 1023);
  motor.safeDirectionChange(OFF);
  //  motor.configLimitPins(INTERNAL_PULLUP);
  motor.setAccelerationTime(5000);
  motor.setSpeed(255);
}

void loop() {

  if (motor.getSpeed() == 255) {
    motor.coast();
    delay(3000);
    motor.setDirection(CCW);
    motor.setSpeed(0);
  }
  else if (!motor.getSpeed()) {
    motor.coast();

    motor.setDirection(CW);
    motor.setSpeed(255);
  }
  motor.update();

  //  Serial.println(motor.getSpeed());
  Serial.print(motor.isBrakeOn());
  Serial.print(" ");
  Serial.print(motor.isCoasting());
  Serial.print(" ");
  //  Serial.print(motor.checkCollision(CW));
  //  Serial.print(" ");
  //  Serial.print(motor.checkCollision(CCW));
  //  Serial.print(" ");
  //  Serial.print(motor.checkPositionLimit(CCW));
  //  Serial.print(" ");
  //  Serial.print(motor.checkPositionLimit(CCW));
  //  Serial.print(" ");
  //  Serial.print(motor.getPosition());
  //  Serial.print(" ");
  Serial.println(motor.getSpeed());
  delay(500);
}
