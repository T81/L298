/*************************************************************************
  L298 Dual Full-Bridge Driver library
  by Christodoulos P. Lekkos <tolis81@gmail.com>

  Unit test coverage

  This file is free software; you can redistribute it and/or modify
  it under the terms of either the GNU General Public License version 3
  published by the Free Software Foundation.
*************************************************************************/

#include <ArduinoUnit.h> // https://github.com/mmurdoch/arduinounit
#include <ArduinoUnitMock.h>
#include <L298.h>

L298 motor;

void setup() {
  motor.begin(2, 3, 4);
  motor.setLimits(5, 6, true);
  motor.setSpeed(255);
    Serial.begin(9600);
}

void loop() {
  motor.update();
  motor.setSpeed(127, 5000);
  Serial.println(motor.getSpeed());
  delay(1000);
}