// Copyright (C) 2026 Alec Henderson
// This file is part of Arduino Reliable Compass.
//
// Arduino Reliable Compass is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option) any
// later version.
//
// Arduino Reliable Compass is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// Arduino Reliable Compass. If not, see <https://www.gnu.org/licenses/>.

#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>
#include <CompassCalibration.h>

void setup() {
    Serial.begin(500000);
    while (!Serial);
    if (!IMU.begin()) {
        Serial.println("IMU init failed");
        while (1);
    }
}

void loop() {
    quaternion_compass::writeCalibrationSamples(Serial);
}
