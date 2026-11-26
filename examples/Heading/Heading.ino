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

#include <Arduino_BMI270_BMM150.h>
#include <QuaternionCompass.h>

void setup() {
    Serial.begin(115200);
    quaternion_compass::Calibration calibration;
    // Replace identity defaults with the output of tools/calibrate.py.
    quaternion_compass::setCalibration(calibration);
    if (!IMU.begin()) {
        Serial.println("IMU init failed");
        while (true) delay(100);
    }
}

void loop() {
    const int heading = quaternion_compass::getHeading();
    static unsigned long printedAt = 0;
    if (millis() - printedAt >= 100) {
        printedAt = millis();
        if (heading != quaternion_compass::COMPASS_ERROR) Serial.println(heading);
    }
}
