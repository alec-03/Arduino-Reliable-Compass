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

#include "CompassCalibration.h"
#include <Arduino_BMI270_BMM150.h>

void quaternion_compass::writeCalibrationSamples(Print &output) {
    float x, y, z;

    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);
        output.print("acc\t"); output.print(x, 5);
        output.print("\t");    output.print(y, 5);
        output.print("\t");    output.println(z, 5);
    }

    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(x, y, z);
        output.print("gyr\t"); output.print(x, 5);
        output.print("\t");    output.print(y, 5);
        output.print("\t");    output.println(z, 5);
    }

    if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(x, y, z);
        x = -x;  // BMM150 X axis reversed on Nano 33 BLE
        output.print("mag\t"); output.print(x, 5);
        output.print("\t");    output.print(y, 5);
        output.print("\t");    output.println(z, 5);
    }
}
