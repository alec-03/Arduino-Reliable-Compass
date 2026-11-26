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

#pragma once
#include <stdint.h>

// Arduino Nano 33 BLE Sense Rev2: BMI270 + BMM150, body +Y forward.
namespace quaternion_compass {
constexpr int COMPASS_ERROR = 400;

struct Calibration {
    float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
    float magOffsetX = 0, magOffsetY = 0, magOffsetZ = 0;
    float magScaleX = 1, magScaleY = 1, magScaleZ = 1;
    float accOffsetX = 0, accOffsetY = 0, accOffsetZ = 0;
    float accScale = 1;
};

// Set before polling. Reconfiguration resets fusion and diagnostic state.
void setCalibration(const Calibration &calibration);
// Poll available IMU samples and return magnetic heading [0, 360), or
// COMPASS_ERROR until gravity and five consistent magnetic samples are ready.
// Call frequently (at least the gyro sample rate); this never waits for data.
int getHeading();

struct Attitude {
    float pitch, roll, yaw; // radians
    Attitude(float p, float r, float y) : pitch(p), roll(r), yaw(y) {}
};
struct CompassDiagnostics {
    uint32_t samples = 0, corrections = 0, lastSampleUs = 0;
    float rawX = 0, rawY = 0, rawZ = 0;
    float field = 0;
    int magneticHeading = COMPASS_ERROR;
    const char *status = "WAIT";
};
CompassDiagnostics getCompassDiagnostics();
Attitude getAttitude();
Attitude getAttitudeRate();
bool read_magnetometer(float &x, float &y, float &z);
bool read_accelerometer(float &x, float &y, float &z);
bool read_gyroscope(float &x, float &y, float &z);
} // namespace quaternion_compass
