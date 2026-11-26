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
#include <Arduino.h>

namespace quaternion_compass {
// Stream one poll of raw calibration samples (acc/gyr/mag) to the host tool.
void writeCalibrationSamples(Print &output);
}
