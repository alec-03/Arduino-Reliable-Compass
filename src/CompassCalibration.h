#pragma once
#include <Arduino.h>

namespace quaternion_compass {
// Stream one poll of raw calibration samples (acc/gyr/mag) to the host tool.
void writeCalibrationSamples(Print &output);
}
