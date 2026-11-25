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
