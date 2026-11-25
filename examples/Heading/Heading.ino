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
