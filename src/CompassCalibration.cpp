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
