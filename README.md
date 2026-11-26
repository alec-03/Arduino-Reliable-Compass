# Reliable Digital Compass for Arduino

Quaternion-based heading fusion and calibration tools for Arduino projects.
The current hardware adapter targets the **Arduino Nano 33 BLE Sense Rev2**
(BMI270 + BMM150), but the fusion algorithm itself is portable to other boards
with an accelerometer, gyroscope, and magnetometer. Supporting another board
requires an adapter for its sensor API and coordinate frame.

Add this directory as a local library using your preferred Arduino build system.
`library.json` describes the hardware dependency for tools that support the
PlatformIO library format, but PlatformIO is not required.

```cpp
#include <Arduino_BMI270_BMM150.h>
#include <QuaternionCompass.h>

void setup() {
    quaternion_compass::Calibration calibration;
    // Insert the calibration tool's assignments here, specific to your device.
    quaternion_compass::setCalibration(calibration);
    if (!IMU.begin()) { /* handle hardware failure */ }
}

void loop() {
    const int heading = quaternion_compass::getHeading();
    if (heading != quaternion_compass::COMPASS_ERROR) {
        // heading is integer magnetic degrees clockwise from north, 0–359.
    }
}
```

See `examples/Heading/Heading.ino` for a runnable serial example.

## Heading contract

`getHeading()` consumes available sensor samples, updates quaternion fusion, and
returns the current heading without waiting for another sample. Call it regularly,
ideally at least 100 times per second, to service the gyro. Calls between samples
retain the estimate. Startup returns `COMPASS_ERROR` (400) until valid gravity and
five consistent, settled magnetic samples establish north.

The existing behavior is preserved: gyro integration continues between magnetic
updates; field-strength disturbances, fast motion, and stale gravity gate magnetic
corrections. After initialization, missing sensor data does **not** invalidate the
last fused heading. Use `getCompassDiagnostics()` to observe magnetic freshness
(`STALE` after 500 ms), rejected samples, and correction counts. This is magnetic
north; no declination correction is applied.

This is a single-IMU, single-caller library. `setCalibration()` resets fusion and
startup state. Identity calibration defaults are placeholders, not measurements.
`getAttitude()` also polls sensors and returns pitch/roll/yaw in radians;
`getAttitudeRate()` consumes an available gyro sample. Prefer just `getHeading()`
for the normal update path so another reader does not consume its samples.

## Coordinate conventions

The quaternion rotates body vectors into a right-handed world frame with +Z up.
Forward is Arduino IMU body +Y (printed board +X on the mounted Rev2). Calibration
magnetometer samples use `(-rawX, rawY, rawZ)`; offsets are subtracted before
scaling, then mapped into the accel/gyro body frame as `(-calY, -calX, calZ)`.
Accelerometer offsets are subtracted and a common scale normalizes gravity to 1g.
Gyro biases are in degrees/second. Other sensors or mounting axes require an
adapter/change to these conventions; they are not automatically detected.

## Calibration

1. Upload `examples/Calibration/Calibration.ino` to the board. It writes raw
   samples through the board's serial interface using
   `writeCalibrationSamples(Print&)` from `CompassCalibration.h`.
2. Install Python dependencies in a virtual environment:
   `python -m pip install -r tools/requirements.txt`.
3. Run `python tools/calibrate.py --collect --fit --port YOUR_SERIAL_PORT --baud YOUR_BAUD_RATE`.
   Choose the port and baud rate reported by your IDE or operating system.
   Keep still for the first 30 seconds, then slowly rotate through all orientations.
4. Paste the emitted `Calibration` assignments into your application before
   `setCalibration()`. The fit uses a sphere center/radius for magnetometer and
   accelerometer, and mean stationary gyro bias. It is not a full soft-iron
   ellipsoid fit.

Use `--no-plot` for headless operation. Saved input is supported with
`--fit --input cal_data.txt`; saved gyro rows must represent stationary samples.

## Verification and release status

Run `python3 tests/calibration_test.py` to check the calibration fit and the
generated C++ assignments. Hardware behavior still needs a physical-board check;
simulated tests cannot establish real-world calibration accuracy.

## License

Copyright (C) 2026 Alec.

This project is licensed under the [GNU General Public License v3.0](LICENSE).
You may use, modify, and redistribute it under that license's terms. There is
no warranty.
