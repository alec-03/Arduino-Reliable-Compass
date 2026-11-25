# QuaternionCompass

Quaternion compass and calibration tools for the **Arduino Nano 33 BLE Sense
Rev2 (BMI270 + BMM150)**. The library has no display, Bluetooth, radio, or goggles
application dependency. Copy this directory into a PlatformIO project's `lib/`
folder; `library.json` declares the Arduino IMU dependency.

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

1. Run `examples/Calibration/Calibration.ino` on the board, or build this parent
   project's `calibrate` firmware environment. Both use
   `writeCalibrationSamples(Print&)` from `CompassCalibration.h`.
2. Install Python dependencies in a virtual environment:
   `python -m pip install -r tools/requirements.txt`.
3. Run `python tools/calibrate.py --collect --fit --port /dev/ttyACM0 --baud 500000`.
   Keep still for the first 30 seconds, then slowly rotate through all orientations.
4. Paste the emitted `Calibration` assignments into your application before
   `setCalibration()`. The fit uses a sphere center/radius for magnetometer and
   accelerometer, and mean stationary gyro bias. It is not a full soft-iron
   ellipsoid fit.

Use `--no-plot` for headless operation. Saved input is supported with
`--fit --input cal_data.txt`; saved gyro rows must represent stationary samples.
The parent project's `calibrate.py` remains a forwarding entry point. Its board
profiles live in `src/model/services/sensors/boardCalibration.h`, outside this
library, with standalone firmware using prototype B's profile.

## Verification and release status

From the parent repository, run `python3 test/sensors/run.py` and repeat with
`PROTOTYPE_ROLE=1`. These exercise the library using simulated sensor timing,
startup, tilt, field rejection, drift, fast turns, calibration frames, and reset.
Calibration checks: `python3 lib/QuaternionCompass/tests/calibration_test.py`.
Firmware builds: `pio run -e nano33ble -e prototype_a -e prototype_b -e calibrate`.

This is a local extraction, not a published release. A distribution license has
not been selected. Hardware behavior still needs a physical-board check after
refactoring; simulated tests cannot establish real-world calibration accuracy.
