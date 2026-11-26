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

#include "QuaternionCompass.h"
#include <Arduino_BMI270_BMM150.h>
#include <math.h>

namespace quaternion_compass {
static Calibration calibration;

// --- filter constants ---
// Correction gain: how strongly the accelerometer nudges orientation toward
// measured gravity per update, when trusted. Analogous to the old
// GYRO_ACCEL_WEIGHT, but applied as a quaternion correction, not a raw
// Euler-angle blend, so it can't leak into the wrong axis.
constexpr float ACCEL_CORRECTION_GAIN       = 0.02f;
constexpr float ACCEL_CORRECTION_GAIN_LOW   = 0.005f; // used when accel is untrustworthy (non-1g)

constexpr float MAG_CORRECTION_TIME_S = 2.0f;
constexpr uint32_t GRAVITY_MAX_AGE_US = 250000;
constexpr unsigned MAG_STARTUP_SAMPLES = 5;
constexpr float MAG_FIELD_TOLERANCE  = 0.3f;   // trust gate relative to the initial calibrated field magnitude
constexpr float MAG_CORRECTION_MAX_ANGULAR_SPEED = 45.0f * DEG_TO_RAD;
constexpr uint32_t MAG_CORRECTION_SETTLE_US = 150000;

// Printed board +X is physical forward. In the mounted Nano 33 BLE Rev2,
// Arduino's transformed IMU frame reports that direction as body +Y.
constexpr float FORWARD_X = 0.0f;
constexpr float FORWARD_Y = 1.0f;
constexpr float FORWARD_Z = 0.0f;
static CompassDiagnostics diagnostics;
static float measuredUpX = 0, measuredUpY = 0, measuredUpZ = 1;

CompassDiagnostics getCompassDiagnostics() {
    CompassDiagnostics result = diagnostics;
    if (result.samples && static_cast<uint32_t>(micros() - result.lastSampleUs) > 500000) {
        result.status = "STALE";
        result.magneticHeading = COMPASS_ERROR;
    }
    return result;
}

// ------------------------------------------------------------------ //
//  raw sensor reads
// ------------------------------------------------------------------ //

bool read_magnetometer(float &x, float &y, float &z) {
    if (!IMU.magneticFieldAvailable()) return false;
    if (!IMU.readMagneticField(x, y, z)) return false;
    diagnostics.rawX = x; diagnostics.rawY = y; diagnostics.rawZ = z;
    // Calibration is stored in the collection frame (-raw X, raw Y, raw Z).
    // Rotate that frame into the board frame used by Arduino's accel/gyro.
    // Keeping calibration and alignment separate preserves existing per-device
    // constants while giving all three sensors one right-handed body frame.
    const float calibrated_x = (-x - calibration.magOffsetX) * calibration.magScaleX;
    const float calibrated_y = ( y - calibration.magOffsetY) * calibration.magScaleY;
    const float calibrated_z = ( z - calibration.magOffsetZ) * calibration.magScaleZ;
    x = -calibrated_y;
    y = -calibrated_x;
    z =  calibrated_z;
    return true;
}

bool read_gyroscope(float &x, float &y, float &z) {
    return IMU.gyroscopeAvailable() && IMU.readGyroscope(x, y, z);
}

bool read_accelerometer(float &x, float &y, float &z) {
    if (!IMU.accelerationAvailable()) return false;
    if (!IMU.readAcceleration(x, y, z)) return false;
    x = (x - calibration.accOffsetX) * calibration.accScale;
    y = (y - calibration.accOffsetY) * calibration.accScale;
    z = (z - calibration.accOffsetZ) * calibration.accScale;
    return true;
}

// ------------------------------------------------------------------ //
//  quaternion math
// ------------------------------------------------------------------ //
//
// Convention: q represents the rotation that takes a vector from the BODY
// frame to the WORLD frame (world_v = q * body_v * q^-1). World frame is
// assumed right-handed with +Z up (gravity = (0, 0, 1) when level, matching
// a normalized accelerometer reading at rest).

struct Quaternion {
    float w, x, y, z;

    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    Quaternion operator*(const Quaternion &o) const {
        return Quaternion(
            w*o.w - x*o.x - y*o.y - z*o.z,
            w*o.x + x*o.w + y*o.z - z*o.y,
            w*o.y - x*o.z + y*o.w + z*o.x,
            w*o.z + x*o.y - y*o.x + z*o.w
        );
    }

    void normalize() {
        float n = sqrtf(w*w + x*x + y*y + z*z);
        if (n < 1e-9f) { w = 1; x = y = z = 0; return; }
        float inv = 1.0f / n;
        w *= inv; x *= inv; y *= inv; z *= inv;
    }

    Quaternion conjugate() const { return Quaternion(w, -x, -y, -z); }

    // rotate a body-frame vector into world frame
    void rotateToWorld(float bx, float by, float bz, float &wx, float &wy, float &wz) const {
        Quaternion v(0, bx, by, bz);
        Quaternion r = (*this) * v * conjugate();
        wx = r.x; wy = r.y; wz = r.z;
    }
};

// ------------------------------------------------------------------ //
//  helpers
// ------------------------------------------------------------------ //

static inline float normalizeHeading(float h) {
    while (h >= 360.0f) h -= 360.0f;
    while (h <    0.0f) h += 360.0f;
    return h;
}

// ------------------------------------------------------------------ //
//  attitude (quaternion-based)
// ------------------------------------------------------------------ //

// Persistent orientation state. Identity = sensor level, facing the
// arbitrary reference "north" of whatever frame it was initialized in
// (heading itself comes from the magnetometer separately, in getHeading).
static Quaternion g_orientation;
static bool g_orientation_init = false;

static bool  g_mag_ref_init = false;
static float g_mag_ref_x, g_mag_ref_y, g_mag_ref_z; // fixed world-frame reference field direction
static float g_mag_ref_magnitude;
static float g_angular_speed = 0.0f;
static bool g_fast_rotation_seen = false;
static uint32_t g_last_fast_rotation_us = 0;
static uint32_t g_last_gravity_us = 0;
static uint32_t g_last_mag_us = 0;
static unsigned g_mag_startup_count = 0;
static float g_mag_sum_x = 0, g_mag_sum_y = 0, g_mag_sum_magnitude = 0;

static uint32_t last_time = 0;

void setCalibration(const Calibration &value) {
    calibration = value;
    g_orientation = Quaternion();
    g_orientation_init = g_mag_ref_init = false;
    g_mag_ref_x = g_mag_ref_y = g_mag_ref_z = g_mag_ref_magnitude = 0;
    g_angular_speed = 0;
    g_fast_rotation_seen = false;
    g_last_fast_rotation_us = g_last_gravity_us = g_last_mag_us = last_time = 0;
    g_mag_startup_count = 0;
    g_mag_sum_x = g_mag_sum_y = g_mag_sum_magnitude = 0;
    measuredUpX = measuredUpY = 0;
    measuredUpZ = 1;
    diagnostics = CompassDiagnostics();
}

static bool readAttitudeRate(Attitude &rate) {
    float x, y, z;
    if (!read_gyroscope(x, y, z)) return false;
    x = - (x - calibration.gyroBiasX) * DEG_TO_RAD;  // raw gyro X-axis rate = roll rate (rotation about X)
    y = - (y - calibration.gyroBiasY) * DEG_TO_RAD;  // raw gyro Y-axis rate = pitch rate (rotation about Y)
    z = - (z - calibration.gyroBiasZ) * DEG_TO_RAD;  // raw gyro Z-axis rate = yaw rate
    // Attitude(pitch, roll, yaw) — note x/y swapped here relative to raw read
    // order, so that .pitch gets the Y-axis (pitch) rate and .roll gets the
    // X-axis (roll) rate, matching the convention already used by the
    // accelerometer formulas (ax -> pitch, ay -> roll in getMeasuredGravityBody
    // callers). Previously this returned Attitude(x, y, z) directly, which
    // silently swapped pitch and roll rates and caused a fast pitch motion to
    // be integrated as roll (and vice versa) in the quaternion update below.
    rate = Attitude(y, x, z);
    return true;
}

Attitude getAttitudeRate() {
    Attitude rate(0, 0, 0);
    readAttitudeRate(rate);
    return rate;
}

// Accelerometer only observes the direction of gravity — pitch/roll, never
// yaw (rotating about the gravity axis doesn't change the reading at all).
// Returns the normalized measured gravity direction in body frame, plus its
// magnitude (for trust-gating), rather than an Euler angle — the correction
// is applied directly to the quaternion, so there's no axis to leak into.
static bool getMeasuredGravityBody(float &gx, float &gy, float &gz, float &mag_out) {
    float ax, ay, az;
    if (!read_accelerometer(ax, ay, az)) return false;

    float mag = sqrtf(ax*ax + ay*ay + az*az);
    mag_out = mag;
    if (!isfinite(mag) || mag < 1e-6f) return false;

    gx = ax / mag;
    gy = ay / mag;
    gz = az / mag;
    return true;
}

Attitude getAttitude() {

    const uint32_t now = micros();
    float gx, gy, gz, gmag;
    const bool new_gravity = getMeasuredGravityBody(gx, gy, gz, gmag);
    const bool trusted_gravity = new_gravity && fabsf(gmag - 1.0f) < 0.2f;
    if (trusted_gravity) {
        g_last_gravity_us = now;
        measuredUpX = gx; measuredUpY = gy; measuredUpZ = gz;
    }

    if (!g_orientation_init) {
        last_time = now;
        if (!trusted_gravity) return Attitude(0, 0, 0);

        // initialize orientation directly from accel so we don't start at a
        // garbage identity pose if the device isn't level at boot
        {
            // measured gravity in body frame should equal world +Z rotated
            // into body frame; find the shortest rotation taking body-frame
            // measured gravity to world +Z: this is body->world.
            float wx = 0, wy = 0, wz = 1; // world up
            float cx = gy*wz - gz*wy;
            float cy = gz*wx - gx*wz;
            float cz = gx*wy - gy*wx;
            float dot = gx*wx + gy*wy + gz*wz;
            float s = sqrtf((1.0f + dot) * 2.0f);
            if (s < 1e-6f) {
                // Exact face-down is the shortest-rotation ambiguity: every
                // horizontal 180-degree axis maps body -Z to world +Z. Rotate
                // around physical forward so startup does not reverse heading.
                g_orientation = Quaternion(0, FORWARD_X, FORWARD_Y, FORWARD_Z);
            } else {
                float invs = 1.0f / s;
                g_orientation = Quaternion(s * 0.5f, cx * invs, cy * invs, cz * invs);
                g_orientation.normalize();
            }
        }
        g_orientation_init = true;
    }

    // Integrate the time between gyro samples, not between display frames.
    // Empty polls must not discard elapsed time; unsigned subtraction handles wrap.
    Attitude rate(0, 0, 0);
    float dt = 0;
    if (readAttitudeRate(rate)) {
        dt = static_cast<uint32_t>(now - last_time) * 1e-6f;
        last_time = now;
        g_angular_speed = sqrtf(rate.roll*rate.roll + rate.pitch*rate.pitch + rate.yaw*rate.yaw);
        if (g_angular_speed > MAG_CORRECTION_MAX_ANGULAR_SPEED) {
            g_fast_rotation_seen = true;
            g_last_fast_rotation_us = now;
        }
    }

    // --- 1. integrate gyro rate directly on the quaternion ---
    // q_dot = 0.5 * q * (0, wx, wy, wz)   (body-frame angular rate)
    Quaternion omega(0, rate.roll, rate.pitch, rate.yaw); // (x=roll rate, y=pitch rate, z=yaw rate) — body-frame gyro axes
    Quaternion qdot = g_orientation * omega;
    g_orientation.w += 0.5f * qdot.w * dt;
    g_orientation.x += 0.5f * qdot.x * dt;
    g_orientation.y += 0.5f * qdot.y * dt;
    g_orientation.z += 0.5f * qdot.z * dt;
    g_orientation.normalize();

    // --- 2. accelerometer correction: nudge orientation toward measured gravity ---
    float up_x = 0, up_y = 0, up_z = 1; // body-frame "world up", filled below if accel valid
    if (new_gravity) {
        Quaternion qc = g_orientation.conjugate();
        qc.rotateToWorld(0, 0, 1, up_x, up_y, up_z); // world up rotated into body frame

        float ex = gy*up_z - gz*up_y;
        float ey = gz*up_x - gx*up_z;
        float ez = gx*up_y - gy*up_x;

        float trust = (fabsf(gmag - 1.0f) < 0.2f) ? ACCEL_CORRECTION_GAIN : ACCEL_CORRECTION_GAIN_LOW;

        Quaternion correction(1.0f, ex * trust, ey * trust, ez * trust);
        g_orientation = g_orientation * correction;
        g_orientation.normalize();
    }
    // Propagate up with the gyro between accelerometer samples. Fresh samples
    // need not coincide with the independent magnetometer's sampling clock.
    g_orientation.conjugate().rotateToWorld(0, 0, 1, up_x, up_y, up_z);
    const bool have_up = static_cast<uint32_t>(now - g_last_gravity_us) <= GRAVITY_MAX_AGE_US;

    // --- 2b. magnetometer correction: yaw-only, via a fixed world-frame reference ---
    float mx, my, mz;
    if (g_orientation_init && read_magnetometer(mx, my, mz)) {
        ++diagnostics.samples;
        diagnostics.lastSampleUs = now;
        diagnostics.magneticHeading = COMPASS_ERROR;
        diagnostics.status = !have_up ? "TILT" : "MOVE";
        // Direct heading uses measured gravity, independent of the fusion's yaw
        // reference and of the field-strength rejection gate.
        const float vertical = mx*measuredUpX + my*measuredUpY + mz*measuredUpZ;
        const float nx = mx - vertical*measuredUpX;
        const float ny = my - vertical*measuredUpY;
        const float nz = mz - vertical*measuredUpZ;
        const float forwardUp = FORWARD_X*measuredUpX + FORWARD_Y*measuredUpY + FORWARD_Z*measuredUpZ;
        const float horizontal2 = nx*nx + ny*ny + nz*nz;
        if (have_up && isfinite(horizontal2) && horizontal2 > 1e-8f && fabsf(forwardUp) < 0.99f) {
            const float cross = measuredUpX*(ny*FORWARD_Z - nz*FORWARD_Y)
                              + measuredUpY*(nz*FORWARD_X - nx*FORWARD_Z)
                              + measuredUpZ*(nx*FORWARD_Y - ny*FORWARD_X);
            diagnostics.magneticHeading = static_cast<int>(normalizeHeading(
                atan2f(cross, nx*FORWARD_X + ny*FORWARD_Y + nz*FORWARD_Z)*RAD_TO_DEG));
        }
        const float mag_dt = fminf(static_cast<uint32_t>(now - g_last_mag_us) * 1e-6f, 0.2f);
        g_last_mag_us = now;
        const bool settled = have_up &&
            g_angular_speed <= MAG_CORRECTION_MAX_ANGULAR_SPEED &&
            (!g_fast_rotation_seen || static_cast<uint32_t>(now - g_last_fast_rotation_us) >= MAG_CORRECTION_SETTLE_US);
        float mnorm = sqrtf(mx*mx + my*my + mz*mz);
        diagnostics.field = mnorm;
        if (!isfinite(mnorm) || mnorm <= 1e-6f) diagnostics.status = "BAD";
        if (settled && isfinite(mnorm) && mnorm > 1e-6f) {
            float mxu = mx / mnorm, myu = my / mnorm, mzu = mz / mnorm;
            float wx, wy, wz;
            g_orientation.rotateToWorld(mxu, myu, mzu, wx, wy, wz);
            const float horizontal = sqrtf(wx*wx + wy*wy);
            if (horizontal < 0.1f) {
                diagnostics.status = "VERT";
                g_mag_startup_count = 0;
            } else {
            wx /= horizontal;
            wy /= horizontal;

            if (!g_mag_ref_init) {
                diagnostics.status = "INIT";
                if (g_mag_startup_count) {
                    const float length = sqrtf(g_mag_sum_x*g_mag_sum_x + g_mag_sum_y*g_mag_sum_y);
                    const float mean = g_mag_sum_magnitude / g_mag_startup_count;
                    if ((wx*g_mag_sum_x + wy*g_mag_sum_y) < 0.98480775f * length ||
                        fabsf(mnorm - mean) > 0.15f * mean) g_mag_startup_count = 0;
                }
                if (!g_mag_startup_count) g_mag_sum_x = g_mag_sum_y = g_mag_sum_magnitude = 0;
                g_mag_sum_x += wx;
                g_mag_sum_y += wy;
                g_mag_sum_magnitude += mnorm;
                if (++g_mag_startup_count >= MAG_STARTUP_SAMPLES) {
                    const float length = sqrtf(g_mag_sum_x*g_mag_sum_x + g_mag_sum_y*g_mag_sum_y);
                    g_mag_ref_x = g_mag_sum_x / length;
                    g_mag_ref_y = g_mag_sum_y / length;
                    g_mag_ref_z = 0;
                    g_mag_ref_magnitude = g_mag_sum_magnitude / g_mag_startup_count;
                    g_mag_ref_init = true;
                    diagnostics.status = "OK";
                }
            } else if (fabsf(mnorm - g_mag_ref_magnitude) < g_mag_ref_magnitude * MAG_FIELD_TOLERANCE) {
                diagnostics.status = "OK";
                ++diagnostics.corrections;
                // Signed horizontal angular error avoids dip-dependent gain and
                // the zero correction of a cross product at 180 degrees.
                const float error = atan2f(wx*g_mag_ref_y - wy*g_mag_ref_x,
                                          wx*g_mag_ref_x + wy*g_mag_ref_y);
                const float half_step = 0.5f * error * (1.0f - expf(-mag_dt / MAG_CORRECTION_TIME_S));
                g_orientation = Quaternion(cosf(half_step), 0, 0, sinf(half_step)) * g_orientation;
                g_orientation.normalize();
            } else {
                diagnostics.status = "FIELD";
            }
            }
        } else if (!g_mag_ref_init) {
            g_mag_startup_count = 0;
        }
    }

    // --- 3. convert quaternion to Euler angles for display / external use ---
    // standard aerospace roll-pitch-yaw extraction (ZYX convention)
    float w = g_orientation.w, x = g_orientation.x, y = g_orientation.y, z = g_orientation.z;

    float roll  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
    float sinp  = 2.0f*(w*y - z*x);
    float pitch = (fabsf(sinp) >= 1.0f) ? copysignf(M_PI / 2.0f, sinp) : asinf(sinp); // clamp — gimbal lock at +/-90 deg pitch
    float yaw   = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));

    return Attitude(pitch, roll, yaw);
}

// ------------------------------------------------------------------ //
//  heading
// ------------------------------------------------------------------ //

int getHeading() {
    getAttitude();
    if (!g_orientation_init || !g_mag_ref_init) return COMPASS_ERROR;

    float wx, wy, wz;
    g_orientation.rotateToWorld(FORWARD_X, FORWARD_Y, FORWARD_Z, wx, wy, wz);


    float cross = g_mag_ref_x * wy - g_mag_ref_y * wx;
    float dot   = g_mag_ref_x * wx + g_mag_ref_y * wy;
    float heading = atan2f(cross, dot) * RAD_TO_DEG;


    return static_cast<int>(normalizeHeading(heading)) % 360;
}

} // namespace quaternion_compass
