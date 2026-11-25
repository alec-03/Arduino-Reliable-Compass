#!/usr/bin/env python3
"""
Magnetometer + Accelerometer + Gyroscope calibration
for Arduino Nano 33 BLE Sense Rev2 (BMI270 + BMM150)

Usage:
    # collect data:
    #   Phase 1 — keep Arduino COMPLETELY STILL for gyro bias (first --gyro-still-s seconds)
    #   Phase 2 — rotate slowly through all orientations for mag/acc sphere fit
    python3 calibrate.py --collect --fit --port /dev/ttyACM0 --baud 500000

    # fit from saved file:
    python3 calibrate.py --fit --input cal_data.txt

    # gyro only (keep still the whole time):
    python3 calibrate.py --collect --fit --gyro-only --duration 30
"""

import argparse
import sys
import time
import numpy as np
from scipy.optimize import least_squares

# ------------------------------------------------------------------ #
#  serial collection
# ------------------------------------------------------------------ #

def collect(port, baud, outfile, duration_s=120, gyro_still_s=30, gyro_only=False):
    import serial
    mag_data = []
    acc_data = []
    gyr_data = []

    print("=" * 60)
    print(f"Phase 1 ({gyro_still_s}s): Keep Arduino COMPLETELY STILL for gyro bias.")
    print("=" * 60)

    with serial.Serial(port, baud, timeout=1) as s, open(outfile, 'w') as f:
        start = time.time()
        phase = 1
        try:
            while time.time() - start < duration_s:
                elapsed = time.time() - start

                # phase transition
                if phase == 1 and elapsed >= gyro_still_s:
                    if gyro_only:
                        break
                    phase = 2
                    print(f"\n{'=' * 60}")
                    print(f"Phase 2 ({duration_s - gyro_still_s}s): Rotate slowly through ALL orientations.")
                    print("=" * 60)

                line = s.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                parts = line.split()
                if len(parts) != 4:
                    continue

                label = parts[0]
                try:
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                except ValueError:
                    continue

                f.write(line + '\n')
                f.flush()

                if label == 'gyr' and phase == 1:
                    gyr_data.append([x, y, z])
                elif label == 'mag' and phase == 2:
                    mag_data.append([x, y, z])
                elif label == 'acc' and phase == 2:
                    acc_data.append([x, y, z])

                remaining = (gyro_still_s if phase == 1 else duration_s) - elapsed
                print(
                    f"\r[Phase {phase}] {remaining:.0f}s left | "
                    f"gyr: {len(gyr_data)}  mag: {len(mag_data)}  acc: {len(acc_data)}",
                    end=''
                )

        except KeyboardInterrupt:
            print("\nStopped early.")

    print(f"\nCollected {len(gyr_data)} gyr, {len(mag_data)} mag, {len(acc_data)} acc → {outfile}")
    return np.array(mag_data), np.array(acc_data), np.array(gyr_data)


# ------------------------------------------------------------------ #
#  sphere fit
# ------------------------------------------------------------------ #

def sphere_fit(data, label):
    """Fit a sphere to data points. Returns (cx, cy, cz, r)."""
    if len(data) < 10:
        print(f"[{label}] Not enough samples ({len(data)}) — need at least 10.")
        return None

    def residuals(params):
        cx, cy, cz, r = params
        return np.sqrt(
            (data[:,0] - cx)**2 +
            (data[:,1] - cy)**2 +
            (data[:,2] - cz)**2
        ) - r

    # initial guess: centroid + mean radius
    cx0, cy0, cz0 = data.mean(axis=0)
    r0 = np.sqrt(((data - [cx0, cy0, cz0])**2).sum(axis=1)).mean()

    result = least_squares(residuals, [cx0, cy0, cz0, r0])
    cx, cy, cz, r = result.x

    # RMS fit error
    errors = residuals(result.x)
    rms = np.sqrt((errors**2).mean())
    print(f"[{label}] center=({cx:.4f}, {cy:.4f}, {cz:.4f})  r={r:.4f}  RMS error={rms:.4f}")

    return cx, cy, cz, r


# ------------------------------------------------------------------ #
#  parse saved file
# ------------------------------------------------------------------ #

def parse_file(infile):
    mag_data, acc_data, gyr_data = [], [], []
    with open(infile) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 4:
                continue
            label = parts[0]
            try:
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            except ValueError:
                continue
            if label == 'mag':
                mag_data.append([x, y, z])
            elif label == 'acc':
                acc_data.append([x, y, z])
            elif label == 'gyr':
                gyr_data.append([x, y, z])
    return np.array(mag_data), np.array(acc_data), np.array(gyr_data)


# ------------------------------------------------------------------ #
#  plot
# ------------------------------------------------------------------ #

def plot_fit(data, cx, cy, cz, r, label):
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(data[:,0], data[:,1], data[:,2], s=2, alpha=0.4, label='raw')

    # draw sphere
    u, v = np.mgrid[0:2*np.pi:40j, 0:np.pi:20j]
    sx = cx + r * np.cos(u) * np.sin(v)
    sy = cy + r * np.sin(u) * np.sin(v)
    sz = cz + r * np.cos(v)
    ax.plot_wireframe(sx, sy, sz, color='red', alpha=0.15, linewidth=0.5)

    ax.set_title(f"{label} sphere fit")
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.legend()
    plt.tight_layout()
    plt.show()


# ------------------------------------------------------------------ #
#  gyro bias
# ------------------------------------------------------------------ #

def gyro_bias(data, label='GYR'):
    """Compute mean bias and noise from still-phase gyro data."""
    if len(data) < 10:
        print(f"[{label}] Not enough samples ({len(data)}) — keep Arduino still during phase 1.")
        return None

    bias = data.mean(axis=0)
    std  = data.std(axis=0)
    print(f"[{label}] bias=({bias[0]:.5f}, {bias[1]:.5f}, {bias[2]:.5f}) deg/s")
    print(f"[{label}] noise std=({std[0]:.5f}, {std[1]:.5f}, {std[2]:.5f}) deg/s")
    return bias


def plot_gyro(data, bias):
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
    labels = ['X', 'Y', 'Z']
    for i, (ax, lbl) in enumerate(zip(axes, labels)):
        ax.plot(data[:, i], linewidth=0.5, label=f'raw {lbl}')
        ax.axhline(bias[i], color='red', linewidth=1, label=f'bias {bias[i]:.4f}')
        ax.set_ylabel(f'{lbl} (deg/s)')
        ax.legend(fontsize=7)
    axes[-1].set_xlabel('sample')
    fig.suptitle('Gyroscope bias (still phase)')
    plt.tight_layout()
    plt.show()


# ------------------------------------------------------------------ #
#  print calibration constants
# ------------------------------------------------------------------ #

def print_constants(mag_result, acc_result, gyr_result):
    print("\n" + "="*60)
    print("Paste these assignments before setCalibration(calibration):")
    print("quaternion_compass::Calibration calibration;")
    if mag_result:
        cx, cy, cz, r = mag_result
        for axis, value in zip('XYZ', (cx, cy, cz)):
            print(f"calibration.magOffset{axis} = {value:.6f}f;")
            print(f"calibration.magScale{axis} = {1/r:.6f}f;")
    if acc_result:
        cx, cy, cz, r = acc_result
        for axis, value in zip('XYZ', (cx, cy, cz)):
            print(f"calibration.accOffset{axis} = {value:.6f}f;")
        print(f"calibration.accScale = {1/r:.6f}f;")
    if gyr_result is not None:
        for axis, value in zip('XYZ', gyr_result):
            print(f"calibration.gyroBias{axis} = {value:.6f}f;")
    print("quaternion_compass::setCalibration(calibration);")
    print("="*60 + "\n")


# ------------------------------------------------------------------ #
#  main
# ------------------------------------------------------------------ #

def main():
    parser = argparse.ArgumentParser(description="IMU calibration — mag/acc sphere fit + gyro bias")
    parser.add_argument('--collect',       action='store_true', help='collect data from serial')
    parser.add_argument('--fit',           action='store_true', help='fit to saved/collected data')
    parser.add_argument('--gyro-only',     action='store_true', help='only collect gyro (stay still whole time)')
    parser.add_argument('--port',          default='/dev/ttyACM0')
    parser.add_argument('--baud',          type=int, default=500000)
    parser.add_argument('--out',           default='cal_data.txt', help='output file for collection')
    parser.add_argument('--input',         default='cal_data.txt', help='input file for fitting')
    parser.add_argument('--duration',      type=int, default=120,  help='total collection duration (s)')
    parser.add_argument('--gyro-still-s',  type=int, default=30,   help='still phase duration for gyro (s)')
    parser.add_argument('--no-plot',       action='store_true',     help='skip plots')
    args = parser.parse_args()

    if not args.collect and not args.fit:
        parser.print_help()
        sys.exit(1)

    empty = np.array([]).reshape(0, 3)
    mag_data, acc_data, gyr_data = empty.copy(), empty.copy(), empty.copy()

    if args.collect:
        mag_data, acc_data, gyr_data = collect(
            args.port, args.baud, args.out,
            duration_s=args.duration,
            gyro_still_s=args.gyro_still_s,
            gyro_only=args.gyro_only
        )

    if args.fit:
        if not args.collect:
            mag_data, acc_data, gyr_data = parse_file(args.input)
            print(f"Loaded {len(gyr_data)} gyr, {len(mag_data)} mag, {len(acc_data)} acc from {args.input}")

        mag_result = sphere_fit(mag_data, 'MAG') if len(mag_data) >= 10 else None
        acc_result = sphere_fit(acc_data, 'ACC') if len(acc_data) >= 10 else None
        gyr_result = gyro_bias(gyr_data,  'GYR') if len(gyr_data) >= 10 else None

        print_constants(mag_result, acc_result, gyr_result)

        if not args.no_plot:
            if mag_result:
                plot_fit(mag_data, *mag_result, 'Magnetometer')
            if acc_result:
                plot_fit(acc_data, *acc_result, 'Accelerometer')
            if gyr_result is not None:
                plot_gyro(gyr_data, gyr_result)


if __name__ == '__main__':
    main()