#!/usr/bin/env python3
"""
Max-rate benchmark for a single BNO085.

Stripped down for throughput testing.
No per-sample printing.
Reports final poll rate and fresh-sample rate.
"""

from __future__ import annotations

import time
import warnings

warnings.filterwarnings("ignore", category=RuntimeWarning, message="I2C frequency is not settable")

from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_GAME_ROTATION_VECTOR,
)


I2C_BUS = 1
I2C_ADDR = 0x4A
BOOT_DELAY = 0.8
DURATION_S = 5.0


def main() -> None:
    i2c = I2C(I2C_BUS)
    bno = BNO08X_I2C(i2c, address=I2C_ADDR)

    time.sleep(BOOT_DELAY)

    bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)

    polls = 0
    fresh = 0
    last_key = None

    t0 = time.perf_counter()

    try:
        while True:
            now = time.perf_counter()
            if now - t0 >= DURATION_S:
                break

            bno._process_available_packets()
            polls += 1

            quat = bno._readings.get(BNO_REPORT_GAME_ROTATION_VECTOR)
            accel = bno._readings.get(BNO_REPORT_ACCELEROMETER)
            gyro = bno._readings.get(BNO_REPORT_GYROSCOPE)

            if quat is None or accel is None or gyro is None:
                continue

            key = (*quat, *accel, *gyro)
            if key != last_key:
                fresh += 1
                last_key = key

    finally:
        elapsed = time.perf_counter() - t0
        poll_hz = polls / elapsed if elapsed > 0 else 0.0
        fresh_hz = fresh / elapsed if elapsed > 0 else 0.0

        print(f"Poll frequency : {poll_hz:.2f} Hz")
        print(f"Fresh frequency: {fresh_hz:.2f} Hz")

        try:
            i2c.deinit()
        except Exception:
            pass


if __name__ == "__main__":
    main()
