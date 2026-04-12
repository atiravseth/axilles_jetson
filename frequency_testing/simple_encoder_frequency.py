#!/usr/bin/env python3
"""
Simple encoder frequency check (AS5600).

Pass a target frequency, and it prints the actual frequency attained.

Usage:
  python3 simple_encoder_frequency.py --freq 200
  python3 simple_encoder_frequency.py --freq 300 --duration 8
"""

from __future__ import annotations

import argparse
import time

import smbus2


I2C_BUS = 1
ENC_ADDR = 0x36
REG_ANGLE = 0x0E


def read_angle_raw(bus: smbus2.SMBus) -> int:
    data = bus.read_i2c_block_data(ENC_ADDR, REG_ANGLE, 2)
    return ((data[0] & 0x0F) << 8) | data[1]


def main() -> None:
    parser = argparse.ArgumentParser(description="Simple encoder frequency test")
    parser.add_argument("--freq", type=float, required=True, help="Target frequency (Hz)")
    parser.add_argument("--duration", type=float, default=6.0, help="Test duration (seconds)")
    args = parser.parse_args()

    if args.freq <= 0:
        raise ValueError("--freq must be > 0")
    if args.duration <= 0:
        raise ValueError("--duration must be > 0")

    interval = 1.0 / args.freq

    with smbus2.SMBus(I2C_BUS) as bus:
        start = time.perf_counter()
        next_tick = start
        samples = 0

        while True:
            now = time.perf_counter()
            if now - start >= args.duration:
                break

            read_angle_raw(bus)
            samples += 1

            next_tick += interval
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.perf_counter()

        elapsed = time.perf_counter() - start

    actual_hz = samples / elapsed if elapsed > 0 else 0.0

    print(f"Target frequency : {args.freq:.2f} Hz")
    print(f"Actual frequency : {actual_hz:.2f} Hz")


if __name__ == "__main__":
    main()
