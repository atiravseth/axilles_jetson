#!/usr/bin/env python3
"""
Simple FSR frequency check.

Pass a target frequency, and it prints the actual frequency attained.

Usage:
  python3 simple_fsr_frequency.py --freq 200
  python3 simple_fsr_frequency.py --freq 150 --duration 8
"""

from __future__ import annotations

import argparse
import struct
import time

import smbus2


I2C_BUS = 1
ADC_ADDR = 0x48

REG_CONVERSION = 0x00
REG_CONFIG = 0x01

# ADS1115 single-shot, 860 SPS, ±4.096V
CFG_AIN0 = [0xC3, 0xE3]  # HEEL
CFG_AIN1 = [0xD3, 0xE3]  # FOOT


def read_channel_raw(bus: smbus2.SMBus, config: list[int]) -> int:
    bus.write_i2c_block_data(ADC_ADDR, REG_CONFIG, config)

    while True:
        cfg = bus.read_i2c_block_data(ADC_ADDR, REG_CONFIG, 2)
        if cfg[0] & 0x80:
            break

    data = bus.read_i2c_block_data(ADC_ADDR, REG_CONVERSION, 2)
    return struct.unpack(">h", bytes(data))[0]


def read_pair(bus: smbus2.SMBus) -> tuple[int, int]:
    heel = read_channel_raw(bus, CFG_AIN0)
    foot = read_channel_raw(bus, CFG_AIN1)
    return foot, heel


def main() -> None:
    parser = argparse.ArgumentParser(description="Simple FSR frequency test")
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

            read_pair(bus)
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
