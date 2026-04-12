#!/usr/bin/env python3
"""
Simple FSR frequency check using ADS1115 continuous mode.

This script configures the ADC once and then repeatedly reads the latest
conversion result to measure the actual attainable frequency.

Usage:
    python3 simple_fsr_frequency_continuous.py --freq 300
    python3 simple_fsr_frequency_continuous.py --freq 500 --duration 8

Channel mapping (fixed):
    HEEL -> AIN0
    FOOT -> AIN1

Note:
    The ADS1115 has one ADC, so both channels are read sequentially.
    This script keeps the ADC in continuous mode while alternating the mux.
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

# ADS1115 continuous mode, 860 SPS, ±4.096V, comparator disabled
# AIN0: 0xC3E3
# AIN1: 0xD3E3
CFG_AIN0 = [0xC3, 0xE3]  # HEEL
CFG_AIN1 = [0xD3, 0xE3]  # FOOT

VOLTS_PER_COUNT = 4.096 / 32767.0


class ADS1115ContinuousReader:
    def __init__(self, bus: smbus2.SMBus):
        self.bus = bus
        self._configs = {
            "heel": CFG_AIN0,
            "foot": CFG_AIN1,
        }
        self._current = "heel"
        self.bus.write_i2c_block_data(ADC_ADDR, REG_CONFIG, self._configs[self._current])

    def _is_ready(self) -> bool:
        cfg = self.bus.read_i2c_block_data(ADC_ADDR, REG_CONFIG, 2)
        return bool(cfg[0] & 0x80)

    def _read_raw_for(self, channel: str) -> int:
        self.bus.write_i2c_block_data(ADC_ADDR, REG_CONFIG, self._configs[channel])
        while not self._is_ready():
            pass
        data = self.bus.read_i2c_block_data(ADC_ADDR, REG_CONVERSION, 2)
        self._current = channel
        return struct.unpack(">h", bytes(data))[0]

    def read_both_raw(self) -> tuple[int, int]:
        heel_raw = self._read_raw_for("heel")
        foot_raw = self._read_raw_for("foot")
        return foot_raw, heel_raw


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ADS1115 continuous-mode frequency test")
    parser.add_argument("--freq", type=float, required=True, help="Target loop frequency (Hz)")
    parser.add_argument("--duration", type=float, default=6.0, help="Test duration (seconds)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.freq <= 0:
        raise ValueError("--freq must be > 0")
    if args.duration <= 0:
        raise ValueError("--duration must be > 0")

    interval = 1.0 / args.freq

    with smbus2.SMBus(I2C_BUS) as bus:
        reader = ADS1115ContinuousReader(bus)

        start = time.perf_counter()
        next_tick = start
        samples = 0

        while True:
            now = time.perf_counter()
            if now - start >= args.duration:
                break

            foot_raw, heel_raw = reader.read_both_raw()
            samples += 1

            foot_v = max(foot_raw, 0) * VOLTS_PER_COUNT
            heel_v = max(heel_raw, 0) * VOLTS_PER_COUNT

            print(
                f"FOOT={foot_raw:7d} ({foot_v:0.4f} V)   "
                f"HEEL={heel_raw:7d} ({heel_v:0.4f} V)"
            )

            next_tick += interval
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.perf_counter()

        elapsed = time.perf_counter() - start

    actual_hz = samples / elapsed if elapsed > 0 else 0.0

    print(f"Target frequency: {args.freq:.2f} Hz")
    print(f"Actual frequency : {actual_hz:.2f} Hz")


if __name__ == "__main__":
    main()
