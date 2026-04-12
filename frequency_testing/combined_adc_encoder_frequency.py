#!/usr/bin/env python3
"""
Combined ADC + encoder frequency test.

Reads:
  - FSRs from ADS1115 (HEEL=A0, FOOT=A1)
  - Encoder from AS5600

Pass a target loop frequency and the script reports the actual
frequency attained while reading both devices in the same loop.

Usage:
  python3 combined_adc_encoder_frequency.py --freq 100
  python3 combined_adc_encoder_frequency.py --freq 200 --duration 8
"""

from __future__ import annotations

import argparse
import struct
import time

import smbus2


I2C_BUS = 1
ADC_ADDR = 0x48
ENC_ADDR = 0x36

REG_CONVERSION = 0x00
REG_CONFIG = 0x01
REG_ANGLE = 0x0E

# ADS1115 continuous-mode configs, 860 SPS, ±4.096V, comparator disabled
CFG_HEEL = [0xC3, 0xE3]  # AIN0
CFG_FOOT = [0xD3, 0xE3]  # AIN1

VOLTS_PER_COUNT = 4.096 / 32767.0


class CombinedReader:
    def __init__(self, bus: smbus2.SMBus):
        self.bus = bus
        self.bus.write_i2c_block_data(ADC_ADDR, REG_CONFIG, CFG_HEEL)

    def _adc_ready(self) -> bool:
        cfg = self.bus.read_i2c_block_data(ADC_ADDR, REG_CONFIG, 2)
        return bool(cfg[0] & 0x80)

    def _read_adc_raw(self, config: list[int]) -> int:
        self.bus.write_i2c_block_data(ADC_ADDR, REG_CONFIG, config)
        while not self._adc_ready():
            pass
        data = self.bus.read_i2c_block_data(ADC_ADDR, REG_CONVERSION, 2)
        return struct.unpack(">h", bytes(data))[0]

    def read_encoder_raw(self) -> int:
        data = self.bus.read_i2c_block_data(ENC_ADDR, REG_ANGLE, 2)
        return ((data[0] & 0x0F) << 8) | data[1]

    def read_all(self) -> tuple[int, int, int]:
        heel_raw = self._read_adc_raw(CFG_HEEL)
        foot_raw = self._read_adc_raw(CFG_FOOT)
        enc_raw = self.read_encoder_raw()
        return foot_raw, heel_raw, enc_raw


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Combined ADC + encoder frequency test")
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
        reader = CombinedReader(bus)

        start = time.perf_counter()
        next_tick = start
        samples = 0

        while True:
            now = time.perf_counter()
            if now - start >= args.duration:
                break

            foot_raw, heel_raw, enc_raw = reader.read_all()
            samples += 1

            # Keep the loop compact; uncomment for debugging:
            # print(f"FOOT={foot_raw:7d} HEEL={heel_raw:7d} ENC={enc_raw:4d}")

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
    print(f"Samples          : {samples}")
    print(f"Duration         : {elapsed:.3f} s")
    print(
        f"Last read values : FOOT={foot_raw:7d} ({max(foot_raw, 0) * VOLTS_PER_COUNT:0.4f} V), "
        f"HEEL={heel_raw:7d} ({max(heel_raw, 0) * VOLTS_PER_COUNT:0.4f} V), ENC={enc_raw:4d}"
    )


if __name__ == "__main__":
    main()
