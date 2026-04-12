#!/usr/bin/env python3
"""
Max-rate benchmark for:
  - ADC FSRs: HEEL=A0, FOOT=A1
  - Encoder: AS5600 angle

Stripped down for throughput testing.
No per-sample printing.
Reports only the final achieved loop rate.
"""

from __future__ import annotations

import struct
import time

import smbus2


I2C_BUS = 1
ADC_ADDR = 0x48
ENC_ADDR = 0x36

REG_CONVERSION = 0x00
REG_CONFIG = 0x01
REG_ANGLE = 0x0E

# ADS1115, continuous-ish use by repeatedly switching mux and reading ready bit
CFG_HEEL = [0xC3, 0xE3]  # AIN0
CFG_FOOT = [0xD3, 0xE3]  # AIN1


class Reader:
    def __init__(self, bus: smbus2.SMBus):
        self.bus = bus
        self.bus.write_i2c_block_data(ADC_ADDR, REG_CONFIG, CFG_HEEL)

    def adc_ready(self) -> bool:
        cfg = self.bus.read_i2c_block_data(ADC_ADDR, REG_CONFIG, 2)
        return bool(cfg[0] & 0x80)

    def read_adc(self, cfg_word: list[int]) -> int:
        self.bus.write_i2c_block_data(ADC_ADDR, REG_CONFIG, cfg_word)
        while not self.adc_ready():
            pass
        data = self.bus.read_i2c_block_data(ADC_ADDR, REG_CONVERSION, 2)
        return struct.unpack(">h", bytes(data))[0]

    def read_encoder(self) -> int:
        data = self.bus.read_i2c_block_data(ENC_ADDR, REG_ANGLE, 2)
        return ((data[0] & 0x0F) << 8) | data[1]

    def read_all(self) -> None:
        self.read_adc(CFG_HEEL)
        self.read_adc(CFG_FOOT)
        self.read_encoder()


def main() -> None:
    duration_s = 5.0

    with smbus2.SMBus(I2C_BUS) as bus:
        reader = Reader(bus)

        start = time.perf_counter()
        samples = 0

        while True:
            if time.perf_counter() - start >= duration_s:
                break
            reader.read_all()
            samples += 1

        elapsed = time.perf_counter() - start

    hz = samples / elapsed if elapsed > 0 else 0.0
    print(f"Actual frequency: {hz:.2f} Hz")


if __name__ == "__main__":
    main()
