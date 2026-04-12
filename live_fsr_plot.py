#!/usr/bin/env python3
"""
Live FSR visualizer (ADS1115): plots FOOT and HEEL in real time.

Channels (fixed mapping):
    HEEL FSR -> AIN0 (single-ended)
    FOOT FSR -> AIN1 (single-ended)

Requirements:
  pip install smbus2 matplotlib

Run:
  python3 live_fsr_plot.py
"""

from __future__ import annotations

import struct
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import smbus2

# Hardware
I2C_BUS = 1
ADC_ADDR = 0x48

# ADS1115 registers
REG_CONVERSION = 0x00
REG_CONFIG = 0x01

# ADS1115 config: ±4.096 V, single-shot, 128 SPS, comparator disabled
_CFG_LO = 0x83
CFG_AIN0 = [0xC3, _CFG_LO]  # AIN0 vs GND
CFG_AIN1 = [0xD3, _CFG_LO]  # AIN1 vs GND

VOLTS_PER_COUNT = 4.096 / 32767.0
CONV_DELAY_S = 1 / 128 + 0.002

# Plot timing
SAMPLE_HZ = 40
WINDOW_SECONDS = 10


class LiveFsrPlotter:
    def __init__(self) -> None:
        self.interval_s = 1.0 / SAMPLE_HZ
        self.maxlen = int(WINDOW_SECONDS * SAMPLE_HZ)

        self.t = deque(maxlen=self.maxlen)
        self.foot = deque(maxlen=self.maxlen)
        self.heel = deque(maxlen=self.maxlen)

        self.bus = smbus2.SMBus(I2C_BUS)
        self.t_start = time.time()

        self.fig, self.ax = plt.subplots(figsize=(10, 5))
        (self.line_foot,) = self.ax.plot([], [], label="Foot FSR (A1)", color="tab:red")
        (self.line_heel,) = self.ax.plot([], [], label="Heel FSR (A0)", color="tab:green")

        self.ax.set_title("Live FSR ADC Values")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("ADC Count")
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc="upper right")

        self.text_box = self.ax.text(
            0.02,
            0.95,
            "",
            transform=self.ax.transAxes,
            va="top",
            ha="left",
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.7},
        )

    def read_channel(self, config: list[int]) -> int:
        self.bus.write_i2c_block_data(ADC_ADDR, REG_CONFIG, config)
        time.sleep(CONV_DELAY_S)
        data = self.bus.read_i2c_block_data(ADC_ADDR, REG_CONVERSION, 2)
        return struct.unpack(">h", bytes(data))[0]

    def read_pair(self) -> tuple[int, int]:
        heel_raw = self.read_channel(CFG_AIN0)
        foot_raw = self.read_channel(CFG_AIN1)
        return foot_raw, heel_raw

    def update(self, _frame_idx: int):
        t0 = time.monotonic()

        foot_raw, heel_raw = self.read_pair()
        t_now = time.time() - self.t_start

        self.t.append(t_now)
        self.foot.append(foot_raw)
        self.heel.append(heel_raw)

        self.line_foot.set_data(self.t, self.foot)
        self.line_heel.set_data(self.t, self.heel)

        if self.t:
            x_min = max(0.0, self.t[-1] - WINDOW_SECONDS)
            x_max = max(WINDOW_SECONDS, self.t[-1])
            self.ax.set_xlim(x_min, x_max)

            y_min = min(min(self.foot), min(self.heel))
            y_max = max(max(self.foot), max(self.heel))
            if y_min == y_max:
                y_min -= 1
                y_max += 1
            pad = max(50, int(0.1 * (y_max - y_min)))
            self.ax.set_ylim(y_min - pad, y_max + pad)

            foot_v = max(foot_raw, 0) * VOLTS_PER_COUNT
            heel_v = max(heel_raw, 0) * VOLTS_PER_COUNT
            self.text_box.set_text(
                f"Foot: {foot_raw:6d} ({foot_v:0.3f} V)\n"
                f"Heel: {heel_raw:6d} ({heel_v:0.3f} V)"
            )

        elapsed = time.monotonic() - t0
        sleep_time = self.interval_s - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

        return self.line_foot, self.line_heel, self.text_box

    def run(self) -> None:
        anim = FuncAnimation(
            self.fig,
            self.update,
            interval=int(self.interval_s * 1000),
            blit=False,
            cache_frame_data=False,
        )

        try:
            plt.tight_layout()
            plt.show()
        finally:
            self.bus.close()
            del anim


def main() -> None:
    print(f"Starting live FSR plot on I2C bus {I2C_BUS}, ADS1115 addr 0x{ADC_ADDR:02X}")
    print(f"Sampling at ~{SAMPLE_HZ} Hz, showing last {WINDOW_SECONDS} s")
    plotter = LiveFsrPlotter()
    plotter.run()


if __name__ == "__main__":
    main()
