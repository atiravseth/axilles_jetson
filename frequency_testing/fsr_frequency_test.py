#!/usr/bin/env python3
"""
FSR frequency tester (ADS1115) for FOOT + HEEL channels.

Goals:
1) Stream live FSR values at a user-requested target frequency.
2) Report achieved frequency and timing misses.
3) Estimate max possible frequency in current setup (no sleep benchmark).

Channel mapping (fixed):
  HEEL -> AIN0
  FOOT -> AIN1

Examples:
  python3 frequency_testing/fsr_frequency_test.py --freq 100 --duration 8
  python3 frequency_testing/fsr_frequency_test.py --freq 200 --duration 10 --quiet
  python3 frequency_testing/fsr_frequency_test.py --max-test-duration 6
"""

from __future__ import annotations

import argparse
import struct
import time
from statistics import mean

import smbus2


I2C_BUS = 1
ADC_ADDR = 0x48

REG_CONVERSION = 0x00
REG_CONFIG = 0x01

# ±4.096 V, single-shot, 860 SPS, comparator disabled
CFG_AIN0 = [0xC3, 0xE3]  # AIN0 (HEEL)
CFG_AIN1 = [0xD3, 0xE3]  # AIN1 (FOOT)

VOLTS_PER_COUNT = 4.096 / 32767.0


class ADS1115Reader:
    def __init__(self, bus_index: int = I2C_BUS, addr: int = ADC_ADDR):
        self.bus_index = bus_index
        self.addr = addr
        self.bus = smbus2.SMBus(bus_index)

    def close(self) -> None:
        self.bus.close()

    def _read_channel_raw(self, config: list[int]) -> int:
        self.bus.write_i2c_block_data(self.addr, REG_CONFIG, config)

        while True:
            cfg = self.bus.read_i2c_block_data(self.addr, REG_CONFIG, 2)
            if cfg[0] & 0x80:
                break

        data = self.bus.read_i2c_block_data(self.addr, REG_CONVERSION, 2)
        return struct.unpack(">h", bytes(data))[0]

    def read_pair_raw(self) -> tuple[int, int]:
        heel_raw = self._read_channel_raw(CFG_AIN0)
        foot_raw = self._read_channel_raw(CFG_AIN1)
        return foot_raw, heel_raw


def run_target_frequency_test(
    reader: ADS1115Reader,
    target_hz: float,
    duration_s: float,
    print_every: int,
    quiet: bool,
) -> dict:
    interval_s = 1.0 / target_hz
    deadline_misses = 0
    loop_times = []
    foot_vals = []
    heel_vals = []

    start = time.perf_counter()
    next_tick = start
    samples = 0

    while True:
        now = time.perf_counter()
        if now - start >= duration_s:
            break

        iter_start = now
        foot_raw, heel_raw = reader.read_pair_raw()
        iter_end = time.perf_counter()

        loop_dt = iter_end - iter_start
        loop_times.append(loop_dt)
        foot_vals.append(foot_raw)
        heel_vals.append(heel_raw)
        samples += 1

        if (not quiet) and (samples % print_every == 0):
            foot_v = max(foot_raw, 0) * VOLTS_PER_COUNT
            heel_v = max(heel_raw, 0) * VOLTS_PER_COUNT
            print(
                f"[{samples:6d}] FOOT={foot_raw:7d} ({foot_v:0.4f} V)  "
                f"HEEL={heel_raw:7d} ({heel_v:0.4f} V)"
            )

        next_tick += interval_s
        sleep_time = next_tick - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            deadline_misses += 1
            next_tick = time.perf_counter()

    elapsed = time.perf_counter() - start
    achieved_hz = samples / elapsed if elapsed > 0 else 0.0

    return {
        "samples": samples,
        "elapsed_s": elapsed,
        "achieved_hz": achieved_hz,
        "deadline_misses": deadline_misses,
        "mean_loop_ms": mean(loop_times) * 1000 if loop_times else 0.0,
        "max_loop_ms": max(loop_times) * 1000 if loop_times else 0.0,
        "min_loop_ms": min(loop_times) * 1000 if loop_times else 0.0,
        "foot_min": min(foot_vals) if foot_vals else 0,
        "foot_max": max(foot_vals) if foot_vals else 0,
        "heel_min": min(heel_vals) if heel_vals else 0,
        "heel_max": max(heel_vals) if heel_vals else 0,
    }


def run_max_throughput_test(reader: ADS1115Reader, duration_s: float) -> dict:
    start = time.perf_counter()
    samples = 0
    loop_times = []

    while True:
        now = time.perf_counter()
        if now - start >= duration_s:
            break

        iter_start = now
        reader.read_pair_raw()
        iter_end = time.perf_counter()

        samples += 1
        loop_times.append(iter_end - iter_start)

    elapsed = time.perf_counter() - start
    max_hz = samples / elapsed if elapsed > 0 else 0.0

    return {
        "samples": samples,
        "elapsed_s": elapsed,
        "max_hz": max_hz,
        "mean_loop_ms": mean(loop_times) * 1000 if loop_times else 0.0,
        "max_loop_ms": max(loop_times) * 1000 if loop_times else 0.0,
        "min_loop_ms": min(loop_times) * 1000 if loop_times else 0.0,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="FSR frequency + throughput tester")
    parser.add_argument("--freq", type=float, default=100.0, help="Target output frequency in Hz")
    parser.add_argument("--duration", type=float, default=8.0, help="Duration for target-frequency test (s)")
    parser.add_argument(
        "--print-every",
        type=int,
        default=20,
        help="Print one FSR sample every N loops (ignored with --quiet)",
    )
    parser.add_argument(
        "--max-test-duration",
        type=float,
        default=5.0,
        help="Duration for max-throughput benchmark (s)",
    )
    parser.add_argument("--quiet", action="store_true", help="Suppress per-sample printing")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.freq <= 0:
        raise ValueError("--freq must be > 0")
    if args.duration <= 0:
        raise ValueError("--duration must be > 0")
    if args.max_test_duration <= 0:
        raise ValueError("--max-test-duration must be > 0")
    if args.print_every <= 0:
        raise ValueError("--print-every must be > 0")

    print(
        f"FSR test start | bus={I2C_BUS} addr=0x{ADC_ADDR:02X} | "
        f"target={args.freq:.1f} Hz for {args.duration:.1f}s"
    )
    print("Mapping: FOOT=AIN1, HEEL=AIN0")

    reader = ADS1115Reader()

    try:
        result = run_target_frequency_test(
            reader=reader,
            target_hz=args.freq,
            duration_s=args.duration,
            print_every=args.print_every,
            quiet=args.quiet,
        )

        print("\n--- Target Frequency Report ---")
        print(f"Target Hz          : {args.freq:.2f}")
        print(f"Achieved Hz        : {result['achieved_hz']:.2f}")
        print(f"Samples            : {result['samples']} in {result['elapsed_s']:.3f} s")
        print(f"Deadline misses    : {result['deadline_misses']}")
        print(
            f"Loop time ms       : mean={result['mean_loop_ms']:.3f} "
            f"min={result['min_loop_ms']:.3f} max={result['max_loop_ms']:.3f}"
        )
        print(
            f"FOOT raw range     : {result['foot_min']} .. {result['foot_max']}"
        )
        print(
            f"HEEL raw range     : {result['heel_min']} .. {result['heel_max']}"
        )

        max_result = run_max_throughput_test(reader, duration_s=args.max_test_duration)
        print("\n--- Max Throughput Estimate ---")
        print(f"Estimated max Hz   : {max_result['max_hz']:.2f}")
        print(
            f"Samples            : {max_result['samples']} in {max_result['elapsed_s']:.3f} s"
        )
        print(
            f"Loop time ms       : mean={max_result['mean_loop_ms']:.3f} "
            f"min={max_result['min_loop_ms']:.3f} max={max_result['max_loop_ms']:.3f}"
        )

        if result["achieved_hz"] + 1e-9 < args.freq:
            print(
                "\nResult: target frequency is above current sustainable rate. "
                "Use achieved/max Hz as practical limit."
            )
        else:
            print("\nResult: target frequency is achievable in current setup.")

    finally:
        reader.close()


if __name__ == "__main__":
    main()
