#!/usr/bin/env python3
"""
Single-BNO085 frequency test using hardware report interval (no sleep in read loop).

This script sets the IMU report period by converting --freq (Hz) to microseconds
and passing it into enable_feature(..., report_interval_us).

Examples
--------
python3 unit_tests/test_bno085_report_interval.py --freq 100
python3 unit_tests/test_bno085_report_interval.py --freq 200 --duration 10 --print-samples
"""

import argparse
import sys
import time
import warnings
from pathlib import Path
from typing import Optional

warnings.filterwarnings(
    "ignore",
    category=RuntimeWarning,
    message="I2C frequency is not settable",
)

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
)
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_extended_bus import ExtendedI2C as I2C

RECONNECT_DELAY = 1.0
BOOT_DELAY = 0.8
FEATURE_RETRIES = 5

# Choose exactly one feature to test: "quat", "accel", or "gyro".
SELECTED_FEATURE = "quat"

FEATURE_MAP = {
    "quat": BNO_REPORT_GAME_ROTATION_VECTOR,
    "accel": BNO_REPORT_ACCELEROMETER,
    "gyro": BNO_REPORT_GYROSCOPE,
}


class FreshBNO085:
    """BNO085 wrapper that emits only changed samples."""

    def __init__(self, bus: int, address: int, report_interval_us: int, feature_name: str):
        self._bus = bus
        self._address = address
        self._report_interval_us = report_interval_us
        self._feature_name = feature_name
        self._feature_id = FEATURE_MAP[feature_name]
        self._i2c = None
        self._bno = None
        self._last_key: Optional[tuple] = None
        self._connect()

    def _connect(self) -> None:
        while True:
            try:
                if self._i2c is not None:
                    try:
                        self._i2c.deinit()
                    except Exception:
                        pass
                    time.sleep(0.2)

                self._i2c = I2C(self._bus)
                self._bno = BNO08X_I2C(self._i2c, address=self._address)
                time.sleep(BOOT_DELAY)

                for attempt in range(FEATURE_RETRIES):
                    try:
                        self._bno.enable_feature(self._feature_id, self._report_interval_us)
                        break
                    except TypeError:
                        # Backward compatibility for versions without interval arg.
                        print("[WARN] enable_feature does not accept report_interval, using default!")
                        self._bno.enable_feature(self._feature_id)
                        break
                    except Exception:
                        if attempt == FEATURE_RETRIES - 1:
                            raise
                        time.sleep(0.2)
                return
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                print(f"[IMU] Connect failed ({exc}), retrying in {RECONNECT_DELAY}s...", flush=True)
                time.sleep(RECONNECT_DELAY)

    def close(self) -> None:
        try:
            self._i2c.deinit()
        except Exception:
            pass

    def read_fresh(self) -> Optional[dict]:
        """Return a sample only when the reading changed from the previous one."""
        try:
            self._bno._process_available_packets(max_packets=1)
            reading = self._bno._readings.get(self._feature_id)
        except KeyError:
            return None
        except (OSError, RuntimeError, AttributeError) as exc:
            print(f"[IMU] Read error ({type(exc).__name__}: {exc}), reconnecting...", flush=True)
            self._connect()
            return None

        if reading is None:
            return None

        sample_key = tuple(reading)
        if sample_key == self._last_key:
            return None

        self._last_key = sample_key
        return {"feature": self._feature_name, "values": reading}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="BNO085 frequency test using report interval.")
    parser.add_argument("--freq", type=float, required=True, help="Target report frequency in Hz.")
    parser.add_argument("--duration", type=float, default=5.0, help="Run duration in seconds (default: 5).")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number (default: 1).")
    parser.add_argument(
        "--addr",
        type=lambda v: int(v, 0),
        default=0x4A,
        help="I2C address, hex or decimal (default: 0x4A).",
    )
    parser.add_argument(
        "--print-samples",
        action="store_true",
        help="Print each fresh sample.",
    )
    return parser.parse_args()


def hz_to_us(freq_hz: float) -> int:
    # Clamp to >= 1000us (1kHz) to avoid invalid zero/negative intervals.
    return max(1000, int(round(1_000_000.0 / freq_hz)))


def main() -> None:
    args = parse_args()

    feature_name = SELECTED_FEATURE.strip().lower()
    if feature_name not in FEATURE_MAP:
        choices = ", ".join(FEATURE_MAP.keys())
        raise SystemExit(f"[ERROR] Invalid SELECTED_FEATURE '{SELECTED_FEATURE}'. Choose one of: {choices}")

    if args.freq <= 0:
        raise SystemExit("[ERROR] --freq must be > 0")
    if args.duration <= 0:
        raise SystemExit("[ERROR] --duration must be > 0")

    report_interval_us = hz_to_us(args.freq)
    expected_hz = 1_000_000.0 / report_interval_us

    print(
        f"Starting BNO085 test: requested={args.freq:.2f} Hz, "
        f"interval={report_interval_us} us (~{expected_hz:.2f} Hz), "
        f"duration={args.duration:.2f}s, bus={args.bus}, addr=0x{args.addr:02X}, "
        f"feature={feature_name}"
    )

    imu = FreshBNO085(
        bus=args.bus,
        address=args.addr,
        report_interval_us=report_interval_us,
        feature_name=feature_name,
    )

    samples = 0
    null_reads = 0
    loop_iters = 0

    start = time.perf_counter()
    t_first = None
    t_last = None

    try:
        while True:
            now = time.perf_counter()
            if now - start >= args.duration:
                break

            loop_iters += 1
            sample = imu.read_fresh()
            if sample is None:
                null_reads += 1
                continue

            samples += 1
            t = time.perf_counter()
            if t_first is None:
                t_first = t
            t_last = t

            if args.print_samples:
                values = sample["values"]
                print(
                    f"[{samples:05d}] t={t - start:8.4f}s "
                    f"{feature_name}={tuple(values)}"
                )
    finally:
        imu.close()

    elapsed = max(1e-9, time.perf_counter() - start)
    if samples > 1 and t_first is not None and t_last is not None and t_last > t_first:
        measured_hz = (samples - 1) / (t_last - t_first)
    else:
        measured_hz = samples / elapsed

    print("\n--- Test Summary ---")
    print(f"Requested frequency:       {args.freq:.2f} Hz")
    print(f"Selected feature:          {feature_name}")
    print(f"Programmed interval:       {report_interval_us} us")
    print(f"Nominal programmed freq:   {expected_hz:.2f} Hz")
    print(f"Fresh samples read:        {samples}")
    print(f"Empty/duplicate polls:     {null_reads}")
    print(f"Total loop iterations:     {loop_iters}")
    print(f"Elapsed time:              {elapsed:.3f} s")
    print(f"Measured fresh frequency:  {measured_hz:.2f} Hz")


if __name__ == "__main__":
    main()
