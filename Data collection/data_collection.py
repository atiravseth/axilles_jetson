#!/usr/bin/env python3
"""
data_collection.py

Standalone CSV logger for:
- IMU foot  (BNO085): accel(3) + gyro(3) + optional orientation (Euler or quat)
- IMU shank (BNO085): accel(3) + gyro(3) + optional orientation (Euler or quat)
- Ankle encoder (AS5600): angle (deg)
- Toe FSR (ADS1115 AIN0): voltage
- Heel FSR (ADS1115 AIN1): voltage

Run:
    python3 "Data collection/data_collection.py"
"""

from __future__ import annotations

import csv
import math
import signal
import time
import warnings
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

import smbus2  # type: ignore[import-not-found]

warnings.filterwarnings(
    "ignore",
    category=RuntimeWarning,
    message="I2C frequency is not settable",
)

from adafruit_bno08x import (  # type: ignore[import-not-found]
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
)
from adafruit_bno08x.i2c import BNO08X_I2C  # type: ignore[import-not-found]
from adafruit_extended_bus import ExtendedI2C as ExtI2C  # type: ignore[import-not-found]


# ========================= User Config =========================
# Per-sensor-type acquisition rates (Hz)
IMU_HZ = 200.0
ENCODER_HZ = 200.0
FSR_HZ = 200.0

# I2C configuration (explicit sensor addresses)
I2C_BUS = 1
IMU_FOOT_ADDR = 0x4A
IMU_SHANK_ADDR = 0x4B
ADS_ADDR = 0x48
AS5600_ADDR = 0x36

# IMU reconnect and setup behavior
RECONNECT_DELAY = 1.0
BOOT_DELAY = 0.8
FEATURE_RETRIES = 5

# Choose orientation output: "euler" or "quat"
IMU_ORIENTATION_MODE = "euler"

# FSR key mapping from standalone SensorHub frame
TOE_FSR_KEY = "toe_fsr_v"
HEEL_FSR_KEY = "heel_fsr_v"

# Output file naming
OUTPUT_DIR = Path(__file__).resolve().parent
OUTPUT_PREFIX = "data_collection"
# ==============================================================


# ADS1115 registers/config
_ADS_REG_CONV = 0x00
_ADS_REG_CFG = 0x01
_ADS_CFG_AIN0 = [0xC3, 0xE3]
_ADS_CFG_AIN1 = [0xD3, 0xE3]
_ADS_LSB_V = 4.096 / 32767.0

# AS5600 registers
_AS5600_REG_ANGLE = 0x0E


def quat_to_euler(qi: float, qj: float, qk: float, qr: float) -> tuple[float, float, float]:
    # Quaternion -> intrinsic XYZ (roll, pitch, yaw) in degrees.
    sinr_cosp = 2.0 * (qr * qi + qj * qk)
    cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (qr * qj - qk * qi)
    if abs(sinp) >= 1.0:
        pitch = math.degrees(math.copysign(math.pi / 2.0, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (qr * qk + qi * qj)
    cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    return roll, pitch, yaw


class _FastIMU:
    """Single BNO085 using packet-drain read for high throughput."""

    def __init__(self, bus: int, address: int, label: str):
        self._bus = bus
        self._address = address
        self._label = label
        self._i2c = None
        self._bno = None
        self._connect()

    def _connect(self) -> None:
        while True:
            try:
                if self._i2c is not None:
                    try:
                        self._i2c.deinit()
                    except Exception:
                        pass
                    time.sleep(0.3)

                self._i2c = ExtI2C(self._bus)
                self._bno = BNO08X_I2C(self._i2c, address=self._address)
                time.sleep(BOOT_DELAY)

                for feat in (
                    BNO_REPORT_GAME_ROTATION_VECTOR,
                    BNO_REPORT_ACCELEROMETER,
                    BNO_REPORT_GYROSCOPE,
                ):
                    for attempt in range(FEATURE_RETRIES):
                        try:
                            self._bno.enable_feature(feat)
                            break
                        except Exception:
                            if attempt == FEATURE_RETRIES - 1:
                                raise
                            time.sleep(0.2)
                return
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                print(
                    f"[{self._label}] Connect failed ({exc}), retrying in {RECONNECT_DELAY}s...",
                    flush=True,
                )
                time.sleep(RECONNECT_DELAY)

    def _reconnect(self) -> None:
        print(f"\n[{self._label}] Reconnecting...", flush=True)
        self._connect()
        print(f"[{self._label}] Reconnected.\n", flush=True)

    def read(self) -> Optional[dict]:
        try:
            self._bno._process_available_packets()
            accel = self._bno._readings.get(BNO_REPORT_ACCELEROMETER)
            gyro = self._bno._readings.get(BNO_REPORT_GYROSCOPE)
            quat = self._bno._readings.get(BNO_REPORT_GAME_ROTATION_VECTOR)
        except (OSError, RuntimeError, AttributeError, KeyError) as exc:
            print(
                f"\n[{self._label}] Error ({type(exc).__name__}: {exc}), reconnecting...",
                flush=True,
            )
            self._reconnect()
            return None

        if accel is None or gyro is None or quat is None:
            return None

        return {"quat": quat, "accel": accel, "gyro": gyro}

    def close(self) -> None:
        try:
            self._i2c.deinit()
        except Exception:
            pass


class _ADS1115:
    """Non-blocking 2-channel ADS1115 reader in single-shot mode."""

    def __init__(self, bus_handle: smbus2.SMBus, address: int = ADS_ADDR):
        self._bus = bus_handle
        self._addr = address
        self._ch = 0
        self._v = [0.0, 0.0]
        self._trigger(0)

    def _trigger(self, ch: int) -> None:
        cfg = _ADS_CFG_AIN0 if ch == 0 else _ADS_CFG_AIN1
        self._bus.write_i2c_block_data(self._addr, _ADS_REG_CFG, cfg)
        self._ch = ch

    def _is_ready(self) -> bool:
        data = self._bus.read_i2c_block_data(self._addr, _ADS_REG_CFG, 2)
        return bool(data[0] & 0x80)

    def _read_raw(self) -> int:
        data = self._bus.read_i2c_block_data(self._addr, _ADS_REG_CONV, 2)
        raw = (data[0] << 8) | data[1]
        return raw if raw < 0x8000 else raw - 0x10000

    def update(self) -> None:
        try:
            if self._is_ready():
                raw = self._read_raw()
                self._v[self._ch] = max(0.0, raw * _ADS_LSB_V)
                self._trigger(1 - self._ch)
        except OSError:
            pass

    @property
    def toe_fsr_v(self) -> float:
        return self._v[0]

    @property
    def heel_fsr_v(self) -> float:
        return self._v[1]


class _AS5600:
    """AS5600 angle reader."""

    def __init__(self, bus_handle: smbus2.SMBus, address: int = AS5600_ADDR):
        self._bus = bus_handle
        self._addr = address
        self._angle = 0.0

    def update(self) -> None:
        try:
            data = self._bus.read_i2c_block_data(self._addr, _AS5600_REG_ANGLE, 2)
            raw = ((data[0] & 0x0F) << 8) | data[1]
            self._angle = raw * (360.0 / 4096.0)
        except OSError:
            pass

    @property
    def angle_deg(self) -> float:
        return self._angle


class SensorHub:
    """Standalone I2C sensor hub for foot/shank IMUs, encoder, and FSR ADC."""

    def __init__(
        self,
        bus: int = I2C_BUS,
        imu_foot_addr: int = IMU_FOOT_ADDR,
        imu_shank_addr: int = IMU_SHANK_ADDR,
        ads_addr: int = ADS_ADDR,
        enc_addr: int = AS5600_ADDR,
    ):
        print(f"[Hub] Connecting IMU-foot (bus {bus}, 0x{imu_foot_addr:02X})...")
        self._imu_foot = _FastIMU(bus, imu_foot_addr, "IMU-foot")
        print("[Hub] IMU-foot connected.")

        print(f"[Hub] Connecting IMU-shank (bus {bus}, 0x{imu_shank_addr:02X})...")
        self._imu_shank = _FastIMU(bus, imu_shank_addr, "IMU-shank")
        print("[Hub] IMU-shank connected.")

        print(f"[Hub] Opening smbus2 on bus {bus} for ADS1115 + AS5600...")
        self._smbus = smbus2.SMBus(bus)

        print(f"[Hub] Connecting ADS1115 (0x{ads_addr:02X})...")
        self._ads = _ADS1115(self._smbus, ads_addr)
        print("[Hub] ADS1115 connected.")

        print(f"[Hub] Connecting AS5600 (0x{enc_addr:02X})...")
        self._enc = _AS5600(self._smbus, enc_addr)
        print("[Hub] AS5600 connected.")

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self) -> None:
        self._imu_foot.close()
        self._imu_shank.close()
        try:
            self._smbus.close()
        except Exception:
            pass

    def read(self) -> dict:
        imu_foot = self._imu_foot.read()
        imu_shank = self._imu_shank.read()
        self._ads.update()
        self._enc.update()
        return {
            "imu_foot": imu_foot,
            "imu_shank": imu_shank,
            "toe_fsr_v": self._ads.toe_fsr_v,
            "heel_fsr_v": self._ads.heel_fsr_v,
            "ankle_encoder_deg": self._enc.angle_deg,
        }


def _period_from_hz(hz: float) -> float:
    if hz <= 0:
        return float("inf")
    return 1.0 / hz


def _next_time(last_t: float, period: float, now_t: float) -> float:
    if period == float("inf"):
        return float("inf")
    next_t = last_t + period
    if next_t <= now_t:
        # Catch up in one step if loop falls behind.
        next_t = now_t + period
    return next_t


def _empty_or(v: Optional[float]) -> str | float:
    return "" if v is None else v


def _extract_imu_fields(imu_packet: Optional[dict], mode: str, prefix: str) -> Dict[str, Optional[float]]:
    fields: Dict[str, Optional[float]] = {
        f"{prefix}_ax": None,
        f"{prefix}_ay": None,
        f"{prefix}_az": None,
        f"{prefix}_gx": None,
        f"{prefix}_gy": None,
        f"{prefix}_gz": None,
    }

    if mode == "euler":
        fields.update(
            {
                f"{prefix}_roll_deg": None,
                f"{prefix}_pitch_deg": None,
                f"{prefix}_yaw_deg": None,
            }
        )
    else:
        fields.update(
            {
                f"{prefix}_qi": None,
                f"{prefix}_qj": None,
                f"{prefix}_qk": None,
                f"{prefix}_qr": None,
            }
        )

    if imu_packet is None:
        return fields

    accel = imu_packet.get("accel")
    gyro = imu_packet.get("gyro")
    quat = imu_packet.get("quat")

    if accel is not None and len(accel) >= 3:
        fields[f"{prefix}_ax"], fields[f"{prefix}_ay"], fields[f"{prefix}_az"] = accel[0], accel[1], accel[2]

    if gyro is not None and len(gyro) >= 3:
        fields[f"{prefix}_gx"], fields[f"{prefix}_gy"], fields[f"{prefix}_gz"] = gyro[0], gyro[1], gyro[2]

    if quat is not None and len(quat) >= 4:
        qi, qj, qk, qr = quat[0], quat[1], quat[2], quat[3]
        if mode == "euler":
            roll, pitch, yaw = quat_to_euler(qi, qj, qk, qr)
            fields[f"{prefix}_roll_deg"] = roll
            fields[f"{prefix}_pitch_deg"] = pitch
            fields[f"{prefix}_yaw_deg"] = yaw
        else:
            fields[f"{prefix}_qi"] = qi
            fields[f"{prefix}_qj"] = qj
            fields[f"{prefix}_qk"] = qk
            fields[f"{prefix}_qr"] = qr

    return fields


def _build_header(mode: str) -> list[str]:
    header = ["timestamp_s"]

    for prefix in ("foot", "shank"):
        header.extend(
            [
                f"{prefix}_ax",
                f"{prefix}_ay",
                f"{prefix}_az",
                f"{prefix}_gx",
                f"{prefix}_gy",
                f"{prefix}_gz",
            ]
        )
        if mode == "euler":
            header.extend([f"{prefix}_roll_deg", f"{prefix}_pitch_deg", f"{prefix}_yaw_deg"])
        else:
            header.extend([f"{prefix}_qi", f"{prefix}_qj", f"{prefix}_qk", f"{prefix}_qr"])

    header.extend(["ankle_encoder_deg", "toe_fsr_v", "heel_fsr_v"])
    return header


def main() -> None:
    mode = IMU_ORIENTATION_MODE.strip().lower()
    if mode not in {"euler", "quat"}:
        raise ValueError("IMU_ORIENTATION_MODE must be 'euler' or 'quat'")

    imu_period = _period_from_hz(IMU_HZ)
    enc_period = _period_from_hz(ENCODER_HZ)
    fsr_period = _period_from_hz(FSR_HZ)

    min_period = min(imu_period, enc_period, fsr_period)
    sleep_hint = 0.0 if min_period == float("inf") else min(0.001, min_period * 0.2)

    timestamp_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_file = OUTPUT_DIR / f"{OUTPUT_PREFIX}_{timestamp_tag}.csv"

    stop_requested = False

    def _handle_stop(_sig, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    header = _build_header(mode)
    latest: Dict[str, Optional[float]] = {k: None for k in header}

    print(f"[Logger] Writing CSV to: {out_file}")
    print(
        f"[Logger] Rates -> IMU: {IMU_HZ} Hz, Encoder: {ENCODER_HZ} Hz, FSR: {FSR_HZ} Hz | "
        f"IMU orientation mode: {mode}"
    )

    with SensorHub() as hub, out_file.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()

        t0 = time.perf_counter()
        next_imu_t = t0
        next_enc_t = t0
        next_fsr_t = t0

        rows_written = 0

        while not stop_requested:
            now = time.perf_counter()
            frame = hub.read()

            wrote_this_cycle = False

            if now >= next_imu_t:
                latest.update(_extract_imu_fields(frame.get("imu_foot"), mode, "foot"))
                latest.update(_extract_imu_fields(frame.get("imu_shank"), mode, "shank"))
                next_imu_t = _next_time(next_imu_t, imu_period, now)
                wrote_this_cycle = True

            if now >= next_enc_t:
                latest["ankle_encoder_deg"] = frame.get("ankle_encoder_deg")
                next_enc_t = _next_time(next_enc_t, enc_period, now)
                wrote_this_cycle = True

            if now >= next_fsr_t:
                latest["toe_fsr_v"] = frame.get(TOE_FSR_KEY)
                latest["heel_fsr_v"] = frame.get(HEEL_FSR_KEY)
                next_fsr_t = _next_time(next_fsr_t, fsr_period, now)
                wrote_this_cycle = True

            if wrote_this_cycle:
                row = {k: _empty_or(latest.get(k)) for k in header}
                row["timestamp_s"] = now - t0
                writer.writerow(row)
                rows_written += 1

            if sleep_hint > 0:
                time.sleep(sleep_hint)

    print(f"[Logger] Stopped. Rows written: {rows_written}")


if __name__ == "__main__":
    main()
