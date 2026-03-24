"""
AK80-9 Trapezoidal Torque Control
==================================
Sends a trapezoidal torque waveform to the AK80-9 motor via CAN bus
using MIT impedance mode (pure torque: kp=0, kd=0).

The waveform ramps from zero up to +PEAK_TORQUE, holds, ramps down through
zero to -PEAK_TORQUE, holds, then ramps back to zero — repeating indefinitely.

A position safety clamp monitors motor feedback and prevents the motor from
exceeding ±POS_MAX degrees (hard limit: ±15°).  The declared operating range
for this profile is -15° to +30°; the tighter ±15° hard stop ensures the motor
stays within the required -15° to +15° window.

Usage:
    sudo ip link set can0 up type can bitrate 1000000
    python3 ak_80_torque_trap.py

Press Ctrl+C to stop.  Motor torque is zeroed on exit.

Adjust PEAK_TORQUE, RAMP_TIME, and HOLD_TIME below to tune the waveform.
"""

import can
import struct
import time
import sys

# ── Configuration ─────────────────────────────────────────────────────────────

CAN_INTERFACE = "can0"
MOTOR_ID      = 0x68          # CAN ID of your AK80-9 motor

# Trapezoidal torque wave parameters
PEAK_TORQUE   = 5.0           # Nm — peak torque magnitude (wave: 0 → +peak → -peak → 0)
RAMP_TIME     = 0.3           # s  — time to ramp between 0 and peak (or between peaks)
HOLD_TIME     = 0.5           # s  — time to dwell at each peak

# Position limits (degrees)
# Operating range declared for this profile: -15° to +30°
# Hard movement limit: motor must not move outside ±POS_LIMIT
POS_RANGE_MIN = -15.0         # declared operating range minimum (°)
POS_RANGE_MAX =  30.0         # declared operating range maximum (°)
POS_LIMIT     =  15.0         # hard movement boundary (°) — applies to both directions

# Loop timing
CONTROL_FREQ  = 200           # Hz
DT            = 1.0 / CONTROL_FREQ

# MIT mode limits for AK80-9
MIT_P_MIN  = -12.56
MIT_P_MAX  =  12.56
MIT_V_MIN  = -65.0
MIT_V_MAX  =  65.0
MIT_T_MIN  = -18.0
MIT_T_MAX  =  18.0
MIT_KP_MIN =   0.0
MIT_KP_MAX = 500.0
MIT_KD_MIN =   0.0
MIT_KD_MAX =   5.0

MODE_MIT = 8

# ── CAN helpers ───────────────────────────────────────────────────────────────

def _clamp(val, lo, hi):
    return max(lo, min(hi, val))


def _float_to_uint(x, x_min, x_max, bits):
    x = _clamp(x, x_min, x_max)
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) / span))


def send_torque(bus, torque_nm):
    """Send a pure torque command via MIT mode (kp=0, kd=0, pos=0, vel=0)."""
    torque_nm = _clamp(torque_nm, MIT_T_MIN, MIT_T_MAX)

    kp_int = _float_to_uint(0.0,       MIT_KP_MIN, MIT_KP_MAX, 12)
    kd_int = _float_to_uint(0.0,       MIT_KD_MIN, MIT_KD_MAX, 12)
    p_int  = _float_to_uint(0.0,       MIT_P_MIN,  MIT_P_MAX,  16)
    v_int  = _float_to_uint(0.0,       MIT_V_MIN,  MIT_V_MAX,  12)
    t_int  = _float_to_uint(torque_nm, MIT_T_MIN,  MIT_T_MAX,  12)

    buf = [0] * 8
    buf[0] =  kp_int >> 4
    buf[1] = ((kp_int & 0xF) << 4) | (kd_int >> 8)
    buf[2] =  kd_int & 0xFF
    buf[3] =  p_int  >> 8
    buf[4] =  p_int  & 0xFF
    buf[5] =  v_int  >> 4
    buf[6] = ((v_int  & 0xF) << 4) | (t_int >> 8)
    buf[7] =  t_int  & 0xFF

    arb_id = (MODE_MIT << 8) | MOTOR_ID
    msg = can.Message(arbitration_id=arb_id, data=buf, is_extended_id=True)
    bus.send(msg)


def read_feedback(bus):
    """Read one feedback frame. Returns a dict or None if nothing arrived."""
    msg = bus.recv(timeout=0.001)
    if msg is None or len(msg.data) < 8:
        return None
    return {
        "position":    struct.unpack(">h", bytes(msg.data[0:2]))[0] * 0.1,   # degrees
        "speed":       struct.unpack(">h", bytes(msg.data[2:4]))[0] * 10.0,  # ERPM
        "current":     struct.unpack(">h", bytes(msg.data[4:6]))[0] * 0.01,  # Amps
        "temperature": msg.data[6],                                            # °C
        "error":       msg.data[7],
    }

# ── Trapezoidal waveform ───────────────────────────────────────────────────────

def trapezoid_torque(t, peak, ramp_time, hold_time):
    """
    Return the trapezoidal torque value at elapsed time *t*.

    One complete cycle (period = 4*ramp_time + 2*hold_time):

      Phase 1  [0,          r)        ramp  0      → +peak
      Phase 2  [r,          r+h)      hold  +peak
      Phase 3  [r+h,        3r+h)     ramp  +peak  → -peak  (through zero)
      Phase 4  [3r+h,       3r+2h)    hold  -peak
      Phase 5  [3r+2h,      4r+2h)    ramp  -peak  → 0
    """
    r = ramp_time
    h = hold_time
    period = 4 * r + 2 * h
    phase  = t % period

    if phase < r:
        # Ramp up: 0 → +peak
        return peak * (phase / r)
    elif phase < r + h:
        # Hold at +peak
        return peak
    elif phase < 3 * r + h:
        # Ramp: +peak → -peak
        progress = (phase - (r + h)) / (2 * r)
        return peak * (1.0 - 2.0 * progress)
    elif phase < 3 * r + 2 * h:
        # Hold at -peak
        return -peak
    else:
        # Ramp up: -peak → 0
        progress = (phase - (3 * r + 2 * h)) / r
        return -peak * (1.0 - progress)

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    try:
        bus = can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan")
    except Exception as e:
        print(f"ERROR: Could not open CAN interface '{CAN_INTERFACE}': {e}")
        print("Run:  sudo ip link set can0 up type can bitrate 1000000")
        sys.exit(1)

    period = 4 * RAMP_TIME + 2 * HOLD_TIME
    print("AK80-9 Trapezoidal Torque Control")
    print(f"  Peak torque:     ±{PEAK_TORQUE} Nm")
    print(f"  Ramp time:       {RAMP_TIME} s")
    print(f"  Hold time:       {HOLD_TIME} s")
    print(f"  Cycle period:    {period:.2f} s  ({1/period:.3f} Hz)")
    print(f"  Loop rate:       {CONTROL_FREQ} Hz")
    print(f"  Operating range: {POS_RANGE_MIN}° to {POS_RANGE_MAX}°")
    print(f"  Hard position limit: ±{POS_LIMIT}°  (motor stays within -{POS_LIMIT}° to +{POS_LIMIT}°)")
    print("\nPress Ctrl+C to stop.\n")
    print(f"  {'Time':>8}  {'Wave Torque':>11}  {'Sent Torque':>11}  {'Position':>10}  {'Current':>9}  {'Temp':>6}")
    print("  " + "-" * 72)

    t_start      = time.perf_counter()
    last_pos     = 0.0          # last known motor position (degrees)
    print_counter = 0

    try:
        while True:
            loop_start = time.perf_counter()
            t = loop_start - t_start

            # ── 1. Compute trapezoidal torque for this instant ──────────────
            wave_torque = trapezoid_torque(t, PEAK_TORQUE, RAMP_TIME, HOLD_TIME)

            # ── 2. Position safety clamp (hard ±POS_LIMIT limit) ───────────
            # If the motor is at or beyond the positive limit, do not allow
            # any further positive torque (only negative torque can pull it back).
            # Mirror logic applies at the negative limit.
            sent_torque = wave_torque
            if last_pos >= POS_LIMIT:
                sent_torque = min(sent_torque, 0.0)
            elif last_pos <= -POS_LIMIT:
                sent_torque = max(sent_torque, 0.0)

            # ── 3. Send command ─────────────────────────────────────────────
            send_torque(bus, sent_torque)

            # ── 4. Read feedback ────────────────────────────────────────────
            fb = read_feedback(bus)
            if fb is not None:
                last_pos = fb["position"]

            # ── 5. Print status (every 20 loops ≈ 10 Hz) ───────────────────
            print_counter += 1
            if print_counter >= 20:
                print_counter = 0
                if fb is not None:
                    print(f"  {t:>7.2f}s  {wave_torque:>+10.2f} Nm  {sent_torque:>+10.2f} Nm"
                          f"  {fb['position']:>9.1f}°"
                          f"  {fb['current']:>+7.2f} A  {fb['temperature']:>5}°C")
                else:
                    print(f"  {t:>7.2f}s  {wave_torque:>+10.2f} Nm  {sent_torque:>+10.2f} Nm"
                          f"  {'(no feedback)':>32}")

            # ── 6. Maintain loop rate ───────────────────────────────────────
            elapsed    = time.perf_counter() - loop_start
            sleep_time = DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\nStopping — zeroing motor torque ...")
        for _ in range(10):
            send_torque(bus, 0.0)
            time.sleep(0.005)
        print("Motor torque zeroed.")

    finally:
        bus.shutdown()
        print("CAN bus closed.  Done.")


if __name__ == "__main__":
    main()
