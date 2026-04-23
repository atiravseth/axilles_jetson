#!/usr/bin/env python3
"""
TBE Controller Dashboard

Live GUI that receives UDP telemetry from the TBE controller and displays:
  - Heel strike / toe off indicator lights
  - Torque vs. gait phase plot
  - Torque history plot
  - System state (Calibration, Activation, Off, E-Stop)

Run:
    python3 tbe_dashboard.py

Requires: matplotlib, tkinter (built-in)
"""

from __future__ import annotations

import socket
import threading
import time
import tkinter as tk
from collections import deque

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# ── UDP Config ────────────────────────────────────────────────────────────────
UDP_HOST = "127.0.0.1"
UDP_PORT = 47270          # Separate port from Teleplot (47269)

# ── Plot history ──────────────────────────────────────────────────────────────
HISTORY_LEN = 500         # ~3.3 s at 150 Hz

# ── Colors ────────────────────────────────────────────────────────────────────
BG_DARK      = "#1a1a2e"
BG_PANEL     = "#16213e"
BG_CARD      = "#0f3460"
TEXT_PRIMARY  = "#e0e0e0"
TEXT_DIM      = "#8a8a9a"
GREEN_ON      = "#00e676"
GREEN_OFF     = "#1b3a1b"
RED_ON        = "#ff1744"
RED_OFF       = "#3a1b1b"
ORANGE_ON     = "#ff9100"
BLUE_ACCENT   = "#00b0ff"
CYAN_ACCENT   = "#00e5ff"
PLOT_BG       = "#0a0a1a"
GRID_COLOR    = "#1a2a3a"

# ── State labels ──────────────────────────────────────────────────────────────
STATE_MAP = {
    0: ("OFF",         "#607d8b"),
    1: ("CALIBRATING", "#ffc107"),
    2: ("ACTIVE",      GREEN_ON),
    3: ("E-STOP",      RED_ON),
}


class TelemetryReceiver:
    """Background thread that receives UDP telemetry packets."""

    def __init__(self, host: str, port: int) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.settimeout(0.05)

        self.data: dict[str, float] = {}
        self.lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while self._running:
            try:
                raw, _ = self.sock.recvfrom(4096)
                self._parse(raw.decode("utf-8", errors="ignore"))
            except socket.timeout:
                continue
            except OSError:
                break

    def _parse(self, msg: str) -> None:
        # Format: "name:timestamp_ms:value"
        try:
            parts = msg.split("|")[0]   # strip flags like |g
            name, _, value = parts.rsplit(":", 2)
            # Strip group suffix if present (e.g. "heel_fsr,FSR" -> "heel_fsr")
            key = name.split(",")[0].strip()
            with self.lock:
                self.data[key] = float(value)
        except (ValueError, IndexError):
            pass

    def snapshot(self) -> dict[str, float]:
        with self.lock:
            return dict(self.data)

    def stop(self) -> None:
        self._running = False
        self.sock.close()


class TBEDashboard:
    """Main dashboard window."""

    def __init__(self) -> None:
        self.receiver = TelemetryReceiver(UDP_HOST, UDP_PORT)

        # History buffers
        self.torque_history = deque(maxlen=HISTORY_LEN)
        self.phase_history = deque(maxlen=HISTORY_LEN)
        self.heel_fsr_history = deque(maxlen=HISTORY_LEN)
        self.toe_fsr_history = deque(maxlen=HISTORY_LEN)
        self.time_history = deque(maxlen=HISTORY_LEN)
        self._t0 = time.time()

        self._build_ui()

    # ── UI Construction ───────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self.root = tk.Tk()
        self.root.title("TBE Controller Dashboard")
        self.root.configure(bg=BG_DARK)
        self.root.geometry("1100x720")
        self.root.minsize(900, 600)

        # Title bar
        title_frame = tk.Frame(self.root, bg=BG_DARK)
        title_frame.pack(fill=tk.X, padx=16, pady=(12, 4))
        tk.Label(
            title_frame, text="TBE CONTROLLER", font=("Courier New", 20, "bold"),
            fg=CYAN_ACCENT, bg=BG_DARK
        ).pack(side=tk.LEFT)
        self.state_label = tk.Label(
            title_frame, text="  OFF  ", font=("Courier New", 14, "bold"),
            fg="#607d8b", bg=BG_DARK, padx=12
        )
        self.state_label.pack(side=tk.RIGHT)

        # Top row: indicators
        ind_frame = tk.Frame(self.root, bg=BG_DARK)
        ind_frame.pack(fill=tk.X, padx=16, pady=8)

        self.heel_card = self._make_indicator(ind_frame, "HEEL STRIKE", GREEN_OFF)
        self.toe_card = self._make_indicator(ind_frame, "TOE OFF", GREEN_OFF)
        self.motor_card = self._make_indicator(ind_frame, "MOTOR", GREEN_OFF)
        self.torque_value_label = self._make_value_card(ind_frame, "TORQUE CMD", "0.00 Nm")
        self.phase_value_label = self._make_value_card(ind_frame, "GAIT PHASE", "0 %")
        self.angle_value_label = self._make_value_card(ind_frame, "ANKLE ANGLE", "0.0 °")

        # Plots
        plot_frame = tk.Frame(self.root, bg=BG_DARK)
        plot_frame.pack(fill=tk.BOTH, expand=True, padx=16, pady=(4, 12))

        self.fig = Figure(figsize=(10, 4), facecolor=PLOT_BG)
        self.fig.subplots_adjust(hspace=0.45, left=0.08, right=0.97, top=0.92, bottom=0.12)

        # Torque history plot
        self.ax_torque = self.fig.add_subplot(2, 1, 1)
        self._style_axis(self.ax_torque, "Torque Command (Nm)")
        self.torque_line, = self.ax_torque.plot([], [], color=CYAN_ACCENT, linewidth=1.5)

        # FSR plot
        self.ax_fsr = self.fig.add_subplot(2, 1, 2)
        self._style_axis(self.ax_fsr, "FSR Filtered")
        self.heel_line, = self.ax_fsr.plot([], [], color=GREEN_ON, linewidth=1.2, label="Heel")
        self.toe_line, = self.ax_fsr.plot([], [], color=ORANGE_ON, linewidth=1.2, label="Toe")
        self.ax_fsr.legend(loc="upper right", fontsize=8, facecolor=PLOT_BG,
                           edgecolor=GRID_COLOR, labelcolor=TEXT_DIM)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Start update loop
        self._update()

    def _style_axis(self, ax, ylabel: str) -> None:
        ax.set_facecolor(PLOT_BG)
        ax.set_ylabel(ylabel, color=TEXT_DIM, fontsize=9)
        ax.tick_params(colors=TEXT_DIM, labelsize=8)
        ax.grid(True, color=GRID_COLOR, linewidth=0.4, alpha=0.6)
        for spine in ax.spines.values():
            spine.set_color(GRID_COLOR)

    def _make_indicator(self, parent, label_text: str, initial_color: str) -> dict:
        card = tk.Frame(parent, bg=BG_CARD, padx=14, pady=10, relief=tk.FLAT,
                        highlightbackground="#1a3a5c", highlightthickness=1)
        card.pack(side=tk.LEFT, padx=4, expand=True, fill=tk.X)

        lbl = tk.Label(card, text=label_text, font=("Courier New", 9),
                       fg=TEXT_DIM, bg=BG_CARD)
        lbl.pack()

        light = tk.Canvas(card, width=28, height=28, bg=BG_CARD, highlightthickness=0)
        light.pack(pady=(6, 0))
        circle = light.create_oval(2, 2, 26, 26, fill=initial_color, outline="")

        return {"canvas": light, "circle": circle}

    def _make_value_card(self, parent, label_text: str, initial_value: str) -> tk.Label:
        card = tk.Frame(parent, bg=BG_CARD, padx=14, pady=10, relief=tk.FLAT,
                        highlightbackground="#1a3a5c", highlightthickness=1)
        card.pack(side=tk.LEFT, padx=4, expand=True, fill=tk.X)

        tk.Label(card, text=label_text, font=("Courier New", 9),
                 fg=TEXT_DIM, bg=BG_CARD).pack()

        val = tk.Label(card, text=initial_value, font=("Courier New", 14, "bold"),
                       fg=TEXT_PRIMARY, bg=BG_CARD)
        val.pack(pady=(4, 0))
        return val

    # ── Indicator helpers ─────────────────────────────────────────────────────

    def _set_light(self, indicator: dict, on: bool, on_color: str = GREEN_ON,
                   off_color: str = GREEN_OFF) -> None:
        color = on_color if on else off_color
        indicator["canvas"].itemconfigure(indicator["circle"], fill=color)

    # ── Update loop ───────────────────────────────────────────────────────────

    def _update(self) -> None:
        snap = self.receiver.snapshot()
        now = time.time() - self._t0

        # Extract values with defaults
        heel_fsr = snap.get("heel_fsr_filt", 0.0)
        toe_fsr = snap.get("toe_fsr_filt", 0.0)
        heel_on = snap.get("heel_strike_on", 0.0) > 0.5
        toe_off = snap.get("toe_off_on", 0.0) > 0.5
        torque = snap.get("torque_cmd", 0.0)
        phase = snap.get("gait_phase", 0.0)
        angle = snap.get("ankle_angle", 0.0)
        motor_alive = snap.get("motor_alive", 1.0) > 0.5
        state = int(snap.get("system_state", 0.0))

        # Update indicators
        self._set_light(self.heel_card, heel_on, GREEN_ON, GREEN_OFF)
        self._set_light(self.toe_card, toe_off, ORANGE_ON, RED_OFF)
        self._set_light(self.motor_card, motor_alive, GREEN_ON, RED_ON)

        # Update value cards
        self.torque_value_label.config(text=f"{torque:.2f} Nm")
        self.phase_value_label.config(text=f"{phase * 100:.0f} %")
        self.angle_value_label.config(text=f"{angle:.1f} °")

        # Update system state
        state_text, state_color = STATE_MAP.get(state, ("UNKNOWN", TEXT_DIM))
        self.state_label.config(text=f"  {state_text}  ", fg=state_color)

        # Append history
        self.time_history.append(now)
        self.torque_history.append(torque)
        self.heel_fsr_history.append(heel_fsr)
        self.toe_fsr_history.append(toe_fsr)

        # Update plots
        t = list(self.time_history)
        if len(t) > 1:
            self.torque_line.set_data(t, list(self.torque_history))
            self.ax_torque.set_xlim(t[0], t[-1])
            torque_vals = list(self.torque_history)
            tmin = min(torque_vals) - 0.5
            tmax = max(torque_vals) + 0.5
            self.ax_torque.set_ylim(tmin, tmax)

            self.heel_line.set_data(t, list(self.heel_fsr_history))
            self.toe_line.set_data(t, list(self.toe_fsr_history))
            self.ax_fsr.set_xlim(t[0], t[-1])
            fsr_all = list(self.heel_fsr_history) + list(self.toe_fsr_history)
            fmin = min(fsr_all) - 500
            fmax = max(fsr_all) + 500
            self.ax_fsr.set_ylim(fmin, fmax)

            self.canvas.draw_idle()

        # Schedule next update at ~30 Hz (GUI refresh rate)
        self.root.after(33, self._update)

    # ── Run ────────────────────────────────────────────────────────────────────

    def run(self) -> None:
        try:
            self.root.mainloop()
        finally:
            self.receiver.stop()


def main() -> None:
    print(f"TBE Dashboard listening on UDP {UDP_HOST}:{UDP_PORT}")
    print("Start your TBE controller to see live data.")
    dashboard = TBEDashboard()
    dashboard.run()


if __name__ == "__main__":
    main()