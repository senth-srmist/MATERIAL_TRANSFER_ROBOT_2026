#!/usr/bin/env python3
"""
Real-time PID tuning plotter.

Subscribes to /pid/debug (Float32MultiArray, 31 floats) and plots a rolling
window of the most useful tuning signals.

Usage:
    ros2 run diff_drive_controller pid_plotter              # 15 s window
    ros2 run diff_drive_controller pid_plotter --window 30  # 30 s window
    python3 pid_plotter.py --window 30                      # standalone

Layout (4 rows):
  Row 1 — Wheel speeds: desired vs actual (left & right)
  Row 2 — Per-wheel error (desired - actual)
  Row 3 — PID term decomposition: FF, P, I, D per wheel
  Row 4 — Body-space tracking: linear error and angular error

/pid/debug index map (31 floats):
  [0]  dt
  [1]  raw_v          [2]  raw_w
  [3]  rl_v           [4]  rl_w
  [5]  des_left       [6]  des_right
  [7]  act_left       [8]  act_right
  [9]  left_err      [10]  lp   [11] li   [12] ld   [13] ff_l  [14] out_l
  [15] right_err     [16]  rp   [17] ri   [18] rd   [19] ff_r  [20] out_r
  [21] output_left   [22]  output_right
  [23] lin_track_err [24]  ang_track_err
  [25] kp_l  [26] ki_l  [27] kd_l
  [28] kp_r  [29] ki_r  [30] kd_r
"""

import argparse
import collections
import sys
import threading

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from std_msgs.msg import Float32MultiArray
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

PUBLISH_RATE = 10.0  # Hz — must match pid_controller control_rate
REFRESH_MS   = 100   # plot refresh interval in ms


def parse_args():
    p = argparse.ArgumentParser(description="PID live plotter")
    p.add_argument("--window", type=float, default=15.0,
                   help="Rolling time window in seconds (default: 15)")
    # ROS2 passes extra --ros-args; ignore them
    known, _ = p.parse_known_args()
    return known


# ──────────────────────────────────────────────────────────────
# Thread-safe ring buffer
# ──────────────────────────────────────────────────────────────

class RingBuffer:
    def __init__(self, maxlen: int, ncols: int):
        self._buf  = collections.deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._ncols = ncols

    def append(self, row):
        with self._lock:
            self._buf.append(row)

    def snapshot(self):
        with self._lock:
            if not self._buf:
                return np.empty((0, self._ncols))
            return np.array(self._buf)


# ──────────────────────────────────────────────────────────────
# ROS2 subscriber (runs in its own thread)
# ──────────────────────────────────────────────────────────────

class PIDListener(Node):
    def __init__(self, buf: RingBuffer):
        super().__init__("pid_plotter")
        self._buf = buf
        self._t   = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Float32MultiArray, "/pid/debug", self._cb, qos)
        self.get_logger().info("pid_plotter listening on /pid/debug")

    def _cb(self, msg):
        if len(msg.data) < 31:
            return
        d = msg.data
        dt = d[0] if d[0] > 0 else 1.0 / PUBLISH_RATE
        self._t += dt
        # row: [t, des_l, des_r, act_l, act_r, err_l, err_r,
        #        lp, li, ld, ff_l, out_l,
        #        rp, ri, rd, ff_r, out_r,
        #        lin_err, ang_err]
        row = [
            self._t,
            d[5], d[6],    # desired left, right
            d[7], d[8],    # actual  left, right
            d[9], d[15],   # error   left, right
            d[10], d[11], d[12], d[13], d[14],   # lP lI lD lFF lOut
            d[16], d[17], d[18], d[19], d[20],   # rP rI rD rFF rOut
            d[23], d[24],  # body lin/ang tracking error
        ]
        self._buf.append(row)


# ──────────────────────────────────────────────────────────────
# Plotter
# ──────────────────────────────────────────────────────────────

COL = {
    "t":      0,
    "des_l":  1, "des_r":  2,
    "act_l":  3, "act_r":  4,
    "err_l":  5, "err_r":  6,
    "lp":     7, "li":     8,  "ld":     9,  "lff": 10, "lout": 11,
    "rp":    12, "ri":    13,  "rd":    14,  "rff": 15, "rout": 16,
    "lin_e": 17, "ang_e": 18,
}
NCOLS = 19


def make_figure():
    fig = plt.figure(figsize=(14, 9))
    fig.suptitle("PID Tuning — /pid/debug", fontsize=11, fontweight="bold")
    gs = gridspec.GridSpec(4, 1, figure=fig, hspace=0.55)

    ax_speed = fig.add_subplot(gs[0])
    ax_error = fig.add_subplot(gs[1])
    ax_terms = fig.add_subplot(gs[2])
    ax_body  = fig.add_subplot(gs[3])

    # ── Row 1: wheel speeds ──
    ax_speed.set_title("Wheel speeds  (rad/s)", fontsize=9)
    ax_speed.set_ylabel("rad/s")
    ln_dl, = ax_speed.plot([], [], "b-",  lw=1.4, label="des left")
    ln_dr, = ax_speed.plot([], [], "r-",  lw=1.4, label="des right")
    ln_al, = ax_speed.plot([], [], "b--", lw=1.0, label="act left")
    ln_ar, = ax_speed.plot([], [], "r--", lw=1.0, label="act right")
    ax_speed.axhline(0, color="k", lw=0.5)
    ax_speed.legend(loc="upper left", fontsize=7, ncol=4)

    # ── Row 2: per-wheel error ──
    ax_error.set_title("Per-wheel error  desired − actual  (rad/s)", fontsize=9)
    ax_error.set_ylabel("rad/s")
    ln_el, = ax_error.plot([], [], "b-", lw=1.2, label="left error")
    ln_er, = ax_error.plot([], [], "r-", lw=1.2, label="right error")
    ax_error.axhline(0, color="k", lw=0.5)
    ax_error.legend(loc="upper left", fontsize=7, ncol=2)

    # ── Row 3: PID term decomposition ──
    ax_terms.set_title("PID terms  (left solid / right dashed)", fontsize=9)
    ax_terms.set_ylabel("rad/s")
    ln_lff, = ax_terms.plot([], [], "g-",  lw=1.2, label="FF")
    ln_rff, = ax_terms.plot([], [], "g--", lw=1.0)
    ln_lp,  = ax_terms.plot([], [], "b-",  lw=1.2, label="P")
    ln_rp,  = ax_terms.plot([], [], "b--", lw=1.0)
    ln_li,  = ax_terms.plot([], [], "m-",  lw=1.2, label="I")
    ln_ri,  = ax_terms.plot([], [], "m--", lw=1.0)
    ln_ld,  = ax_terms.plot([], [], "c-",  lw=1.2, label="D")
    ln_rd,  = ax_terms.plot([], [], "c--", lw=1.0)
    ax_terms.axhline(0, color="k", lw=0.5)
    ax_terms.legend(loc="upper left", fontsize=7, ncol=4)

    # ── Row 4: body tracking ──
    ax_body.set_title("Body-space tracking errors", fontsize=9)
    ax_body.set_ylabel("m/s  or  rad/s")
    ax_body.set_xlabel("time  (s)")
    ln_lin, = ax_body.plot([], [], "b-", lw=1.2, label="linear  (m/s)")
    ln_ang, = ax_body.plot([], [], "r-", lw=1.2, label="angular (rad/s)")
    ax_body.axhline(0, color="k", lw=0.5)
    ax_body.legend(loc="upper left", fontsize=7, ncol=2)

    lines = (ln_dl, ln_dr, ln_al, ln_ar,
             ln_el, ln_er,
             ln_lff, ln_rff, ln_lp, ln_rp, ln_li, ln_ri, ln_ld, ln_rd,
             ln_lin, ln_ang)
    axes  = (ax_speed, ax_error, ax_terms, ax_body)
    return fig, axes, lines


def update_plot(buf: RingBuffer, window: float, axes, lines, fig):
    data = buf.snapshot()
    if data.shape[0] < 2:
        return

    t   = data[:, COL["t"]]
    t0  = t[-1] - window
    mask = t >= t0
    if mask.sum() < 2:
        return
    d = data[mask]
    t = d[:, COL["t"]]

    (ln_dl, ln_dr, ln_al, ln_ar,
     ln_el, ln_er,
     ln_lff, ln_rff, ln_lp, ln_rp, ln_li, ln_ri, ln_ld, ln_rd,
     ln_lin, ln_ang) = lines
    ax_speed, ax_error, ax_terms, ax_body = axes

    def _set(ln, y):
        ln.set_data(t, d[:, y])

    _set(ln_dl, COL["des_l"]);  _set(ln_dr, COL["des_r"])
    _set(ln_al, COL["act_l"]);  _set(ln_ar, COL["act_r"])
    _set(ln_el, COL["err_l"]);  _set(ln_er, COL["err_r"])
    _set(ln_lff, COL["lff"]);   _set(ln_rff, COL["rff"])
    _set(ln_lp,  COL["lp"]);    _set(ln_rp,  COL["rp"])
    _set(ln_li,  COL["li"]);    _set(ln_ri,  COL["ri"])
    _set(ln_ld,  COL["ld"]);    _set(ln_rd,  COL["rd"])
    _set(ln_lin, COL["lin_e"]); _set(ln_ang, COL["ang_e"])

    tmin, tmax = t[0], t[-1]
    for ax in axes:
        ax.set_xlim(tmin, tmax + 0.1)
        ax.relim()
        ax.autoscale_view(scalex=False, scaley=True)

    fig.canvas.draw_idle()


def main():
    args = parse_args()
    window = args.window

    maxlen = int(window * PUBLISH_RATE * 2)
    buf = RingBuffer(maxlen=maxlen, ncols=NCOLS)

    if HAS_ROS:
        rclpy.init()
        node = PIDListener(buf)
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
    else:
        print("[WARN] rclpy not found — running in demo mode (random noise)")
        import time
        def _fake_data():
            t = 0.0
            while True:
                t += 0.1
                row = [t] + list(np.random.randn(NCOLS - 1) * 0.5)
                buf.append(row)
                time.sleep(0.1)
        threading.Thread(target=_fake_data, daemon=True).start()

    fig, axes, lines = make_figure()

    def _tick(frame):
        update_plot(buf, window, axes, lines, fig)

    ani = matplotlib.animation.FuncAnimation(
        fig, _tick, interval=REFRESH_MS, cache_frame_data=False
    )

    plt.show()

    if HAS_ROS:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    import matplotlib.animation
    main()
