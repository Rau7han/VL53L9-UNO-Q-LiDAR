#!/usr/bin/env python3
"""USB-LiDAR-VL53L9 live dashboard: LiDAR depth map + top-down room mapping
+ IMU (accel/gyro/orientation).

Usage:
    python3 dashboard.py --swd                 # real full raw frames via ST-Link
    python3 dashboard.py --sim                 # synthetic data, no hardware
    python3 dashboard.py --port /dev/ttyACM0   # read the real board

Keys:  q = quit,  c = clear the accumulated map
"""

import sys
# The real raw dashboard needs only the Python standard library.
if __name__ == "__main__" and ("--swd" in sys.argv or "--usb" in sys.argv):
    from raw_server import main as raw_main
    raw_main()
    raise SystemExit(0)

import argparse
import collections
import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

import protocol
from sources import SerialSource, SimSource

HFOV_DEG = 60.0          # VL53L9CX approximate horizontal field of view
VFOV_DEG = 45.0
MAP_MAX_PTS = 6000       # accumulated top-down map points
PLOT_HISTORY = 200       # IMU samples kept on the strip charts


class Dashboard:
    def __init__(self, source, sim=False):
        self.source = source
        self.sim = sim
        self.heading = 0.0               # integrated yaw (rad)
        self.last_t = time.time()
        self.map_pts = collections.deque(maxlen=MAP_MAX_PTS)
        self.t_hist = collections.deque(maxlen=PLOT_HISTORY)
        self.acc_hist = [collections.deque(maxlen=PLOT_HISTORY) for _ in range(3)]
        self.gyr_hist = [collections.deque(maxlen=PLOT_HISTORY) for _ in range(3)]
        self.frames = 0
        self.t_start = time.time()
        self._build_ui()

    # ------------------------------------------------------------------
    def _build_ui(self):
        plt.rcParams["figure.facecolor"] = "#101418"
        plt.rcParams["axes.facecolor"] = "#161b22"
        plt.rcParams["text.color"] = "#d0d7de"
        plt.rcParams["axes.labelcolor"] = "#d0d7de"
        plt.rcParams["xtick.color"] = "#8b949e"
        plt.rcParams["ytick.color"] = "#8b949e"
        plt.rcParams["axes.edgecolor"] = "#30363d"

        self.fig = plt.figure(figsize=(14, 8))
        self.fig.canvas.manager.set_window_title("USB-LiDAR-VL53L9 Dashboard")
        gs = GridSpec(2, 3, figure=self.fig, height_ratios=[1.3, 1],
                      hspace=0.3, wspace=0.28,
                      left=0.05, right=0.97, top=0.93, bottom=0.08)

        # --- LiDAR depth heatmap ---
        self.ax_depth = self.fig.add_subplot(gs[0, 0:2])
        self.ax_depth.set_title("LiDAR depth (mm)")
        self.im = self.ax_depth.imshow(np.zeros((42, 54)), cmap="turbo",
                                       vmin=100, vmax=4000, aspect="auto",
                                       origin="upper", interpolation="nearest")
        self.cbar = self.fig.colorbar(self.im, ax=self.ax_depth, fraction=0.04, pad=0.02)

        # --- orientation / info ---
        self.ax_info = self.fig.add_subplot(gs[0, 2])
        self.ax_info.set_title("Status / orientation")
        self.ax_info.axis("off")
        self.info_txt = self.ax_info.text(0.02, 0.97, "", va="top", ha="left",
                                          family="monospace", fontsize=11,
                                          transform=self.ax_info.transAxes)
        # a little attitude indicator
        self.att_line, = self.ax_info.plot([], [], lw=4, color="#58a6ff",
                                           transform=self.ax_info.transAxes)

        # --- top-down room map ---
        self.ax_map = self.fig.add_subplot(gs[1, 0])
        self.ax_map.set_title("Top-down map (rotate to scan)")
        self.ax_map.set_xlabel("X (m)"); self.ax_map.set_ylabel("Y (m)")
        self.ax_map.set_xlim(-4, 4); self.ax_map.set_ylim(-0.5, 5)
        self.ax_map.set_aspect("equal")
        self.ax_map.grid(True, color="#30363d", lw=0.5)
        self.map_scat = self.ax_map.scatter([], [], s=3, c=[], cmap="viridis",
                                            vmin=0, vmax=5)
        self.ax_map.plot(0, 0, "^", color="#f78166", ms=10)   # sensor origin

        # --- IMU accel ---
        self.ax_acc = self.fig.add_subplot(gs[1, 1])
        self.ax_acc.set_title("Accelerometer (g)")
        self.ax_acc.set_ylim(-2, 2); self.ax_acc.grid(True, color="#30363d", lw=0.5)
        self.acc_lines = [self.ax_acc.plot([], [], label=l)[0] for l in ("ax", "ay", "az")]
        self.ax_acc.legend(loc="upper right", fontsize=8, facecolor="#161b22")

        # --- IMU gyro ---
        self.ax_gyr = self.fig.add_subplot(gs[1, 2])
        self.ax_gyr.set_title("Gyroscope (dps)")
        self.ax_gyr.set_ylim(-250, 250); self.ax_gyr.grid(True, color="#30363d", lw=0.5)
        self.gyr_lines = [self.ax_gyr.plot([], [], label=l)[0] for l in ("gx", "gy", "gz")]
        self.ax_gyr.legend(loc="upper right", fontsize=8, facecolor="#161b22")

        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

    def _on_key(self, event):
        if event.key == "q":
            plt.close(self.fig)
        elif event.key == "c":
            self.map_pts.clear()
            self.heading = 0.0

    # ------------------------------------------------------------------
    def _update_map(self, frame, dt):
        # integrate yaw from gyro Z to build a rotating scan
        gz = frame.gyro[2]
        self.heading += np.deg2rad(gz) * dt

        h, w = frame.depth_mm.shape
        mid = slice(max(0, h // 2 - 1), h // 2 + 2)       # a few central rows
        cols = (np.arange(w) - (w - 1) / 2) / w * np.deg2rad(HFOV_DEG)
        d = frame.depth_mm[mid].astype(float).mean(axis=0) / 1000.0  # m
        v = frame.valid[mid].any(axis=0)
        ang = cols + self.heading
        xs = d * np.sin(ang)
        ys = d * np.cos(ang)
        for x, y, dd, ok in zip(xs, ys, d, v):
            if ok and 0.05 < dd < 8.0:
                self.map_pts.append((x, y, dd))

    def update(self, _):
        frame = self.source.read()
        now = time.time()
        dt = now - self.last_t
        self.last_t = now
        if frame is None:
            return []
        self.frames += 1

        # depth heatmap
        depth = frame.depth_mm.astype(float)
        depth[~frame.valid] = np.nan
        if self.im.get_array().shape != depth.shape:
            self.im.set_data(depth)
            self.im.set_extent([0, frame.width, frame.height, 0])
        self.im.set_data(depth)

        # top-down map
        self._update_map(frame, min(dt, 0.1))
        if self.map_pts:
            arr = np.array(self.map_pts)
            self.map_scat.set_offsets(arr[:, :2])
            self.map_scat.set_array(arr[:, 2])

        # IMU history
        t = now - self.t_start
        self.t_hist.append(t)
        for i in range(3):
            self.acc_hist[i].append(frame.accel[i])
            self.gyr_hist[i].append(frame.gyro[i])
        tt = list(self.t_hist)
        for i, ln in enumerate(self.acc_lines):
            ln.set_data(tt, list(self.acc_hist[i]))
        for i, ln in enumerate(self.gyr_lines):
            ln.set_data(tt, list(self.gyr_hist[i]))
        if tt:
            self.ax_acc.set_xlim(max(0, tt[-1] - 10), tt[-1] + 0.1)
            self.ax_gyr.set_xlim(max(0, tt[-1] - 10), tt[-1] + 0.1)

        # orientation from accel
        ax, ay, az = frame.accel
        pitch = np.degrees(np.arctan2(-ax, np.hypot(ay, az)))
        roll = np.degrees(np.arctan2(ay, az))
        # attitude indicator line (roll) centred in the info axis
        cx, cy, r = 0.5, 0.35, 0.18
        a = np.deg2rad(roll)
        self.att_line.set_data([cx - r * np.cos(a), cx + r * np.cos(a)],
                               [cy - r * np.sin(a), cy + r * np.sin(a)])

        fps = self.frames / max(1e-3, now - self.t_start)
        valid_pct = 100.0 * frame.valid.mean()
        self.info_txt.set_text(
            f"source : {'SIMULATOR' if self.sim else 'USB serial'}\n"
            f"seq    : {frame.seq}\n"
            f"res    : {frame.width} x {frame.height}  ({frame.width*frame.height} zones)\n"
            f"fps    : {fps:5.1f}\n"
            f"valid  : {valid_pct:5.1f} %\n"
            f"temp   : {frame.temp_c:5.1f} C\n\n"
            f"accel g: {ax:+.2f} {ay:+.2f} {az:+.2f}\n"
            f"gyro dps:{frame.gyro[0]:+6.1f}{frame.gyro[1]:+6.1f}{frame.gyro[2]:+6.1f}\n\n"
            f"pitch  : {pitch:+6.1f}\n"
            f"roll   : {roll:+6.1f}\n"
            f"heading: {np.degrees(self.heading)%360:6.1f}\n"
        )
        return []

    def run(self):
        from matplotlib.animation import FuncAnimation
        self.source.start()
        self._anim = FuncAnimation(self.fig, self.update, interval=40,
                                   blit=False, cache_frame_data=False)
        plt.show()
        self.source.close()


def main():
    ap = argparse.ArgumentParser(description="USB-LiDAR-VL53L9 dashboard")
    source_args = ap.add_mutually_exclusive_group(required=True)
    source_args.add_argument("--port", help="serial port, requires separate USB firmware")
    ap.add_argument("--baud", type=int, default=921600)
    source_args.add_argument("--sim", action="store_true", help="explicit synthetic demo")
    ap.add_argument("--width", type=int, default=54)
    ap.add_argument("--height", type=int, default=42)
    args = ap.parse_args()

    if args.sim:
        src = SimSource(width=args.width, height=args.height)
        dash = Dashboard(src, sim=True)
    else:
        src = SerialSource(args.port, args.baud)
        dash = Dashboard(src, sim=False)
    dash.run()


if __name__ == "__main__":
    main()
