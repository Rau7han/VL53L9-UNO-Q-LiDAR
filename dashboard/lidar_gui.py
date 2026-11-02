#!/usr/bin/env python3
"""LiDAR Dashboard — native GTK3 viewer for the VL53L9 sensor and its IMU.

Local USB:    python3 dashboard/lidar_gui.py
Over SSH:     python3 dashboard/lidar_gui.py --usb socket://127.0.0.1:7000   (see view_remote.sh)
Nothing is written to disk.
"""
import argparse
import math
import os
import queue
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import gi
gi.require_version('Gtk', '3.0')
gi.require_version('Gdk', '3.0')
from gi.repository import Gtk, Gdk, GLib
import cairo

sys.path.insert(0, str(Path(__file__).resolve().parent))
from usb_source import USBSource, SensorError            # noqa: E402
from cloud_geometry import display_mask, surface_triangles
from processing import Processor, ProcessingError        # noqa: E402

W, H = 54, 42
PIXELS = W * H
CENTRE = 21 * W + 27
HFOV, VFOV = math.radians(55), math.radians(42)
ZONE_PITCH = HFOV / W          # angular size of one sensor zone (rad)

FONT = 'Inter, Noto Sans, DejaVu Sans'
CAIRO_FONT = 'Noto Sans'

# Near (warm) to far (cool). Hand-tuned for even perceived steps on a dark background.
RANGE_STOPS = [(255, 228, 150), (255, 170, 80), (250, 104, 104), (205, 98, 214),
               (120, 118, 255), (64, 170, 255), (60, 222, 230)]
BG_TOP, BG_BOTTOM = (0.055, 0.067, 0.090), (0.024, 0.031, 0.043)


def build_lut(stops, n=256):
    s = np.asarray(stops, dtype=np.float32) / 255.0
    lin = s ** 2.2                       # interpolate in linear light for smoother blends
    xs = np.linspace(0, 1, len(s))
    t = np.linspace(0, 1, n)
    out = np.stack([np.interp(t, xs, lin[:, c]) for c in range(3)], axis=1)
    return (out ** (1 / 2.2)).astype(np.float32)


LUT = build_lut(RANGE_STOPS)


def colors_for(values, lo, hi):
    t = np.clip((values - lo) / max(hi - lo, 1e-6), 0, 1)
    return LUT[(t * 255).astype(np.int32)]


def pack(rgb):
    """(N,3) float 0..1 -> opaque ARGB32 uint32."""
    c = np.clip(rgb * 255 + 0.5, 0, 255).astype(np.uint32)
    return np.uint32(0xff000000) | (c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]


def surface_from(buffer, w, h, fmt=cairo.FORMAT_ARGB32):
    return cairo.ImageSurface.create_for_data(buffer, fmt, w, h, w * 4)


# ---------------------------------------------------------------- acquisition
class Acquisition:
    """USB thread reads at sensor rate; compute thread runs the ST transform."""

    def __init__(self, port, temporal=True):
        self.source = USBSource(port)
        self.temporal = temporal
        self.processor = Processor(self.source.calibration, temporal=temporal)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.paused = threading.Event()
        self.inbox = queue.Queue(maxsize=1)
        self.times = deque(maxlen=120)
        self.fps = 0.0
        self.received = 0
        self.failures = 0
        self.message = 'Connecting'
        self.error = None
        self.processing_error = None
        self.generation = 0
        self.processed = 0
        self.dropped = 0
        self.source.cancel_event = self.stop
        self.latest = None
        self.imu_queue = deque(maxlen=64)
        self.imu_latest = None
        self.source.on_imu = self._on_imu
        self.usb_thread = threading.Thread(target=self._usb, daemon=True)
        self.cpu_thread = threading.Thread(target=self._compute, daemon=True)

    def start(self):
        self.cpu_thread.start()
        self.usb_thread.start()

    def _on_imu(self, imu):
        with self.lock:
            if self.imu_latest is None or imu['sequence'] != self.imu_latest['sequence']:
                self.imu_queue.append(imu)
            self.imu_latest = imu

    def _offer(self, item):
        try:
            self.inbox.put_nowait(item)
        except queue.Full:
            try:
                self.inbox.get_nowait()
                self.dropped += 1
            except queue.Empty:
                pass
            try:
                self.inbox.put_nowait(item)
            except queue.Full:
                pass

    def _usb(self):
        try:
            self._usb_work()
        finally:
            try:self.source.close()
            except Exception:pass

    def _usb_work(self):
        recovery = False
        reset = False
        while not self.stop.is_set():
            if self.paused.is_set():
                if getattr(self.source, 'streaming', False):
                    try:
                        self.source.stop_stream()
                    except Exception as exc:
                        recovery = True
                        self.error = str(exc)
                self.stop.wait(0.1)
                continue
            try:
                if recovery:
                    self.source.reconnect()
                    self.generation += 1
                    self.source.cancel_event = self.stop
                    frame, payload = self.source.read(recover=reset, decode=False)
                    recovery = False
                else:
                    frame, payload = self.source.read(decode=False)
                now = time.monotonic()
                self.times.append(now)
                while self.times and now - self.times[0] > 1:
                    self.times.popleft()
                with self.lock:
                    n = len(self.times)
                    span = self.times[-1] - self.times[0] if n > 1 else 0
                    self.fps = (n - 1) / span if span >= 0.2 else 0.0
                    self.received += 1
                    self.failures = 0
                    self.message = 'Live'
                    self.error = None
                self._offer(('frame', frame, payload, self.source.calibration, self.generation))
            except Exception as exc:
                if self.stop.is_set():break
                self.failures += 1
                retry = (not isinstance(exc, ProcessingError)) and self.failures <= 3
                with self.lock:
                    self.error = str(exc)
                    self.message = 'Reconnecting' if retry else 'Offline'
                if retry:
                    recovery = True
                    reset = isinstance(exc, SensorError)
                    self.stop.wait(min(self.failures, 3))
                else:
                    self.paused.set()

    def _compute(self):
        try:
            self._compute_work()
        finally:
            self.processor.close()

    def _compute_work(self):
        generation = 0
        while not self.stop.is_set():
            try:
                item = self.inbox.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                _, frame, payload, calibration, frame_generation = item
                if generation != frame_generation:
                    self.processor.close()
                    self.processor = Processor(calibration, temporal=self.temporal)
                    generation = frame_generation
                xyz, depth, valid, confidence, flags, ms = self.processor.process_arrays(
                    payload, frame['sequence'], frame['frame_error_code'], frame['frame_error_flags'])
                with self.lock:
                    self.latest = {'sequence': frame['sequence'], 'capture_ms': frame['capture_ms'],
                                   'process_ms': ms, 'xyz': xyz, 'depth': depth, 'valid': valid,
                                   'confidence': confidence, 'filter_flags': flags,
                                   'reflectance': self.processor.last_reflectance,
                                   'calibrated': calibration is not None, 'updated': time.monotonic()}
                    self.processed += 1
                    self.processing_error = None
            except Exception as exc:
                with self.lock:
                    self.processing_error = f'Processing: {exc}'
                    self.latest = None

    def close(self):
        self.stop.set()
        self.usb_thread.join(timeout=3)
        self.cpu_thread.join(timeout=2)
        if self.usb_thread.is_alive() or self.cpu_thread.is_alive():
            raise RuntimeError('Acquisition did not shut down cleanly')


# ---------------------------------------------------------------- IMU attitude
def wrap180(d):
    return (d + 180) % 360 - 180


def ang_lerp(a, b, t):
    return a + wrap180(b - a) * t


class Attitude:
    """Complementary filter: gyro integration, corrected by gravity while still."""

    def __init__(self):
        self.flipped = None            # IMU board mounted upside-down; detected from gravity
        self.zero = None
        self.reset()
        self.rel = (0.0, 0.0, 0.0)
        self.accel = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.norm = 0.0
        self.temp = None
        self.last_seq = None
        self.hist_a = deque(maxlen=240)
        self.hist_g = deque(maxlen=240)

    def reset(self):
        self.roll = self.pitch = self.yaw = 0.0
        self.t = None
        self.ready = False

    def update(self, imu):
        if self.last_seq is not None and imu['sequence'] < self.last_seq:
            self.reset()
        self.last_seq = imu['sequence']
        valid = bool(imu['valid'])
        self.temp = imu.get('temperature_c')
        a = imu['accel_g'] or [0, 0, 0]
        g = imu['gyro_dps'] or [0, 0, 0]
        if valid and self.flipped is None and abs(a[2]) > 0.7:
            self.flipped = a[2] < 0
        if self.flipped:
            a = [a[0], -a[1], -a[2]]
            g = [g[0], -g[1], -g[2]]
        self.accel, self.gyro = tuple(a), tuple(g)
        self.norm = math.sqrt(sum(x * x for x in a))
        self.hist_a.append(self.accel)
        self.hist_g.append(self.gyro)
        if not valid:
            return
        ts = imu['timestamp_ms']
        dt = 0.0
        if self.t is not None:
            dt = (ts - self.t) / 1000
            if not (0 < dt < 0.25):
                dt = 0.0
        self.t = ts
        if dt:
            self.roll += g[0] * dt
            self.pitch += g[1] * dt
            self.yaw += g[2] * dt
        if 0.75 < self.norm < 1.25:
            a_roll = math.degrees(math.atan2(a[1], a[2]))
            a_pitch = math.degrees(math.atan2(-a[0], math.hypot(a[1], a[2])))
            if not self.ready:
                self.roll, self.pitch, self.ready = a_roll, a_pitch, True
            else:
                trust = 0.12 if abs(self.norm - 1) < 0.08 else 0.04
                self.roll = ang_lerp(self.roll, a_roll, trust)
                self.pitch = ang_lerp(self.pitch, a_pitch, trust)
        z = self.zero
        self.rel = ((wrap180(self.roll - z[0]) if z else self.roll),
                    (wrap180(self.pitch - z[1]) if z else self.pitch),
                    (wrap180(self.yaw - z[2]) if z else wrap180(self.yaw)))

    def toggle_zero(self):
        if self.zero is not None:
            self.zero = None
            self.rel = (self.roll, self.pitch, wrap180(self.yaw))
        elif self.ready:
            self.zero = (self.roll, self.pitch, self.yaw)
            self.rel = (0.0, 0.0, 0.0)


# ---------------------------------------------------------------- 3D camera and renderer
class Camera:
    """Orbit camera. Mouse input moves the goal; the view eases toward it every tick."""
    FOV = math.radians(45)

    def __init__(self):
        self.goal = {'yaw': -0.55, 'pitch': 0.30, 'dist': 3.0, 'target': np.array([0.0, 0.0, -1.4], np.float32)}
        self.yaw, self.pitch, self.dist = self.goal['yaw'], self.goal['pitch'], self.goal['dist']
        self.target = self.goal['target'].copy()

    def step(self, dt):
        k = 1 - math.exp(-dt / 0.055)
        g = self.goal
        moved = (abs(g['yaw'] - self.yaw) + abs(g['pitch'] - self.pitch) +
                 abs(g['dist'] - self.dist) + float(np.abs(g['target'] - self.target).sum()))
        if moved < 1e-4:
            return False
        self.yaw += (g['yaw'] - self.yaw) * k
        self.pitch += (g['pitch'] - self.pitch) * k
        self.dist += (g['dist'] - self.dist) * k
        self.target += (g['target'] - self.target) * k
        return True

    def rotation(self):
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        ry = np.array([[cy, 0, -sy], [0, 1, 0], [sy, 0, cy]], np.float32)
        rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]], np.float32)
        return rx @ ry

    def focal(self, h):
        return 0.5 * h / math.tan(self.FOV / 2)

    def project(self, pts, w, h):
        v = (pts - self.target) @ self.rotation().T
        depth = self.dist - v[:, 2]
        safe = np.maximum(depth, 1e-3)
        f = self.focal(h)
        return w * 0.5 + f * v[:, 0] / safe, h * 0.5 - f * v[:, 1] / safe, depth

    def orbit(self, dx, dy):
        self.goal['yaw'] -= dx * 0.0065
        self.goal['pitch'] = max(-1.45, min(1.45, self.goal['pitch'] + dy * 0.0065))

    def pan(self, dx, dy, h):
        r = self.rotation()
        scale = self.goal['dist'] / self.focal(h)
        self.goal['target'] = self.goal['target'] - r[0] * dx * scale + r[1] * dy * scale

    def zoom(self, steps):
        self.goal['dist'] = max(0.4, min(16.0, self.goal['dist'] * math.exp(steps * 0.12)))

    def fit(self, pts):
        if pts is None or len(pts) < 20:
            self.goal.update(yaw=-0.55, pitch=0.30, dist=3.0, target=np.array([0.0, 0.0, -1.4], np.float32))
            return
        lo, hi = np.percentile(pts, 3, axis=0), np.percentile(pts, 97, axis=0)
        centre = ((lo + hi) / 2).astype(np.float32)
        extent = float(np.linalg.norm(hi - lo))
        self.goal.update(yaw=-0.55, pitch=0.30, target=centre,
                         dist=max(1.2, extent * 0.5 / math.tan(self.FOV / 2) * 1.55))


def _kernel(r):
    rr = r + 0.5
    ys, xs = np.mgrid[-r:r + 1, -r:r + 1]
    d2 = (xs * xs + ys * ys) / (rr * rr)
    keep = d2 <= 1.0
    ox, oy, d2 = xs[keep], ys[keep], d2[keep]
    nz = np.sqrt(1 - d2)
    # Soft key light from the upper left; mostly flat so colour still reads as distance.
    shade = 0.88 + 0.10 * nz + 0.04 * np.clip((-ox - oy) / (rr * 1.414), -1, 1)
    return ox.astype(np.int32), oy.astype(np.int32), nz.astype(np.float32), shade.astype(np.float32)


class CloudRenderer:
    """Z-buffered sphere-impostor splats with eye-dome lighting and distance fog."""
    RMAX = 9

    def __init__(self):
        self.kernels = {r: _kernel(r) for r in range(1, self.RMAX + 1)}
        self.ms = 0.0
        self.size = None

    def _buffers(self, w, h):
        if self.size != (w, h):
            self.size = (w, h)
            self.zbuf = np.full(h * w, np.inf, np.float32)
            self.pad = np.full((h + 2) * (w + 2), np.inf, np.float32)
        return self.zbuf, self.pad

    def render(self, cam, pts, rgb, rng, w, h, size=6.0):
        started = time.monotonic()
        out = np.zeros(h * w, np.uint32)
        zbuf, pad = self._buffers(w, h)
        if pts is None or not len(pts):
            return out.reshape(h, w)
        sx, sy, depth = cam.project(pts, w, h)
        f = cam.focal(h)
        front = depth > 0.06
        sx, sy, depth, rgb, rng = sx[front], sy[front], depth[front], rgb[front], rng[front]
        # Fixed screen-space diameter, matching the GPU point-size control.
        rad_px = np.full_like(depth,np.clip(size/2,.5,self.RMAX))
        rad_w = rad_px * depth / f
        bucket = np.clip(np.rint(rad_px).astype(np.int32), 1, self.RMAX)
        cx, cy = np.rint(sx).astype(np.int32), np.rint(sy).astype(np.int32)
        onscreen = (cx > -self.RMAX) & (cx < w + self.RMAX) & (cy > -self.RMAX) & (cy < h + self.RMAX)
        idx_parts, z_parts, pt_parts, sh_parts = [], [], [], []
        for r in np.unique(bucket[onscreen]):
            sel = np.nonzero(onscreen & (bucket == r))[0]
            ox, oy, nz, shade = self.kernels[int(r)]
            px = cx[sel, None] + ox[None, :]
            py = cy[sel, None] + oy[None, :]
            ok = (px >= 0) & (px < w) & (py >= 0) & (py < h)
            idx_parts.append((py * w + px)[ok])
            z_parts.append((depth[sel, None] - rad_w[sel, None] * nz[None, :])[ok])
            pt_parts.append(np.broadcast_to(sel[:, None], ok.shape)[ok])
            sh_parts.append(np.broadcast_to(shade[None, :], ok.shape)[ok])
        if not idx_parts:
            return out.reshape(h, w)
        idx = np.concatenate(idx_parts)
        z = np.concatenate(z_parts)
        pt = np.concatenate(pt_parts)
        sh = np.concatenate(sh_parts)
        np.minimum.at(zbuf, idx, z)                        # z-test: nearest fragment owns the pixel
        win = z <= zbuf[idx]
        zbuf[idx] = np.inf                                 # reset only what was touched
        idx, z, pt, sh = idx[win], z[win], pt[win], sh[win]

        # Eye-dome lighting on covered pixels only. Uses each point's centre depth, so a single
        # surface stays smooth while silhouettes and depth steps get a soft dark edge.
        stride = w + 2
        pidx = (idx // w + 1) * stride + idx % w + 1
        lz = np.log(np.maximum(depth[pt], 1e-3)).astype(np.float32)
        pad[pidx] = lz
        resp = np.zeros(len(idx), np.float32)
        for off in (-1, 1, -stride, stride, -stride - 1, -stride + 1, stride - 1, stride + 1):
            nb = pad[pidx + off]
            resp += np.where(np.isinf(nb), np.float32(0.12), np.minimum(np.maximum(lz - nb, 0), 0.5))
        pad[pidx] = np.inf
        edl = 0.42 + 0.58 * np.exp(-1.8 * resp)

        near, far = np.percentile(z, 5), np.percentile(z, 95)
        fog = 1 - 0.30 * np.clip((z - near) / max(far - near, 1e-3), 0, 1)
        colour = rgb[pt] * (sh * edl * fog)[:, None]
        out[idx] = pack(colour)
        self.ms = (time.monotonic() - started) * 1000
        return out.reshape(h, w)


# ---------------------------------------------------------------- drawing helpers
def rounded(ctx, x, y, w, h, r):
    ctx.new_sub_path()
    ctx.arc(x + w - r, y + r, r, -math.pi / 2, 0)
    ctx.arc(x + w - r, y + h - r, r, 0, math.pi / 2)
    ctx.arc(x + r, y + h - r, r, math.pi / 2, math.pi)
    ctx.arc(x + r, y + r, r, math.pi, 1.5 * math.pi)
    ctx.close_path()


def label(ctx, x, y, s, size=11, rgba=(0.55, 0.59, 0.65, 1), weight=cairo.FONT_WEIGHT_NORMAL, align='left'):
    ctx.select_font_face(CAIRO_FONT, cairo.FONT_SLANT_NORMAL, weight)
    ctx.set_font_size(size)
    ext = ctx.text_extents(s)
    if align == 'right':
        x -= ext.x_advance
    elif align == 'center':
        x -= ext.x_advance / 2
    ctx.set_source_rgba(*rgba)
    ctx.move_to(round(x), round(y))
    ctx.show_text(s)


_background = {}


def paint_background(ctx, w, h):
    surf = _background.get((w, h))
    if surf is None:
        _background.clear()
        surf = cairo.ImageSurface(cairo.FORMAT_ARGB32, w, h)
        c = cairo.Context(surf)
        rounded(c, 0, 0, w, h, 10)
        c.clip()
        grad = cairo.LinearGradient(0, 0, 0, h)
        grad.add_color_stop_rgb(0, *BG_TOP)
        grad.add_color_stop_rgb(1, *BG_BOTTOM)
        c.set_source(grad)
        c.paint()
        glow = cairo.RadialGradient(w * 0.5, h * 0.42, 0, w * 0.5, h * 0.42, max(w, h) * 0.6)
        glow.add_color_stop_rgba(0, 0.30, 0.45, 0.70, 0.06)
        glow.add_color_stop_rgba(1, 0.30, 0.45, 0.70, 0.0)
        c.set_source(glow)
        c.paint()
        _background[(w, h)] = surf
    rounded(ctx, 0, 0, w, h, 10)
    ctx.clip()
    ctx.set_source_surface(surf, 0, 0)
    ctx.paint()


def draw_legend(ctx, x, y, lo, hi, caption='Range'):
    bw, bh = 150, 6
    ramp = np.ascontiguousarray(pack(LUT))
    surf = surface_from(ramp, 256, 1)
    ctx.save()
    rounded(ctx, x, y, bw, bh, 3)
    ctx.clip()
    ctx.translate(x, y)
    ctx.scale(bw / 256, bh)
    ctx.set_source_surface(surf, 0, 0)
    ctx.get_source().set_filter(cairo.FILTER_BILINEAR)
    ctx.paint()
    ctx.restore()
    label(ctx, x, y - 8, caption.upper(), 9.5, (0.49, 0.52, 0.56, 1), cairo.FONT_WEIGHT_BOLD)
    label(ctx, x, y + bh + 14, f'{lo:.2f} m', 10.5, (0.79, 0.82, 0.85, 1))
    label(ctx, x + bw, y + bh + 14, f'{hi:.2f} m', 10.5, (0.79, 0.82, 0.85, 1), align='right')


# ---------------------------------------------------------------- application
CSS = f"""
* {{ font-family: {FONT}; }}
window {{ background-color: #07090d; color: #e6edf3; }}
.header {{ background-color: #0a0d13; border-bottom: 1px solid rgba(255,255,255,0.06); padding: 10px 16px; }}
.title {{ font-size: 15px; font-weight: 600; color: #f0f6fc; }}
.muted {{ color: #7d8590; font-size: 11px; }}
.metric {{ color: #e6edf3; font-size: 13px; font-weight: 600; font-feature-settings: "tnum"; }}
.pill {{ border-radius: 999px; padding: 3px 11px; font-size: 12px; font-weight: 600; }}
.pill.live {{ background-color: rgba(63,185,80,0.13); color: #3fb950; }}
.pill.warn {{ background-color: rgba(210,153,34,0.14); color: #d29922; }}
.pill.bad  {{ background-color: rgba(248,81,73,0.14); color: #f85149; }}
.pill.idle {{ background-color: rgba(139,148,158,0.12); color: #8b949e; }}
button {{ background-image: none; background-color: #121722; color: #c9d1d9; border: 1px solid rgba(255,255,255,0.07);
          border-radius: 6px; box-shadow: none; text-shadow: none; padding: 4px 12px; min-height: 0; }}
button:hover {{ background-color: #19202d; color: #f0f6fc; }}
button:checked {{ background-color: #1d2738; color: #f0f6fc; border-color: rgba(88,166,255,0.40); }}
button.flat {{ background-color: transparent; border-color: transparent; padding: 3px 8px; }}
button.flat:hover {{ background-color: rgba(255,255,255,0.06); }}
button.overlay {{ background-color: rgba(13,17,23,0.72); border-color: rgba(255,255,255,0.08); padding: 5px 7px; }}
.card {{ background-color: #0d1117; border: 1px solid rgba(255,255,255,0.06); border-radius: 10px; padding: 14px 14px 12px 14px; }}
.card-title {{ color: #7d8590; font-size: 10.5px; font-weight: 700; letter-spacing: 1.2px; }}
.row-label {{ color: #8b949e; font-size: 12px; }}
.row-value {{ color: #e6edf3; font-size: 12px; font-weight: 600; font-feature-settings: "tnum"; }}
.big {{ color: #f0f6fc; font-size: 17px; font-weight: 600; font-feature-settings: "tnum"; }}
.axis {{ color: #7d8590; font-size: 10.5px; }}
tooltip {{ background-color: #161b22; color: #e6edf3; }}
"""


class Viewer(Gtk.Window):
    def __init__(self, acq):
        super().__init__(title='LiDAR Dashboard')
        self.acq = acq
        self.att = Attitude()
        self.cam = Camera()
        self.renderer = CloudRenderer()
        self.render_scale = 1.0
        self.point_size = 6.0
        self.confidence_min = 0.0
        self.surface_jump = 0.15
        self.surface_mode = False
        self.reflectance_mode = False
        self.valid = np.zeros(PIXELS, bool)
        self.all_rgb = np.zeros((PIXELS,3), np.float32)
        self.gpu = None
        self.view = '3d'
        self.frame = None
        self.seq = None
        self.pts = self.rgb = self.rng = None
        self.range_lo, self.range_hi = 0.3, 3.0
        self.depth_lo, self.depth_hi = 300.0, 3000.0
        self.floor_y = None
        self.fitted = False
        self.drag = None
        self.hover = None
        self.dirty = True
        self.last_tick = time.monotonic()
        self.draw_times = deque(maxlen=60)
        self.set_default_size(1280, 760)

        settings = Gtk.Settings.get_default()
        settings.set_property('gtk-application-prefer-dark-theme', True)
        css = Gtk.CssProvider()
        css.load_from_data(CSS.encode())
        Gtk.StyleContext.add_provider_for_screen(Gdk.Screen.get_default(), css,
                                                 Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)

        root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.add(root)
        root.pack_start(self.build_header(), False, False, 0)

        body = Gtk.Box(spacing=14)
        body.set_border_width(14)
        root.pack_start(body, True, True, 0)

        self.area = Gtk.DrawingArea()
        self.area.set_size_request(480, 320)
        self.area.add_events(Gdk.EventMask.POINTER_MOTION_MASK | Gdk.EventMask.BUTTON_PRESS_MASK |
                             Gdk.EventMask.BUTTON_RELEASE_MASK | Gdk.EventMask.SCROLL_MASK |
                             Gdk.EventMask.SMOOTH_SCROLL_MASK | Gdk.EventMask.LEAVE_NOTIFY_MASK)
        self.area.connect('draw', self.on_draw)
        self.area.connect('motion-notify-event', self.on_motion)
        self.area.connect('button-press-event', self.on_press)
        self.area.connect('button-release-event', lambda *_: setattr(self, 'drag', None))
        self.area.connect('scroll-event', self.on_scroll)
        self.area.connect('leave-notify-event', self.on_leave)
        overlay = Gtk.Overlay()
        self.view_stack = Gtk.Stack()
        self.view_stack.add_named(self.area, 'cpu')
        try:
            if os.environ.get('LIDAR_CPU_RENDER')=='1':raise ImportError('CPU rendering selected')
            from gpu_cloud import GPUCloud
            self.gpu = GPUCloud(self)
            self.gpu.set_size_request(480,320)
            self.gpu.add_events(Gdk.EventMask.POINTER_MOTION_MASK | Gdk.EventMask.BUTTON_PRESS_MASK |
                                Gdk.EventMask.BUTTON_RELEASE_MASK | Gdk.EventMask.SCROLL_MASK |
                                Gdk.EventMask.SMOOTH_SCROLL_MASK)
            self.gpu.connect('motion-notify-event',self.on_motion)
            self.gpu.connect('button-press-event',self.on_press)
            self.gpu.connect('button-release-event',lambda *_:setattr(self,'drag',None))
            self.gpu.connect('scroll-event',self.on_scroll)
            self.view_stack.add_named(self.gpu,'gpu')
            self.gpu.show()
            self.view_stack.set_visible_child_name('gpu')
        except ImportError as exc:
            print('CPU renderer fallback:',exc,flush=True)
        overlay.add(self.view_stack)
        self.view_note=Gtk.Label(label='Measured points · calibrated sensor geometry')
        self.view_note.set_halign(Gtk.Align.START);self.view_note.set_valign(Gtk.Align.START)
        self.view_note.set_margin_top(12);self.view_note.set_margin_start(12)
        overlay.add_overlay(self.view_note)
        self.cloud_note=Gtk.Label()
        self.cloud_note.set_halign(Gtk.Align.START);self.cloud_note.set_valign(Gtk.Align.END)
        self.cloud_note.set_margin_start(12);self.cloud_note.set_margin_bottom(12)
        overlay.add_overlay(self.cloud_note)
        self.fit_btn = Gtk.Button()
        self.fit_btn.add(Gtk.Image.new_from_icon_name('zoom-fit-best-symbolic', Gtk.IconSize.BUTTON))
        self.fit_btn.set_tooltip_text('Fit view (double-click)')
        self.fit_btn.get_style_context().add_class('overlay')
        self.fit_btn.set_halign(Gtk.Align.END)
        self.fit_btn.set_valign(Gtk.Align.START)
        self.fit_btn.set_margin_top(12)
        self.fit_btn.set_margin_end(12)
        self.fit_btn.connect('clicked', lambda *_: self.cam.fit(self.pts))
        overlay.add_overlay(self.fit_btn)
        overlay.set_hexpand(True)
        overlay.set_vexpand(True)
        body.pack_start(overlay, True, True, 0)

        side = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=14)
        side.set_size_request(300, -1)
        side.set_hexpand(False)
        side.pack_start(self.build_lidar_card(), False, False, 0)
        side.pack_start(self.build_imu_card(), False, False, 0)
        body.pack_start(side, False, False, 0)

        self.connect('destroy', self.on_destroy)
        self.connect('key-press-event', self.on_key)
        GLib.timeout_add(16, self.tick)
        GLib.timeout_add(250, self.refresh_text)

    # ---------- layout
    def build_header(self):
        bar = Gtk.Box(spacing=18)
        bar.get_style_context().add_class('header')
        title = Gtk.Label(label='LiDAR Dashboard')
        title.get_style_context().add_class('title')
        bar.pack_start(title, False, False, 0)

        seg = Gtk.Box()
        seg.get_style_context().add_class('linked')
        self.tabs = {}
        group = None
        for key, name in (('3d', '3D'), ('depth', 'Depth'), ('top', 'Top')):
            b = Gtk.RadioButton.new_with_label_from_widget(group, name)
            b.set_mode(False)
            b.set_size_request(64, -1)
            group = group or b
            b.connect('toggled', self.on_tab, key)
            seg.pack_start(b, False, False, 0)
            self.tabs[key] = b
        bar.set_center_widget(seg)

        self.pause_icon = Gtk.Image.new_from_icon_name('media-playback-pause-symbolic', Gtk.IconSize.BUTTON)
        pause = Gtk.Button()
        pause.add(self.pause_icon)
        pause.set_tooltip_text('Pause (Space)')
        pause.connect('clicked', self.on_pause)
        bar.pack_end(pause, False, False, 0)

        self.pill = Gtk.Label(label='● Connecting')
        self.pill.get_style_context().add_class('pill')
        self.pill.get_style_context().add_class('idle')
        bar.pack_end(self.pill, False, False, 0)

        self.metrics = {}
        for key, unit in (('latency', 'ms'), ('fps', 'fps')):
            box = Gtk.Box(spacing=5)
            value = Gtk.Label(label='—')
            value.get_style_context().add_class('metric')
            u = Gtk.Label(label=unit)
            u.get_style_context().add_class('muted')
            box.pack_start(value, False, False, 0)
            box.pack_start(u, False, False, 0)
            bar.pack_end(box, False, False, 0)
            self.metrics[key] = value
        return bar

    def card(self, title, extra=None):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        box.get_style_context().add_class('card')
        head = Gtk.Box()
        t = Gtk.Label(label=title, xalign=0)
        t.get_style_context().add_class('card-title')
        head.pack_start(t, True, True, 0)
        if extra:
            head.pack_end(extra, False, False, 0)
        box.pack_start(head, False, False, 0)
        return box

    def rows(self, parent, names):
        grid = Gtk.Grid(row_spacing=7, column_spacing=12)
        values = {}
        for i, name in enumerate(names):
            k = Gtk.Label(label=name, xalign=0)
            k.get_style_context().add_class('row-label')
            v = Gtk.Label(label='—', xalign=1)
            v.set_hexpand(True)
            v.get_style_context().add_class('row-value')
            grid.attach(k, 0, i, 1, 1)
            grid.attach(v, 1, i, 1, 1)
            values[name] = v
        parent.pack_start(grid, False, False, 0)
        return values

    def build_lidar_card(self):
        box = self.card('LIDAR')
        self.lidar_rows = self.rows(box, ('Points', 'Nearest', 'Farthest', 'Centre', 'Coverage'))
        controls=Gtk.Grid(row_spacing=6,column_spacing=8)
        def slider(row,name,lo,hi,step,value,attribute):
            controls.attach(Gtk.Label(label=name,xalign=0),0,row,1,1)
            scale=Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL,lo,hi,step)
            scale.set_value(value);scale.set_hexpand(True);scale.set_size_request(130,-1)
            scale.connect('value-changed',lambda w:self.set_option(attribute,w.get_value()))
            controls.attach(scale,1,row,1,1)
        slider(0,'Point size (px)',1,18,1,self.point_size,'point_size')
        slider(1,'Min confidence',0,1,.05,self.confidence_min,'confidence_min')
        slider(2,'Surface gap (m)',.03,.5,.01,self.surface_jump,'surface_jump')
        box.pack_start(controls,False,False,0)
        surface=Gtk.CheckButton(label='Surface (interpolated)')
        self.surface_button=surface
        surface.connect('toggled',lambda w:self.set_option('surface_mode',w.get_active()))
        box.pack_start(surface,False,False,0)
        reflect=Gtk.CheckButton(label='Depth view: ST reflectance estimate')
        self.reflectance_button=reflect
        reflect.connect('toggled',lambda w:self.set_option('reflectance_mode',w.get_active()))
        box.pack_start(reflect,False,False,0)
        note=Gtk.Label(label='ST validity filters always active · max 2,268 measured points')
        note.set_line_wrap(True);box.pack_start(note,False,False,0)
        return box

    def build_imu_card(self):
        zero = Gtk.Button(label='Zero')
        zero.get_style_context().add_class('flat')
        zero.set_tooltip_text('Use the current orientation as 0° (click again to clear)')
        zero.connect('clicked', self.on_zero)
        self.zero_btn = zero
        box = self.card('IMU', zero)
        self.horizon = Gtk.DrawingArea()
        self.horizon.set_size_request(-1, 150)
        self.horizon.connect('draw', self.on_horizon)
        box.pack_start(self.horizon, False, False, 0)

        angles = Gtk.Box(homogeneous=True)
        self.angle_values = {}
        for name in ('Roll', 'Pitch', 'Yaw'):
            col = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=1)
            v = Gtk.Label(label='—')
            v.get_style_context().add_class('big')
            k = Gtk.Label(label=name)
            k.get_style_context().add_class('axis')
            col.pack_start(v, False, False, 0)
            col.pack_start(k, False, False, 0)
            angles.pack_start(col, True, True, 0)
            self.angle_values[name] = v
        box.pack_start(angles, False, False, 0)

        self.charts = Gtk.DrawingArea()
        self.charts.set_size_request(-1, 112)
        self.charts.connect('draw', self.on_charts)
        box.pack_start(self.charts, False, False, 0)
        self.imu_rows = self.rows(box, ('Acceleration', 'Temperature'))
        return box

    def set_option(self,name,value):
        setattr(self,name,value)
        if self.frame is not None:self.ingest(self.frame)
        self.dirty=True

    def gpu_failed(self,error):
        print('OpenGL unavailable; CPU fallback:',error,flush=True)
        self.view_stack.set_visible_child_name('cpu')
        self.dirty=True
        return False

    # ---------- events
    def on_tab(self, button, key):
        if button.get_active():
            self.view = key
            self.view_stack.set_visible_child_name('gpu' if key=='3d' and self.gpu and not self.gpu.failed else 'cpu')
            self.fit_btn.set_visible(key == '3d')
            self.hover = None
            self.dirty = True

    def on_pause(self, *_):
        if self.acq.paused.is_set():
            self.acq.failures = 0
            self.acq.paused.clear()
            self.pause_icon.set_from_icon_name('media-playback-pause-symbolic', Gtk.IconSize.BUTTON)
        else:
            self.acq.paused.set()
            self.pause_icon.set_from_icon_name('media-playback-start-symbolic', Gtk.IconSize.BUTTON)

    def on_zero(self, *_):
        self.att.toggle_zero()
        self.zero_btn.set_label('Clear zero' if self.att.zero else 'Zero')
        self.horizon.queue_draw()

    def on_key(self, _, ev):
        key = Gdk.keyval_name(ev.keyval)
        if key == 'space':
            self.on_pause()
        elif key in ('1', '2', '3'):
            self.tabs[('3d', 'depth', 'top')[int(key) - 1]].set_active(True)
        elif key in ('f', 'F'):
            self.cam.fit(self.pts)
        return False

    def on_press(self, _, ev):
        if ev.type == Gdk.EventType.DOUBLE_BUTTON_PRESS and self.view == '3d':
            self.cam.fit(self.pts)
            return
        pan = ev.button in (2, 3) or bool(ev.state & Gdk.ModifierType.SHIFT_MASK)
        self.drag = (ev.x, ev.y, pan)

    def on_motion(self, widget, ev):
        if self.view == '3d' and self.drag:
            x0, y0, pan = self.drag
            self.drag = (ev.x, ev.y, pan)
            if pan:
                self.cam.pan(ev.x - x0, ev.y - y0, widget.get_allocated_height())
            else:
                self.cam.orbit(ev.x - x0, ev.y - y0)
        elif self.view == 'depth':
            self.hover = (ev.x, ev.y)
            self.dirty = True

    def on_leave(self, *_):
        if self.hover:
            self.hover = None
            self.dirty = True

    def on_scroll(self, _, ev):
        if self.view != '3d':
            return
        if ev.direction == Gdk.ScrollDirection.SMOOTH:
            self.cam.zoom(ev.delta_y)
        elif ev.direction in (Gdk.ScrollDirection.UP, Gdk.ScrollDirection.DOWN):
            self.cam.zoom(-1 if ev.direction == Gdk.ScrollDirection.UP else 1)

    def on_destroy(self, *_):
        self.acq.close()
        Gtk.main_quit()

    # ---------- frame intake
    def ingest(self, frame):
        self.frame = frame
        valid = display_mask(frame['xyz'],frame['valid'],frame.get('confidence'),self.confidence_min)
        self.valid = valid
        pts = frame['xyz'][valid]
        rng = np.linalg.norm(pts, axis=1) if len(pts) else np.empty(0, np.float32)
        if len(rng) > 20:
            lo, hi = np.percentile(rng, 2), np.percentile(rng, 98)
            self.range_lo += (lo - self.range_lo) * 0.12
            self.range_hi += (max(hi, lo + 0.2) - self.range_hi) * 0.12
            d = frame['depth'][valid]
            self.depth_lo += (np.percentile(d, 2) - self.depth_lo) * 0.12
            self.depth_hi += (max(np.percentile(d, 98), self.depth_lo + 200) - self.depth_hi) * 0.12
            fy = float(np.percentile(pts[:, 1], 2)) - 0.03
            self.floor_y = fy if self.floor_y is None else self.floor_y + (fy - self.floor_y) * 0.05
            if not self.fitted:
                self.cam.fit(pts)
                self.cam.yaw, self.cam.pitch, self.cam.dist = (self.cam.goal[k] for k in ('yaw', 'pitch', 'dist'))
                self.cam.target = self.cam.goal['target'].copy()
                self.fitted = True
        all_ranges=np.linalg.norm(frame['xyz'],axis=1)
        self.all_rgb=colors_for(np.where(np.isfinite(all_ranges),all_ranges,0),self.range_lo,self.range_hi)
        self.pts, self.rng = pts, rng
        self.rgb = colors_for(rng, self.range_lo, self.range_hi) if len(rng) else None

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        a = self.acq
        with a.lock:
            frame = a.latest
            imus = list(a.imu_queue)
            a.imu_queue.clear()
        for imu in imus:
            self.att.update(imu)
        if imus:
            self.horizon.queue_draw()
            self.charts.queue_draw()
        if frame is None and a.processing_error:
            self.frame=None;self.pts=None;self.dirty=True
        if frame is not None and frame['sequence'] != self.seq:
            self.seq = frame['sequence']
            self.ingest(frame)
            self.dirty = True
        if self.cam.step(dt) and self.view == '3d':
            self.dirty = True
        if self.dirty:
            self.dirty = False
            if self.view=='3d' and self.gpu and not self.gpu.failed:self.gpu.queue_render()
            else:self.area.queue_draw()
        return True

    def refresh_text(self):
        a = self.acq
        with a.lock:
            msg, err, fps = a.message, a.processing_error or a.error, a.fps
        paused = a.paused.is_set()
        state, cls = ((msg, 'bad') if paused and err else ('Paused', 'idle') if paused else
                      (msg, 'warn') if err else ('Live', 'live') if self.frame else ('Connecting', 'idle'))
        ctx = self.pill.get_style_context()
        for c in ('live', 'warn', 'bad', 'idle'):
            ctx.remove_class(c)
        ctx.add_class(cls)
        if a.processing_error:
            state='Processing error';ctx.remove_class('live');ctx.add_class('bad')
        elif self.frame and not paused and time.monotonic()-self.frame.get('updated',0)>1:
            state='Stale depth data';ctx.remove_class('live');ctx.add_class('warn')
        self.pill.set_text(f'●  {state}')
        mode='Interpolated surface · not additional measurements' if self.surface_mode and self.view=='3d' else 'Measured points · ST calibration'
        if self.view=='depth':mode='ST reflectance estimate (%)' if self.reflectance_mode else 'Perpendicular depth (m)'
        if self.frame and not self.frame.get('calibrated'):mode='UNCALIBRATED preview · nominal geometry'
        backend='OpenGL' if self.gpu and self.gpu.ready and not self.gpu.failed else 'CPU'
        if self.gpu and self.gpu.ready and any(x in self.gpu.renderer_name.lower() for x in ('llvmpipe','softpipe','software')):backend='OpenGL software'
        self.view_note.set_text(f'{mode} · {backend}')
        self.cloud_note.set_text(f'{int(self.valid.sum()):,} / 2,268 measured points · range colour {self.range_lo:.2f}–{self.range_hi:.2f} m' if self.view=='3d' else '')
        self.pill.set_tooltip_text(err or None)
        self.metrics['fps'].set_tooltip_text(f'Received frame rate · processed {a.processed} · skipped display frames {a.dropped}')
        self.metrics['fps'].set_text(f'{fps:.1f}' if fps and not paused else '—')
        f = self.frame
        if f:
            self.metrics['latency'].set_text(str(f['capture_ms']))
            rng = self.rng
            rows = self.lidar_rows
            n = int(len(rng))
            rows['Points'].set_text(f'{n:,}')
            rows['Nearest'].set_text(f'{rng.min():.2f} m' if n else '—')
            rows['Farthest'].set_text(f'{rng.max():.2f} m' if n else '—')
            rows['Centre'].set_text(f'{f["depth"][CENTRE] / 1000:.2f} m' if self.valid[CENTRE] else '—')
            rows['Coverage'].set_text(f'{100 * n / PIXELS:.1f} %')
        t = self.att
        if t.last_seq is not None:
            for name, v in zip(('Roll', 'Pitch', 'Yaw'), t.rel):
                self.angle_values[name].set_text(f'{v:+.1f}°')
            self.imu_rows['Acceleration'].set_text(f'{t.norm:.3f} g')
            self.imu_rows['Temperature'].set_text('—' if t.temp is None else f'{t.temp:.1f} °C')
        return True

    # ---------- main view
    def on_draw(self, widget, ctx):
        w, h = widget.get_allocated_width(), widget.get_allocated_height()
        started = time.monotonic()
        paint_background(ctx, w, h)
        if self.frame is None:
            label(ctx, w / 2, h / 2, 'Waiting for sensor…', 13, (0.55, 0.59, 0.65, 1), align='center')
            return
        if self.view == '3d':
            self.draw_cloud(ctx, w, h)
        elif self.view == 'depth':
            self.draw_depth(ctx, w, h)
        else:
            self.draw_top(ctx, w, h)
        self.draw_times.append((time.monotonic() - started) * 1000)

    def draw_cloud(self, ctx, w, h):
        if self.surface_mode and self.frame is not None:
            t=surface_triangles(self.frame['xyz'],self.valid,self.surface_jump)
            x,y,d=self.cam.project(self.frame['xyz'],w,h)
            t=t[np.all(d[t]>.06,axis=1)]
            for tri in t[np.argsort(d[t].mean(axis=1))[::-1]]:
                ctx.set_source_rgb(*self.all_rgb[tri].mean(axis=0))
                ctx.move_to(x[tri[0]],y[tri[0]])
                for i in tri[1:]:ctx.line_to(x[i],y[i])
                ctx.close_path();ctx.fill()
            return
        self.draw_grid(ctx, w, h)
        self.draw_sensor(ctx, w, h)
        s = self.render_scale
        rw, rh = max(2, int(w * s)), max(2, int(h * s))
        buf = self.renderer.render(self.cam, self.pts, self.rgb, self.rng, rw, rh, size=self.point_size)
        ctx.save()
        ctx.scale(w / rw, h / rh)
        ctx.set_source_surface(surface_from(buf, rw, rh), 0, 0)
        ctx.get_source().set_filter(cairo.FILTER_GOOD if s < 0.99 else cairo.FILTER_NEAREST)
        ctx.paint()
        ctx.restore()
        # Hold ~35 FPS on slower machines by trading internal resolution, never frame rate.
        if self.renderer.ms > 16 and s > 0.5:
            self.render_scale = max(0.5, s - 0.05)
        elif self.renderer.ms < 8 and s < 1.0:
            self.render_scale = min(1.0, s + 0.05)
        self.draw_gizmo(ctx, w, h)
        if self.pts is not None and len(self.pts):
            draw_legend(ctx, 16, h - 34, self.range_lo, self.range_hi)
        else:
            label(ctx, w / 2, h / 2, 'No returns', 13, (0.55, 0.59, 0.65, 1), align='center')

    def draw_grid(self, ctx, w, h):
        floor = -0.6 if self.floor_y is None else min(self.floor_y, -0.05)
        cx, cz = round(float(self.cam.target[0])), round(float(self.cam.target[2]))
        key = (cx, cz, round(floor, 2))
        if getattr(self, '_grid_key', None) != key:
            span, step, seg = 4.0, 0.5, 0.5
            segments, alphas = [], []
            for t in np.arange(-span, span + 1e-6, step):
                base = 0.17 if abs(t - round(t)) < 1e-6 else 0.075
                for a0 in np.arange(-span, span - seg + 1e-6, seg):
                    for p, q in (((t, a0), (t, a0 + seg)), ((a0, t), (a0 + seg, t))):
                        fade = max(0.0, 1 - math.hypot((p[0] + q[0]) / 2, (p[1] + q[1]) / 2) / span) ** 1.6
                        if fade > 0.02:
                            segments.append((cx + p[0], floor, cz + p[1], cx + q[0], floor, cz + q[1]))
                            alphas.append(base * fade)
            self._grid = (np.asarray(segments, np.float32), np.asarray(alphas))
            self._grid_key = key
        s, alphas = self._grid
        ax, ay, ad = self.cam.project(s[:, :3], w, h)
        bx, by, bd = self.cam.project(s[:, 3:], w, h)
        ok = (ad > 0.1) & (bd > 0.1)
        bins = np.minimum((alphas / 0.17 * 8).astype(int), 7)
        ctx.set_line_width(1)
        for b in range(8):
            sel = np.nonzero(ok & (bins == b))[0]
            if not len(sel):
                continue
            ctx.set_source_rgba(0.62, 0.70, 0.80, (b + 0.5) / 8 * 0.17)
            for i in sel:
                ctx.move_to(ax[i], ay[i])
                ctx.line_to(bx[i], by[i])
            ctx.stroke()

    def draw_sensor(self, ctx, w, h):
        reach = 0.32
        th, tv = math.tan(HFOV / 2), math.tan(VFOV / 2)
        corners = np.array([(0, 0, 0)] + [(sx * th * reach, sy * tv * reach, -reach)
                                          for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))], np.float32)
        x, y, d = self.cam.project(corners, w, h)
        if (d < 0.1).any():
            return
        ctx.set_line_width(1)
        ctx.set_source_rgba(0.35, 0.75, 0.95, 0.45)
        for i in range(1, 5):
            ctx.move_to(x[0], y[0])
            ctx.line_to(x[i], y[i])
        ctx.move_to(x[1], y[1])
        for i in (2, 3, 4, 1):
            ctx.line_to(x[i], y[i])
        ctx.stroke()
        ctx.set_source_rgba(0.35, 0.75, 0.95, 0.9)
        ctx.arc(x[0], y[0], 2.5, 0, 2 * math.pi)
        ctx.fill()

    def draw_gizmo(self, ctx, w, h):
        r = self.cam.rotation()
        ox, oy, length = w - 42, h - 42, 20
        axes = [('X', r[:, 0], (0.97, 0.47, 0.45)), ('Y', r[:, 1], (0.49, 0.90, 0.53)),
                ('Z', -r[:, 2], (0.47, 0.75, 1.0))]
        axes.sort(key=lambda a: a[1][2])
        ctx.set_line_width(1.6)
        for name, v, colour in axes:
            ex, ey = ox + v[0] * length, oy - v[1] * length
            ctx.set_source_rgba(*colour, 0.9)
            ctx.move_to(ox, oy)
            ctx.line_to(ex, ey)
            ctx.stroke()
            label(ctx, ox + v[0] * (length + 9), oy - v[1] * (length + 9) + 4, name, 9.5, (*colour, 0.95),
                  cairo.FONT_WEIGHT_BOLD, 'center')

    def draw_depth(self, ctx, w, h):
        f = self.frame
        values=f.get('reflectance') if self.reflectance_mode else f['depth']
        if values is None:
            label(ctx,w/2,h/2,'Reflectance unavailable without ST calibration',13,align='center');return
        lo,hi=(0,100) if self.reflectance_mode else (self.depth_lo,self.depth_hi)
        rgb = colors_for(np.where(np.isfinite(values),values,0),lo,hi)
        img = pack(rgb)
        img[~self.valid | ~np.isfinite(values)] = np.uint32(0xff0e131b)
        self._depth_img = np.ascontiguousarray(img.reshape(H, W))
        scale = min((w - 48) / W, (h - 96) / H)
        iw, ih = W * scale, H * scale
        ox, oy = (w - iw) / 2, (h - ih) / 2 - 12
        ctx.save()
        rounded(ctx, ox, oy, iw, ih, 6)
        ctx.clip()
        ctx.translate(ox, oy)
        ctx.scale(scale, scale)
        ctx.set_source_surface(surface_from(self._depth_img, W, H), 0, 0)
        ctx.get_source().set_filter(cairo.FILTER_NEAREST)
        ctx.paint()
        ctx.restore()
        rounded(ctx, ox, oy, iw, ih, 6)
        ctx.set_source_rgba(1, 1, 1, 0.06)
        ctx.set_line_width(1)
        ctx.stroke()
        if self.reflectance_mode:label(ctx,16,h-20,'ST reflectance estimate · colour range 0–100% (values above 100% saturate)',10)
        else:draw_legend(ctx,16,h-34,self.depth_lo/1000,self.depth_hi/1000,'Depth')
        if self.hover:
            c = int((self.hover[0] - ox) // scale)
            r = int((self.hover[1] - oy) // scale)
            if 0 <= c < W and 0 <= r < H:
                i = r * W + c
                ctx.set_source_rgba(1, 1, 1, 0.85)
                ctx.set_line_width(1.5)
                ctx.rectangle(ox + c * scale, oy + r * scale, scale, scale)
                ctx.stroke()
                text = (f'{values[i]:.1f} %' if self.reflectance_mode else f'{values[i]/1000:.3f} m') if self.valid[i] else 'no return'
                tx, ty = min(self.hover[0] + 14, w - 96), max(self.hover[1] - 12, 20)
                rounded(ctx, tx - 8, ty - 15, 90, 22, 5)
                ctx.set_source_rgba(0.05, 0.07, 0.09, 0.88)
                ctx.fill()
                label(ctx, tx, ty, text, 11.5, (0.94, 0.96, 0.98, 1), cairo.FONT_WEIGHT_BOLD)

    def draw_top(self, ctx, w, h):
        reach = max(2.0, math.ceil(self.range_hi + 0.5))
        ox, oy = w / 2, h - 46
        scale = min((h - 90) / reach, (w / 2 - 30) / (reach * math.sin(math.radians(60))))
        ctx.set_line_width(1)
        for r in range(1, int(reach) + 1):
            ctx.set_source_rgba(0.62, 0.70, 0.80, 0.10)
            ctx.new_sub_path()
            ctx.arc(ox, oy, r * scale, math.pi, 2 * math.pi)
            ctx.stroke()
            label(ctx, ox + 5, oy - r * scale + 13, f'{r} m', 10, (0.49, 0.52, 0.56, 1))
        half = HFOV / 2
        ctx.move_to(ox, oy)
        ctx.line_to(ox + math.sin(-half) * reach * scale, oy - math.cos(half) * reach * scale)
        ctx.line_to(ox + math.sin(half) * reach * scale, oy - math.cos(half) * reach * scale)
        ctx.close_path()
        ctx.set_source_rgba(0.35, 0.75, 0.95, 0.035)
        ctx.fill_preserve()
        ctx.set_source_rgba(0.35, 0.75, 0.95, 0.22)
        ctx.stroke()
        if self.pts is not None and len(self.pts):
            buf = np.zeros((h, w), np.uint32)
            order = np.argsort(self.pts[:, 1])                      # higher points drawn last
            sx = np.rint(ox + self.pts[order, 0] * scale).astype(np.int32)
            sy = np.rint(oy + self.pts[order, 2] * scale).astype(np.int32)
            col = pack(self.rgb[order])
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    x, y = sx + dx, sy + dy
                    ok = (x >= 0) & (x < w) & (y >= 0) & (y < h)
                    buf[y[ok], x[ok]] = col[ok]
            ctx.set_source_surface(surface_from(buf, w, h), 0, 0)
            ctx.paint()
            draw_legend(ctx, 16, h - 34, self.range_lo, self.range_hi)
        ctx.set_source_rgba(0.35, 0.75, 0.95, 0.95)
        ctx.arc(ox, oy, 3.5, 0, 2 * math.pi)
        ctx.fill()

    # ---------- IMU widgets
    def on_horizon(self, widget, ctx):
        w, h = widget.get_allocated_width(), widget.get_allocated_height()
        cx, cy, r = w / 2, h / 2 + 6, min(w, h) / 2 - 10
        roll, pitch = self.att.rel[0], self.att.rel[1]
        ctx.save()
        ctx.arc(cx, cy, r, 0, 2 * math.pi)
        ctx.clip()
        ctx.translate(cx, cy)
        ctx.rotate(-math.radians(roll))
        shift = max(-r * 1.4, min(r * 1.4, pitch * r / 40))
        sky = cairo.LinearGradient(0, -r * 2 + shift, 0, shift)
        sky.add_color_stop_rgb(0, 0.05, 0.10, 0.20)
        sky.add_color_stop_rgb(1, 0.13, 0.23, 0.38)
        ctx.set_source(sky)
        ctx.rectangle(-r * 3, -r * 3 + shift, r * 6, r * 3)
        ctx.fill()
        ground = cairo.LinearGradient(0, shift, 0, shift + r * 2)
        ground.add_color_stop_rgb(0, 0.20, 0.15, 0.10)
        ground.add_color_stop_rgb(1, 0.09, 0.07, 0.05)
        ctx.set_source(ground)
        ctx.rectangle(-r * 3, shift, r * 6, r * 3)
        ctx.fill()
        ctx.set_source_rgba(1, 1, 1, 0.85)
        ctx.set_line_width(1.4)
        ctx.move_to(-r * 3, shift)
        ctx.line_to(r * 3, shift)
        ctx.stroke()
        ctx.set_source_rgba(1, 1, 1, 0.30)
        ctx.set_line_width(1)
        for deg in (-20, -10, 10, 20):
            y = shift - deg * r / 40
            half = 16 if deg % 20 else 26
            ctx.move_to(-half, y)
            ctx.line_to(half, y)
        ctx.stroke()
        ctx.restore()
        # roll scale
        ctx.set_source_rgba(1, 1, 1, 0.22)
        ctx.set_line_width(1)
        for deg in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            a = math.radians(deg) - math.pi / 2
            inner = r + (2 if deg % 30 else -1)
            ctx.move_to(cx + math.cos(a) * inner, cy + math.sin(a) * inner)
            ctx.line_to(cx + math.cos(a) * (r + 6), cy + math.sin(a) * (r + 6))
        ctx.stroke()
        a = -math.radians(roll) - math.pi / 2
        px, py = cx + math.cos(a) * (r - 3), cy + math.sin(a) * (r - 3)
        ctx.set_source_rgba(0.94, 0.71, 0.16, 1)
        ctx.move_to(px, py)
        ctx.line_to(px + math.cos(a + 2.7) * 7, py + math.sin(a + 2.7) * 7)
        ctx.line_to(px + math.cos(a - 2.7) * 7, py + math.sin(a - 2.7) * 7)
        ctx.close_path()
        ctx.fill()
        # fixed reference marks
        ctx.set_line_width(2.4)
        ctx.set_line_cap(cairo.LINE_CAP_ROUND)
        ctx.move_to(cx - 30, cy)
        ctx.line_to(cx - 10, cy)
        ctx.move_to(cx + 10, cy)
        ctx.line_to(cx + 30, cy)
        ctx.stroke()
        ctx.arc(cx, cy, 2.2, 0, 2 * math.pi)
        ctx.fill()
        ctx.set_source_rgba(1, 1, 1, 0.08)
        ctx.set_line_width(1)
        ctx.arc(cx, cy, r, 0, 2 * math.pi)
        ctx.stroke()

    def on_charts(self, widget, ctx):
        w, h = widget.get_allocated_width(), widget.get_allocated_height()
        plots = ((self.att.hist_a, 2.0, 'ACCEL', 'g'), (self.att.hist_g, 250.0, 'GYRO', '°/s'))
        ph = h / 2
        colours = ((1.0, 0.48, 0.45), (0.49, 0.91, 0.53), (0.47, 0.75, 1.0))
        for k, (hist, rng, name, unit) in enumerate(plots):
            top = k * ph
            label(ctx, 0, top + 11, name, 9.5, (0.49, 0.52, 0.56, 1), cairo.FONT_WEIGHT_BOLD)
            lx = w
            for axis in (2, 1, 0):
                value = hist[-1][axis] if hist else 0.0
                text = f'{"xyz"[axis]} {value:+.2f}' if rng < 10 else f'{"xyz"[axis]} {value:+.1f}'
                ctx.select_font_face(CAIRO_FONT, cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
                ctx.set_font_size(9.5)
                lx -= ctx.text_extents(text).x_advance + 8
                label(ctx, lx, top + 11, text, 9.5, (*colours[axis], 0.95))
            y0, y1 = top + 17, top + ph - 4
            mid = (y0 + y1) / 2
            ctx.set_source_rgba(1, 1, 1, 0.06)
            ctx.set_line_width(1)
            ctx.move_to(0, round(mid) + 0.5)
            ctx.line_to(w, round(mid) + 0.5)
            ctx.stroke()
            if len(hist) < 2:
                continue
            arr = np.asarray(hist, np.float32)
            xs = np.linspace(w * (1 - len(arr) / hist.maxlen), w, len(arr))
            ys = mid - np.clip(arr / rng, -1, 1) * (y1 - y0) / 2
            ctx.set_line_width(1.25)
            ctx.set_line_join(cairo.LINE_JOIN_ROUND)
            for axis in range(3):
                ctx.set_source_rgba(*colours[axis], 0.9)
                ctx.move_to(xs[0], ys[0, axis])
                for x, y in zip(xs[1:], ys[1:, axis]):
                    ctx.line_to(x, y)
                ctx.stroke()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--usb', default='auto', help='serial device or socket:// URL (default: auto-detect)')
    ap.add_argument('--no-temporal', action='store_true', help='disable ST temporal filtering')
    ap.add_argument('--log', action='store_true', help='print FPS and timings every 2 s')
    args = ap.parse_args()
    acq = Acquisition(args.usb, temporal=not args.no_temporal)
    win = Viewer(acq)
    if args.log:
        def report():
            f = win.frame
            draws = list(win.draw_times)
            print(f'usb {acq.fps:.2f} FPS · frames {acq.received}'
                  + (f' · capture {f["capture_ms"]} ms · process {f["process_ms"]:.1f} ms' if f else '')
                  + (f' · draw {np.median(draws):.1f} ms (cloud {win.renderer.ms:.1f})' if draws else '')
                  + (f' · error {acq.error}' if acq.error else ''), flush=True)
            return True
        GLib.timeout_add(2000, report)
    win.show_all()
    acq.start()
    Gtk.main()


if __name__ == '__main__':
    main()
