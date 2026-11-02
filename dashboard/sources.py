"""Frame sources for the dashboard: real USB serial, or a built-in simulator."""

import threading
import time
import numpy as np

import protocol


class BaseSource:
    def start(self):
        pass

    def read(self):
        """Return the latest Frame or None."""
        raise NotImplementedError

    def close(self):
        pass


class SerialSource(BaseSource):
    """Reads frames from the board's USB CDC port in a background thread."""

    def __init__(self, port, baud=921600):
        import serial
        self._ser = serial.Serial(port, baud, timeout=0.05)
        self._buf = bytearray()
        self._latest = None
        self._lock = threading.Lock()
        self._run = False
        self._thread = None
        self.frames = 0

    def start(self):
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._run:
            chunk = self._ser.read(4096)
            if chunk:
                self._buf += chunk
                self._drain()
            else:
                time.sleep(0.001)

    def _drain(self):
        buf = self._buf
        while True:
            i = buf.find(protocol.MAGIC)
            if i < 0:
                # keep only a tail that might be a partial magic
                if len(buf) > 3:
                    del buf[:-3]
                return
            if i > 0:
                del buf[:i]
            frame, total = protocol.parse_body(bytes(buf))
            if frame is None:
                if total == -1:      # bad magic/checksum: skip this magic
                    del buf[:4]
                    continue
                return               # need more bytes
            del buf[:total]
            with self._lock:
                self._latest = frame
                self.frames += 1

    def read(self):
        with self._lock:
            return self._latest

    def close(self):
        self._run = False
        try:
            self._ser.close()
        except Exception:
            pass


class SimSource(BaseSource):
    """Synthetic data so the dashboard is demonstrable with no hardware.

    Scene: the sensor sits in a virtual room and slowly pans; a box sits in
    front of a back wall, so the depth map and top-down map show real
    structure. The IMU gently oscillates (tilt + rotation).
    """

    def __init__(self, width=54, height=42, fps=15):
        self.width = width
        self.height = height
        self.period = 1.0 / fps
        self._seq = 0
        self._t0 = time.time()
        self._last = 0.0
        # per-zone horizontal / vertical angles (radians), FoV ~ 60 x 45 deg
        self._hfov = np.deg2rad(60.0)
        self._vfov = np.deg2rad(45.0)

    def read(self):
        now = time.time()
        if now - self._last < self.period:
            return None
        self._last = now
        t = now - self._t0
        w, h = self.width, self.height

        cols = (np.arange(w) - (w - 1) / 2) / w * self._hfov
        rows = (np.arange(h) - (h - 1) / 2) / h * self._vfov
        ang_x = cols[None, :] + 0.25 * np.sin(t * 0.5)      # slow pan
        ang_y = rows[:, None]

        # back wall at 2.5 m (perpendicular distance), range = d / (cos ax cos ay)
        wall = 2500.0 / (np.cos(ang_x) * np.cos(ang_y) + 1e-3)
        # a box ~1.2 m ahead, offset to the right, moving
        bx = 0.35 + 0.25 * np.sin(t * 0.7)
        box = (np.abs(ang_x - bx) < 0.22) & (np.abs(ang_y) < 0.25)
        depth = np.where(box, 1200.0 / (np.cos(ang_x) * np.cos(ang_y) + 1e-3), wall)
        depth += np.random.normal(0, 8, size=depth.shape)   # sensor noise
        depth = np.clip(depth, 50, 8800)

        valid = np.random.random(depth.shape) > 0.03        # a few invalid zones
        words = (depth.astype(np.uint16) & 0x7FFF) | (valid.astype(np.uint16) << 15)

        # IMU: gentle tilt + yaw rotation
        ax = int(16384 * 0.15 * np.sin(t * 0.6))
        ay = int(16384 * 0.12 * np.cos(t * 0.5))
        az = int(16384 * (0.98 + 0.01 * np.sin(t)))
        gz = int((20.0 * np.sin(t * 0.7)) / protocol.GYRO_DPS_PER_LSB)
        gx = int((5.0 * np.sin(t * 1.3)) / protocol.GYRO_DPS_PER_LSB)
        gy = int((5.0 * np.cos(t * 1.1)) / protocol.GYRO_DPS_PER_LSB)
        temp = int((28.0 - protocol.TEMP_OFFSET_C) / protocol.TEMP_C_PER_LSB)

        raw = protocol.pack_frame(self._seq, (ax, ay, az), (gx, gy, gz), temp,
                                  w, h, words.flatten().tolist())
        self._seq += 1
        frame, _ = protocol.parse_body(raw)
        return frame
