"""ST processing with real calibration, or explicitly uncalibrated preview.

Never substitute another device's calibration or synthesize ST confidence.
Raw planes remain byte-for-byte available alongside processed output.
"""
import ctypes as ct
from functools import lru_cache
import hashlib
import math
from pathlib import Path
import time
import numpy as np

PIXELS = 54 * 42

class ProcessingError(RuntimeError):
    """Host processing failed; resetting the sensor cannot repair this."""

@lru_cache(maxsize=8)
def nominal_rays(width, height):
    tx, ty = math.tan(math.radians(55/2)), math.tan(math.radians(42/2))
    rays=[]
    for i in range(width*height):
        x = ((i % width + .5) / width * 2 - 1) * tx
        y = ((i // width + .5) / height * 2 - 1) * ty
        z = 1 / math.sqrt(1 + x*x + y*y)
        rays.append((x*z,y*z,z))
    return tuple(rays)

def nominal_projection(depth, width=54, height=42):
    """Radial distance along a unit ray; approximate nominal 55 x 42 deg FoV."""
    if len(depth)!=width*height:raise ProcessingError('Depth dimensions do not match')
    return [[round(radius*x,3),round(radius*y,3),round(radius*z,3)]
            for radius,(x,y,z) in zip(depth,nominal_rays(width,height))]

class Processor:
    def __init__(self, calibration=None, temporal=False):
        self.calibration = calibration
        self.temporal = temporal
        self.handle = None
        self.error = None
        self.previous_time = None
        self.previous_sequence = None
        self.calibration_hash=hashlib.sha256(calibration).hexdigest() if calibration else None
        self.xy=(ct.c_float*(PIXELS*4))()
        self.flags=(ct.c_uint32*PIXELS)()
        self.reflectance=(ct.c_float*PIXELS)()
        self.last_reflectance=None
        if calibration is not None:
            if len(calibration) != 2332 or len(set(calibration)) < 2:
                raise ProcessingError('Invalid sensor calibration blob')
            path = Path(__file__).with_name('build') / 'libvl53l9_transform.so'
            try:
                self.lib = ct.CDLL(str(path))
            except OSError as exc:
                raise ProcessingError('Build the ST library with python3 dashboard/build_transform.py') from exc
            self.lib.l9_create.argtypes = [ct.c_void_p, ct.c_uint, ct.c_int]
            self.lib.l9_create.restype = ct.c_void_p
            self.lib.l9_process.argtypes = [ct.c_void_p, ct.c_void_p, ct.c_uint, ct.c_void_p, ct.c_void_p]
            self.lib.l9_process.restype = ct.c_int
            self.lib.l9_process_full.argtypes = [ct.c_void_p,ct.c_void_p,ct.c_uint,ct.c_void_p,ct.c_void_p,ct.c_void_p]
            self.lib.l9_process_full.restype = ct.c_int
            self.lib.l9_destroy.argtypes = [ct.c_void_p]
            self.lib.l9_destroy.restype = None
            self.reset()

    def reset(self):
        self.close()
        self.handle = self.lib.l9_create(self.calibration, len(self.calibration), self.temporal)
        if not self.handle:
            raise ProcessingError('ST transform rejected factory calibration/configuration')

    def close(self):
        if self.handle:
            self.lib.l9_destroy(self.handle)
            self.handle = None

    def process(self, frame, payload):
        started = time.monotonic()
        if frame.get('frame_error_code') or frame.get('frame_error_flags'):
            points = [[0, 0, 0]] * PIXELS
            valid = [False] * PIXELS
            confidence = None
            filter_status = None
            description = 'Sensor reported a frame error; all points suppressed'
            if self.handle:
                self.reset()
        elif self.calibration is None:
            points = nominal_projection(frame['depth_values'], frame['width'], frame['height'])
            valid = [0 < d <= 8800 for d in frame['depth_values']]
            confidence = None
            filter_status = None
            description = 'Uncalibrated preview; nominal FoV; ST confidence unavailable'
        else:
            if self.previous_time is not None and (started-self.previous_time > 1 or
                    frame['sequence'] != (self.previous_sequence+1) & 0xffffffff):
                self.reset()
            xy,flags=self.xy,self.flags
            result = self.lib.l9_process(self.handle, payload, len(payload), xy, flags)
            if result:
                # Fail visibly; do not silently turn a processing fault into raw valid depth.
                self.reset()
                raise ProcessingError(f'ST transform processing failed: {result}')
            data=np.ctypeslib.as_array(xy).reshape(PIXELS,4)
            flag_array=np.ctypeslib.as_array(flags)
            valid=((flag_array==0)&np.isfinite(data[:,:3]).all(axis=1)&(data[:,2]>0)).tolist()
            # Millimetres to 0.001 mm for JSON, well below the sensor precision.
            points=np.round(data[:,:3].astype(np.float64),3).tolist()
            filter_status=flag_array.tolist()
            c=data[:,3].astype(np.float64)
            confidence=np.where(np.isfinite(c),np.floor(np.maximum(0,c)*1000)/1000,0).tolist()
            description = 'ST transform 1.5.0; factory calibration; distortion and confidence filters'
        self.previous_time = started
        self.previous_sequence = frame['sequence']
        frame['points_xyz_mm'] = [p if ok else None for p, ok in zip(points, valid)]
        frame['depth_mm'] = [round(p[2], 3) if ok else None for p, ok in zip(points, valid)]
        frame['valid'] = valid
        frame['confidence'] = confidence
        frame['filter_status'] = filter_status
        frame['processing'] = {
            'calibrated': self.calibration is not None, 'description': description,
            'temporal_filter': self.temporal if self.calibration is not None else False,
            'valid_points': sum(valid), 'confidence_available': confidence is not None,
            'calibration_sha256': self.calibration_hash,
            'milliseconds': round((time.monotonic()-started)*1000, 3),
            'coordinates': 'x right, y down, z forward; mm (nominal only without calibration)'}
        return frame

    def process_arrays(self, payload, sequence, error_code=0, error_flags=0):
        """Return (xyz_m, depth_mm, valid_mask, confidence, filter_flags, elapsed_ms) as NumPy arrays."""
        started = time.monotonic()
        self.last_reflectance=None
        raw = np.frombuffer(payload, dtype='<u2', count=PIXELS * 3)
        depth_raw = raw[:PIXELS]
        depth_vals = (depth_raw & 0x7fff).astype(np.float32)
        if error_code or error_flags:
            if self.handle:
                self.reset()
            zeros = np.zeros((PIXELS, 3), dtype=np.float32)
            valid = np.zeros(PIXELS, dtype=bool)
            ms = round((time.monotonic() - started) * 1000, 3)
            self.previous_time = started
            self.previous_sequence = sequence
            return zeros, np.zeros(PIXELS, dtype=np.float32), valid, None, None, ms
        if self.calibration is None:
            rays = np.asarray(nominal_rays(54, 42), dtype=np.float32)
            xyz_mm = rays * depth_vals[:, None]
            valid = (depth_vals > 0) & (depth_vals <= 8800)
            xyz_m = np.column_stack((xyz_mm[:, 0], -xyz_mm[:, 1], -xyz_mm[:, 2])) * 0.001
            ms = round((time.monotonic() - started) * 1000, 3)
            self.previous_time = started
            self.previous_sequence = sequence
            return xyz_m, xyz_mm[:, 2], valid, None, None, ms
        if self.previous_time is not None and (started - self.previous_time > 1 or
                sequence != (self.previous_sequence + 1) & 0xffffffff):
            self.reset()
        xy, flags = self.xy, self.flags
        result = self.lib.l9_process_full(self.handle, payload, len(payload), xy, flags, self.reflectance)
        if result:
            self.reset()
            raise ProcessingError(f'ST transform processing failed: {result}')
        self.last_reflectance=np.ctypeslib.as_array(self.reflectance).copy()
        data = np.ctypeslib.as_array(xy).reshape(PIXELS, 4)
        flag_array = np.ctypeslib.as_array(flags)
        valid = (flag_array == 0) & np.isfinite(data[:, :3]).all(axis=1) & (data[:, 2] > 0)
        xyz_m = np.empty((PIXELS, 3), dtype=np.float32)
        xyz_m[:, 0] = data[:, 0] * 0.001
        xyz_m[:, 1] = -data[:, 1] * 0.001
        xyz_m[:, 2] = -data[:, 2] * 0.001
        depth_mm = np.where(valid, data[:, 2], 0.0).astype(np.float32)
        conf = np.where(np.isfinite(data[:, 3]), np.maximum(0.0, data[:, 3]), 0.0).astype(np.float32)
        self.previous_time = started
        self.previous_sequence = sequence
        ms = round((time.monotonic() - started) * 1000, 3)
        return xyz_m, depth_mm, valid, conf, flag_array.copy(), ms
