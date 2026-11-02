import ctypes as ct
import math
from pathlib import Path
import struct
import unittest
from processing import nominal_projection, Processor, ProcessingError
from raw_source import FRAME_SIZE, PIXELS

class ProcessingTests(unittest.TestCase):
    def test_projection_preserves_radial_range_and_symmetry(self):
        points=nominal_projection([2000]*PIXELS)
        for point in points:
            self.assertAlmostEqual(math.sqrt(sum(v*v for v in point)), 2000, delta=.001)
        self.assertEqual(points[0][0], -points[-1][0])
        self.assertEqual(points[0][1], -points[-1][1])
        self.assertLess(points[0][2], points[PIXELS//2+27][2])

    def test_failed_sensor_frame_has_no_shown_points(self):
        frame={'sequence':1,'width':54,'height':42,'depth_values':[2000]*PIXELS,
               'frame_error_flags':0x40}
        result=Processor().process(frame,b'')
        self.assertFalse(any(result['valid']))
        self.assertTrue(all(p is None for p in result['points_xyz_mm']))
        self.assertFalse(result['processing']['calibrated'])

    def test_preview_does_not_fabricate_confidence(self):
        frame={'sequence':1,'width':54,'height':42,'depth_values':[2000]*PIXELS}
        result=Processor().process(frame,b'')
        self.assertIsNone(result['confidence'])
        self.assertFalse(result['processing']['confidence_available'])

    def test_constant_calibration_rejected(self):
        with self.assertRaises(ProcessingError):Processor(bytes(2332))

class NativeMetadataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=Path(__file__).resolve().parents[1]/'build/libvl53l9_transform.so'
        cls.lib=ct.CDLL(str(path))
        cls.lib.l9_validate_frame.argtypes=[ct.c_void_p,ct.c_uint]

    def valid_packet(self):
        raw=bytearray(FRAME_SIZE)
        struct.pack_into('<HHBBH',raw,14742+52,54,42,0x10,4,0x7080)
        return raw

    def check(self,raw):return self.lib.l9_validate_frame(bytes(raw),len(raw))

    def test_supported_metadata(self):self.assertEqual(self.check(self.valid_packet()),0)

    def test_malformed_dimensions_rejected_before_vendor_pipeline(self):
        raw=self.valid_packet();struct.pack_into('<H',raw,14742+52,65535)
        self.assertEqual(self.check(raw),-2)

    def test_reserved_dss_index_rejected(self):
        raw=self.valid_packet();raw[PIXELS*6]=7
        self.assertEqual(self.check(raw),-3)

    def test_cropping_rejected_by_fixed_frame_abi(self):
        raw=self.valid_packet();raw[14742+75]=1
        self.assertEqual(self.check(raw),-2)

    def test_truncation_rejected(self):self.assertEqual(self.check(self.valid_packet()[:-1]),-1)
