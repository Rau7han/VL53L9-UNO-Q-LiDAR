import sys
from pathlib import Path
import struct
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from raw_source import decode_frame,fnv1a,PIXELS,FRAME_SIZE

class RawFrameTests(unittest.TestCase):
    def setUp(self):
        self.payload=(struct.pack('<%dH'%PIXELS,*range(PIXELS))+
                      struct.pack('<%dH'%PIXELS,*[32768+i for i in range(PIXELS)])+
                      struct.pack('<%dH'%PIXELS,*[65535-i for i in range(PIXELS)])+
                      bytes(i%256 for i in range(PIXELS//2))+
                      struct.pack('<I',17)+bytes(range(96)))
        self.header=dict(width=54,height=42,payload_size=FRAME_SIZE,
                         fnv1a=fnv1a(self.payload),sequence=4,frame_counter=17,capture_ms=100)
    def test_full_payload_and_boundaries(self):
        f=decode_frame(self.payload,self.header)
        self.assertEqual(len(self.payload),14842)
        self.assertEqual([len(f[k]) for k in ['depth','amplitude','ambient','dss','status_bytes']],[2268,2268,2268,1134,100])
        self.assertEqual(f['depth'][-1],2267)
        self.assertEqual(f['amplitude'][0],32768)
        self.assertEqual(f['ambient'][-1],65535-2267)
        self.assertEqual(f['status_bytes'][:4],[17,0,0,0])
        self.assertEqual(f['status_bytes'][-1],95)
    def test_corrupt_rejected(self):
        p=bytearray(self.payload);p[500]^=1
        with self.assertRaisesRegex(ValueError,'checksum'):decode_frame(p,self.header)
    def test_truncated_rejected(self):
        with self.assertRaisesRegex(ValueError,'Incomplete'):decode_frame(self.payload[:-1],self.header)
    def test_wrong_resolution_rejected(self):
        with self.assertRaisesRegex(ValueError,'dimensions'):decode_frame(self.payload,self.header|{'width':4})
    def test_mixed_frame_counter_rejected(self):
        with self.assertRaisesRegex(ValueError,'counter'):decode_frame(self.payload,self.header|{'frame_counter':16})
    def test_standard_fnv_vector(self):
        self.assertEqual(fnv1a(b'hello'),0x4f9f2cab)
    def test_native_and_fallback_checksum_match(self):
        import raw_source
        from unittest.mock import patch
        native=raw_source._native_fnv
        self.assertIsNotNone(native, 'Rebuild the native transform library')
        for length in (0,1,255,256,14842):
            data=bytes((i*37+13)%256 for i in range(length))
            with patch.object(raw_source, '_native_fnv', None):
                expected=fnv1a(data)
            for value in (data,bytearray(data),memoryview(data)):
                self.assertEqual(fnv1a(value), expected)

    def test_frame_error_reported_even_when_stopped_status_clears(self):
        payload=bytearray(self.payload)
        payload[14742+60:14742+63]=b'\x00\x00\x40'
        header=self.header|{'fnv1a':fnv1a(payload),'error_flags':0}
        frame=decode_frame(payload,header)
        self.assertEqual(frame['frame_error_flags'],0x40)
        self.assertEqual(frame['sensor_warnings'],['reference array'])

if __name__=='__main__':unittest.main()
