import sys,struct,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from usb_source import decode_imu
from raw_source import fnv1a,decode_frame
import test_usb

def imu_packet(status=0):
    data=b'I6R1'+struct.pack('<III7hH',10,1000,status,256,-100,200,-300,0,0,16384,0)
    return data+struct.pack('<I',fnv1a(data))

class IMUTests(unittest.TestCase):
    def test_units_signed_values_and_tilt(self):
        m=decode_imu(imu_packet())
        self.assertEqual(m['temperature_c'],26)
        self.assertEqual(m['gyro_dps'],[-.875,1.75,-2.625])
        self.assertAlmostEqual(m['accel_g'][2],.999424)
        self.assertEqual(m['tilt'],{'roll_deg':0.,'pitch_deg':0.})
    def test_failed_imu_not_presented_as_valid(self):
        m=decode_imu(imu_packet(2));self.assertFalse(m['valid']);self.assertIsNone(m['accel_g'])
    def test_bad_checksum(self):
        p=bytearray(imu_packet());p[19]^=1
        with self.assertRaisesRegex(ValueError,'checksum'):decode_imu(p)
    def test_interleaved_fragmented_imu_and_lidar(self):
        s=test_usb.USBTests().source(imu_packet()+imu_packet()+b'L9R2'+test_usb.packet()[4:]);samples=[];s.on_imu=samples.append
        f,p=s.read();self.assertEqual(len(samples),2);self.assertTrue(f['imu']['measurements']['valid']);self.assertEqual(len(p),14842)
    def test_recovery_command_and_header_only_error(self):
        p=bytearray(b'L9R2'+test_usb.packet()[4:76]);struct.pack_into('<I',p,4+2*4,3)
        s=test_usb.USBTests().source(p)
        with self.assertRaisesRegex(RuntimeError,'sensor acquisition failed'):s.read(recover=True)
        self.assertEqual(s.serial.written,b'R')
    def test_depth_flag_separated_without_altering_raw(self):
        import raw_source
        p=bytearray(14842);struct.pack_into('<2H',p,0,0x8123,0x123)
        h=dict(width=54,height=42,payload_size=14842,fnv1a=fnv1a(p),frame_counter=0,sequence=1,capture_ms=1)
        f=decode_frame(p,h);self.assertEqual(f['depth'][:2],[0x8123,0x123]);self.assertEqual(f['depth_values'][:2],[0x123,0x123]);self.assertEqual(f['depth_main_flags'][:2],[True,False])
