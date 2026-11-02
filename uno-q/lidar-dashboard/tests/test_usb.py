import sys,struct,time
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from usb_source import USBSource
from raw_source import FRAME_SIZE,fnv1a

class FakeSerial:
    def __init__(self,data):self.data=bytearray(data);self.written=b''
    def write(self,data):self.written+=data;return len(data)
    def read(self,n):
        size=min(n,37,len(self.data));out=bytes(self.data[:size]);del self.data[:size];return out

def packet(sequence=2):
    payload=bytearray(FRAME_SIZE)
    struct.pack_into('<I',payload,14742,1)
    payload[14742+62]=0x40
    header=[0x4c394652,1,2,0,sequence,54,42,FRAME_SIZE,fnv1a(payload),100,0,0,2,0,0,1]
    return b'L9R1'+struct.pack('<18I',*header,0x70,0x53334c39)+payload

class USBTests(unittest.TestCase):
    def source(self,data):
        source=USBSource.__new__(USBSource);source.serial=FakeSerial(data)
        source.port='test';source.last_sequence=None;return source
    def test_fragmented_complete_frame(self):
        s=self.source(packet());f,p=s.read()
        self.assertEqual(s.serial.written,b'F')
        self.assertEqual(len(p),FRAME_SIZE)
        self.assertEqual(f['transport'],'USB CDC')
        self.assertEqual(f['sensor_warnings'],['reference array'])
    def test_post_capture_sensor_error_rejected(self):
        data=bytearray(packet())
        struct.pack_into('<I',data,4+13*4,1)
        with self.assertRaisesRegex(RuntimeError,'sensor acquisition failed'):
            self.source(data).read()
    def test_sequence_gap_rejected(self):
        s=self.source(packet(5));s.last_sequence=3
        with self.assertRaisesRegex(RuntimeError,'Nonconsecutive'):s.read()
    def test_sequence_wrap_accepted(self):
        s=self.source(packet(0));s.last_sequence=0xffffffff
        frame,_=s.read();self.assertEqual(frame['sequence'],0)
    def test_information_packet_fragmentation_and_checksum(self):
        data=bytes(range(64))
        wire=b'L9D1'+struct.pack('<III',1,len(data),fnv1a(data))+data
        self.assertEqual(self.source(wire).read_info(b'D',b'L9D1',64),data)
        bad=bytearray(wire);bad[-1]^=1
        with self.assertRaisesRegex(RuntimeError,'checksum'):
            self.source(bad).read_info(b'D',b'L9D1',64)
    def test_information_abi_rejected(self):
        wire=b'L9D1'+struct.pack('<III',2,64,0)
        with self.assertRaisesRegex(RuntimeError,'ABI'):
            self.source(wire).read_info(b'D',b'L9D1',64)
    def test_streaming_abi_and_single_start_command(self):
        def streaming_packet(seq):
            data=bytearray(packet(seq));data[:4]=b'L9R3'
            struct.pack_into('<I',data,8,2)
            struct.pack_into('<I',data,4+12*4,3)
            return data
        s=self.source(streaming_packet(2)+streaming_packet(3))
        s.diagnostics={'version':2,'mode':2}
        self.assertEqual(s.read()[0]['sequence'],2)
        self.assertEqual(s.read()[0]['sequence'],3)
        self.assertEqual(s.serial.written,b'Q')
        s.stop_stream()
        self.assertEqual(s.serial.written,b'QP')
        self.assertIsNone(s.last_sequence)

    def test_legacy_firmware_cannot_report_streaming_state(self):
        data=bytearray(packet());struct.pack_into('<I',data,4+12*4,3)
        with self.assertRaisesRegex(RuntimeError,'sensor acquisition failed'):
            self.source(data).read()

    def test_wrong_magic_rejected(self):
        s=self.source(b'BAD!'+packet()[4:])
        with self.assertRaisesRegex(RuntimeError,'framing'):s.read()
    def test_corrupt_payload_rejected(self):
        data=bytearray(packet());data[-1]^=1
        with self.assertRaisesRegex(ValueError,'checksum'):self.source(data).read()
    def test_short_transfer_timeout(self):
        s=self.source(b'abc')
        with self.assertRaisesRegex(RuntimeError,'timeout'):s.exact(5,time.monotonic()-1)

if __name__=='__main__':unittest.main()
