import sys,threading,queue,unittest
from pathlib import Path
from unittest.mock import patch,Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from lidar_gui import Acquisition
from usb_source import USBSource
class NativeRecoveryTests(unittest.TestCase):
    def test_dropped_frame_keeps_reconnect_calibration(self):
        a=Acquisition.__new__(Acquisition);a.inbox=queue.Queue(maxsize=1);a.dropped=0
        a._offer(('frame',{},b'old',b'calibration',1))
        a._offer(('frame',{},b'new',b'calibration',1))
        value=a.inbox.get_nowait()
        self.assertEqual(value[2:],(b'new',b'calibration',1));self.assertEqual(a.dropped,1)
    def test_processing_failure_does_not_keep_old_cloud(self):
        a=Acquisition.__new__(Acquisition);a.stop=threading.Event();a.lock=threading.Lock()
        a.inbox=queue.Queue();a.latest={'old':True};a.processing_error=None
        a.processor=Mock()
        def fail(*args):a.stop.set();raise RuntimeError('bad transform')
        a.processor.process_arrays.side_effect=fail
        a.inbox.put(('frame',{'sequence':1,'frame_error_code':0,'frame_error_flags':0},b'',None,0))
        a._compute()
        self.assertIsNone(a.latest);self.assertIn('bad transform',a.processing_error)
        a.processor.close.assert_called_once()
    def test_cancelled_read_exits_without_waiting_for_frame_timeout(self):
        s=USBSource.__new__(USBSource);s.cancel_event=threading.Event();s.cancel_event.set();s.serial=Mock()
        with self.assertRaisesRegex(RuntimeError,'stopped'):s.exact(100,1e20)
        s.serial.read.assert_not_called()
