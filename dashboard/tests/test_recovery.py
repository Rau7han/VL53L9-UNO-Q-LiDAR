import json,tempfile,time,unittest
from pathlib import Path
from raw_server import CaptureSession
import test_usb
from processing import ProcessingError

class Source:
    def __init__(self,persistent):self.persistent=persistent;self.reads=0;self.reconnects=0;self.on_imu=None;self.recovery_flags=[]
    def reconnect(self):self.reconnects+=1
    def read(self,**kwargs):
        self.reads+=1;self.recovery_flags.append(kwargs.get('recover',False))
        if self.persistent or self.reads==1:raise RuntimeError('test sensor fault')
        self.session.stopping.set()
        return test_usb.USBTests().source(test_usb.packet()).read()

class RecoveryTests(unittest.TestCase):
    def run_session(self,persistent):
        with tempfile.TemporaryDirectory() as directory:
            source=Source(persistent);session=CaptureSession(source,directory,0);source.session=session
            session.stopping.wait=lambda timeout:session.stopping.is_set()
            session.thread.start();deadline=time.monotonic()+2
            while session.thread.is_alive() and session.running.is_set() and time.monotonic()<deadline:time.sleep(.005)
            session.stopping.set();session.thread.join(1)
            self.assertFalse(session.thread.is_alive())
            errors=[json.loads(s) for s in (Path(directory)/'errors.jsonl').read_text().splitlines()]
            return session,source,errors
    def test_transient_fault_recovers_with_error_history(self):
        session,source,errors=self.run_session(False)
        self.assertEqual(source.recovery_flags,[False,False]);self.assertEqual(source.reconnects,1);self.assertEqual(session.recoveries,1);self.assertIsNone(session.error);self.assertEqual(len(errors),1);self.assertEqual(session.frames,1)
    def test_persistent_fault_stops_after_three_retries(self):
        session,source,errors=self.run_session(True)
        self.assertEqual(source.reconnects,3);self.assertFalse(session.running.is_set());self.assertEqual(len(errors),4);self.assertFalse(errors[-1]['retry'])

    def test_processing_error_does_not_reset_hardware(self):
        with tempfile.TemporaryDirectory() as directory:
            source=Source(False);source.reads=1
            session=CaptureSession(source,directory,0);source.session=session
            session.handle_result({'processing_error':'host library failed'})
            self.assertEqual(source.reconnects,0)
            self.assertFalse(session.running.is_set())
            self.assertEqual(session.error,'host library failed')
            self.assertTrue(session.processing_failed)
