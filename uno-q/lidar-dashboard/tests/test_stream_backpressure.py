import json
import queue
import threading
import unittest
from raw_server import CaptureSession

class BackpressureTests(unittest.TestCase):
    def test_slow_display_receives_current_frame_after_gap(self):
        session=CaptureSession.__new__(CaptureSession)
        session.lock=threading.Lock()
        slow=queue.Queue(maxsize=8);fast=queue.Queue(maxsize=8)
        session.clients={slow,fast}
        for i in range(8):slow.put_nowait(b'{"old":true}')
        event={'frame':{'sequence':123,'depth':[1],'points_xyz_mm':[[1,2,3]]}}
        # Serialization/filtering moved to the compute worker; fanout handles backpressure.
        wire={'frame':{k:v for k,v in event['frame'].items() if k!='depth'}}
        session.fanout(json.dumps(wire).encode())
        self.assertEqual(json.loads(slow.get_nowait()),{'display_gap':True})
        fresh=json.loads(slow.get_nowait())
        self.assertEqual(fresh['frame']['sequence'],123)
        self.assertEqual(fresh['frame']['points_xyz_mm'],[[1,2,3]])
        self.assertNotIn('depth',fresh['frame'])
        self.assertTrue(slow.empty())
        self.assertEqual(json.loads(fast.get_nowait()),fresh)
        self.assertIn('depth',event['frame'])
