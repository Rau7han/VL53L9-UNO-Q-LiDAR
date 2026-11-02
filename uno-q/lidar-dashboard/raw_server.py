"""Local live full-frame dashboard; USB uses pyserial, no synthetic fallback."""
from collections import deque
import argparse
import fcntl
import json
import multiprocessing
from pathlib import Path
import secrets
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
from raw_source import RawSource, ROOT, decode_frame
from usb_source import USBSource, SensorError
from processing import Processor, ProcessingError

# Planes the browser does not read. Encoding them on the USB process stalls the next read.
_SSE_SKIP = ('depth', 'dss', 'status_bytes', 'confidence', 'filter_status')


def run_compute(inbox, outbox, calibration, temporal, directory, stop):
    """ST transform, capture files, and JSON. Separate process so its GIL cannot stall USB."""
    directory = Path(directory)
    processor = Processor(calibration, temporal=temporal)
    last_json = 0.0
    generation = None
    try:
        while not stop.is_set():
            try:
                item = inbox.get(timeout=0.2)
            except queue.Empty:
                continue
            kind = item[0]
            try:
                if kind == 'cal':
                    processor.close()
                    processor = Processor(item[1], temporal=temporal)
                    continue
                frame, payload, status = item[1], item[2], item[3]
                if len(item)>4 and generation != status['session_id']:
                    processor.close()
                    processor=Processor(item[4],temporal=temporal)
                    generation=status['session_id']
                decoded = decode_frame(payload, frame['mailbox'])
                decoded.update(frame)
                frame = processor.process(decoded, payload)
                stem = f'{time.time_ns()}-seq{frame["sequence"]}'
                (directory/f'{stem}.bin').write_bytes(payload)
                metadata = {k: v for k, v in frame.items() if k not in (
                    'depth', 'depth_values', 'depth_main_flags', 'amplitude', 'ambient', 'dss',
                    'status_bytes', 'points_xyz_mm', 'depth_mm', 'valid', 'confidence', 'filter_status')}
                metadata['file'] = f'{stem}.bin'
                with (directory/'index.jsonl').open('a') as log:
                    log.write(json.dumps(metadata)+'\n')
                now = time.monotonic()
                if now-last_json >= 1:
                    temp = directory/'latest.json.tmp'
                    temp.write_text(json.dumps(frame, separators=(',', ':')))
                    temp.replace(directory/'latest.json')
                    last_json = now
                event = {
                    'session_id': status['session_id'],
                    'frame': {k: v for k, v in frame.items() if k not in _SSE_SKIP},
                    'message': status['message'], 'error': status['error'], 'running': status['running'],
                    'frames_received': status['frames_received'], 'age_seconds': 0,
                    'host_fps': status['host_fps'], 'control_token': status['control_token'],
                    'imu': status['imu'], 'imu_received': status['imu_received'],
                    'imu_age_seconds': status['imu_age_seconds'], 'recoveries': status['recoveries'],
                    'retry_attempt': status['retry_attempt'],
                }
                data = json.dumps(event, separators=(',', ':')).encode()
                try:
                    outbox.put_nowait(data)
                except queue.Full:
                    try: outbox.get_nowait()
                    except queue.Empty: pass
                    try: outbox.put_nowait(data)
                    except queue.Full: pass
                if frame['sequence'] % 30 == 0:
                    print(f'Frame {frame["sequence"]}: {len(payload)} bytes, capture {frame["capture_ms"]} ms, '
                          f'host {status["host_fps"]:.1f} FPS, FNV {frame["checksum"]}, '
                          f'frame flags 0x{frame["frame_error_flags"]:02x}, FSM {frame["mailbox"]["fsm"]}', flush=True)
            except Exception as exc:
                outbox.put({'processing_error':str(exc)},timeout=2)
                break
    finally:
        processor.close()


class CaptureSession:
    def __init__(self, source, directory, interval=1.0, temporal=True):
        self.source = source
        self.directory = Path(directory)
        self.directory.mkdir(parents=True,exist_ok=True)
        calibration=getattr(source, 'calibration', None)
        self.processor=Processor(calibration, temporal=temporal)
        if calibration:
            (self.directory/'sensor-calibration.bin').write_bytes(calibration)
        self.interval = interval
        self.lock = threading.Lock()
        self.running = threading.Event()
        self.running.set()
        self.stopping = threading.Event()
        self.frame = None
        self.payload = b''
        self.message = 'Connecting to real board…'
        self.error = None
        self.updated = None
        self.frames = 0
        self.clients = set()
        self.token = secrets.token_urlsafe(24)
        self.session_id=str(time.time_ns())
        self.started = time.monotonic()
        self.last_cycle_s = None
        self.host_fps = 0.0
        self.frame_times=deque(maxlen=120)
        self.latest_event=None
        self.processing_failed=False
        self.imu=None;self.imu_updated=None;self.imu_received=0
        self.failures=0;self.recoveries=0;self.recovery_pending=False;self.sensor_reset_pending=False
        if hasattr(source,'on_imu'):source.on_imu=self.accept_imu
        self.outgoing=queue.Queue(maxsize=2)
        mp = multiprocessing.get_context('spawn')
        self.inbox = mp.Queue(maxsize=1)
        self.results = mp.Queue(maxsize=2)
        self.proc_stop = mp.Event()
        self.worker = mp.Process(
            target=run_compute,
            args=(self.inbox, self.results, calibration, temporal, str(self.directory), self.proc_stop),
            daemon=True)
        self.thread = threading.Thread(target=self.work,daemon=True)
        self.publisher = threading.Thread(target=self.publish,daemon=True)

    def emit(self,event):
        if isinstance(event, dict):
            event=json.dumps(event,separators=(',',':')).encode()
        try:
            self.outgoing.put_nowait(event)
        except queue.Full:
            try:self.outgoing.get_nowait()
            except queue.Empty:pass
            try:self.outgoing.put_nowait(event)
            except queue.Full:pass

    def offer(self,item):
        try:
            self.inbox.put_nowait(item)
        except queue.Full:
            try:self.inbox.get_nowait()
            except queue.Empty:pass
            try:self.inbox.put_nowait(item)
            except queue.Full:pass

    def fanout(self,data):
        with self.lock:clients=list(self.clients)
        for client in clients:
            try:client.put_nowait(data)
            except queue.Full:
                while not client.empty():
                    try:client.get_nowait()
                    except queue.Empty:break
                client.put_nowait(b'{"display_gap":true}')
                client.put_nowait(data)

    def handle_result(self,data):
        if isinstance(data,dict) and "processing_error" in data:
            with self.lock:
                self.processing_failed=True
                self.error=data["processing_error"]
                self.message="Host processing stopped · see error"
                self.running.clear()
            self.emit(self.snapshot())
            return
        self.latest_event=data
        self.fanout(data)

    def publish(self):
        while not self.stopping.is_set():
            try:
                data=self.results.get(timeout=0.2)
            except queue.Empty:
                data=None
            if data is not None:
                self.handle_result(data)
            while True:
                try:extra=self.outgoing.get_nowait()
                except queue.Empty:break
                self.fanout(extra)

    def accept_imu(self,imu):
        with self.lock:
            fresh=self.imu is None or imu['sequence']!=self.imu['sequence']
            self.imu=imu;self.imu_updated=time.time()
            if fresh:self.imu_received+=1
        if fresh:
            with (self.directory/'imu.jsonl').open('a') as log:
                log.write(json.dumps({'received_at':time.time(),**imu})+'\n')
        if fresh:self.emit({'kind':'imu','session_id':self.session_id,'imu':imu})

    def progress(self,message):
        with self.lock:
            self.message=message

    def work(self):
        first = True
        while not self.stopping.is_set():
            if not self.running.wait(0.2):
                if getattr(self.source, 'streaming', False):
                    try:self.source.stop_stream()
                    except Exception as exc:
                        self.error=str(exc);self.recovery_pending=True
                continue
            try:
                cycle_start=time.monotonic()
                if self.recovery_pending:
                    self.source.reconnect()
                    calibration=getattr(self.source, 'calibration', None)
                    if calibration != self.processor.calibration or calibration is not None:
                        replacement=Processor(calibration, temporal=self.processor.temporal)
                        self.processor.close()
                        self.processor=replacement
                        if calibration:
                            (self.directory/'sensor-calibration.bin').write_bytes(calibration)
                    self.session_id=str(time.time_ns())
                    # Calibration travels with frames so a bounded queue cannot drop it.
                    frame,payload=self.pull(progress=self.progress,recover=self.sensor_reset_pending)
                    self.recovery_pending=False;self.recoveries+=1
                else:
                    frame,payload = self.pull(request=not first,progress=self.progress)
                first=False
                now=time.monotonic()
                self.frame_times.append(now)
                while self.frame_times and now-self.frame_times[0]>1:
                    self.frame_times.popleft()
                with self.lock:
                    count=len(self.frame_times)
                    span=(self.frame_times[-1]-self.frame_times[0]) if count>1 else 0
                    self.host_fps=((count-1)/span) if span>=0.2 else 0.0
                    self.last_cycle_s=(1/self.host_fps) if self.host_fps else None
                    self.frames+=1
                    self.payload=payload
                    self.updated=time.time()
                    self.failures=0
                    self.message='Streaming'
                    if self.processing_failed:
                        self.running.clear()
                        continue
                    self.error=None
                    status=self.status()
                self.offer(('frame', frame, payload, status, self.processor.calibration))
            except Exception as exc:
                self.failures+=1
                retry=not isinstance(exc, ProcessingError) and hasattr(self.source,'reconnect') and self.failures<=3
                with self.lock:
                    self.error=str(exc)
                    self.message=('Host processing stopped · see error' if isinstance(exc, ProcessingError) else
                                  f'Recovering sensor · attempt {self.failures}/3' if retry else
                                  'Connection stopped · retry or check the board')
                with (self.directory/'errors.jsonl').open('a') as log:
                    log.write(json.dumps({'time':time.time(),'error':str(exc),'attempt':self.failures,'retry':retry})+'\n')
                self.emit(self.snapshot())
                if retry:
                    self.recovery_pending=True
                    self.sensor_reset_pending=isinstance(exc, SensorError)
                    self.stopping.wait(min(self.failures,3))
                else:self.running.clear()
                print(f'ERROR: {exc}',flush=True)
            self.stopping.wait(self.interval)

    def pull(self, **kwargs):
        if isinstance(self.source, USBSource):
            kwargs['decode'] = False
        return self.source.read(**kwargs)

    def status(self):
        return {'session_id':self.session_id,'message':self.message,'error':self.error,
                'running':self.running.is_set(),'frames_received':self.frames,'host_fps':self.host_fps,
                'control_token':self.token,'imu':self.imu,'imu_received':self.imu_received,
                'imu_age_seconds':None if self.imu_updated is None else time.time()-self.imu_updated,
                'recoveries':self.recoveries,'retry_attempt':self.failures}

    def snapshot(self):
        with self.lock:
            state=self.status()
        return {'session_id':state['session_id'],'frame':self.frame,'message':state['message'],'error':state['error'],
                'running':state['running'],'frames_received':state['frames_received'],
                'age_seconds':None if self.updated is None else time.time()-self.updated,
                'host_fps':state['host_fps'],'control_token':state['control_token'],'imu':state['imu'],
                'imu_received':state['imu_received'],'imu_age_seconds':state['imu_age_seconds'],
                'recoveries':state['recoveries'],'retry_attempt':state['retry_attempt']}


def serve(port=8765,directory=None,interval=0.0,usb=None,temporal=True):
    lock_path=(ROOT/'analysis/.stlink.lock') if (ROOT/'analysis').exists() else (Path(__file__).resolve().parent/'.stlink.lock')
    lock_path.parent.mkdir(exist_ok=True)
    with lock_path.open('w') as device_lock:
        try:
            fcntl.flock(device_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('ST-Link already in use; stop the other dashboard/checker')
        default_dir=Path(__file__).resolve().parent/'captures'
        source=USBSource(usb) if usb else RawSource()
        try:
            session=CaptureSession(source,directory or default_dir,interval,temporal)
        except Exception:
            if hasattr(source,'close'):source.close()
            raise
        html_file=Path(__file__).with_name('raw_dashboard.html')
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):
                pass
            def reply(self,content,kind='application/json',code=200):
                self.send_response(code)
                self.send_header('Content-Type',kind)
                self.send_header('Cache-Control','no-store')
                self.send_header('Content-Length',str(len(content)))
                self.end_headers()
                self.wfile.write(content)
            def do_GET(self):
                path=urlsplit(self.path).path
                if path=='/':
                    self.reply(html_file.read_bytes(),'text/html; charset=utf-8')
                elif path=='/events':
                    self.send_response(200)
                    self.send_header('Content-Type','text/event-stream')
                    self.send_header('Cache-Control','no-cache')
                    self.end_headers()
                    events=queue.Queue(maxsize=8)
                    with session.lock:session.clients.add(events)
                    try:
                        self.wfile.write(b'data: '+json.dumps(session.snapshot()).encode()+b'\n\n')
                        self.wfile.flush()
                        while not session.stopping.is_set():
                            try:message=b'data: '+events.get(timeout=5)+b'\n\n'
                            except queue.Empty:message=b': keepalive\n\n'
                            self.wfile.write(message);self.wfile.flush()
                    except (BrokenPipeError,ConnectionResetError):pass
                    finally:
                        with session.lock:session.clients.discard(events)
                elif path=='/api/frame':
                    self.reply(json.dumps(session.snapshot()).encode())
                elif path in ('/download/frame.bin','/download/frame.json'):
                    with session.lock:
                        content=session.payload if path.endswith('.bin') else json.dumps(session.frame).encode()
                    self.reply(content,'application/octet-stream' if path.endswith('.bin') else 'application/json')
                else:
                    self.reply(b'Not found','text/plain',404)
            def do_POST(self):
                if self.headers.get('X-Control-Token') != session.token:
                    self.reply(b'Forbidden','text/plain',403);return
                path=urlsplit(self.path).path
                if path=='/api/pause':
                    session.running.clear()
                elif path=='/api/resume':
                    session.failures=0
                    session.running.set()
                else:
                    self.reply(b'Not found','text/plain',404);return
                self.reply(b'{"ok":true}')
        server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
        session.publisher.start()
        session.worker.start()
        session.thread.start()
        print(f'Real full-frame dashboard: http://127.0.0.1:{port}\nCaptures: {session.directory}',flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            session.stopping.set()
            session.proc_stop.set()
            session.thread.join(timeout=5)
            session.worker.join(timeout=5)
            if session.worker.is_alive():
                session.worker.terminate()
            server.server_close()
            session.processor.close()
            if hasattr(session.source,"close"):session.source.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--swd',action='store_true',help='real full-frame ST-Link source')
    modes.add_argument('--usb',nargs='?',const='auto',help='native USB full frames, optional device path')
    parser.add_argument('--port','--web-port',type=int,default=8765)
    parser.add_argument('--capture-dir',type=Path)
    parser.add_argument('--interval',type=float,default=None)
    parser.add_argument('--temporal',action=argparse.BooleanOptionalAction,default=True,help='ST motion-aware temporal filtering (default on); --no-temporal for latency comparisons')
    args=parser.parse_args()
    interval=(0.0 if args.usb else 1.0) if args.interval is None else max(0.0,args.interval)
    serve(args.port,args.capture_dir,interval,args.usb,args.temporal)


if __name__=='__main__':
    main()
