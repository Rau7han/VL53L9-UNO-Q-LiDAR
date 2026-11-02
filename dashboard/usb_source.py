"""Framed USB transport: full LiDAR frames plus independently sampled IMU packets."""
import datetime
import math
import struct
import time
from raw_source import check_payload, decode_frame,FRAME_SIZE,KEYS,fnv1a

class SensorError(RuntimeError):
    pass

def decode_imu(packet):
    if len(packet)!=36 or packet[:4]!=b'I6R1':raise ValueError('Invalid IMU packet')
    if fnv1a(packet[:32])!=struct.unpack_from('<I',packet,32)[0]:raise ValueError('IMU checksum mismatch')
    sequence,timestamp,status=struct.unpack_from('<III',packet,4)
    raw=list(struct.unpack_from('<7h',packet,16))
    valid=status==0 and sequence>0
    accel=[round(x*0.000061,6) for x in raw[4:7]]
    gyro=[round(x*0.00875,5) for x in raw[1:4]]
    norm=math.sqrt(sum(x*x for x in accel))
    tilt=None
    if valid and .8<norm<1.2:
        tilt={'roll_deg':round(math.degrees(math.atan2(accel[1],accel[2])),2),
              'pitch_deg':round(math.degrees(math.atan2(-accel[0],math.hypot(accel[1],accel[2]))),2)}
    return {'sequence':sequence,'timestamp_ms':timestamp,'status':status,'valid':valid,
            'accel_g':accel if valid else None,'accel_ms2':[round(x*9.80665,5) for x in accel] if valid else None,
            'gyro_dps':gyro if valid else None,'temperature_c':round(25+raw[0]/256,2) if valid else None,
            'accel_norm_g':round(norm,4) if valid else None,'tilt':tilt,
            'raw':{'temperature':raw[0],'gyro':raw[1:4],'accel':raw[4:7]},
            'odr_hz':60,'accel_range_g':2,'gyro_range_dps':250,'checksum_ok':True}

class USBSource:
    def __init__(self,port='auto'):
        import serial
        from serial.tools import list_ports
        self.selector=port
        if port=='auto':
            found=[p.device for p in list_ports.comports() if (p.vid,p.pid)==(0xcafe,0x4010)]
            if len(found)!=1:raise RuntimeError('USB board not found. Check the data cable.')
            port=found[0]
        self.port=port
        if '://' in port:
            # e.g. socket://127.0.0.1:7000 — the board's serial port relayed over an SSH tunnel.
            self.serial=serial.serial_for_url(port,timeout=0.1,write_timeout=2)
        else:
            self.serial=serial.Serial(port,115200,timeout=0.1,write_timeout=2,exclusive=True)
        self.serial.reset_input_buffer()
        deadline=time.monotonic()+4;quiet_since=time.monotonic()
        while time.monotonic()-quiet_since<0.4:
            if self.serial.read(4096):quiet_since=time.monotonic()
            if time.monotonic()>deadline:
                self.serial.close();raise RuntimeError('USB did not become idle during reconnect')
        self.last_sequence=None;self.imu=None;self.on_imu=None
        self.streaming = False
        self.diagnostics = None
        self.calibration = None
        try:
            self.negotiate()
        except Exception:
            self.serial.close()
            raise

    def negotiate(self):
        # Old firmware ignores D/C. Only a completely silent response is legacy.
        info = self.read_info(b'D', b'L9D1', 64, optional=True)
        if info is not None:
            names = ('version mode stage status kernel_hz bus_hz address timing0 timing1 '
                     'error pid_lo pid_hi reads writes bytes failures').split()
            self.diagnostics = dict(zip(names, struct.unpack('<16I', info)))
            self.calibration = self.read_info(b'C', b'L9C1', 2332)

    def read_info(self, command, magic, expected_size, optional=False):
        self.serial.write(command)
        started = time.monotonic()
        first = bytearray()
        while not first and time.monotonic()-started < .5:
            first.extend(self.serial.read(4))
        if not first and optional:
            return None
        deadline = time.monotonic()+3
        first.extend(self.exact(4-len(first), deadline))
        if bytes(first) != magic:
            raise RuntimeError('Invalid USB information packet magic')
        version, size, checksum = struct.unpack('<III', self.exact(12, deadline))
        if version != 1 or size != expected_size:
            raise RuntimeError(f'USB information ABI mismatch: version={version}, size={size}')
        data = self.exact(size, deadline)
        if fnv1a(data) != checksum:
            raise RuntimeError('USB information checksum mismatch')
        return data

    def reconnect(self):
        callback=self.on_imu
        cancel=getattr(self,"cancel_event",None)
        # A broken socket may reject the stop command; still replace the transport.
        try:self.close()
        except Exception:pass
        self.__init__(self.selector);self.on_imu=callback
        self.cancel_event=cancel

    def exact(self,size,deadline):
        result=bytearray()
        while len(result)<size:
            if getattr(self,'cancel_event',None) is not None and self.cancel_event.is_set():
                raise RuntimeError('Acquisition stopped')
            result.extend(self.serial.read(size-len(result)))
            if time.monotonic()>deadline:raise RuntimeError(f'USB frame timeout ({len(result)}/{size} bytes)')
        return bytes(result)

    def read(self,request=True,progress=None,recover=False,decode=True):
        started=time.monotonic()
        if progress:progress('Recovering sensor…' if recover else 'Streaming')
        continuous = (getattr(self, 'diagnostics', None) or {}).get('version', 1) >= 2
        if recover:
            self.streaming=False
            self.serial.write(b'R')
        elif continuous:
            if not getattr(self, 'streaming', False):
                self.serial.write(b'Q');self.streaming=True
        else:
            self.serial.write(b'F')
        deadline=started+(30 if recover else 10)
        while True:
            magic=self.exact(4,deadline)
            if magic==b'I6R1':
                self.imu=decode_imu(magic+self.exact(32,deadline))
                if getattr(self,'on_imu',None):self.on_imu(self.imu)
                continue
            if magic not in (b'L9R1',b'L9R2',b'L9R3'):raise RuntimeError('USB framing mismatch')
            break
        header_raw=magic+self.exact(72,deadline)
        header=dict(zip(KEYS,struct.unpack_from('<16I',header_raw,4)))
        for field in ('result','stop_result'):
            if header[field]>=0x80000000:header[field]-=0x100000000
        if header['magic']!=0x4c394652 or header['version']!=(2 if magic==b'L9R3' else 1) or header['payload_size']!=FRAME_SIZE:
            raise RuntimeError('USB firmware ABI mismatch')
        if header['state']!=2 or header['result'] or header['stop_result'] or header['fsm'] not in ((2,3) if magic==b'L9R3' else (2,)) or header['firmware_error'] or header['error_flags']:
            raise SensorError(f'USB sensor acquisition failed: {header}')
        payload=self.exact(FRAME_SIZE,deadline)
        if self.last_sequence is not None and header['sequence']!=(self.last_sequence+1)&0xffffffff:
            raise RuntimeError(f'Nonconsecutive USB sequence {self.last_sequence} → {header["sequence"]}')
        imu,lidar=struct.unpack_from('<II',header_raw,68)
        if lidar!=0x53334c39:raise RuntimeError('Unexpected USB LiDAR identity')
        if decode:
            frame=decode_frame(payload,header)
        else:
            counter, frame_error_code, frame_error_flags, status = check_payload(payload, header)
            error_names = ('VHV overvoltage', 'VHV undervoltage', 'SPAD supply overload',
                           'current limit', 'SOF outside blanking', 'PLL lock',
                           'reference array', 'internal firmware')
            warnings = [name for bit, name in enumerate(error_names) if frame_error_flags & (1 << bit)]
            if frame_error_code:
                warnings.append(f'firmware code {frame_error_code}')
            frame={
                'received_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                'width': header['width'], 'height': header['height'], 'bytes': len(payload),
                'sequence': header['sequence'], 'sensor_counter': counter,
                'capture_ms': header['capture_ms'], 'checksum': f'{header["fnv1a"]:08x}',
                'checksum_ok': True, 'frame_error_code': frame_error_code,
                'frame_error_flags': frame_error_flags, 'sensor_warnings': warnings,
                'mailbox': header, 'status_hex': status.hex(' '),
            }
        diag = getattr(self, 'diagnostics', None)
        if recover and diag is not None:
            data = self.read_info(b'D', b'L9D1', 64)
            diag = dict(zip(diag, struct.unpack('<16I', data)))
            self.diagnostics = diag
        bus = {0:'GPIO I2C', 1:'Peripheral I2C', 2:'I3C SDR'}.get(diag['mode'], 'Unknown bus') if diag else 'GPIO I2C'
        frame.update(source=f'REAL BOARD · {bus} → native USB CDC',transport='USB CDC',usb_device=self.port,
                     bus_diagnostics=diag,
                     delivery_ms=round((time.monotonic()-started)*1000,2),lidar_id=f'0x{lidar:08X}',
                     imu={'whoami':f'0x{imu:02X}','identity_ok':imu==0x70,'measurements':getattr(self,'imu',None)})
        self.last_sequence=header['sequence']
        return frame,payload

    def stop_stream(self):
        if getattr(self, 'streaming', False):
            self.serial.write(b'P');self.streaming=False
            # Drain at most two complete buffered frames, with a bounded timeout.
            deadline=time.monotonic()+2
            while self.serial.read(16384):
                if time.monotonic()>deadline:raise RuntimeError('USB stream did not stop')
            self.last_sequence=None

    def close(self):
        try:
            self.stop_stream()
        finally:
            self.serial.close()
