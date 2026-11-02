"""Decode full VL53L9 frames and provide the ST-Link/SWD capture source.

USB and SWD readers share payload checks, metadata validation and FNV-1a.
Reject incomplete or corrupted frames before point-cloud processing.
"""
from pathlib import Path
import ctypes
import datetime
import glob
import importlib.util
import os
import re
import shutil
import struct
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
checker = None
checker_path = ROOT / 'firmware/tools/check_sensors.py'
if checker_path.exists():
    spec = importlib.util.spec_from_file_location('sensor_checker', checker_path)
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)
WIDTH, HEIGHT = 54, 42
PIXELS = WIDTH * HEIGHT
FRAME_SIZE = PIXELS * 6 + PIXELS // 2 + 100
KEYS = ('magic version state request sequence width height payload_size fnv1a '
        'capture_ms result stop_result fsm firmware_error error_flags frame_counter').split()


try:
    _checksum_lib=ctypes.CDLL(str(Path(__file__).with_name('build')/'libvl53l9_transform.so'))
    _native_fnv=_checksum_lib.l9_fnv1a
    _native_fnv.argtypes=[ctypes.c_void_p,ctypes.c_uint]
    _native_fnv.restype=ctypes.c_uint32
except (OSError,AttributeError):
    _native_fnv=None

def fnv1a(payload):
    if _native_fnv is not None:
        data=payload if isinstance(payload,bytes) else bytes(payload)
        return _native_fnv(data,len(data))
    value = 2166136261
    for byte in payload:
        value = ((value ^ byte) * 16777619) & 0xffffffff
    return value


def check_payload(payload, header):
    """Reject a bad frame before any plane is expanded into Python lists."""
    if len(payload) != FRAME_SIZE or header['payload_size'] != FRAME_SIZE:
        raise ValueError('Incomplete full frame')
    if (header['width'], header['height']) != (WIDTH, HEIGHT):
        raise ValueError('Unexpected frame dimensions')
    if fnv1a(payload) != header['fnv1a']:
        raise ValueError('Frame checksum mismatch; rejected')
    status_start = PIXELS * 6 + PIXELS // 2
    status = payload[status_start:]
    counter, = struct.unpack_from('<I', status)
    if counter != header['frame_counter']:
        raise ValueError('Status counter does not match mailbox')
    frame_error_code, = struct.unpack_from('<H', status, 60)
    frame_error_flags = status[62]
    return counter, frame_error_code, frame_error_flags, status


def decode_frame(payload, header):
    counter, frame_error_code, frame_error_flags, status = check_payload(payload, header)
    planes = struct.unpack_from(f'<{PIXELS * 3}H', payload)
    dss_start = PIXELS * 6
    status_start = dss_start + PIXELS // 2
    error_names = ('VHV overvoltage', 'VHV undervoltage', 'SPAD supply overload',
                   'current limit', 'SOF outside blanking', 'PLL lock',
                   'reference array', 'internal firmware')
    warnings = [name for bit, name in enumerate(error_names) if frame_error_flags & (1 << bit)]
    if frame_error_code:
        warnings.append(f'firmware code {frame_error_code}')
    return {
        'source': 'REAL BOARD · GPIO I2C → STM32 → ST-Link/SWD',
        'received_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'width': WIDTH, 'height': HEIGHT, 'bytes': len(payload),
        'sequence': header['sequence'], 'sensor_counter': counter,
        'capture_ms': header['capture_ms'], 'checksum': f'{header["fnv1a"]:08x}',
        'checksum_ok': True,
        'depth': list(planes[:PIXELS]),
        # ST extract.c: distance bits 0..14, main/short flag bit 15.
        'depth_values': [v & 0x7fff for v in planes[:PIXELS]],
        'depth_main_flags': [bool(v & 0x8000) for v in planes[:PIXELS]],
        'amplitude': list(planes[PIXELS:2*PIXELS]),
        'ambient': list(planes[2*PIXELS:]),
        'dss': list(payload[dss_start:status_start]),
        'status_bytes': list(status), 'status_hex': status.hex(' '),
        'frame_error_code': frame_error_code, 'frame_error_flags': frame_error_flags,
        'sensor_warnings': warnings,
        'mailbox': header,
        'units': 'Unmodified raw uint16 words; no calibrated distance or validity interpretation',
        'imu': {'whoami': '0x70', 'measurements': None},
    }


class RawSource:
    def __init__(self, elf=None):
        if checker is None:
            raise RuntimeError(f'Sensor checker not found at {checker_path}; required for ST-Link/SWD')
        if elf is None:
            elf = checker.ELF
        self.elf = Path(elf)
        self.programmer = os.environ.get('STM32_PROGRAMMER_CLI') or shutil.which('STM32_Programmer_CLI')
        if not self.programmer:
            choices = sorted(glob.glob('/opt/st/stm32cubeide_*/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_*/tools/bin/STM32_Programmer_CLI'))
            if not choices:
                raise RuntimeError('STM32CubeProgrammer not found')
            self.programmer = choices[-1]
        symbols = checker.run(['arm-none-eabi-nm', '-S', self.elf])
        self.symbols = {}
        for name, expected in [('g_capture',64), ('g_lidar_frame',FRAME_SIZE), ('g_diag',96)]:
            match = re.search(r'^([\da-fA-F]+)\s+([\da-fA-F]+)\s+\w\s+'+name+r'$', symbols, re.M)
            if not match or int(match[2],16) != expected:
                raise RuntimeError(f'{name} ABI mismatch; build/flash full-frame firmware first')
            self.symbols[name] = int(match[1],16)
        self.connection = [self.programmer, '-c', 'port=SWD', 'mode=HOTPLUG']
        self.last_sequence = None

    def header(self):
        addr = self.symbols['g_capture']
        out = checker.run(self.connection + ['-r32', hex(addr), '0x40'])
        values = checker.parse_words(out, addr, 16)
        header = dict(zip(KEYS, values))
        for field in ('result', 'stop_result'):
            if header[field] >= 0x80000000:
                header[field] -= 0x100000000
        if header['magic'] != 0x4c394652 or header['version'] not in (1,2):
            raise RuntimeError('Full-frame firmware mailbox missing; ELF must match board')
        if header['state'] == 3:
            raise RuntimeError(f'Sensor capture failed: {header}')
        return header

    def read(self, request=True, progress=None):
        before = self.header()
        if request:
            if before['state'] != 2 or before['request']:
                raise RuntimeError('Sensor mailbox busy; another reader may be active')
            # Only this aligned request word is written; never touch frame memory.
            checker.run(self.connection + ['-w32', hex(self.symbols['g_capture']+12), '0x1'])
        deadline = time.monotonic() + 120
        while True:
            header = self.header()
            ready = header['state'] == 2 and header['request'] == 0
            fresh = not request or header['sequence'] != before['sequence']
            if ready and fresh:
                break
            if progress:
                progress('Capturing full frame over GPIO I2C…')
            if time.monotonic() >= deadline:
                raise RuntimeError('Full-frame capture timed out; no frame fabricated')
            time.sleep(0.5)
        if header['result'] or header['stop_result'] or header['fsm'] not in ((2,3) if header['version']==2 else (2,)) or header['firmware_error'] or header['error_flags']:
            raise RuntimeError(f'Invalid sensor completion status: {header}')
        if self.last_sequence == header['sequence']:
            raise RuntimeError('Duplicate frame sequence; rejected')
        if progress:
            progress('Reading and checking 14,842 bytes through ST-Link…')
        with tempfile.TemporaryDirectory(prefix='vl53l9-frame-') as directory:
            path = Path(directory)/'frame.bin'
            checker.run(self.connection + ['-u', hex(self.symbols['g_lidar_frame']), str(FRAME_SIZE), path])
            payload = path.read_bytes()
        after = self.header()
        if after != header:
            raise RuntimeError('Mailbox changed during transfer; frame rejected')
        frame = decode_frame(payload,header)
        # Read real IMU identity rather than substituting a constant.
        addr = self.symbols['g_diag']
        out = checker.run(self.connection + ['-r32', hex(addr), '0x60'])
        diag = dict(zip(checker.FIELDS, checker.parse_words(out, addr, len(checker.FIELDS))))
        if diag['magic'] != checker.MAGIC or diag['soft_id'] != 0x53334c39:
            raise RuntimeError('Board diagnostics do not match full-frame firmware')
        frame['imu']['whoami'] = f'0x{diag["imu_whoami"]:02X}'
        frame['imu']['identity_ok'] = bool(diag['imu_ok'])
        frame['lidar_id'] = f'0x{diag["soft_id"]:08X}'
        self.last_sequence = header['sequence']
        return frame, payload
