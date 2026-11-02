"""USB-LiDAR-VL53L9 streaming protocol (host side).

A single compact binary frame carries one IMU sample + one LiDAR depth grid.
The firmware emits these back-to-back over the USB CDC virtual COM port; the
dashboard (and the simulator) share this one definition.

Frame layout (little-endian), no padding:

    offset  size  field
    0       4     magic  = b"LDR1"
    4       4     seq         uint32   frame counter
    8       2     ax          int16    accel X  (raw LSB, +-2g => /16384 = g)
    10      2     ay          int16
    12      2     az          int16
    14      2     gx          int16    gyro  X  (raw LSB, 70 mdps/LSB)
    16      2     gy          int16
    18      2     gz          int16
    20      2     temp        int16    IMU temperature (raw)
    22      1     width       uint8    LiDAR columns
    23      1     height      uint8    LiDAR rows
    24      w*h*2 depth[]     uint16   per zone: bit15 = valid, bits0..14 = mm
    ...     1     checksum    uint8    XOR of all bytes from seq..last depth

Everything after `magic` up to and including `checksum` is the body. The
reader re-syncs on `magic`, so a dropped byte only costs one frame.
"""

import struct

MAGIC = b"LDR1"

# scaling for the LSM6DSV16BX at its default full-scale ranges
ACCEL_G_PER_LSB = 1.0 / 16384.0        # +-2 g
GYRO_DPS_PER_LSB = 70.0 / 1000.0       # 70 mdps/LSB (+-2000 dps)
TEMP_C_PER_LSB = 1.0 / 256.0
TEMP_OFFSET_C = 25.0

_HEADER = struct.Struct("<4sI7hBB")    # magic..height (24 bytes)
HEADER_SIZE = _HEADER.size


class Frame:
    __slots__ = ("seq", "accel", "gyro", "temp_c", "width", "height", "depth_mm", "valid")

    def __init__(self, seq, accel, gyro, temp_c, width, height, depth_mm, valid):
        self.seq = seq
        self.accel = accel          # (ax, ay, az) in g
        self.gyro = gyro            # (gx, gy, gz) in dps
        self.temp_c = temp_c
        self.width = width
        self.height = height
        self.depth_mm = depth_mm    # numpy (h, w) uint16 millimetres
        self.valid = valid          # numpy (h, w) bool


def pack_frame(seq, accel_raw, gyro_raw, temp_raw, width, height, depth_words):
    """Build a wire frame. depth_words: flat iterable of uint16 (valid<<15 | mm)."""
    body = bytearray()
    body += _HEADER.pack(MAGIC, seq & 0xFFFFFFFF,
                         accel_raw[0], accel_raw[1], accel_raw[2],
                         gyro_raw[0], gyro_raw[1], gyro_raw[2],
                         temp_raw, width, height)
    body += struct.pack("<%dH" % (width * height), *depth_words)
    chk = 0
    for b in body[4:]:
        chk ^= b
    body.append(chk)
    return bytes(body)


def parse_body(buf):
    """Parse one frame whose first bytes are MAGIC. Returns (Frame, total_len)
    or (None, need_more_or_resync)."""
    import numpy as np
    if len(buf) < HEADER_SIZE:
        return None, 0
    magic, seq, ax, ay, az, gx, gy, gz, temp, w, h = _HEADER.unpack(buf[:HEADER_SIZE])
    if magic != MAGIC:
        return None, -1
    n = w * h
    total = HEADER_SIZE + n * 2 + 1
    if len(buf) < total:
        return None, 0
    depth = np.frombuffer(buf, dtype="<u2", count=n, offset=HEADER_SIZE)
    chk = 0
    for b in buf[4:total - 1]:
        chk ^= b
    if chk != buf[total - 1]:
        return None, -1
    depth = depth.reshape(h, w)
    valid = (depth & 0x8000) != 0
    depth_mm = (depth & 0x7FFF).astype("u2")
    f = Frame(
        seq=seq,
        accel=(ax * ACCEL_G_PER_LSB, ay * ACCEL_G_PER_LSB, az * ACCEL_G_PER_LSB),
        gyro=(gx * GYRO_DPS_PER_LSB, gy * GYRO_DPS_PER_LSB, gz * GYRO_DPS_PER_LSB),
        temp_c=TEMP_OFFSET_C + temp * TEMP_C_PER_LSB,
        width=w, height=h, depth_mm=depth_mm, valid=valid,
    )
    return f, total
