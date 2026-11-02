#!/usr/bin/env python3
"""Board smoke test. Exit 0: identities and full-frame capture pass; 2: sensor failure; 1: tool failure."""
import argparse
import glob
import fcntl
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / 'cube/examples/hal/cortex/bringup/USB-LiDAR/cmake/build/release_GCC_NUCLEO-C542RC'
ELF = BUILD / 'hal_cortex_systick_delay.elf'
FIELDS = ('magic spi_init_ok imu_whoami imu_ok i3c_init_ok i3c_kclk scan_count '
          'scan_addresses idxwrite_status model_read lidar_model_id lidar_detected '
          'i3c_err treg0 treg1 loops bus_idle soft_ack soft_status soft_id hse_status '
          'ap_clock_hz imu_status sync_level').split()
MAGIC = 0x5E450005
HAL_STATUS = {0xEAEAEAEA: "OK", 0xF5F5F5F5: "ERROR", 0x55555555: "BUSY", 0x5A5A5A5A: "TIMEOUT"}
def status(v): return HAL_STATUS.get(v, f"0x{v:08X}")

def run(args):
    r = subprocess.run([str(a) for a in args], text=True, stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, timeout=90)
    out = re.sub(r'\x1b\[[0-9;]*m', '', r.stdout)
    if r.returncode or re.search(r'\bError:', out, re.I):
        raise RuntimeError(out)
    return out

def parse_words(text, address, count):
    memory = {}
    for line in text.splitlines():
        m = re.match(r'\s*0x([0-9a-fA-F]{8})\s*:\s*(.*)', line)
        if not m:
            continue
        base = int(m[1], 16)
        for i, value in enumerate(m[2].split()):
            if not re.fullmatch(r'[0-9a-fA-F]{8}', value):
                break
            memory[base + 4*i] = int(value, 16)
    try:
        return [memory[address + 4*i] for i in range(count)]
    except KeyError as e:
        raise RuntimeError('Incomplete SWD memory read; no result inferred') from e

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--no-flash', action='store_true', help='read current diagnostics only; ELF must match firmware')
    ap.add_argument('--log', type=Path, help='save complete programmer output')
    args = ap.parse_args()
    prog = os.environ.get('STM32_PROGRAMMER_CLI') or shutil.which('STM32_Programmer_CLI')
    if not prog:
        candidates = sorted(glob.glob('/opt/st/stm32cubeide_*/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_*/tools/bin/STM32_Programmer_CLI'))
        if not candidates:
            raise RuntimeError('Set STM32_PROGRAMMER_CLI to the installed programmer executable')
        prog = candidates[-1]
    if not args.no_flash:
        print(run(['cmake', '--build', BUILD, '-j4']), end='')
        run(['arm-none-eabi-objcopy', '-O', 'ihex', ELF, ROOT/'build/usb-lidar-sensors.hex'])
    symbols = run(['arm-none-eabi-nm', '-S', ELF])
    match = re.search(r'^([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+\w\s+g_diag$', symbols, re.M)
    if not match:
        raise RuntimeError('g_diag not found in ELF')
    address, size = (int(x, 16) for x in match.groups())
    if size != len(FIELDS)*4:
        raise RuntimeError(f'Diagnostic ABI size mismatch: {size}, expected {len(FIELDS)*4}')
    logs = []
    try:
        if not args.no_flash:
            print('Flashing and verifying freshly built ELF via SWD...')
            logs.append(run([prog, '-c', 'port=SWD', 'mode=UR', '-w', ELF, '-v', '-rst']))
        deadline = time.monotonic() + 120
        while True:
            time.sleep(0.5)
            raw = run([prog, '-c', 'port=SWD', 'mode=HOTPLUG', '-r32', hex(address), hex(size)])
            logs.append(raw)
            d = dict(zip(FIELDS, parse_words(raw, address, len(FIELDS))))
            if d['magic'] == MAGIC:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(f'Diagnostics incomplete or wrong firmware: magic 0x{d["magic"]:08X}')
        print(f'Firmware completed; heartbeat={d["loops"]}')
        print(f'IMU WHO_AM_I: 0x{d["imu_whoami"]:02X}; SPI status={status(d["imu_status"])}; identity {"PASS (0x70 family)" if d["imu_ok"] else "FAIL"}')
        print(f'AP clock configured: {d["ap_clock_hz"]} Hz; HSE status={status(d["hse_status"])} (frequency needs scope verification)')
        print(f'GPIO bus idle at MCU: SDA={d["bus_idle"]&1}, SCL={(d["bus_idle"]>>1)&1}')
        print(f'GPIO I2C: ACK={d["soft_ack"]}, status={d["soft_status"]}, ID=0x{d["soft_id"]:08X}')
        packed = d['scan_addresses'].to_bytes(4, 'little')
        print(f'I3C peripheral in 100 kHz legacy I2C: init={d["i3c_init_ok"]}; kernel={d["i3c_kclk"]} Hz')
        print(f'Target probe: {d["scan_count"]} device(s); addresses: '+', '.join(f'0x{x:02X}' for x in packed[:min(4,d['scan_count'])]))
        print(f'LiDAR ID=0x{d["lidar_model_id"]:08X}; index status={status(d["idxwrite_status"])}, read status={status(d["model_read"])}, error=0x{d["i3c_err"]:08X}')
        m = re.search(r'^([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+\w\s+g_lidar$', symbols, re.M)
        if not m or int(m[2],16) != 48:
            raise RuntimeError('Missing or incompatible g_lidar')
        addr = int(m[1],16)
        raw = run([prog, '-c', 'port=SWD', 'mode=HOTPLUG', '-r32', hex(addr), '0x30'])
        logs.append(raw)
        r = parse_words(raw,addr,12)
        signed = lambda x: x if x < 0x80000000 else x-0x100000000
        print(f'ST driver: stage={r[0]} result={signed(r[1])} patch=0x{r[3]:04X}, frames={r[4]}, counter={r[5]}')
        print(f'Raw depth words min/max={r[6]}/{r[7]} (uncalibrated); FSM={r[8]}, firmware error={r[9]}, flags={r[10]}, stop={signed(r[11])}')
        passed = bool(d['imu_ok'] and d['soft_id']==0x53334C39 and r[0]==7 and r[1]==0 and r[4]>=1 and r[8]==2 and r[9]==0 and r[10]==0 and r[11]==0)
        print('PASS: identities, ST boot and full 54x42 raw capture via GPIO I2C. This SWD check does not measure USB throughput or range accuracy.' if passed else
              'FAIL: sensor communication incomplete. A NACK alone does not identify the failed hardware component.')
        return 0 if passed else 2
    finally:
        if args.log:
            args.log.parent.mkdir(parents=True, exist_ok=True)
            args.log.write_text('\n'.join(logs))

if __name__ == '__main__':
    try:
        lock_path = ROOT.parent / "analysis/.stlink.lock"
        lock_path.parent.mkdir(exist_ok=True)
        with lock_path.open("w") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError("ST-Link is in use by the dashboard; stop it before flashing")
            sys.exit(main())
    except (RuntimeError, OSError, subprocess.TimeoutExpired) as e:
        print(f'ERROR: {e}', file=sys.stderr)
        sys.exit(1)
