#!/usr/bin/env python3
"""Configure and build the actual STM32C552 board; export ELF, HEX and BIN."""
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / 'cube/examples/hal/cortex/bringup/USB-LiDAR/cmake'
PRESET = 'release_GCC_NUCLEO-C542RC'  # Generated preset name; target is C552KEU6.

if __name__ == '__main__':
    for name in ('cmake', 'ninja', 'arm-none-eabi-gcc', 'arm-none-eabi-objcopy'):
        if not shutil.which(name):
            raise SystemExit(f'Missing {name}; install it and add it to PATH.')
    subprocess.run(['cmake', '--preset', PRESET], cwd=PROJECT, check=True)
    subprocess.run(['cmake', '--build', '--preset', PRESET, '--parallel', '4'], cwd=PROJECT, check=True)
    elf = PROJECT / 'build' / PRESET / 'hal_cortex_systick_delay.elf'
    output = ROOT / 'build'
    output.mkdir(exist_ok=True)
    shutil.copy2(elf, output / 'usb-lidar-sensors.elf')
    for fmt, extension in (('ihex', 'hex'), ('binary', 'bin')):
        subprocess.run(['arm-none-eabi-objcopy', '-O', fmt, str(elf), str(output / f'usb-lidar-sensors.{extension}')], check=True)
    print(f'Build complete: {output}')
