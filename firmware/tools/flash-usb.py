#!/usr/bin/env python3
"""Flash and verify the matching board over USB DFU, then start the application."""
import argparse
import glob
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', nargs='?', type=Path, default=Path(__file__).resolve().parents[1] / 'build/usb-lidar-sensors.hex')
    parser.add_argument('--port', default='USB1', help='DFU port reported by STM32_Programmer_CLI -l usb')
    parser.add_argument('--dry-run', action='store_true', help='print commands without accessing a board')
    args = parser.parse_args()
    if not re.fullmatch(r'USB[0-9]+', args.port, re.I):
        parser.error('--port must be a DFU USB port such as USB1')
    image = args.image.resolve()
    if not image.is_file() or image.suffix.lower() not in ('.hex', '.bin', '.elf'):
        parser.error('image must be an existing .hex, .bin or .elf file')
    prog = os.environ.get('STM32_PROGRAMMER_CLI') or shutil.which('STM32_Programmer_CLI')
    if not prog:
        found = sorted(glob.glob('/opt/st/stm32cubeide_*/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_*/tools/bin/STM32_Programmer_CLI'))
        prog = found[-1] if found else None
    if not prog and not args.dry_run:
        parser.error('Install STM32CubeProgrammer and set STM32_PROGRAMMER_CLI to its executable, or add it to PATH')
    prog = prog or 'STM32_Programmer_CLI'
    write = [prog, '-c', f'port={args.port.upper()}', '-w', str(image)]
    if image.suffix.lower() == '.bin':
        write += ['0x08000000']
    write += ['-v']
    for command in (write, [prog, '-c', f'port={args.port.upper()}', '-s', '0x08000000']):
        print(shlex.join(command), flush=True)
        if not args.dry_run:
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            output = re.sub(r'\x1b\[[0-9;]*m', '', result.stdout)
            print(output, end='')
            if result.returncode or re.search(r'\bError:', output, re.I):
                raise SystemExit('Programmer failed; application start was not attempted after a failed write.')
            if command is write and 'Download verified successfully' not in output:
                raise SystemExit('Verification success was not confirmed; application start was not attempted.')


if __name__ == '__main__':
    main()
