"""Execute ST's actual timing calculator on the host with real register constants.

No HAL/peripheral emulation: only the MCU's arithmetic-independent timing
definitions are extracted, avoiding ARM intrinsics in a native compiler.
"""
import ctypes as ct
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]

class Config(ct.Structure):
    _fields_ = [(key, ct.c_uint32) for key in
                ('kernel', 'pp', 'od', 'duty', 'activity', 'bus')]

class TimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.directory.cleanup)
        path = Path(cls.directory.name)
        defines = (ROOT/'firmware/cube/stm32c5xx_dfp/Include/stm32c552xx.h').read_text()
        (path/'stm32c5xx.h').write_text('#include <stdint.h>\n' + '\n'.join(
            line for line in defines.splitlines() if line.startswith('#define I3C_TIMINGR')))
        utility = ROOT/'firmware/cube/stm32c5xx_drivers/utils/i3c_timing'
        subprocess.run(['cc', '-shared', '-fPIC', f'-I{path}', f'-I{utility}',
                        str(utility/'stm32_utils_i3c.c'), '-o', str(path/'timing.so')], check=True)
        cls.lib = ct.CDLL(str(path/'timing.so'))

    def compute(self, pp=1000000, od=400000, pure=True):
        config = Config(72000000, pp, od, 50, 0, 0 if pure else 1)
        a, b = ct.c_uint32(), ct.c_uint32()
        status = self.lib.STM32_UTILS_I3C_CTRL_ComputeTiming(ct.byref(config), ct.byref(a), ct.byref(b))
        return status, a.value, b.value

    def test_original_100khz_overflows_registers(self):
        self.assertNotEqual(self.compute(od=100000, pure=False)[0], 0)

    def test_legacy_configuration_fits(self):
        status, reg, _ = self.compute(pure=False)
        self.assertEqual(status, 0)
        period = (((reg >> 16) & 255)+1 + ((reg >> 24) & 255)+1)/72000000
        self.assertLessEqual(1/period, 400000)

    def test_daa_and_sdr_constraints(self):
        for speed in (1000000, 6000000, 12000000):
            with self.subTest(speed=speed):
                status, reg, _ = self.compute(pp=speed)
                self.assertEqual(status, 0)
                pp_cycles = (reg & 255)+1 + ((reg >> 8)&255)+1
                od_cycles = ((reg >> 16)&255)+1
                self.assertLessEqual(72000000/pp_cycles, speed)
                self.assertGreaterEqual(od_cycles/72000000, 200e-9)
                self.assertLessEqual(od_cycles/72000000, 500e-9)

if __name__ == '__main__':
    unittest.main()
