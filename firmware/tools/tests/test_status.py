"""Run the actual ST status reader against deterministic register reads."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[3]
class StatusTests(unittest.TestCase):
    def test_burst_decode_and_transport_errors(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp=Path(tmp)
            (tmp/'test.c').write_text(r'''#include "vl53l9.h"
#include "vl53l9_reg.h"
#include <assert.h>
#include <string.h>
static int calls, fail;
int vl53l9_read(void *const dev, uint16_t addr, uint8_t *out, uint32_t size) {
    ++calls;
    if (calls==fail) return -77;
    if (calls==1) {
        assert(addr==VL53L9_REGADDR_SYSTEM_FSM && size==2);
        out[0]=3; out[1]=0x25;
    } else {
        assert(calls==2 && addr==VL53L9_REGADDR_ERROR_CODE && size==8);
        const uint8_t data[]={0x34,0x12,0xa5,11,22,33,44,55};
        memcpy(out,data,8);
    }
    return 0;
}
int main(void) {
    vl53l9_status_t s={0}; int dev=1;
    assert(vl53l9_get_status(&dev,&s)==0 && calls==2);
    assert(s.fsm==3 && s.command==0x25 && s.firmware==0x1234);
    assert(s.error.vhv_overvoltage && !s.error.vhv_undervoltage);
    assert(s.error.spad_supply_overload && s.error.pll_lock && s.error.internal_fw);
    for(int i=0;i<5;++i) assert(s.laser_driver[i]==11*(i+1));
    for(fail=1;fail<=2;++fail) {
        calls=0; assert(vl53l9_get_status(&dev,&s)==-77 && calls==fail);
    }
    assert(vl53l9_get_status(0,&s)!=0);
    assert(vl53l9_get_status(&dev,0)!=0);
    return 0;
}
''')
            vendor=ROOT/'firmware/vendor/vl53l9'
            subprocess.run(['cc','-ffunction-sections','-fdata-sections',f'-I{vendor}',
                str(vendor/'vl53l9.c'),str(tmp/'test.c'),'-Wl,--gc-sections','-lm','-o',str(tmp/'test')],check=True)
            subprocess.run([str(tmp/'test')],check=True)
