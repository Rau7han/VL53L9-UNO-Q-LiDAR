import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

TOOLS=Path(__file__).resolve().parents[1]
ROOT=TOOLS.parents[1]
spec=importlib.util.spec_from_file_location('check_sensors',TOOLS/'check_sensors.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class DiagnosticTests(unittest.TestCase):
    def test_addressed_read_not_unrelated_hex(self):
        raw='0x20000000 : 00000001 EAEAEAEA\n0x20000008 : 53334C39 00000003\n'
        self.assertEqual(module.parse_words(raw,0x20000004,2),[0xEAEAEAEA,0x53334C39])
    def test_short_read_rejected(self):
        with self.assertRaises(RuntimeError):module.parse_words('0x20000000 : 00000001',0x20000000,2)
    def test_hal_status_encoding(self):
        self.assertEqual(module.status(0xEAEAEAEA),'OK')
        self.assertEqual(module.status(0x5A5A5A5A),'TIMEOUT')
        self.assertNotEqual(module.status(0),'OK')
    def test_hal_fifo_bounds(self):
        # Compile the actual HAL helpers against a level-asserted FIFO mock.
        # Regression: CFNF staying high previously consumed beyond the last descriptor.
        path=ROOT/'firmware/cube/stm32c5xx_drivers/hal/stm32c5xx_hal_i3c.c'
        source=path.read_text()
        functions=[]
        for name in ('I3C_ControlDataTreatment','I3C_ReceiveByteTreatment'):
            marker='static void '+name+'(hal_i3c_handle_t *hi3c)\n{'
            start=source.index(marker);end=source.index('\n}',start)+2
            functions.append(source[start:end])
        pre='''#include <stdint.h>
#include <assert.h>
typedef struct {int unused;} I3C_TypeDef;
typedef struct {uint32_t *p_tc_data;uint32_t tc_count_word;uint8_t *p_rx_data;uint32_t rx_count_byte;} hal_i3c_handle_t;
#define I3C_GET_INSTANCE(h) ((I3C_TypeDef*)0)
static int writes,reads;
static uint32_t LL_I3C_IsActiveFlag_CFNF(I3C_TypeDef *p){(void)p;return 1;}
static void LL_I3C_WriteControlWord(I3C_TypeDef *p,uint32_t v){(void)p;assert(v==42);writes++;}
static uint32_t LL_I3C_IsActiveFlag_RXFNE(I3C_TypeDef *p){(void)p;return reads<2;}
static uint8_t LL_I3C_ReceiveData8(I3C_TypeDef *p){(void)p;reads++;return 99;}
'''
        post='''
int main(void){uint32_t tc=42;uint8_t rx[2]={0,77};hal_i3c_handle_t h={&tc,1,rx,1};
I3C_ControlDataTreatment(&h);I3C_ControlDataTreatment(&h);
assert(writes==1&&h.tc_count_word==0&&h.p_tc_data==&tc+1);
I3C_ReceiveByteTreatment(&h);assert(reads==1&&h.rx_count_byte==0&&rx[0]==99&&rx[1]==77);
return 0;}
'''
        with tempfile.TemporaryDirectory() as d:
            src=Path(d)/'test.c';exe=Path(d)/'test'
            src.write_text(pre+'\n'.join(functions)+post)
            subprocess.run(['cc','-Wall','-Wextra','-Werror',str(src),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

if __name__=='__main__':unittest.main()
