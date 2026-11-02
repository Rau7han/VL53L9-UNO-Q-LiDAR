#!/usr/bin/env python3
"""STM32C552 SWD programming using ST's bundled, unmodified XLDR Keil entrypoints.
Requires pyocd and pyelftools. Writes only image sectors, verifies every byte.
"""
import argparse,struct,hashlib
from pathlib import Path
from elftools.elf.elffile import ELFFile
from pyocd.core.helpers import ConnectHelper
from pyocd.coresight.coresight_target import CoreSightTarget
from pyocd.core.memory_map import MemoryMap,FlashRegion,RamRegion
from pyocd.target import TARGET
from pyocd.flash.flash import Flash
root=Path(__file__).resolve().parents[1]
loader=root/'cube/stm32c5xx_dfp/Flash/STM32C5[56]x.xldr'
with loader.open('rb') as f:
 elf=ELFFile(f);code=elf.get_section_by_name('PrgCode').data()
 symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
# A breakpoint return trampoline precedes the position-independent ST image.
base=0x20000004
algo={'load_address':0x20000000,'instructions':[0xe7febe00]+list(struct.unpack('<%dI'%(len(code)//4),code)),
 'static_base':base,'begin_stack':0x2001f000,'end_stack':0x2001e000,
 'page_buffers':[0x20005000],'min_program_length':16,'analyzer_supported':False}
for key,symbol in [('pc_init','Keil_Init'),('pc_unInit','UnInit'),('pc_erase_sector','EraseSector'),('pc_program_page','ProgramPage')]:
 algo[key]=(base+symbols[symbol])|1
class STFlash(Flash):
 def prepare_target(self):
  # Reset-and-halt can stop after startup installed MPU and enabled ICACHE.
  # Use a stack above the application limit, and bypass cached system-flash reads.
  self.target.write32(0xe000ed94,0)  # MPU CTRL
  self.target.write32(0xe000e010,0)  # SysTick CTRL
  self.target.write32(0x40030400,0)  # ICACHE CR: direct system-flash reads
  self.target.write_core_register('primask',1)
class C552(CoreSightTarget):
 VENDOR='STMicroelectronics'
 def __init__(self,session):
  super().__init__(session,MemoryMap(FlashRegion(start=0x08000000,length=0x80000,blocksize=0x800,page_size=0x800,is_boot_memory=True,algo=algo,flash_class=STFlash),RamRegion(start=0x20000000,length=0x20000)))
TARGET['lidar_c552']=C552
ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('image',nargs='?',type=Path);ap.add_argument('--backup',type=Path)
args=ap.parse_args()
image=args.image.read_bytes() if args.image else None
if image:
 if len(image)>0x80000 or len(image)<8:ap.error('Image outside 512 KB flash')
 sp,pc=struct.unpack_from('<II',image)
 if not (0x20000000<sp<=0x20020000 and 0x08000000<pc<0x08000000+len(image) and pc&1):
  ap.error('Image vectors do not describe this STM32C552 application')
with ConnectHelper.session_with_chosen_probe(target_override='lidar_c552',frequency=1000000,auto_unlock=False,connect_mode='halt',resume_on_disconnect=False) as session:
 t=session.target
 print('Flash size KB:',t.read16(0x08fff80c),flush=True)
 if t.read16(0x08fff80c)!=512:raise RuntimeError('Not the expected 512 KB device')
 if args.backup:
  if args.backup.exists():raise RuntimeError('Refusing to overwrite backup')
  args.backup.write_bytes(bytes(t.read_memory_block8(0x08000000,0x80000)))
 if image:
  flash=t.memory_map.get_region_for_address(0x08000000).flash
  flash.init(Flash.Operation.ERASE)
  for offset in range(0,len(image),0x800):flash.erase_sector(0x08000000+offset)
  flash.uninit();flash.init(Flash.Operation.PROGRAM)
  padded=image+b'\xff'*((-len(image))%16)
  for offset in range(0,len(padded),0x800):flash.program_page(0x08000000+offset,list(padded[offset:offset+0x800]))
  flash.cleanup()
  actual=bytes(t.read_memory_block8(0x08000000,len(image)))
  if actual!=image:raise RuntimeError('Flash readback differs from input; target remains halted')
  print('Verified SHA256:',hashlib.sha256(actual).hexdigest(),flush=True)
 t.reset()
