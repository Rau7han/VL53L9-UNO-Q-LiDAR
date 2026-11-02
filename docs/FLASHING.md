# Flash the LiDAR STM32C552

Use the same PCB and STM32C552KEU6. Power the LiDAR through its own 5 V USB connection. Use ST-Link SWD; normal USB CDC streaming is not a firmware programming interface, and automatic USB DFU entry is not a validated installation method for this package.

J3 pin assignment (confirm connector orientation in the PCB files):

| Pin | Signal |
|---|---|
| 1 | 3.3 V VTref, programmer voltage reference |
| 2 | SWDIO |
| 3 | SWCLK |
| 4 | SWO (optional) |
| 5 | NRST |
| 6 | GND |

VTref is a reference, not a second power source. Use a programmer compatible with the board voltage. Close viewers before flashing.

## STM32CubeProgrammer

Use a version supporting STM32C552. Select ST-Link/SWD and connect. Open `firmware/build/usb-lidar-sensors.hex`, program with verification enabled, then reset. HEX contains its addresses. If selecting `usb-lidar-sensors.bin` instead, set start address **0x08000000**. Do not write this image to the UNO Q MCU.

## Included Linux ST-Link helper

The supplied helper uses ST's bundled XLDR flash algorithm and verifies flash readback:

```bash
python3 -m venv .flash-tools
.flash-tools/bin/pip install pyocd==0.46.0 pyelftools
.flash-tools/bin/python firmware/tools/flash-stlink.py firmware/build/usb-lidar-sensors.bin
```

Install the appropriate ST-Link USB access rules if your Linux account cannot access the probe. The helper targets the 512 KB STM32C552 and does not change option bytes. Do not run it just to inspect USB connectivity: connecting resets/halts the MCU during programming.

After verification, remove the entire SWD connection and test a cold boot from the LiDAR's own USB supply. Do not leave an unpowered debug probe attached. On the USB host:

```bash
lsusb
ls -l /dev/serial/by-id/
```

Expected USB ID: `cafe:4010`; device name contains `USB_LiDAR_VL53L9`, normally mapped to `/dev/ttyACM0` or another ACM number. Each sensor supplies its own calibration at runtime; never substitute another sensor's calibration data.
