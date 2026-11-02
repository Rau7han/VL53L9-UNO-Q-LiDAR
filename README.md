# DIY 3D Depth Mapping with VL53L9CX and Arduino UNO Q

Companion source package for [the DIY Projects Labs tutorial](https://diyprojectslabs.com/3d-lidar-arduino-uno-q-vl53l9cx/).
Original USB-LiDAR hardware: [RemiFM/USB-LiDAR-VL53L9](https://github.com/RemiFM/USB-LiDAR-VL53L9). This project adds firmware improvements, UNO Q integration and a native Python/GTK dashboard. Original copyright and third-party licenses are retained.

## Start here

1. Build the same PCB using `pcb/` and check its components and power rails.
2. [Flash the LiDAR board](docs/FLASHING.md) using the supplied BIN or HEX.
3. [Install and run on Arduino UNO Q](docs/UNO_Q_SETUP.md).
4. To modify the code, follow [Build from source](docs/BUILD.md).

**The firmware runs on the LiDAR PCB's STM32C552KEU6, not the UNO Q's onboard microcontroller.** UNO Q runs the Linux dashboard and receives data over USB. This is not an Arduino IDE sketch.

## Included

| Folder | Contents |
|---|---|
| `firmware/build/` | Ready-to-flash `.bin`, `.hex`, debugging `.elf` |
| `firmware/cube/` | MCU application, generated configuration, HAL, linker files and flash loader |
| `firmware/vendor/` | Sensor, IMU and USB stack source and licenses |
| `dashboard/` | Python/GTK GUI, relay, ST processing C source, tests, native library prebuilts |
| `uno-q/` | Prepared ARM dashboard and Python dependency snapshot used by the article |
| `pcb/` | KiCad project, BOM and manufacturing files |
| `docs/` | Setup, flashing, building and validation notes |

The native dashboard provides 3D, Depth and Top views, confidence filtering, adjustable point size, reflectance estimates, optional triangulated surfaces, and relative IMU orientation. Up to 54 × 42 = 2,268 measured points are available per frame; invalid measurements remain absent. Surface interpolation does not add measured points. IMU yaw can drift; this is not SLAM.

The tested configuration received approximately 35–36 frames/s. This is not a guarantee of screen refresh rate, accuracy or end-to-end latency. ST's maximum sensor specifications do not all apply simultaneously to this setup. UNO Q VNC uses CPU rendering; the optional laptop viewer supports OpenGL.

Use only one viewer/relay at a time. The native GUI does not save captures. The retained legacy `raw_server.py` web application can record captures and is not the tutorial launcher.

See [validation](docs/VALIDATION.md) and [third-party notices](THIRD_PARTY_NOTICES.md). Before flashing, check release checksums with `sha256sum -c SHA256SUMS` from the repository root.
