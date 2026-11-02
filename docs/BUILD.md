# Build from source (Linux)

Run commands from the repository root. Bundled sources are included; no submodule checkout is needed.

## STM32 firmware

Install the ARM bare-metal toolchain, Ninja, Python and a recent CMake:

```bash
sudo apt update
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-dev ninja-build python3-venv
python3 -m venv .build-tools
.build-tools/bin/python -m pip install cmake==4.4.4
export PATH="$PWD/.build-tools/bin:$PATH"
arm-none-eabi-gcc --version
cmake --version
python3 firmware/tools/build.py
```

CMake must be at least 3.30. The clean package was built with CMake 4.4.4 and ARM GCC 13.2.1. The generated preset/toolchain names mention NUCLEO-C542RC and GCC 14.3.1; the configured application actually targets STM32C552KEU6 and uses the compiler on PATH. Do not substitute a Nucleo firmware image.

Outputs: `firmware/build/usb-lidar-sensors.bin`, `.hex` and `.elf`. BIN base address is `0x08000000`.

Application source: `firmware/cube/examples/hal/cortex/bringup/application/`. Build configuration: `firmware/cube/examples/hal/cortex/bringup/USB-LiDAR/cmake/`. Keep the custom application and pin/clock configuration when regenerating ST configuration files. The build script is the supported build entry point.

## Host processing library

Build on the machine that will run the dashboard, so the shared library matches its CPU architecture:

```bash
sudo apt install build-essential python3
python3 dashboard/build_transform.py
```

This compiles the bundled ST transform sources and `dashboard/transform_bridge.c` into `dashboard/build/libvl53l9_transform.so`. Alternatively, copy the matching supplied library:

```bash
mkdir -p dashboard/build
# UNO Q ARM64:
cp dashboard/prebuilt/linux-aarch64/libvl53l9_transform.so dashboard/build/
# For an x86_64 laptop instead, use linux-x86_64 in the command above.
```

Do not install an x86_64 library on UNO Q. GUI dependencies and launching are documented in UNO_Q_SETUP.md.

## Regression tests

```bash
python3 -m unittest discover -s dashboard/tests -v
python3 -m unittest discover -s firmware/tools/tests -v
```

These tests do not replace flashing, USB enumeration, live ranging and measured-distance verification on your assembled board.
