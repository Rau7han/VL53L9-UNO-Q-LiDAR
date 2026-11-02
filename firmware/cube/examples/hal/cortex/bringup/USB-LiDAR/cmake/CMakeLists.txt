# file-format: 1.0.0
cmake_minimum_required(VERSION 3.20)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

include(cmake/target.cmake)
include(cmake/${TOOLCHAIN_FILE})

project(hal_cortex_systick_delay)
add_executable(hal_cortex_systick_delay)
include(cmake/files.cmake)
include(cmake/flags.cmake)
include(cmake/components.cmake)

# Stubs to be able to add compiler flag overrides, e.g. -Os
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
  ../../../../../../../vendor/lsm6dsv/lsm6dsv_reg.c
  ../../application/usb_stream.c
  ../../application/usb_descriptors.c
  ../../../../../../../vendor/tinyusb-src/tusb.c
  ../../../../../../../vendor/tinyusb-src/common/tusb_fifo.c
  ../../../../../../../vendor/tinyusb-src/device/usbd.c
  ../../../../../../../vendor/tinyusb-src/class/cdc/cdc_device.c
  ../../../../../../../vendor/tinyusb-src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
  ../../../../../../../vendor/tinyusb-src/portable/st/stm32_fsdev/fsdev_common.c
  ../../application/lidar_bus.c
  ../../application/bootloader.c
  ../../application/soft_i2c.c
  ../../application/lidar_smoke.c
  ../../application/vl53l9_platform.c
  ../../../../../../../vendor/vl53l9/vl53l9.c
  # Add additional source files here
  ../../../../../../stm32c5xx_drivers/hal/stm32c5xx_hal_spi.c
  ../../../../../../stm32c5xx_drivers/hal/stm32c5xx_hal_i3c.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
  ../../../../../../../vendor/lsm6dsv
  ../../../../../../../vendor/vl53l9
  ../../../../../../../vendor/tinyusb-src
  # Add additional include directories here
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC
  # Add additional defined symbols here
)

target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC
  # Add additional compiler flags here
)

target_link_options(${CMAKE_PROJECT_NAME} PUBLIC
  # Add additional linker flags here
)

target_link_directories(${CMAKE_PROJECT_NAME} PUBLIC
  # Add additional linker search directories here
)

target_link_libraries(${CMAKE_PROJECT_NAME}
  m
  # Add additional linker libraries here
)

# Keep the timing-critical GPIO bus inlined; other firmware retains -Os.
set_source_files_properties(../../application/soft_i2c.c PROPERTIES COMPILE_OPTIONS "-O3")
