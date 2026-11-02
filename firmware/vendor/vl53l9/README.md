# ST VL53L9 driver used by this board

Source: STSW-IMG053, `vl53l9-stm32n6570-dk-1.0.1/drivers/vl53l9`, obtained from the existing reference checkout. Exact upstream repository, commit, local modification and hashes are in `PROVENANCE.json`. This is the portable ST driver with firmware patch 0.17, not a VL53L8 substitute.

The component license and the enclosing package's license table are retained. The enclosing table identifies this component as BSD-3-Clause. No separately licensed transform/postprocessing library is integrated here.

The STM32 platform implementation lives in `firmware/cube/examples/hal/cortex/bringup/application/vl53l9_platform.c`. It uses cycle-timed GPIO I2C, explicit 1.8 V I/O and 2.8 V analog settings, the configured AP clock frequency, 128-byte chunks, big-endian register indices and little-endian register values. Async reads return unsupported.

`lidar_smoke.c` captures one complete 54×42 frame and stops the sensor, then accepts one-at-a-time capture requests through native USB CDC or a versioned SWD mailbox. Raw frame bytes are available in `g_lidar_frame`; they are not a calibrated/postprocessed depth product. The last frame remains in RAM after the test.

Local driver fix: initialize `sync_mode` before the original one-byte read into an enum in `vl53l9_trigger_frame()`. FSM and command polling also use single-byte variables, and failed FSM reads return an invalid state. This prevents indeterminate upper enum bytes from causing spurious state/command errors. Other files retain ST's implementation.
