#ifndef LIDAR_SMOKE_H
#define LIDAR_SMOKE_H
#include <stdint.h>
typedef struct {
  uint32_t stage;
  int32_t result;
  uint32_t device_id, patch, frames, frame_counter, raw_min, raw_max;
  uint32_t fsm, firmware_error, error_flags;
  int32_t stop_result;
} lidar_smoke_result_t;
extern volatile lidar_smoke_result_t g_lidar;
#define LIDAR_WIDTH 54U
#define LIDAR_HEIGHT 42U
#define LIDAR_PIXELS (LIDAR_WIDTH * LIDAR_HEIGHT)
#define LIDAR_FRAME_SIZE (LIDAR_PIXELS * 6U + LIDAR_PIXELS / 2U + 100U)
/* ABI v2: 16 LE words; request 1=capture, 2=reinitialize.
 * state: 1=busy, 2=ready (FSM 2 standby or 3 streaming), 3=error.
 * Stop USB streaming before SWD access; USB continuous mode requests new frames. */
typedef struct {
  uint32_t magic, version, state, request, sequence, width, height, payload_size;
  uint32_t fnv1a, capture_ms;
  int32_t result, stop_result;
  uint32_t fsm, firmware_error, error_flags, frame_counter;
} lidar_capture_t;
extern volatile lidar_capture_t g_capture;
extern uint8_t g_lidar_frame[LIDAR_FRAME_SIZE];
void lidar_smoke_run(uint32_t clock_hz);
void lidar_capture_service(void);
extern uint8_t g_lidar_calibration[2332];
extern uint32_t g_lidar_calibration_size;
#endif
