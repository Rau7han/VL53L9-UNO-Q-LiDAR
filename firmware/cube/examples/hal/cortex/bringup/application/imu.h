#ifndef IMU_H
#define IMU_H
#include <stdint.h>
/* status: 0 valid; 1 configuration failed; 2 SPI failed; 3 waiting. */
typedef struct {
 uint32_t sequence, timestamp_ms, status;
 int16_t temperature, gyro[3], accel[3];
 uint16_t reserved;
} imu_sample_t;
extern volatile imu_sample_t g_imu;
void imu_poll(void);
#endif
