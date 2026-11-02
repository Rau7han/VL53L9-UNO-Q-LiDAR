#ifndef LIDAR_BUS_H
#define LIDAR_BUS_H
#include <stdint.h>
/* mode: 0 GPIO I2C, 1 peripheral I2C, 2 dynamically addressed I3C. */
typedef struct {
  uint32_t version, mode, stage, status, kernel_hz, bus_hz, address;
  uint32_t timing0, timing1, error, pid_lo, pid_hi;
  uint32_t reads, writes, bytes, failures;
} lidar_bus_diag_t;
extern volatile lidar_bus_diag_t g_lidar_bus;
int lidar_bus_init(void);
void lidar_bus_fallback(void);
int lidar_bus_speed(uint32_t hz);
int lidar_bus_read(uint16_t reg, uint8_t *data, uint32_t n);
int lidar_bus_write(uint16_t reg, const uint8_t *data, uint32_t n);
#endif
