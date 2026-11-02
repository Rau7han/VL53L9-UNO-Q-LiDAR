#ifndef USB_STREAM_H
#define USB_STREAM_H
#include <stdint.h>
int usb_stream_init(uint32_t imu_id, uint32_t lidar_id);
void usb_stream_poll(void);
#endif
