/* STM32C552 bring-up platform for ST's portable VL53L9 driver.
 * STOP between register-index write and read; LE register data, BE index.
 * The original asynchronous API is deliberately unsupported. */
#include "stm32_hal.h"
#include "lidar_bus.h"
#include "usb_stream.h"
#include "vl53l9_platform.h"
#define ADDRESS 0x29U
int vl53l9_read(void *const d, uint16_t reg, uint8_t *p, uint32_t n) {
  (void)d;
  for(uint32_t i=0;i<n;) {
    uint32_t k=n-i; uint32_t limit=g_lidar_bus.mode==2?1024U:128U; if(k>limit)k=limit;
    if(lidar_bus_read(reg+i,p+i,k))return VL53L9_ERROR_PLATFORM;
    i+=k;
    usb_stream_poll();
  }
  return 0;
}
int vl53l9_write(void *const d,uint16_t reg,uint8_t *p,uint32_t n) {
  (void)d;
  for(uint32_t i=0;i<n;) {
    uint32_t k=n-i; if(k>128)k=128;
    if(lidar_bus_write(reg+i,p+i,k))return VL53L9_ERROR_PLATFORM;
    i+=k;
    usb_stream_poll();
  }
  return 0;
}
int vl53l9_read8(void *const d,uint16_t r,uint8_t *p){return vl53l9_read(d,r,p,1);}
int vl53l9_read16(void *const d,uint16_t r,uint16_t *p){
  uint8_t b[2];int e=vl53l9_read(d,r,b,2);if(!e)*p=(uint16_t)b[0]|((uint16_t)b[1]<<8);return e;
}
int vl53l9_read32(void *const d,uint16_t r,uint32_t *p){
  uint8_t b[4];int e=vl53l9_read(d,r,b,4);if(!e)*p=(uint32_t)b[0]|((uint32_t)b[1]<<8)|((uint32_t)b[2]<<16)|((uint32_t)b[3]<<24);return e;
}
int vl53l9_write8(void *const d,uint16_t r,uint8_t v){return vl53l9_write(d,r,&v,1);}
int vl53l9_write16(void *const d,uint16_t r,uint16_t v){uint8_t b[2]={v,v>>8};return vl53l9_write(d,r,b,2);}
int vl53l9_write32(void *const d,uint16_t r,uint32_t v){uint8_t b[4]={v,v>>8,v>>16,v>>24};return vl53l9_write(d,r,b,4);}
int vl53l9_wait_ms(void *const d,uint32_t t){(void)d;uint32_t start=HAL_GetTick();while(HAL_GetTick()-start<t)usb_stream_poll();return 0;}
int vl53l9_get_config_vddio(void *const d,vl53l9_vddio_t *v){(void)d;*v=VDDIO_1V8;return 0;}
int vl53l9_get_config_vdda(void *const d,vl53l9_vdda_t *v){(void)d;*v=VDDA_2V8;return 0;}
int vl53l9_get_config_ext_clock(void *const d,uint32_t *v){*v=*(const uint32_t *)d;return 0;}
int vl53l9_read_async(void *const d,uint16_t r,volatile uint8_t *p,uint32_t n){
  (void)d;(void)r;(void)p;(void)n;return VL53L9_ERROR_INVALID_OPERATION;
}
