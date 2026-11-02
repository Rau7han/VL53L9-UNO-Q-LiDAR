#include <string.h>
#include "stm32_hal.h"
#include "stm32c5xx_ll_crs.h"
#include "tusb.h"
#include "usb_stream.h"
#include "lidar_smoke.h"
#include "imu.h"
#include "lidar_bus.h"
#include "bootloader.h"

/* L9R3 + 64-byte mailbox + IMU identity u32 + LiDAR identity u32 + exact payload.
 * F is single-shot; Q enables continuous double-buffered delivery, P stops it.
 * Outgoing data stays frozen under USB backpressure while one frame is acquired. */
extern volatile uint32_t g_lidar_timing[8];
static uint8_t wire_header[76];
/* Freeze outgoing data while the sensor fills its next frame. */
static uint8_t wire_frame[LIDAR_FRAME_SIZE];
static int continuous;
static uint32_t imu_identity,lidar_identity,previous_seq,offset,total;
static int initialized,pending,sending,auxiliary;
static uint8_t auxiliary_packet[2348];
static unsigned boot_match;

static void info_packet(const char *magic,const void *payload,uint32_t length){
 uint32_t version=1,hash=2166136261U;
 memcpy(auxiliary_packet,magic,4);memcpy(auxiliary_packet+4,&version,4);
 memcpy(auxiliary_packet+8,&length,4);memcpy(auxiliary_packet+16,payload,length);
 for(uint32_t i=0;i<length;i++)hash=(hash^auxiliary_packet[16+i])*16777619U;
 memcpy(auxiliary_packet+12,&hash,4);
 total=16+length;offset=0;auxiliary=1;sending=1;
}
volatile uint32_t g_usb_diag[8];
uint32_t tusb_time_millis_api(void){return HAL_GetTick();}
void USB_DRD_FS_IRQHandler(void){tud_int_handler(0);}

int usb_stream_init(uint32_t imu_id,uint32_t lidar_id){
 imu_identity=imu_id;lidar_identity=lidar_id;
 HAL_RCC_USB_EnableClock();HAL_RCC_USB_Reset();
 if(HAL_RCC_CK48_SetKernelClkSource(HAL_RCC_CK48_CLK_SRC_HSIDIV3)!=HAL_OK)return -1;
 HAL_RCC_CRS_EnableClock();HAL_RCC_CRS_Reset();
 LL_CRS_ConfigSynchronization(CRS,LL_CRS_SYNC_SOURCE_USB|LL_CRS_SYNC_DIV_1|LL_CRS_SYNC_POLARITY_RISING,
                              LL_CRS_RELOADVALUE_DEFAULT,LL_CRS_ERRORLIMIT_DEFAULT);
 LL_CRS_EnableAutoTrimming(CRS);LL_CRS_EnableFreqErrorCounter(CRS);
 NVIC_SetPriority(USB_DRD_FS_IRQn,2);
 const tusb_rhport_init_t config={.role=TUSB_ROLE_DEVICE,.speed=TUSB_SPEED_FULL};
 initialized=tusb_init(0,&config);
 g_usb_diag[0]=initialized;
 return initialized?0:-1;
}

void usb_stream_poll(void){
 if(!initialized)return;
 tud_task();imu_poll();g_usb_diag[1]++;
 g_usb_diag[2]=tud_cdc_connected();
 if(!tud_cdc_connected()){pending=0;sending=0;continuous=0;offset=0;boot_match=0;return;}
 while(tud_cdc_available()){
  uint8_t command; if(tud_cdc_read(&command,1)!=1)break;
  if(command=='P'){continuous=0;pending=0;continue;}
  if(!pending && !sending && g_capture.state!=1U){
   static const char boot_command[]="BOOT\n";
   if(command==(uint8_t)boot_command[boot_match]){
    if(++boot_match==sizeof(boot_command)-1){boot_match=0;bootloader_request();}
    continue;
   }
   boot_match=0;
   if(command=='C'){info_packet("L9C1",g_lidar_calibration,g_lidar_calibration_size);continue;}
   if(command=='T'){info_packet("L9T1",(const void *)g_lidar_timing,sizeof(g_lidar_timing));continue;}
   if(command=='D'){info_packet("L9D1",(const void *)&g_lidar_bus,sizeof(g_lidar_bus));continue;}
  }
  if((command=='F'||command=='R'||command=='Q') && !pending && !sending && g_capture.state!=1U){
   continuous=command=='Q';
   previous_seq=g_capture.sequence;g_capture.request=command=='R'?2U:1U;pending=1;g_usb_diag[3]++;
   if(command=='R')g_usb_diag[5]++;
  }
 }
 if(pending && g_capture.state!=1U &&
    (g_capture.sequence!=previous_seq || (g_capture.state==3U && g_capture.request!=2U))){
  memcpy(wire_header,"L9R3",4);
  memcpy(wire_header+4,(const void *)&g_capture,64);
  memcpy(wire_header+68,&imu_identity,4);memcpy(wire_header+72,&lidar_identity,4);
  pending=0;sending=1;offset=0;auxiliary=0;
  previous_seq=g_capture.sequence;
  if(g_capture.state==2U)memcpy(wire_frame,g_lidar_frame,LIDAR_FRAME_SIZE);
  else continuous=0;
  total=sizeof(wire_header)+(g_capture.state==2U?LIDAR_FRAME_SIZE:0U);
  if(continuous)g_capture.request=1U;
 }
 /* IMU packets are only inserted between complete LiDAR packets. */
 static uint32_t last_imu_ms;
 if(!sending && pending && HAL_GetTick()-last_imu_ms>=20 && tud_cdc_write_available()>=36) {
  uint8_t packet[36];memcpy(packet,"I6R1",4);memcpy(packet+4,(const void *)&g_imu,28);
  uint32_t hash=2166136261U;for(unsigned i=0;i<32;i++)hash=(hash^packet[i])*16777619U;
  memcpy(packet+32,&hash,4);tud_cdc_write(packet,36);tud_cdc_write_flush();last_imu_ms=HAL_GetTick();
 }
 if(sending){
  uint32_t avail=tud_cdc_write_available();
  if(avail){
   const uint8_t *data;
   uint32_t n;
   if(auxiliary){data=auxiliary_packet+offset;n=total-offset;}
   else if(offset<sizeof(wire_header)){data=wire_header+offset;n=sizeof(wire_header)-offset;}
   else{data=wire_frame+offset-sizeof(wire_header);n=total-offset;}
   if(n>avail)n=avail;
   offset+=tud_cdc_write(data,n);
   tud_cdc_write_flush();
   if(offset==total){sending=0;g_usb_diag[4]++;if(continuous)pending=1;}
  }
 }
}
