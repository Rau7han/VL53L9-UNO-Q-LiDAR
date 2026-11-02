#include "stm32_hal.h"
#include "lidar_smoke.h"
#include "vl53l9_platform.h"
#include "vl53l9_reg.h"
#include "soft_i2c.h"
#include "lidar_bus.h"
#include "usb_stream.h"

volatile lidar_smoke_result_t g_lidar;
volatile lidar_capture_t g_capture;
uint8_t g_lidar_frame[LIDAR_FRAME_SIZE];
static uint32_t ap_clock_hz;
static int configured, streaming;
static uint32_t last_capture_ms;
volatile uint32_t g_lidar_timing[8];
uint8_t g_lidar_calibration[VL53L9_CALIB_DATA_SIZE];
uint32_t g_lidar_calibration_size;
#define TRY(call) do {ret=(call); if(ret)goto done;}while(0)

static void publish(int ret, uint32_t started) {
  vl53l9_status_t status={0};
  int status_ret=vl53l9_get_status(&ap_clock_hz,&status);
  if(!ret && status_ret)ret=status_ret;
  g_lidar.result=ret;
  g_lidar.fsm=status.fsm;
  g_lidar.firmware_error=status.firmware;
  g_lidar.error_flags=*(uint8_t *)&status.error;
  g_capture.result=ret;g_capture.stop_result=g_lidar.stop_result;
  g_capture.fsm=g_lidar.fsm;g_capture.firmware_error=g_lidar.firmware_error;
  g_capture.error_flags=g_lidar.error_flags;
  g_capture.capture_ms=HAL_GetTick()-started;
  if(ret || g_lidar.stop_result || (g_lidar.fsm!=2U && g_lidar.fsm!=3U) ||
     g_lidar.firmware_error || g_lidar.error_flags) {
    configured=0;streaming=0;
    HAL_GPIO_WritePin(HAL_GPIOB,HAL_GPIO_PIN_7,HAL_GPIO_PIN_RESET);
    __DMB();g_capture.state=3U;
  } else {
    __DMB();g_capture.state=2U;
  }
}

static void capture_one(void) {
  int ret=0;
  uint32_t started=HAL_GetTick();
  g_capture.state=1U;__DMB();
  g_lidar.stop_result=0;
  g_lidar.stage=4;
  uint32_t mark=HAL_GetTick();
  if(!streaming){TRY(vl53l9_start(&ap_clock_hz));streaming=1;}
  g_lidar_timing[0]=HAL_GetTick()-mark;mark=HAL_GetTick();
  TRY(lidar_bus_speed(12000000U));
  g_lidar.stage=5;
  TRY(vl53l9_trigger_frame(&ap_clock_hz));
  g_lidar_timing[1]=HAL_GetTick()-mark;
  uint32_t waiting=HAL_GetTick();uint8_t ready=0;
  do {
    TRY(vl53l9_poll_frame(&ap_clock_hz,&ready));
    if(!ready){(void)vl53l9_wait_ms(&ap_clock_hz,1);}
    if(HAL_GetTick()-waiting>2000U){ret=VL53L9_ERROR_TIMEOUT;goto done;}
  }while(!ready);
  g_lidar_timing[2]=HAL_GetTick()-waiting;mark=HAL_GetTick();
  g_lidar.stage=6;
  TRY(vl53l9_get_frame(&ap_clock_hz,g_lidar_frame,sizeof(g_lidar_frame)));
  g_lidar_timing[3]=HAL_GetTick()-mark;mark=HAL_GetTick();
  const uint32_t offset=LIDAR_PIXELS*6U+LIDAR_PIXELS/2U;
  uint32_t counter=(uint32_t)g_lidar_frame[offset]|((uint32_t)g_lidar_frame[offset+1]<<8)|
                   ((uint32_t)g_lidar_frame[offset+2]<<16)|((uint32_t)g_lidar_frame[offset+3]<<24);
  /* The sensor counter can restart on start/stop; host sequence is monotonic. */
  g_lidar.frame_counter=counter;
  uint32_t lo=65535U,hi=0U,hash=2166136261U;
  for(unsigned i=0;i<LIDAR_PIXELS;i++) {
    uint32_t raw=(uint32_t)g_lidar_frame[2*i]|((uint32_t)g_lidar_frame[2*i+1]<<8);
    if(raw<lo)lo=raw;
    if(raw>hi)hi=raw;
  }
  for(unsigned i=0;i<sizeof(g_lidar_frame);i++)hash=(hash^g_lidar_frame[i])*16777619U;
  g_lidar.raw_min=lo;g_lidar.raw_max=hi;g_lidar.frames++;
  g_capture.sequence=g_lidar.frames;g_capture.frame_counter=counter;g_capture.fnv1a=hash;
  g_lidar.stage=7;
done:
  if(ret && streaming){
    (void)lidar_bus_speed(1000000U);
    g_lidar.stop_result=vl53l9_stop(&ap_clock_hz);streaming=0;
  }
  last_capture_ms=HAL_GetTick();
  publish(ret,started);
  g_lidar_timing[4]=HAL_GetTick()-mark;
  g_lidar_timing[5]=6;g_lidar_timing[6]=12000000;g_lidar_timing[7]=1024;
}

void lidar_smoke_run(uint32_t clock_hz) {
  int ret=0;
  uint32_t id=0,started=HAL_GetTick();uint16_t patch=0,size=0;
  ap_clock_hz=clock_hz;
  configured=0;streaming=0;g_lidar.stop_result=0;
  g_capture.magic=0x4C394652U;g_capture.version=2U;g_capture.state=1U;
  g_capture.width=LIDAR_WIDTH;g_capture.height=LIDAR_HEIGHT;
  g_capture.payload_size=sizeof(g_lidar_frame);
  g_lidar_calibration_size=0;
  g_lidar.stage=1;
  TRY(vl53l9_get_device_id(&ap_clock_hz,&id));g_lidar.device_id=id;
  if(id!=0x53334c39U){ret=VL53L9_ERROR_INVALID_STATE;goto done;}
  g_lidar.stage=2;
  TRY(vl53l9_init(&ap_clock_hz));
  TRY(vl53l9_read16(&ap_clock_hz,VL53L9_REGADDR_PATCH_REVISION,&patch));g_lidar.patch=patch;
  TRY(vl53l9_get_calib_data(&ap_clock_hz,g_lidar_calibration));
  g_lidar_calibration_size=sizeof(g_lidar_calibration);
  g_lidar.stage=3;
  vl53l9_hw_config_t hw={0};
  TRY(vl53l9_get_hw_config(&ap_clock_hz,&hw));
  hw.output_interface=true;hw.signaling_mode=true;hw.interrupt_pad_mode=true;
  TRY(vl53l9_set_hw_config(&ap_clock_hz,hw));
  TRY(vl53l9_set_sync_mode(&ap_clock_hz,VL53L9_SYNC_MANUAL));
  TRY(vl53l9_set_power_mode(&ap_clock_hz,VL53L9_POWER_ULTRA_LOW));
  TRY(vl53l9_set_context(&ap_clock_hz,VL53L9_CONTEXT_SHORT));
  TRY(vl53l9_set_binning(&ap_clock_hz,VL53L9_CONTEXT_SHORT,2));
  TRY(vl53l9_set_exposure(&ap_clock_hz,VL53L9_CONTEXT_SHORT,6));
  TRY(vl53l9_get_raw_buffer_size(2,&size));
  if(size!=sizeof(g_lidar_frame)){ret=VL53L9_ERROR_INVALID_PARAM;goto done;}
  configured=1;
  capture_one();
  return;
done:
  publish(ret,started);
}

void lidar_capture_service(void) {
  /* Manual mode emits only when triggered; stop after host inactivity. */
  if(streaming && !g_capture.request && g_capture.state==2U && HAL_GetTick()-last_capture_ms>500U){
    uint32_t started=HAL_GetTick();
    int ret=lidar_bus_speed(1000000U);
    g_lidar.stop_result=vl53l9_stop(&ap_clock_hz);streaming=0;
    if(ret || g_lidar.stop_result)publish(ret,started);
  }
  if(g_capture.request==2U && g_capture.state!=1U) {
    g_capture.request=0U;g_capture.state=1U;configured=0;
    soft_i2c_relax();lidar_bus_fallback();(void)lidar_bus_init();g_lidar.stop_result=0;
    lidar_smoke_run(ap_clock_hz);
    return;
  }
  if(g_capture.request==1U && configured && g_capture.state==2U) {
    g_capture.request=0U;
    capture_one();
  }
}
