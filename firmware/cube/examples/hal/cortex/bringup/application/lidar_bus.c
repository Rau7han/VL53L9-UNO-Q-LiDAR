/* VL53L9 transport: STOP-delimited register access, bounded transactions,
 * DAA at <=1 MHz and a reset back to the known GPIO path on probe failure.
 * DS14879 rev9 section 4.2; ST HAL2 controller API.
 */
#include <string.h>
#include "stm32_hal.h"
#include "stm32_utils_i3c.h"
#include "soft_i2c.h"
#include "lidar_bus.h"

static hal_i3c_handle_t bus;
volatile lidar_bus_diag_t g_lidar_bus = {.version=2};
static uint8_t tx[258];

static int timing(uint32_t hz, int pure) {
  stm32_utils_i3c_ctrl_timing_config_t c = {
    .clock_src_freq_hz=HAL_RCC_I3C1_GetKernelClkFreq(),
    .i3c_pp_freq_hz=hz, .i2c_od_freq_hz=400000U,
    .duty_cycle_purcent=50U, .wait_time=STM32_UTILS_I3C_ACTIVITY_STATE_0,
    .bus_type=pure ? STM32_UTILS_I3C_PURE_I3C_BUS : STM32_UTILS_I3C_I2C_MIXED_BUS
  };
  hal_i3c_ctrl_config_t cc={0};
  g_lidar_bus.kernel_hz=c.clock_src_freq_hz;
  g_lidar_bus.status=STM32_UTILS_I3C_CTRL_ComputeTiming(&c,&cc.timing_reg0,&cc.timing_reg1);
  if(g_lidar_bus.status!=STM32_UTILS_I3C_OK)return -1;
  g_lidar_bus.timing0=cc.timing_reg0;g_lidar_bus.timing1=cc.timing_reg1;
  g_lidar_bus.status=HAL_I3C_CTRL_SetConfig(&bus,&cc);
  if(g_lidar_bus.status!=HAL_OK)return -1;
  g_lidar_bus.bus_hz=pure ? hz : 400000U;
  return 0;
}

static int transfer(uint8_t *data,uint32_t n,int read) {
  hal_i3c_transfer_ctx_t ctx;
  uint32_t control;
  hal_i3c_private_desc_t desc={g_lidar_bus.address,n,read?HAL_I3C_DIRECTION_READ:HAL_I3C_DIRECTION_WRITE};
  hal_status_t s=HAL_I3C_CTRL_ResetTransferCtx(&ctx);
  if(s==HAL_OK)s=HAL_I3C_CTRL_InitTransferCtxTc(&ctx,&control,1);
  if(s==HAL_OK)s=read?HAL_I3C_CTRL_InitTransferCtxRx(&ctx,data,n):HAL_I3C_CTRL_InitTransferCtxTx(&ctx,data,n);
  if(s==HAL_OK)s=HAL_I3C_CTRL_BuildTransferCtxPrivate(&ctx,&desc,1,
    g_lidar_bus.mode==2 ? HAL_I3C_PRIVATE_WITH_ARB_STOP : HAL_I2C_PRIVATE_WITHOUT_ARB_STOP);
  if(s==HAL_OK)s=HAL_I3C_CTRL_Transfer(&bus,&ctx,20);
  g_lidar_bus.status=s;
  if(s!=HAL_OK){g_lidar_bus.error=HAL_I3C_GetLastErrorCodes(&bus);g_lidar_bus.failures++;return -1;}
  g_lidar_bus.bytes+=n;
  return 0;
}
int lidar_bus_read(uint16_t reg,uint8_t *data,uint32_t n) {
  g_lidar_bus.reads++;
  if(!g_lidar_bus.mode)return soft_i2c_read(0x29,reg,data,n);
  uint8_t index[2]={reg>>8,reg};
  if(transfer(index,2,0))return -1;
  return transfer(data,n,1);
}
int lidar_bus_write(uint16_t reg,const uint8_t *data,uint32_t n) {
  g_lidar_bus.writes++;
  if(!g_lidar_bus.mode)return soft_i2c_write(0x29,reg,data,n);
  if(n>sizeof(tx)-2)return -1;
  tx[0]=reg>>8;tx[1]=reg;memcpy(tx+2,data,n);
  return transfer(tx,n+2,0);
}
int lidar_bus_speed(uint32_t hz) {
  if(g_lidar_bus.mode!=2)return 0;
  if(hz!=1000000U && hz!=6000000U && hz!=12000000U)return -1;
  return timing(hz,1);
}
void lidar_bus_fallback(void) {
  /* A target with a dynamic address cannot be silently reused as legacy I2C. */
  HAL_GPIO_WritePin(HAL_GPIOB,HAL_GPIO_PIN_7,HAL_GPIO_PIN_RESET);
  HAL_RCC_I3C1_Reset();HAL_RCC_I3C1_DisableClock();
  g_lidar_bus.mode=0;g_lidar_bus.address=0x29;g_lidar_bus.bus_hz=0;
  soft_i2c_init();HAL_Delay(10);
  HAL_GPIO_WritePin(HAL_GPIOB,HAL_GPIO_PIN_7,HAL_GPIO_PIN_SET);HAL_Delay(50);
}
int lidar_bus_init(void) {
  uint32_t id=0;uint64_t payload=0;
  hal_i3c_target_detection_status_t detected;
  g_lidar_bus.stage=1;
  HAL_RCC_I3C1_EnableClock();HAL_RCC_I3C1_Reset();
  hal_gpio_config_t g={0};g.mode=HAL_GPIO_MODE_ALTERNATE;
  g.output_type=HAL_GPIO_OUTPUT_PUSHPULL;g.pull=HAL_GPIO_PULL_NO;g.speed=HAL_GPIO_SPEED_FREQ_HIGH;
  g.alternate=HAL_GPIO_AF_2;
  if(HAL_GPIO_Init(HAL_GPIOA,HAL_GPIO_PIN_8,&g)!=HAL_OK)goto fail;
  g.alternate=HAL_GPIO_AF_3;
  if(HAL_GPIO_Init(HAL_GPIOB,HAL_GPIO_PIN_4,&g)!=HAL_OK)goto fail;
  if(HAL_I3C_Init(&bus,HAL_I3C1)!=HAL_OK)goto fail;
  if(HAL_RCC_I3C1_SetKernelClkSource(HAL_RCC_I3C1_CLK_SRC_PCLK1)!=HAL_OK)goto fail;
  g_lidar_bus.stage=2;
  /* 100 kHz at 72 MHz needs 360 cycles/phase, beyond the 8-bit field. */
  if(timing(1000000U,0))goto fail;
  hal_i3c_ctrl_fifo_config_t fifo={.tx_fifo_threshold=HAL_I3C_TX_FIFO_THRESHOLD_1_8,
    .rx_fifo_threshold=HAL_I3C_RX_FIFO_THRESHOLD_1_8,.ctrl_fifo=HAL_I3C_CTRL_FIFO_NONE};
  if(HAL_I3C_CTRL_SetConfigFifo(&bus,&fifo)!=HAL_OK)goto fail;
  g_lidar_bus.mode=1;g_lidar_bus.address=0x29;g_lidar_bus.stage=3;
  if(lidar_bus_read(0,(uint8_t *)&id,4) || id!=0x53334c39U)goto fail;
  g_lidar_bus.stage=4;
  if(timing(1000000U,1))goto fail;
  g_lidar_bus.status=HAL_I3C_CTRL_DynAddrAssign(&bus,&payload,HAL_I3C_DYN_ADDR_ONLY_ENTDAA,&detected,20);
  if(g_lidar_bus.status!=HAL_OK || detected!=HAL_I3C_TGT_DETECTED)goto fail;
  g_lidar_bus.pid_lo=payload;g_lidar_bus.pid_hi=payload>>32;
  if(HAL_I3C_CTRL_SetDynAddr(&bus,0x30)!=HAL_OK)goto fail;
  payload=0;
  g_lidar_bus.status=HAL_I3C_CTRL_DynAddrAssign(&bus,&payload,HAL_I3C_DYN_ADDR_ONLY_ENTDAA,&detected,20);
  if(g_lidar_bus.status!=HAL_OK || detected!=HAL_I3C_TGT_NOT_DETECTED)goto fail;
  hal_i3c_ctrl_device_config_t device={0};device.tgt_dynamic_addr=0x30;
  if(HAL_I3C_CTRL_SetConfigBusDevices(&bus,&device,1)!=HAL_OK)goto fail;
  g_lidar_bus.mode=2;g_lidar_bus.address=0x30;g_lidar_bus.stage=5;
  for(unsigned i=0;i<8;i++)if(lidar_bus_read(0,(uint8_t *)&id,4) || id!=0x53334c39U)goto fail;
  g_lidar_bus.stage=6;return 0;
fail:
  g_lidar_bus.error=HAL_I3C_GetLastErrorCodes(&bus);
  lidar_bus_fallback();return -1;
}
