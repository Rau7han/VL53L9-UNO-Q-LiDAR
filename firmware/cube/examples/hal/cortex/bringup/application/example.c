/**
  ******************************************************************************
  * file           : example.c
  * brief          : USB-LiDAR-VL53L9 sensor bring-up (IMU + LiDAR).
  *
  *   IMU  : LSM6DSV-family on SPI1 (SCK=PA5, MISO=PA6, MOSI=PA7, CS=PA4).
  *          WHO_AM_I (0x0F) must read 0x70.
  *   LiDAR: VL53L9CX on I3C1 (SCL=PB4/AF3, SDA=PA8/AF2), via LSF0102.
  *          Needs AP clock 6-27 MHz on MCO2/PA9 and XSHUT (PB7) high, then
  *          uses legacy I2C for this diagnostic (no DAA).
  *
  *   Results are stored in g_diag and read back live over SWD.
  *   LED (PB8): fast blink after IMU identity and full-frame capture pass.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "example.h"
#include "stm32_hal.h"
#include "stm32_utils_i3c.h"
#include "soft_i2c.h"
#include "lidar_bus.h"
#include "lidar_smoke.h"
#include "usb_stream.h"
#include "imu.h"
#include "lsm6dsv_reg.h"
#include <string.h>

/* ---- board pins ---- */
#define LED_PORT     HAL_GPIOB
#define LED_PIN      HAL_GPIO_PIN_8        /* LED_ONn, active low */
#define IMU_CS_PORT  HAL_GPIOA
#define IMU_CS_PIN   HAL_GPIO_PIN_4        /* SPI_CSn to LSM6DSV  */

/* ---- LSM6DSV family; 0x70 does not uniquely identify the exact variant ---- */
#define LSM6DSV_REG_WHOAMI   0x0FU
#define LSM6DSV_WHOAMI_VAL   0x70U

typedef struct
{
  uint32_t magic;          /* 0x5E450005 after full-frame test completes */
  uint32_t spi_init_ok;    /* 1 if SPI1 initialized                */
  uint32_t imu_whoami;     /* WHO_AM_I byte                        */
  uint32_t imu_ok;         /* 1 if WHO_AM_I == 0x70                */
  uint32_t i3c_init_ok;    /* 1 if I3C1 initialized                */
  uint32_t i3c_kclk;       /* I3C1 kernel clock Hz (sanity)        */
  uint32_t scan_count;     /* successful legacy I2C index writes  */
  uint32_t scan_addresses;/* first four ACKing addresses         */
  uint32_t idxwrite_status;/* HAL status of reg-index private write*/
  uint32_t model_read;     /* HAL status of DEVICE_ID private read */
  uint32_t lidar_model_id; /* VL53L9CX DEVICE_ID (reg 0x0000, u32) */
  uint32_t lidar_detected; /* 1 if DEVICE_ID == 0x53334C39 (S3L9)  */
  uint32_t i3c_err;        /* HAL_I3C_GetLastErrorCodes after xfer  */
  uint32_t treg0;          /* computed I3C timing reg0             */
  uint32_t treg1;          /* computed I3C timing reg1             */
  uint32_t loops;          /* heartbeat counter                   */
  uint32_t bus_idle;       /* bit0 SDA, bit1 SCL, at MCU           */
  uint32_t soft_ack;       /* GPIO I2C address ACK at 0x29         */
  uint32_t soft_status;    /* 0 success; 1 NACK; 2 stuck bus       */
  uint32_t soft_id;        /* independent GPIO I2C ID read        */
  uint32_t hse_status;     /* HAL status starting crystal         */
  uint32_t ap_clock_hz;    /* configured frequency, not measured  */
  uint32_t imu_status;     /* last SPI transfer HAL status        */
  uint32_t sync_level;     /* PA3 released, external 1.8V pull-up  */
} diag_t;

#define VL53L9_DEVICE_ID   0x53334C39U   /* "S3L9" */
#define VL53L9_STATIC_ADDR 0x29U         /* I2C static address (7-bit) */

#define LIDAR_SHDN_PORT  HAL_GPIOB
#define LIDAR_SHDN_PIN   HAL_GPIO_PIN_7   /* XSHUT, high = enabled */

volatile diag_t g_diag __attribute__((used)) = {0};

static hal_spi_handle_t hspi1;

static uint32_t led_delay_ms = 400U;

/* ============================ IMU (SPI1) ================================== */
static int spi1_init(void)
{
  hal_gpio_config_t g;

  HAL_RCC_GPIOA_EnableClock();

  g.mode = HAL_GPIO_MODE_OUTPUT; g.speed = HAL_GPIO_SPEED_FREQ_LOW;
  g.pull = HAL_GPIO_PULL_UP; g.output_type = HAL_GPIO_OUTPUT_PUSHPULL;
  g.init_state = HAL_GPIO_PIN_SET;                       /* CS idle high */
  if (HAL_GPIO_Init(IMU_CS_PORT, IMU_CS_PIN, &g) != HAL_OK) return 0;

  g.mode = HAL_GPIO_MODE_ALTERNATE; g.output_type = HAL_GPIO_OUTPUT_PUSHPULL;
  g.pull = HAL_GPIO_PULL_NO; g.speed = HAL_GPIO_SPEED_FREQ_MEDIUM;
  g.alternate = HAL_GPIO_AF_5;
  if (HAL_GPIO_Init(HAL_GPIOA, HAL_GPIO_PIN_5 | HAL_GPIO_PIN_6 | HAL_GPIO_PIN_7, &g) != HAL_OK) return 0;

  if (HAL_SPI_Init(&hspi1, HAL_SPI1) != HAL_OK) return 0;
  HAL_RCC_SPI1_EnableClock();
  if (HAL_RCC_SPI1_SetKernelClkSource(HAL_RCC_SPI1_CLK_SRC_PCLK2) != HAL_OK) return 0;

  hal_spi_config_t c;
  c.mode = HAL_SPI_MODE_MASTER; c.direction = HAL_SPI_DIRECTION_FULL_DUPLEX;
  c.data_width = HAL_SPI_DATA_WIDTH_8_BIT;
  c.clock_polarity = HAL_SPI_CLOCK_POLARITY_HIGH;  /* mode 3 */
  c.clock_phase = HAL_SPI_CLOCK_PHASE_2_EDGE;
  c.baud_rate_prescaler = HAL_SPI_BAUD_RATE_PRESCALER_32;
  c.first_bit = HAL_SPI_MSB_FIRST;
  c.nss_pin_management = HAL_SPI_NSS_PIN_MGMT_INTERNAL;
  if (HAL_SPI_SetConfig(&hspi1, &c) != HAL_OK) return 0;

  return 1;
}

static uint8_t imu_read_reg(uint8_t reg)
{
  uint8_t tx[2] = { (uint8_t)(reg | 0x80U), 0x00U };
  uint8_t rx[2] = { 0U, 0U };
  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, HAL_GPIO_PIN_RESET);
  g_diag.imu_status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2U, 100U);
  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, HAL_GPIO_PIN_SET);
  return rx[1];
}

/* ST LSM6DSV driver: 60 Hz, +/-2 g, +/-250 dps, coherent burst reads. */
volatile imu_sample_t g_imu = {.status=3};
static int imu_configured;
static int32_t imu_bus_read(void *ctx,uint8_t reg,uint8_t *data,uint16_t n) {
 (void)ctx;uint8_t tx[33]={0},rx[33]={0};if(n>32)return -1;
 tx[0]=reg|0x80;
 HAL_GPIO_WritePin(IMU_CS_PORT,IMU_CS_PIN,HAL_GPIO_PIN_RESET);
 hal_status_t ret=HAL_SPI_TransmitReceive(&hspi1,tx,rx,n+1,20);
 HAL_GPIO_WritePin(IMU_CS_PORT,IMU_CS_PIN,HAL_GPIO_PIN_SET);
 if(ret!=HAL_OK)return -1;memcpy(data,rx+1,n);return 0;
}
static int32_t imu_bus_write(void *ctx,uint8_t reg,const uint8_t *data,uint16_t n) {
 (void)ctx;uint8_t tx[33]={0},rx[33]={0};if(n>32)return -1;
 tx[0]=reg&0x7f;memcpy(tx+1,data,n);
 HAL_GPIO_WritePin(IMU_CS_PORT,IMU_CS_PIN,HAL_GPIO_PIN_RESET);
 hal_status_t ret=HAL_SPI_TransmitReceive(&hspi1,tx,rx,n+1,20);
 HAL_GPIO_WritePin(IMU_CS_PORT,IMU_CS_PIN,HAL_GPIO_PIN_SET);
 return ret==HAL_OK?0:-1;
}
static stmdev_ctx_t imu_ctx={.read_reg=imu_bus_read,.write_reg=imu_bus_write,.mdelay=HAL_Delay};
static void imu_configure(void) {
 g_imu.status=1;imu_configured=0;
 if(!g_diag.imu_ok)return;
 if(lsm6dsv_sw_reset(&imu_ctx) || lsm6dsv_auto_increment_set(&imu_ctx,1) ||
    lsm6dsv_block_data_update_set(&imu_ctx,1) ||
    lsm6dsv_xl_full_scale_set(&imu_ctx,LSM6DSV_2g) ||
    lsm6dsv_gy_full_scale_set(&imu_ctx,LSM6DSV_250dps) ||
    lsm6dsv_xl_data_rate_set(&imu_ctx,LSM6DSV_ODR_AT_60Hz) ||
    lsm6dsv_gy_data_rate_set(&imu_ctx,LSM6DSV_ODR_AT_60Hz))return;
 lsm6dsv_xl_full_scale_t xl;lsm6dsv_gy_full_scale_t gy;
 lsm6dsv_data_rate_t xr,gr;
 if(lsm6dsv_xl_full_scale_get(&imu_ctx,&xl)||lsm6dsv_gy_full_scale_get(&imu_ctx,&gy)||
    lsm6dsv_xl_data_rate_get(&imu_ctx,&xr)||lsm6dsv_gy_data_rate_get(&imu_ctx,&gr)||
    xl!=LSM6DSV_2g||gy!=LSM6DSV_250dps||xr!=LSM6DSV_ODR_AT_60Hz||gr!=LSM6DSV_ODR_AT_60Hz)return;
 imu_configured=1;g_imu.status=3;
}
void imu_poll(void) {
 static uint32_t last;uint32_t now=HAL_GetTick();
 if(!imu_configured || now-last<5)return;last=now;
 lsm6dsv_data_ready_t ready={0};
 if(lsm6dsv_flag_data_ready_get(&imu_ctx,&ready)){g_imu.status=2;return;}
 if(!ready.drdy_xl || !ready.drdy_gy)return;
 uint8_t data[14];
 if(imu_bus_read(0,LSM6DSV_OUT_TEMP_L,data,sizeof(data))){g_imu.status=2;return;}
 imu_sample_t sample={0};sample.sequence=g_imu.sequence+1;sample.timestamp_ms=now;
 sample.temperature=(int16_t)((uint16_t)data[0]|((uint16_t)data[1]<<8));
 for(unsigned i=0;i<3;i++) {
  sample.gyro[i]=(int16_t)((uint16_t)data[2+2*i]|((uint16_t)data[3+2*i]<<8));
  sample.accel[i]=(int16_t)((uint16_t)data[8+2*i]|((uint16_t)data[9+2*i]<<8));
 }
 g_imu=sample;
}

/* ============================ LiDAR (I3C1) ================================ */

/* Prefer Y1's 16 MHz crystal on MCO2 / PA9. The SYSCLK/4 fallback
   supports diagnosis but has not established ranging clock accuracy. */
static void lidar_apclock_init(void)
{
  hal_gpio_config_t g;
  HAL_RCC_GPIOA_EnableClock();
  g.mode = HAL_GPIO_MODE_ALTERNATE; g.output_type = HAL_GPIO_OUTPUT_PUSHPULL;
  g.pull = HAL_GPIO_PULL_NO; g.speed = HAL_GPIO_SPEED_FREQ_HIGH;
  g.alternate = HAL_GPIO_AF_0;                           /* MCO2 = AF0 */
  (void)HAL_GPIO_Init(HAL_GPIOA, HAL_GPIO_PIN_9, &g);
  g_diag.hse_status = HAL_RCC_HSE_Enable(HAL_RCC_HSE_ON);
  if (g_diag.hse_status == HAL_OK) {
    HAL_RCC_SetConfigMCO(HAL_RCC_MCO2_SRC_HSE, HAL_RCC_MCO2_PRESCALER1);
    g_diag.ap_clock_hz = 16000000U; /* Y1 = 16 MHz */
  } else {
    HAL_RCC_SetConfigMCO(HAL_RCC_MCO2_SRC_SYSCLK, HAL_RCC_MCO2_PRESCALER4);
    g_diag.ap_clock_hz = HAL_RCC_GetSYSCLKFreq() / 4U;
  }
}

/* ============================ application ================================= */
app_status_t app_init(void)
{
  hal_gpio_config_t g;
  g_diag.magic = 0U;

  /* PA2 is NC on this board. PA3/SYNC is directly connected to 1.8V U1.
     Release it open-drain: never drive this signal to 3.3V. R6 pulls to 1.8V. */
  HAL_RCC_GPIOA_EnableClock();
  g.mode = HAL_GPIO_MODE_OUTPUT; g.speed = HAL_GPIO_SPEED_FREQ_LOW;
  g.pull = HAL_GPIO_PULL_NO; g.output_type = HAL_GPIO_OUTPUT_OPENDRAIN;
  g.init_state = HAL_GPIO_PIN_SET;
  (void)HAL_GPIO_Init(HAL_GPIOA, HAL_GPIO_PIN_3, &g);
  g_diag.sync_level = HAL_GPIO_ReadPin(HAL_GPIOA, HAL_GPIO_PIN_3);

  /* LiDAR power-up ordering: make sure the AP clock is running, THEN assert
     XSHUT, so the VL53L9CX sees a clock as it comes out of reset. */
  g.mode = HAL_GPIO_MODE_OUTPUT; g.speed = HAL_GPIO_SPEED_FREQ_LOW;
  g.pull = HAL_GPIO_PULL_NO; g.output_type = HAL_GPIO_OUTPUT_PUSHPULL;
  g.init_state = HAL_GPIO_PIN_RESET;
  HAL_RCC_GPIOB_EnableClock();
  (void)HAL_GPIO_Init(LIDAR_SHDN_PORT, LIDAR_SHDN_PIN, &g);   /* XSHUT low */
  lidar_apclock_init();                                       /* AP clock on PA9 */
  HAL_Delay(50U);                                             /* conservative reset hold */
  HAL_GPIO_WritePin(LIDAR_SHDN_PORT, LIDAR_SHDN_PIN, HAL_GPIO_PIN_SET); /* XSHUT high */
  HAL_Delay(50U);                                             /* 50 ms after release */

  /* ---- IMU over SPI ---- */
  g_diag.spi_init_ok = (uint32_t)spi1_init();
  if (g_diag.spi_init_ok)
  {
    HAL_Delay(20U);
    (void)imu_read_reg(LSM6DSV_REG_WHOAMI);      /* 1st xfer selects SPI mode */
    g_diag.imu_whoami = imu_read_reg(LSM6DSV_REG_WHOAMI);
    g_diag.imu_ok = (g_diag.imu_status == HAL_OK && g_diag.imu_whoami == LSM6DSV_WHOAMI_VAL);
  }

  imu_configure();

  /* ---- LiDAR over I3C ---- */
  HAL_Delay(150U);                               /* let VL53L9CX reach READY-TO-BOOT */
  soft_i2c_init();
  g_diag.bus_idle = soft_i2c_lines();
  g_diag.soft_status = soft_i2c_probe(VL53L9_STATIC_ADDR);
  g_diag.soft_ack = (g_diag.soft_status == 0U);
  if (g_diag.soft_ack) {
    uint8_t id[4] = {0};
    g_diag.soft_status = soft_i2c_read(VL53L9_STATIC_ADDR, 0U, id, 4U);
    g_diag.soft_id = (uint32_t)id[0] | ((uint32_t)id[1] << 8)
                    | ((uint32_t)id[2] << 16) | ((uint32_t)id[3] << 24);
  }
  (void)lidar_bus_init();
  g_diag.i3c_init_ok=g_lidar_bus.mode==2;
  g_diag.i3c_kclk=g_lidar_bus.kernel_hz;
  g_diag.treg0=g_lidar_bus.timing0;g_diag.treg1=g_lidar_bus.timing1;
  g_diag.i3c_err=g_lidar_bus.error;
  if(g_diag.soft_status==0 && g_diag.soft_id==VL53L9_DEVICE_ID) {
    lidar_smoke_run(g_diag.ap_clock_hz);
    if(g_capture.state==3 && g_lidar_bus.mode) {
      lidar_bus_fallback();
      lidar_smoke_run(g_diag.ap_clock_hz);
    }
  }

  led_delay_ms = (g_diag.imu_ok && g_lidar.frames >= 1U &&
                 g_lidar.result == 0 && g_lidar.stop_result == 0) ? 80U : 400U;
  g_diag.magic = 0x5E450005U;
  (void)usb_stream_init(g_diag.imu_whoami,g_diag.soft_id);
  return EXEC_STATUS_INIT_OK;
}

app_status_t app_process(void)
{
  static uint32_t last_led;
  usb_stream_poll();
  lidar_capture_service();
  usb_stream_poll();
  if(HAL_GetTick()-last_led>=led_delay_ms){
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);last_led=HAL_GetTick();
  }
  g_diag.loops++;
  return EXEC_STATUS_OK;
}

app_status_t app_deinit(void)
{
  return EXEC_STATUS_OK;
}
