/* Cycle-timed open-drain transport. External pull-ups only.
 * Return 0 on success, 1 on NACK, 2 on stuck/clock-stretched bus timeout.
 * This deliberately bypasses the peripheral and its alternate-function mux. */
#include "stm32_hal.h"
#include "soft_i2c.h"

/* Cycle-timed phases at the actual CPU frequency; use Fast-mode Plus minimum phase times. GPIO register access avoids per-edge HAL call overhead. */
static uint32_t low_cycles, high_cycles;
static int recovery_timing;
void soft_i2c_relax(void){ recovery_timing=1; }
static inline __attribute__((always_inline)) void cycles(uint32_t n) {
  uint32_t start=DWT->CYCCNT;
  while((uint32_t)(DWT->CYCCNT-start)<n) { __NOP(); }
}
static inline __attribute__((always_inline)) void delay(void) { cycles(low_cycles); }
static inline __attribute__((always_inline)) void sda(int high) { GPIOA->BSRR=high ? (1U<<8) : (1U<<24); }
static inline __attribute__((always_inline)) void scl(int high) { GPIOB->BSRR=high ? (1U<<4) : (1U<<20); }
static inline __attribute__((always_inline)) int rd_sda(void) { return (GPIOA->IDR & (1U<<8)) != 0; }
static inline __attribute__((always_inline)) int high_scl(void) {
  scl(1);
  if(!(GPIOB->IDR & (1U<<4))) {
    uint32_t start=HAL_GetTick();
    while(!(GPIOB->IDR & (1U<<4)))
      if(HAL_GetTick()-start>=5U)return 2;
  }
  cycles(high_cycles);
  return 0;
}
static int stop(void) {
  scl(0); sda(0); delay();
  int ret = high_scl(); sda(1); delay(); return ret;
}
static int start(void) {
  sda(1);
  if (high_scl() || !rd_sda()) return 2;
  sda(0); delay(); scl(0); return 0;
}
static int put(uint8_t b) {
  for (unsigned i=0;i<8;i++) {
    sda((b & 0x80U) != 0); b <<= 1; delay();
    if (high_scl()) return 2;
    scl(0);
  }
  sda(1); delay(); if (high_scl()) return 2;
  int nack = rd_sda(); scl(0); return nack;
}
void soft_i2c_init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  uint32_t hz=HAL_RCC_GetSYSCLKFreq();
  low_cycles=(hz+1999999U)/2000000U; /* >= 500 ns */
  high_cycles=(hz/1000000U)*27U/100U+1U; /* >= 270 ns */
  if(recovery_timing){
    low_cycles=(hz/1000000U)*13U/10U+1U; /* >= 1.3 us */
    high_cycles=(hz/1000000U)*7U/10U+1U; /* >= 0.7 us */
  }
  hal_gpio_config_t g = {0};
  g.mode=HAL_GPIO_MODE_OUTPUT; g.output_type=HAL_GPIO_OUTPUT_OPENDRAIN;
  g.pull=HAL_GPIO_PULL_NO; g.speed=HAL_GPIO_SPEED_FREQ_LOW; g.init_state=HAL_GPIO_PIN_SET;
  HAL_GPIO_Init(HAL_GPIOA, HAL_GPIO_PIN_8, &g);
  HAL_GPIO_Init(HAL_GPIOB, HAL_GPIO_PIN_4, &g);
  delay();
}
uint32_t soft_i2c_lines(void) {
  return (uint32_t)rd_sda() | ((HAL_GPIO_ReadPin(HAL_GPIOB,HAL_GPIO_PIN_4)==HAL_GPIO_PIN_SET) ? 2U : 0U);
}
int soft_i2c_probe(uint8_t address) {
  int ret=start(); if (!ret) ret=put(address<<1);
  int end=stop(); return ret ? ret : end;
}
int soft_i2c_read(uint8_t address, uint16_t reg, uint8_t *data, uint32_t size) {
  int ret=start();
  if (!ret) ret=put(address<<1);
  if (!ret) ret=put(reg>>8);
  if (!ret) ret=put(reg);
  int end=stop(); if (ret || end) return ret ? ret : end;
  ret=start(); if (!ret) ret=put((address<<1)|1U);
  for(uint32_t n=0;!ret && n<size;n++) {
    uint8_t b=0; sda(1);
    for(unsigned i=0;i<8;i++) {
      delay(); ret=high_scl(); if(ret) break;
      b=(uint8_t)((b<<1)|rd_sda()); scl(0);
    }
    if(ret) break;
    data[n]=b; sda(n==size-1); delay(); ret=high_scl(); scl(0); sda(1);
  }
  end=stop(); return ret ? ret : end;
}
int soft_i2c_write(uint8_t address, uint16_t reg, const uint8_t *data, uint32_t size) {
  int ret=start();
  if (!ret) ret=put(address<<1);
  if (!ret) ret=put(reg>>8);
  if (!ret) ret=put(reg);
  for(uint32_t n=0;!ret && n<size;n++) ret=put(data[n]);
  int end=stop(); return ret ? ret : end;
}
