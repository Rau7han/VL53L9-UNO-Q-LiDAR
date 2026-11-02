#include "stm32_hal.h"
#include "tusb.h"
#include "bootloader.h"

/* AN2606 rev70, Table 23: STM32C55xxx ROM vectors start at 0x0BF80080.
 * A reset restores clocks/peripherals first. No option bytes are modified. */
static volatile uint32_t request[2] __attribute__((section(".noinit.boot")));
void bootloader_request(void) {
  HAL_GPIO_WritePin(HAL_GPIOB,HAL_GPIO_PIN_7,HAL_GPIO_PIN_RESET);
  tud_disconnect();HAL_Delay(30);
  request[0]=0x4c394446U;request[1]=~0x4c394446U;
  __DSB();NVIC_SystemReset();
}
__attribute__((naked,noreturn)) static void enter_rom(uint32_t sp,uint32_t pc) {
  __asm volatile("msr msp, r0\n movs r2, #0\n msr control, r2\n msr basepri, r2\n isb\n cpsie i\n bx r1");
}
void bootloader_check(void) {
  if(request[0]!=0x4c394446U || request[1]!=~0x4c394446U)return;
  request[0]=0;request[1]=0;
  const uint32_t *vectors=(const uint32_t *)0x0BF80080U;
  if(vectors[0]<0x20000000U || vectors[0]>0x20020000U ||
     vectors[1]<0x0BF80081U || vectors[1]>=0x0BF88400U || !(vectors[1]&1U))return;
  __disable_irq();SysTick->CTRL=0;
  for(unsigned i=0;i<sizeof(NVIC->ICER)/sizeof(NVIC->ICER[0]);i++){
    NVIC->ICER[i]=0xffffffffU;NVIC->ICPR[i]=0xffffffffU;
  }
  SCB->VTOR=0x0BF80080U;__DSB();__ISB();
  enter_rom(vectors[0],vectors[1]);
}
