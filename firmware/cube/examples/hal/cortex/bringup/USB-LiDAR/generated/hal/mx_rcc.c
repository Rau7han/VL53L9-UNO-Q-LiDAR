/**
  ******************************************************************************
  * @file           : mx_rcc.c
  * @brief          : STM32 RCC program body (USB-LiDAR-VL53L9 bring-up)
  ******************************************************************************
  * @attention
  *
  * Bring-up clock configuration.
  *
  * Run from verified Y1/HSE (16 MHz) through PSI at the rated 144 MHz.
  * PCLK1 is 72 MHz; USB uses HSIDIV3 at 48 MHz with USB SOF trimming.
  * If HSE does not start, retain the reset clock and update core timing.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mx_rcc.h"

/******************************************************************************/
/* Exported functions for RCC in HAL layer                                    */
/******************************************************************************/

system_status_t mx_rcc_init(void)
{
  /* Y1 is verified at 16 MHz. Use ST's rated 144 MHz PSI configuration
     for GPIO transport throughput, while USB remains on HSIDIV3 at 48 MHz. */
  if (HAL_RCC_HSE_Enable(HAL_RCC_HSE_ON) != HAL_OK)
    return HAL_UpdateCoreClock()==HAL_OK ? SYSTEM_OK : SYSTEM_CLOCK_ERROR;
  hal_rcc_psi_config_t psi = {
    .psi_source=HAL_RCC_PSI_SRC_HSE,
    .psi_ref=HAL_RCC_PSI_REF_16MHZ,
    .psi_out=HAL_RCC_PSI_OUT_144MHZ
  };
  if(HAL_RCC_PSI_SetConfig(&psi)!=HAL_OK || HAL_RCC_PSIS_Enable()!=HAL_OK)
    return SYSTEM_CLOCK_ERROR;
  hal_rcc_bus_clk_config_t bus = {
    .hclk_prescaler=HAL_RCC_HCLK_PRESCALER1,
    .pclk1_prescaler=HAL_RCC_PCLK_PRESCALER2,
    .pclk2_prescaler=HAL_RCC_PCLK_PRESCALER1,
    .pclk3_prescaler=HAL_RCC_PCLK_PRESCALER1
  };
  if(HAL_RCC_SetBusClockConfig(&bus)!=HAL_OK)return SYSTEM_CLOCK_ERROR;
  HAL_FLASH_ITF_SetLatency(HAL_FLASH,HAL_FLASH_ITF_LATENCY_4);
  if(HAL_RCC_SetSYSCLKSource(HAL_RCC_SYSCLK_SRC_PSIS)!=HAL_OK)return SYSTEM_CLOCK_ERROR;
  HAL_FLASH_ITF_SetProgrammingDelay(HAL_FLASH,HAL_FLASH_ITF_PROGRAM_DELAY_2);
  return HAL_UpdateCoreClock()==HAL_OK ? SYSTEM_OK : SYSTEM_CLOCK_ERROR;
}

void mx_rcc_deinit(void)
{
  HAL_RCC_Reset();
}

system_status_t mx_rcc_peripherals_clock_config(void)
{
  return SYSTEM_OK;
}
