/**
  ******************************************************************************
  * @file           : mx_gpio_default.c
  * @brief          : gpio_default Peripheral initialization (USB-LiDAR-VL53L9)
  ******************************************************************************
  * @attention
  *
  * Pin map for this board (STM32C552KEU6, UFQFPN32):
  *   PB8  -> LED_ONn     (status LED, ACTIVE-LOW: drive LOW = LED on)
  *   PB7  -> LIDAR_SHDN  (VL53L9 enable, drive HIGH = powered/awake)
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mx_gpio_default.h"

/******************************************************************************/
/* Exported functions for GPIO in HAL layer                                   */
/******************************************************************************/
system_status_t mx_gpio_default_init(void)
{
  hal_gpio_config_t gpio_config;

  HAL_RCC_GPIOB_EnableClock();

  /* PB8 = LED_ONn (active low). Start HIGH so the LED is OFF at boot. */
  gpio_config.mode        = HAL_GPIO_MODE_OUTPUT;
  gpio_config.speed       = HAL_GPIO_SPEED_FREQ_LOW;
  gpio_config.pull        = HAL_GPIO_PULL_NO;
  gpio_config.output_type = HAL_GPIO_OUTPUT_PUSHPULL;
  gpio_config.init_state  = HAL_GPIO_PIN_SET;       /* LED off */
  if (HAL_GPIO_Init(HAL_GPIOB, HAL_GPIO_PIN_8, &gpio_config) != HAL_OK)
  {
    return SYSTEM_PERIPHERAL_ERROR;
  }

  /* Hold XSHUT low until app_init has started the AP clock. */
  gpio_config.mode        = HAL_GPIO_MODE_OUTPUT;
  gpio_config.speed       = HAL_GPIO_SPEED_FREQ_LOW;
  gpio_config.pull        = HAL_GPIO_PULL_NO;
  gpio_config.output_type = HAL_GPIO_OUTPUT_PUSHPULL;
  gpio_config.init_state  = HAL_GPIO_PIN_RESET;
  if (HAL_GPIO_Init(HAL_GPIOB, HAL_GPIO_PIN_7, &gpio_config) != HAL_OK)
  {
    return SYSTEM_PERIPHERAL_ERROR;
  }

  return SYSTEM_OK;
}

system_status_t mx_gpio_default_deinit(void)
{
  HAL_GPIO_DeInit(HAL_GPIOB, HAL_GPIO_PIN_8);
  HAL_GPIO_DeInit(HAL_GPIOB, HAL_GPIO_PIN_7);
  return SYSTEM_OK;
}
