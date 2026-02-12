#include "gpio_config.h"

void GPIO_Config()
{
  GPIO_InitTypeDef GPIO_InitStruct_green_led = {0};
  GPIO_InitTypeDef GPIO_InitStruct_red_led = {0};
  GPIO_InitTypeDef GPIO_InitStruct_user1_button = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOO_CLK_ENABLE();

  // Configure green led
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct_green_led.Pin = GREEN_LED_Pin;
  GPIO_InitStruct_green_led.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct_green_led.Pull = GPIO_NOPULL;
  GPIO_InitStruct_green_led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct_green_led);

  // Configure red led
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct_red_led.Pin = RED_LED_Pin;
  GPIO_InitStruct_red_led.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct_red_led.Pull = GPIO_NOPULL;
  GPIO_InitStruct_red_led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct_red_led);

  // Configure user button
  GPIO_InitStruct_user1_button.Pin = USER1_BUTTON_Pin;
  GPIO_InitStruct_user1_button.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct_user1_button.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct_user1_button.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER1_BUTTON_GPIO_Port, &GPIO_InitStruct_user1_button);
}