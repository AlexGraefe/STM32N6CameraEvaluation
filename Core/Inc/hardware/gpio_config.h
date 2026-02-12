#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

#include "stm32n6xx_hal.h"

// LEDS
#define RED_LED_GPIO_Port GPIOG 
#define RED_LED_Pin GPIO_PIN_10

#define GREEN_LED_Pin GPIO_PIN_1
#define GREEN_LED_GPIO_Port GPIOO

// Buttons
#define USER1_BUTTON_Pin GPIO_PIN_13
#define USER1_BUTTON_GPIO_Port GPIOC

void GPIO_Config();

#endif /* GPIO_CONFIG_H */