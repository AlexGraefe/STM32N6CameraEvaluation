 /**
 ******************************************************************************
 * @file    main.h
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAIN_H
#define MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32n6xx_hal.h"

// LEDS
#define RED_LED_GPIO_Port GPIOG 
#define RED_LED_Pin GPIO_PIN_10

#define GREEN_LED_Pin GPIO_PIN_1
#define GREEN_LED_GPIO_Port GPIOO

// Buttons
#define USER1_BUTTON_Pin GPIO_PIN_13
#define USER1_BUTTON_GPIO_Port GPIOC

#endif /* MAIN_H */

