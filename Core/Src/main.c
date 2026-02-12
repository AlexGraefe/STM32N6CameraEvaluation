 /**
 ******************************************************************************
 * @file    main.c
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
#include <string.h>
#include <unistd.h>

#include "console_config.h"
#include "gpio_config.h"
#include "iac_config.h"
#include "mylcd.h"
#include "mpu_config.h"
#include "sd_card.h"
#include "security_config.h"
#include "system_clock_config.h"
#include "venc.h"

#include "cmw_camera.h"
#include "stm32n6570_discovery_bus.h"
#include "stm32n6570_discovery_lcd.h"
#include "stm32n6570_discovery_xspi.h"
#include "stm32n6570_discovery_sd.h"
#include "stm32n6570_discovery.h"
#include "stm32_lcd.h"
#include "app_fuseprogramming.h"
#include "stm32_lcd_ex.h"

#include "h264encapi.h"
#include "stm32n6xx_ll_venc.h"

#include "app_camerapipeline.h"
#include "main.h"
#include <stdio.h>
#include "app_config.h"
#include "crop_img.h"
#include "stlogo.h"

CLASSES_TABLE;

#ifndef APP_GIT_SHA1_STRING
#define APP_GIT_SHA1_STRING "dev"
#endif
#ifndef APP_VERSION_STRING
#define APP_VERSION_STRING "unversioned"
#endif


// venc parts
#define FRAMERATE 30
#define VENC_WIDTH    300
#define VENC_HEIGHT   300
uint16_t * pipe_buffer[2];
volatile uint8_t buf_index_changed = 0;
uint32_t img_addr = 0;

volatile uint32_t output_buffer[VENC_WIDTH*VENC_HEIGHT/8] __attribute__((aligned(8))); // __NON_CACHEABLE __attribute__((aligned(8)));


volatile int32_t cameraFrameReceived;
void* pp_input;

#define ALIGN_TO_16(value) (((value) + 15) & ~15)

// // __attribute__ ((section (".psram_bss")))
__attribute__ ((aligned (32)))
uint8_t secondary_pipe_buffer1[300 * 300 * 3]; // needs to be aligned on 32 bytes for DCMIPP output buffer

// // __attribute__ ((section (".psram_bss")))
__attribute__ ((aligned (32)))
uint8_t secondary_pipe_buffer2[300 * 300 * 3]; // needs to be aligned on 32 bytes for DCMIPP output buffer

extern DCMIPP_HandleTypeDef hcamera_dcmipp;

static void Hardware_init(void);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  Hardware_init();

  /*** App header *************************************************************/
  printf("========================================\n");
  printf("STM32N6-GettingStarted-ObjectDetection %s (%s)\n", APP_VERSION_STRING, APP_GIT_SHA1_STRING);
  printf("Build date & time: %s %s\n", __DATE__, __TIME__);
  #if defined(__GNUC__)
  printf("Compiler: GCC %d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
#elif defined(__ICCARM__)
  printf("Compiler: IAR EWARM %d.%d.%d\n", __VER__ / 1000000, (__VER__ / 1000) % 1000 ,__VER__ % 1000);
#else
  printf("Compiler: Unknown\n");
#endif
  printf("HAL: %lu.%lu.%lu\n", __STM32N6xx_HAL_VERSION_MAIN, __STM32N6xx_HAL_VERSION_SUB1, __STM32N6xx_HAL_VERSION_SUB2);
  printf("========================================\n\n");

  PRINTF_START("Camera Init");
  CameraPipeline_Init(&(get_lcd_lcd_bg_area()->XSize), &(get_lcd_lcd_bg_area()->YSize), VENC_WIDTH, VENC_HEIGHT);
  PRINTF_END("Camera Init");

  PRINTF_START("LCD Init");
  LCD_init();
  PRINTF_END("LCD Init");

  PRINTF_START("VENC Init");
  LL_VENC_Init();
  encoder_prepare(VENC_WIDTH, VENC_HEIGHT, FRAMERATE, output_buffer);
  PRINTF_END("VENC Init");

  CameraPipeline_DisplayPipe_Start(get_lcd_bg_buffer(), CMW_MODE_CONTINUOUS);
  CameraPipeline_SecondaryPipe_Start(secondary_pipe_buffer1, secondary_pipe_buffer2, CMW_MODE_CONTINUOUS);

  while (!enc_end_reached())
  {
    // if(BSP_CAMERA_BackgroundProcess() != BSP_ERROR_NONE)
    // {
    //   printf("Error in BSP image processing\n");
    // }
    CameraPipeline_IspUpdate();
    while (!buf_index_changed) {}
    /* new frame available */
    buf_index_changed = 0;
    Encode_frame(img_addr);
  }
  /* after encoding a certain nb of frames, end program */
  encoder_end();
  flush_out_buffer();

    while (1)
    {
      CameraPipeline_IspUpdate();
        // check if both are finished
        while (cameraFrameReceived < 2) {};
        cameraFrameReceived = 0;
    }


  // while (1)
  // {
  //   CameraPipeline_IspUpdate();
  //     /* Start NN camera single capture Snapshot */
  //     CameraPipeline_SecondaryPipe_Start(secondary_pipe_buffer1, secondary_pipe_buffer2, CMW_MODE_SNAPSHOT);
  //     SCB_CleanInvalidateDCache_by_Addr(secondary_pipe_buffer1, 300 * 300 * 3);  
  //     SCB_CleanInvalidateDCache_by_Addr(secondary_pipe_buffer2, 300 * 300 * 3);  
  //     // TODO: Invalidate cache?

  //     CameraPipeline_DisplayPipe_Start(get_lcd_bg_buffer(), CMW_MODE_SNAPSHOT);

  //     // check if both are finished
  //     while (cameraFrameReceived < 2) {};
  //     cameraFrameReceived = 0;

    
  //   while (HAL_GPIO_ReadPin(USER1_BUTTON_GPIO_Port, USER1_BUTTON_Pin) == GPIO_PIN_SET){
  //     HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
  //     HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
  //     HAL_Delay(10);
  //   }
  //   HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
  //   HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
  // }

}

static void Hardware_init(void)
{
  /* enable MPU configuration to create non cacheable sections */
  MPU_Config();

  /* Power on ICACHE */
  MEMSYSCTL->MSCR |= MEMSYSCTL_MSCR_ICACTIVE_Msk;

  /* Set back system and CPU clock source to HSI */
  __HAL_RCC_CPUCLK_CONFIG(RCC_CPUCLKSOURCE_HSI);
  __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);

  HAL_Init();

  SCB_EnableICache();

#if defined(USE_DCACHE)
  /* Power on DCACHE */
  MEMSYSCTL->MSCR |= MEMSYSCTL_MSCR_DCACTIVE_Msk;
  SCB_EnableDCache();
#endif

  SystemClock_Config();

  GPIO_Config();

  CONSOLE_Config();
  printf("\n\n\n\n");

  Fuse_Programming();

  /*** External RAM and NOR Flash *********************************************/
  PRINTF_START("External Memory Init");
  BSP_XSPI_RAM_Init(0);
  BSP_XSPI_RAM_EnableMemoryMappedMode(0);

  BSP_XSPI_NOR_Init_t NOR_Init;
  NOR_Init.InterfaceMode = BSP_XSPI_NOR_OPI_MODE;
  NOR_Init.TransferRate = BSP_XSPI_NOR_DTR_TRANSFER;
  BSP_XSPI_NOR_Init(0, &NOR_Init);
  BSP_XSPI_NOR_EnableMemoryMappedMode(0);
  PRINTF_END("External Memory Init");

  // init SD card
  PRINTF_START("SD Card Init");
  // SD_Card_Init();
  PRINTF_END("SD Card Init");

  /* Set all required IPs as secure privileged */
  Security_Config();

  IAC_Config();
  set_clk_sleep_mode();

}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  UNUSED(file);
  UNUSED(line);
  __BKPT(0);
  while (1)
  {
  }
}

#endif
