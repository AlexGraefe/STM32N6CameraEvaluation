 /**
 ******************************************************************************
 * @file    app_camerapipeline.c
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

#include <assert.h>
#include "cmw_camera.h"
#include "app_camerapipeline.h"
#include "app_config.h"
#include "crop_img.h"


/* Leave the driver use the default resolution */
#define CAMERA_WIDTH  0
#define CAMERA_HEIGHT 0
#define CAMERA_FPS 30

extern int32_t cameraFrameReceived;
extern uint8_t buf_index_changed;
extern uint32_t img_addr;

static void DCMIPP_PipeInitDisplay(CMW_CameraInit_t *camConf, uint32_t *bg_width, uint32_t *bg_height)
{
  CMW_Aspect_Ratio_Mode_t aspect_ratio;
  CMW_DCMIPP_Conf_t dcmipp_conf = {0};
  int ret;

  if (ASPECT_RATIO_MODE == ASPECT_RATIO_CROP)
  {
    aspect_ratio = CMW_Aspect_ratio_crop;
  }
  else if (ASPECT_RATIO_MODE == ASPECT_RATIO_FIT)
  {
    aspect_ratio = CMW_Aspect_ratio_fit;
  }
  else if (ASPECT_RATIO_MODE == ASPECT_RATIO_FULLSCREEN)
  {
    aspect_ratio = CMW_Aspect_ratio_fullscreen;
  }

  int lcd_bg_width;
  int lcd_bg_height;

  lcd_bg_height = (camConf->height <= SCREEN_HEIGHT) ? camConf->height : SCREEN_HEIGHT;

#if ASPECT_RATIO_MODE == ASPECT_RATIO_FULLSCREEN
  lcd_bg_width = (((camConf->width*lcd_bg_height)/camConf->height) - ((camConf->width*lcd_bg_height)/camConf->height) % 16);
#else
  lcd_bg_width = (camConf->height <= SCREEN_HEIGHT) ? camConf->height : SCREEN_HEIGHT;
#endif

  *bg_width = lcd_bg_width;
  *bg_height = lcd_bg_height;

  dcmipp_conf.output_width = lcd_bg_width;
  dcmipp_conf.output_height = lcd_bg_height;
  dcmipp_conf.output_format = DCMIPP_PIXEL_PACKER_FORMAT_RGB565_1;
  dcmipp_conf.output_bpp = 2;
  dcmipp_conf.mode = aspect_ratio;
  dcmipp_conf.enable_gamma_conversion = 0;
  uint32_t pitch;
  // first pipeconfiguration is for the display.
  ret = CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE2, &dcmipp_conf, &pitch);
  printf("Display pipe configured: output %lux%lu, format %d, bpp %d, mode %d\r\n",
         dcmipp_conf.output_width,
         dcmipp_conf.output_height,
         dcmipp_conf.output_format,
         dcmipp_conf.output_bpp,
         dcmipp_conf.mode);
  assert(ret == HAL_OK);
  assert(dcmipp_conf.output_width * dcmipp_conf.output_bpp == pitch);
}

static void DCMIPP_PipeInitSecondary(int width, int height)
{
  CMW_Aspect_Ratio_Mode_t aspect_ratio;
  CMW_DCMIPP_Conf_t dcmipp_conf;
  int ret;

  if (ASPECT_RATIO_MODE == ASPECT_RATIO_CROP)
  {
    aspect_ratio = CMW_Aspect_ratio_crop;
  }
  else if (ASPECT_RATIO_MODE == ASPECT_RATIO_FIT)
  {
    aspect_ratio = CMW_Aspect_ratio_fit;
  }
  else if (ASPECT_RATIO_MODE == ASPECT_RATIO_FULLSCREEN)
  {
    aspect_ratio = CMW_Aspect_ratio_fit;
  }

  dcmipp_conf.output_width = width;
  dcmipp_conf.output_height = height;
  dcmipp_conf.output_format = DCMIPP_PIXEL_PACKER_FORMAT_RGB565_1;  // DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1;
  dcmipp_conf.output_bpp = 2;
  dcmipp_conf.mode = aspect_ratio;
  dcmipp_conf.enable_swap = COLOR_MODE;
  dcmipp_conf.enable_gamma_conversion = 0;
  uint32_t pitch;
  ret = CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE1, &dcmipp_conf, &pitch);
  assert(ret == HAL_OK);
}

/**
* @brief Init the camera and the 2 DCMIPP pipes
* @param lcd_bg_width display width
* @param lcd_bg_height display height
*/
void CameraPipeline_Init(uint32_t *lcd_bg_width, uint32_t *lcd_bg_height, int secondary_pipe_width, int secondary_pipe_height)
{
  int ret;
  CMW_CameraInit_t cam_conf;

  cam_conf.width = CAMERA_WIDTH;
  cam_conf.height = CAMERA_HEIGHT;
  cam_conf.fps = CAMERA_FPS;
  cam_conf.mirror_flip = CAMERA_FLIP;

  // this determines the connected camera sensor
  ret = CMW_CAMERA_Init(&cam_conf, NULL);
  assert(ret == CMW_ERROR_NONE);

  printf("Camera initialized: %lux%lu @ %d fps\r\n",
         cam_conf.width,
         cam_conf.height,
         cam_conf.fps);

  DCMIPP_PipeInitDisplay(&cam_conf, lcd_bg_width, lcd_bg_height);
  // DCMIPP_PipeInitSecondary(secondary_pipe_width, secondary_pipe_height);
}

void CameraPipeline_DeInit(void)
{
  int ret;
  ret = CMW_CAMERA_DeInit();
  assert(ret == CMW_ERROR_NONE);
}

void CameraPipeline_DisplayPipe_Start(uint8_t *display_pipe_dst, uint32_t cam_mode)
{
  int ret;
  ret = CMW_CAMERA_Start(DCMIPP_PIPE2, display_pipe_dst, cam_mode);
  assert(ret == CMW_ERROR_NONE);
}

void CameraPipeline_SecondaryPipe_Start(uint8_t *secondary_pipe_dst1, uint8_t *secondary_pipe_dst2, uint32_t cam_mode)
{
  int ret;
  ret = CMW_CAMERA_DoubleBufferStart(DCMIPP_PIPE1, secondary_pipe_dst1, secondary_pipe_dst2, cam_mode);
  // ret = CMW_CAMERA_Start(DCMIPP_PIPE1, secondary_pipe_dst, cam_mode);
  assert(ret == CMW_ERROR_NONE);
}

void CameraPipeline_DisplayPipe_Stop()
{
  int ret;
  ret = CMW_CAMERA_Suspend(DCMIPP_PIPE2);
  assert(ret == CMW_ERROR_NONE);
}

void CameraPipeline_IspUpdate(void)
{
  int ret = CMW_ERROR_NONE;
  ret = CMW_CAMERA_Run();
  assert(ret == CMW_ERROR_NONE);
}

/**
  * @brief  Frame event callback
  * @param  hdcmipp pointer to the DCMIPP handle
  * @retval None
  */
int CMW_CAMERA_PIPE_FrameEventCallback(uint32_t pipe)
{
  switch (pipe)
  {
    case DCMIPP_PIPE2 :
      cameraFrameReceived++;
      break;
    case DCMIPP_PIPE1 :
      cameraFrameReceived++;
      img_addr = DCMIPP->P1STM0AR;
      // printf("%d, %d\n", buf_index_changed, cameraFrameReceived);
      buf_index_changed = 1;
      break;
      
  }
  return 0;
}

void CMW_CAMERA_PIPE_ErrorCallback(uint32_t pipe)
{
  /* FIXME : Need to tune sensor/ipplug so we can remove this implementation */
}
