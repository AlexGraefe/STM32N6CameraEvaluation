 /**
 ******************************************************************************
 * @file    app_camerapipeline.h
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
#ifndef APP_CAMERAPIPELINE
#define APP_CAMERAPIPELINE

#define SCREEN_HEIGHT (480)
#define SCREEN_WIDTH  (800)

void CameraPipeline_Init(uint32_t *lcd_bg_width, uint32_t *lcd_bg_height, int secondary_pipe_width, int secondary_pipe_height);
void CameraPipeline_DeInit(void);
void CameraPipeline_Start(void);
void CameraPipeline_DisplayPipe_Start(uint8_t *display_pipe_dst, uint32_t cam_mode);
void CameraPipeline_DisplayPipe_Stop(void);
void CameraPipeline_SecondaryPipe_Start(uint8_t *secondary_pipe_dst1, uint8_t *secondary_pipe_dst2, uint32_t cam_mode);
void CameraPipeline_IspUpdate(void);

#endif