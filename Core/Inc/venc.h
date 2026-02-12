#ifndef VENC_H
#define VENC_H

#include "stm32n6xx_hal.h"

#define VIDEO_FRAME_NB 600

int Encode_frame(uint32_t img_addr);
int encoder_prepare(uint32_t width, uint32_t height, int framerate, uint32_t * output_buffer);
int get_frame_nb(void);
int enc_end_reached();
int encoder_end(void);

#endif /* VENC_H */