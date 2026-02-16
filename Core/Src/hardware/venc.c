#include "h264encapi.h"

#include "venc.h"
#include "sd_card.h"

H264EncIn encIn= {0};
H264EncOut encOut= {0};
H264EncInst encoder= {0};
H264EncConfig cfg= {0};
uint32_t output_size = 0;
EWLLinearMem_t outbuf;
static int frame_nb = 0;

int Encode_frame(uint32_t img_addr){
  int ret = H264ENC_FRAME_READY;
  if(!img_addr){
    printf("Error : NULL image address");
    return -1;
  }
  if (!frame_nb)
  {
    /* if frame is the first : set as intra coded */
    encIn.timeIncrement = 0;
    encIn.codingType = H264ENC_INTRA_FRAME;
  }
  else
  {
    /* if there was a frame previously, set as predicted */
    encIn.timeIncrement = 1;
    encIn.codingType = H264ENC_PREDICTED_FRAME;
  }
  encIn.ipf = H264ENC_REFERENCE_AND_REFRESH;
  encIn.ltrf = H264ENC_REFERENCE;
  /* set input buffers to structures */
  encIn.busLuma = img_addr;
  ret = H264EncStrmEncode(encoder, &encIn, &encOut, NULL, NULL, NULL);
  switch (ret)
  {
  case H264ENC_FRAME_READY:
    /*save stream */
    // printf("encoded frame %d - size : %d bytes\n", frame_nb, encOut.streamSize);

    if (save_stream(output_size, encIn.pOutBuf,  encOut.streamSize))
    {
      printf("error saving stream frame %d\n", frame_nb);
      return -1;
    }
    // for (int i = 0; i < encOut.streamSize; i++) {
    //   printf("%02x, ", ((uint8_t*)encIn.pOutBuf)[i]);
    //   if ((i + 1) % 16 == 0) printf("\n");
    // }
    // printf("\n");
    // while (1) {};
    output_size += encOut.streamSize;
    break;
  case H264ENC_SYSTEM_ERROR:
    printf("fatal error while encoding\n");
    break;
  default:
    printf("error encoding frame %d : %d\n", frame_nb, ret);
    break;
  }
  frame_nb++;
  return 0;
}


int encoder_prepare(uint32_t width, uint32_t height, int framerate, uint32_t * output_buffer)
{
  H264EncRet ret;

  H264EncPreProcessingCfg preproc_cfg = {0};

  /* software workaround for Linemult triggering VENC interrupt. Make it happen as little as possible */
  MODIFY_REG(DCMIPP->P1PPCR, DCMIPP_P1PPCR_LINEMULT_Msk,DCMIPP_MULTILINE_128_LINES);

  frame_nb = 0;
  /* Step 1: Initialize an encoder instance */
  /* set config to 1 ref frame */
  cfg.refFrameAmount = 1;
  /* 30 fps frame rate */
  cfg.frameRateDenom = 1;
  cfg.frameRateNum = framerate;
  /* Image resolution */
  cfg.width = width;
  cfg.height = height;
  /* Stream type */
  cfg.streamType = H264ENC_BYTE_STREAM;

  /* encoding level*/
  /*See API guide for level depending on resolution and framerate*/
  cfg.level = H264ENC_LEVEL_2_2;
  cfg.svctLevel = 0;

  /* Output buffer size */
  outbuf.size = cfg.width * cfg.height;

  ret = H264EncInit(&cfg, &encoder);
  if (ret != H264ENC_OK)
  {
    printf("error initializing encoder %d\n", ret);
    return -1;
  }

  /* set format conversion for preprocessing */
  ret = H264EncGetPreProcessing(encoder, &preproc_cfg);
  if(ret != H264ENC_OK){
    printf("error getting preproc data\n");
    return -1;
  }
  preproc_cfg.inputType = H264ENC_RGB565;
  ret = H264EncSetPreProcessing(encoder, &preproc_cfg);
  if(ret != H264ENC_OK){
    printf("error setting preproc data\n");
    return -1;
  }

  /*assign buffers to input structure */
  encIn.pOutBuf = output_buffer;
  encIn.busOutBuf = (uint32_t) output_buffer;
  encIn.outBufSize = width * height/2;

  /* create stream */
  ret = H264EncStrmStart(encoder, &encIn, &encOut);
  if (ret != H264ENC_OK)
  {
    printf("error starting stream %d\n", ret);
    return -1;
  }
  printf("stream started. saved %d bytes\n", encOut.streamSize);
    //   for (int i = 0; i < encOut.streamSize; i++) {
    //   printf("%02x, ", ((uint8_t*)encIn.pOutBuf)[i]);
    //   if ((i + 1) % 16 == 0) printf("\n");
    // }
  /* save the stream header */
  printf("%d\n", encOut.streamSize);
  if (save_stream(output_size, encIn.pOutBuf,  encOut.streamSize))
  {
    printf("error saving stream\n");
    return -1;
  }
  printf("stream started. saved %d bytes\n", encOut.streamSize);
  output_size+= encOut.streamSize;
  return 0;
}

int encoder_end(void){
  int ret = H264EncStrmEnd(encoder, &encIn, &encOut);
  printf("done encoding %d frames. size : %d - Blocks : %d\n",frame_nb ,output_size, (output_size+511)/512);
  if (ret != H264ENC_OK)
  {
    printf("error ending encoder %d\n", ret);
    return -1;
  }
  else
  {
    /* save stream tail */
    if (save_stream(output_size, encIn.pOutBuf,  encOut.streamSize))
    {
      printf("error saving stream\n");
      return -1;
    }
    output_size+=encOut.streamSize;
  }

  return 0;
}

int enc_end_reached(){
  return (frame_nb >= VIDEO_FRAME_NB);
}

int get_frame_nb(void){
  return frame_nb;
}
