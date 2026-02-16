#include <stdio.h>

#include "sd_card.h"
#include "stm32n6570_discovery_sd.h"

uint32_t sd_buf1[NB_WORDS_TO_WRITE] __NON_CACHEABLE; 
uint32_t sd_buf2[NB_WORDS_TO_WRITE] __NON_CACHEABLE;

uint32_t *curr_buf = sd_buf1;
size_t buf_index = 0;
size_t SD_index = 0;

/**
* @brief  Save an encoded buffer fragment at the given offset.
* @param  offset
* @param  buf  pointer to the buffer to save
* @param  size  size (in bytes) of the buffer to save
* @retval err error code. 0 On success.
*/
int save_stream(uint32_t offset, uint32_t * buf, size_t size){
  int err = 0;
  size += 15; /* Alignment*/
  size = size / sizeof(uint32_t);
  
  for(int i = 0; i<size; i++){
    curr_buf[buf_index] = buf[i];
    buf_index++;
    /* upload to sd every 512 blocks to limit the impact of access latency */
    if(buf_index >= NB_WORDS_TO_WRITE){
      int32_t res = BSP_SD_WriteBlocks_DMA(0, curr_buf, SD_index, NB_BLOCKS_TO_WRITE);
      if(res!= BSP_ERROR_NONE){
        printf("%d\n", res);
        err = -1;
      }
      SD_index+=NB_BLOCKS_TO_WRITE;
      /* swap buffers */
      buf_index = 0;
      curr_buf = (curr_buf == sd_buf1)?sd_buf2:sd_buf1;
    }
  }
  return err;
}

int flush_out_buffer(void){
  if(BSP_SD_WriteBlocks(0, (uint32_t *) curr_buf, SD_index, NB_BLOCKS_TO_WRITE)!= BSP_ERROR_NONE){
        return -1;
      }
  return 0;
}

/**
* @brief  erases data in output medium
* @retval err error code. 0 On success.
*/
int erase_enc_output(void){
  /* Erase beginning of SDCard */
  if (BSP_SD_Erase(0, 0, NB_BLOCKS_ERASED) != BSP_ERROR_NONE)
  {
    printf("failed to erase external flash block nb \n");
    return -1;
  }
  return 0;
}

void SD_Card_Init(void)
{
  /* Initialize SD Card */
  if (BSP_SD_Init(0) != BSP_ERROR_NONE){
    printf("error initializing SD Card\n");
    while(1);
  }
  BSP_SD_CardInfo card_info;
  BSP_SD_GetCardInfo(0, &card_info);
  printf("SD card info : \nblock Nbr : %d\nblock size : %d\ncard speed : %d\n", card_info.BlockNbr, card_info.BlockSize, card_info.CardSpeed);

  /* erase output*/
  printf("erasing flash output blocks\n");
  erase_enc_output();
  printf("Done erasing output flash blocks\n");

#if USE_SD_AS_OUTPUT
  /* wait for erase operation to be done */
  while(BSP_SD_GetCardState(0) != SD_TRANSFER_OK);
#endif
  

}