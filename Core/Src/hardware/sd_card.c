#include "sd_card.h"
#include "stm32n6570_discovery_sd.h"

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
  printf("SD_CARD_INIT START========================================\n");
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

  printf("SD_CARD_INIT END========================================\n");

#if USE_SD_AS_OUTPUT
  /* wait for erase operation to be done */
  while(BSP_SD_GetCardState(0) != SD_TRANSFER_OK);
#endif
  

}