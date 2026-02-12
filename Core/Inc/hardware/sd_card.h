#ifndef SD_CARD_H
#define SD_CARD_H

#include "stm32n6xx_hal.h"

/* Each write = 512 Blocks of 512 Bytes */
#define BLOCK_SIZE         (512) /* 1 Block = 512 Bytes */
#define NB_BLOCKS_TO_WRITE (128)  /* Write 128 Blocks at once*/
#define NB_BYTES_TO_WRITE (NB_BLOCKS_TO_WRITE*BLOCK_SIZE)
#define NB_WORDS_TO_WRITE (NB_BYTES_TO_WRITE/4)
#define NB_BLOCKS_ERASED  (1024*1024) /*Erase 512 MBytes = 512*1024*1024 */

int save_stream(uint32_t offset, uint32_t * buf, size_t size);
int flush_out_buffer(void);
int erase_enc_output(void);
void SD_Card_Init(void);

#endif /* SD_CARD_H */