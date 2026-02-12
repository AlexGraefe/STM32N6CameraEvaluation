#ifndef MYLCD_H
#define MYLCD_H

#include <stdint.h>

typedef struct
{
  uint32_t X0;
  uint32_t Y0;
  uint32_t XSize;
  uint32_t YSize;
} Rectangle_TypeDef;

Rectangle_TypeDef *get_lcd_lcd_bg_area(void);
uint8_t *get_lcd_bg_buffer(void);
void LCD_init(void);

#endif /* MYLCD_H */