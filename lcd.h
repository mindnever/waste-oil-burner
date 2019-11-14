#ifndef _WASTE_OIL_BURNER_LCD_H_

#include "hw.h"
#include <stdbool.h>

void lcd_init();
void lcd_move(uint8_t x, uint8_t y);
void lcd_puts(const char *str);
void lcd_printf(const char *fmt, ...);
void lcd_printf_P(const char *fmt, ...);

void lcd_write(uint8_t value, bool is_data);
void lcd_backlight(bool on_off);

#ifdef LCD_DATA_PORT
void lcd_gpio_init();
#endif

extern bool lcd_bus_error;

#endif /* _WASTE_OIL_BURNER_LCD_H_ */
