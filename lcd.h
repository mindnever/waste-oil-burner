#ifndef _WASTE_OIL_BURNER_LCD_H_

#include <stdbool.h>

void lcd_init();
void lcd_move(uint8_t x, uint8_t y);
void lcd_puts(const char *str);
void lcd_printf(const char *fmt, ...);
void lcd_printf_P(const char *fmt, ...);
void lcd_backlight(bool on_off);

extern bool lcd_bus_error;

#endif /* _WASTE_OIL_BURNER_LCD_H_ */
