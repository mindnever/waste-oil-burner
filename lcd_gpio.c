#include <avr/io.h>
#include <util/delay.h>
#include "hw.h"
#include "lcd.h"


void lcd_write(uint8_t value, bool is_data)
{
    if (is_data) {
        IO_PIN_HIGH(LCD_RS);
    } else {
        IO_PIN_LOW(LCD_RS);
    }
    _delay_us(0.1); // 0.1
    LCD_DATA_PORT = (LCD_DATA_PORT & ~(0x0f << LCD_DATA_SHIFT)) | ((value & 0x0f) << LCD_DATA_SHIFT);

    IO_PIN_HIGH(LCD_E);

    _delay_us(0.25); // 0.25
    IO_PIN_LOW(LCD_E);
    _delay_us(0.1); // 0.1
}


void lcd_backlight(bool on_off)
{}

void lcd_gpio_init()
{
    IO_DIR_OUT(LCD_RS);
    IO_DIR_OUT(LCD_E);

    LCD_DATA_DIR |= 0x0f << LCD_DATA_SHIFT;
}
