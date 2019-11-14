#include <avr/io.h>
#include <stdbool.h>
#include "lcd.h"
#include "twi.h"

#define LCD_RS _BV(0)
#define LCD_RW _BV(1)
#define LCD_EN _BV(2)
#define LCD_BT _BV(3)

#define LCD_D4 _BV(4)
#define LCD_D5 _BV(5)
#define LCD_D6 _BV(6)
#define LCD_D7 _BV(7)



static bool backlight;

#define LCD_I2C_ADDR (0x27 << 1)

void lcd_write(uint8_t value, bool is_data)
{
  value <<= 4;

  if(is_data) {
    value |= LCD_RS;
  }
  if(backlight) {
    value |= LCD_BT;
  }
  
  if(twi_write_bytes(LCD_I2C_ADDR, value, 0, 0) < 0) {
    lcd_bus_error = true;
  }
  if(twi_write_bytes(LCD_I2C_ADDR, value | LCD_EN, 0, 0) < 0) {
    lcd_bus_error = true;
  }
  if(twi_write_bytes(LCD_I2C_ADDR, value, 0, 0) < 0) {
    lcd_bus_error = true;
  }
}

void lcd_backlight(bool on_off)
{
  backlight = on_off;
  twi_write_bytes(LCD_I2C_ADDR, backlight ? LCD_BT : 0, 0, 0);
}

