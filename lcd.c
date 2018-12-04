#include <avr/io.h>
#include <util/delay.h>
#include <stdarg.h>
#include <stdio.h>

#include "lcd.h"
#include "twi.h"

#include <stdbool.h>

#define LCD_COLS 16
#define LCD_ROWS 2

#define LCD_I2C_ADDR (0x27 << 1)

#define LCD_RS _BV(0)
#define LCD_RW _BV(1)
#define LCD_EN _BV(2)
#define LCD_BT _BV(3)

#define LCD_D4 _BV(4)
#define LCD_D5 _BV(5)
#define LCD_D6 _BV(6)
#define LCD_D7 _BV(7)


/*!
 @defined 
 @abstract   All these definitions shouldn't be used unless you are writing 
 a driver.
 @discussion All these definitions are for driver implementation only and
 shouldn't be used by applications.
 */
// LCD Commands
// ---------------------------------------------------------------------------
#define LCD_CLEARDISPLAY        0x01
#define LCD_RETURNHOME          0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAYCONTROL      0x08
#define LCD_CURSORSHIFT         0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80

// flags for display entry mode
// ---------------------------------------------------------------------------
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off and cursor control
// ---------------------------------------------------------------------------
#define LCD_DISPLAYON           0x04
#define LCD_DISPLAYOFF          0x00
#define LCD_CURSORON            0x02
#define LCD_CURSOROFF           0x00
#define LCD_BLINKON             0x01
#define LCD_BLINKOFF            0x00

// flags for display/cursor shift
// ---------------------------------------------------------------------------
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00

// flags for function set
// ---------------------------------------------------------------------------
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00


static bool backlight;

bool lcd_bus_error;

static void lcd_write(uint8_t value, bool is_data)
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

static void lcd_command(uint8_t cmd)
{
  lcd_write(cmd >> 4, false);
  lcd_write(cmd & 0x0f, false);
}

static void lcd_data(uint8_t data)
{
  lcd_write(data >> 4, true);
  lcd_write(data & 0x0f, true);
}

void lcd_backlight(bool on_off)
{
  backlight = on_off;
  twi_write_bytes(LCD_I2C_ADDR, backlight ? LCD_BT : 0, 0, 0);
}

void lcd_puts(const char *str)
{
  char ch;
  while((ch = *str++)) {
    lcd_data(ch);
  }
}


void lcd_printf(const char *fmt, ...)
{
  va_list ap;
  char tmp[LCD_ROWS * LCD_COLS];
  
  va_start(ap, fmt);
  
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  char ch;
  
  for(int i = 0; i < sizeof(tmp) && (ch = tmp[i]); ++i) {
    lcd_data(ch);
  }

  va_end(ap);
}

void lcd_printf_P(const char *fmt, ...)
{
  va_list ap;
  char tmp[LCD_ROWS * LCD_COLS];
  
  va_start(ap, fmt);
  
  vsnprintf_P(tmp, sizeof(tmp), fmt, ap);
  char ch;
  
  for(int i = 0; i < sizeof(tmp) && (ch = tmp[i]); ++i) {
    lcd_data(ch);
  }

  va_end(ap);
}

void lcd_move(uint8_t x, uint8_t y)
{
  const uint8_t row_offsetsDef[]   = { 0x00, 0x40, 0x14, 0x54 }; // For regular LCDs
//  const byte row_offsetsLarge[] = { 0x00, 0x40, 0x10, 0x50 }; // For 16x4 LCDs

  if(y >= LCD_ROWS) {
    y = LCD_ROWS - 1;
  }

  lcd_command( LCD_SETDDRAMADDR | (x + row_offsetsDef[y]));
}

void lcd_init()
{
  lcd_bus_error = false;
  
  // set 4 bit mode
  lcd_write( 0x03, false );
  _delay_ms( 5 );
  lcd_write( 0x03, false );
  _delay_us( 150 );
  lcd_write( 0x03, false );
  _delay_us( 150 );
  lcd_write( 0x02, false );
  _delay_us( 150 );
  
  lcd_command( LCD_FUNCTIONSET | LCD_2LINE );
  lcd_command( LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF );
  lcd_command( LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT );

  lcd_command( LCD_CLEARDISPLAY );
  _delay_ms(2);

  lcd_backlight(true);
}
