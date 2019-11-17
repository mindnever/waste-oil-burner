#include <avr/io.h>
#include <util/delay.h>
#include <stdarg.h>
#include <stdio.h>

#include "lcd.h"

#include <stdbool.h>

#define LCD_COLS 16
#define LCD_ROWS 2

bool lcd_bus_error;

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
#define LCD_VFLIP		0x01
#define LCD_HFLIP		0x02

static void lcd_command(uint8_t cmd)
{
  lcd_write(cmd >> 4, false);
  lcd_write(cmd & 0x0f, false);

#ifdef LCD_COMMAND_DELAY_US
  _delay_us(LCD_COMMAND_DELAY_US);
#endif
}


static void lcd_data(uint8_t data)
{
  lcd_write(data >> 4, true);
  lcd_write(data & 0x0f, true);
#ifdef LCD_DATA_DELAY_US
  _delay_us(LCD_DATA_DELAY_US);
#endif

}

static void lcd_data_xlate(uint8_t data)
{
 if(data == 'B') { data = 0x02; } // B
 lcd_data(data);
}

void lcd_puts(const char *str)
{
  char ch;
  while((ch = *str++)) {
    lcd_data_xlate(ch);
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
    lcd_data_xlate(ch);
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
    lcd_data_xlate(ch);
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
  
  // define degree symbol
  lcd_command( LCD_SETCGRAMADDR + 8);
  lcd_data( 0x8 );
  lcd_data( 0x14 );
  lcd_data( 0x8 );
  lcd_data( 0 );
  lcd_data( 0 );
  lcd_data( 0 );
  lcd_data( 0 );
  lcd_data( 0 );  
  
  lcd_command( LCD_SETCGRAMADDR + 16 ); // Nicer capital B letter.
  lcd_data( 0x1e );
  lcd_data( 0x11 );
  lcd_data( 0x11 );
  lcd_data( 0x1e );
  lcd_data( 0x11 );
  lcd_data( 0x11 );
  lcd_data( 0x1e );
  lcd_data( 0 );

  lcd_backlight(true);
}
