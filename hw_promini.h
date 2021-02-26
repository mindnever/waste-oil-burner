#ifndef __WOB_HW_PROMINI_H__
#define __WOB_HW_PROMINI_H__

#include "io.h"

// UART
#define UART_RX_PORT   D
#define UART_RX_PIN    0

#define UART_TX_PORT   D
#define UART_TX_PIN    1

// RXB6 module

#define RX_DATA_PORT   B
#define RX_DATA_PIN    0

// rotary encoder button
#define BUTTON_R_PORT  B
#define BUTTON_R_PIN   5

// rotary encoder A & B
#define ENCODER_A_PORT B
#define ENCODER_A_PIN  1
#define ENCODER_B_PORT B
#define ENCODER_B_PIN  4

// LCD
// PD0 (D3) - SCL
// PD1 (D2) - SDA

#define LCD_RS_PORT     C
#define LCD_RS_PIN      1

#define LCD_E_PORT      C
#define LCD_E_PIN       0

#define LCD_DATA_PORT   PORTD
#define LCD_DATA_DIR    DDRD
#define LCD_DATA_SHIFT  4


// flame sensor (PC3, A3, 10kΩ resistor from vcc to sensor, sensor to ground)
#define SENSOR_A_PORT   C
#define SENSOR_A_PIN    3
#define SENSOR_A_ADCMUX 3
#define SENSOR_A_DIDR0  3

// oil_temperature (ADC6, A6, 220Ω from vcc to sensor, sensor to ground)
#define SENSOR_B_ADCMUX 6

// water temperature (ADC7, A7, 220Ω from vcc to sensor, sensor to ground)
#define SENSOR_C_ADCMUX 7

// LM35 calibration helper
#define SENSOR_D_ADCMUX 4
#define SENSOR_D_PORT   C
#define SENSOR_D_PIN    4
#define SENSOR_D_DIDR0  4

// reset button (PC1, A1)
// #define BUTTON_A_PORT C
// #define BUTTON_A_PIN 1

// external thermostat trigger (PC2, ADC2)
#define BUTTON_B_PORT C
#define BUTTON_B_PIN  2


// #define LED_A_PORT B
// #define LED_A_PIN 5

// #define LED_B_PORT B
// #define LED_B_PIN 5

// K1 - D2 / PD2
#define K1_PORT     D
#define K1_PIN      2
#define K1_INVERTED true

// K2 - D3 / PD3
#define K2_PORT     D
#define K2_PIN      3
#define K2_INVERTED true

// K3 - D10 / PB2
#define K3_PORT     B
#define K3_PIN      2
#define K3_INVERTED true

// K4 - MOSI / PB2
#define K4_PORT     B
#define K4_PIN      3
#define K4_INVERTED true

#define DEFAULT_RELAY_CONFIG \
	{ \
		[RELAY_FAN]   = 2, \
		[RELAY_AIR]   = 1, \
		[RELAY_SPARK] = 3, \
		[RELAY_EXT1]  = 4, \
	}

#define LED_ON(n)     IO_PIN_LOW(n)
#define LED_OFF(n)    IO_PIN_HIGH(n)
#define LED_TOGGLE(n) IO_PIN_TOGGLE(n)

#endif /* __WOB_HW_PROMINI_H__ */
