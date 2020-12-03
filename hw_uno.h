#ifndef __WOB_HW_UNO_H__
#define __WOB_HW_UNO_H__

#include "io.h"
#include "relay.h"

// UART
#define UART_RX_PORT    D
#define UART_RX_PIN     0

#define UART_TX_PORT    D
#define UART_TX_PIN     1

// RXB6 module

#define RX_DATA_PORT    B
#define RX_DATA_PIN     0

// Rotary encoder
// rotary encoder button
#define BUTTON_R_PORT   B
#define BUTTON_R_PIN    5

// rotary encoder A & B
#define ENCODER_A_PORT  C
#define ENCODER_A_PIN   4
#define ENCODER_B_PORT  C
#define ENCODER_B_PIN   5


// LCD
#define LCD_RS_PORT     D
#define LCD_RS_PIN      2

#define LCD_E_PORT      D
#define LCD_E_PIN       3

#define LCD_DATA_PORT   PORTD
#define LCD_DATA_DIR    DDRD
#define LCD_DATA_SHIFT  4


// flame sensor (PC3, A3, 10kΩ resistor from vcc to sensor, sensor to ground)
#define SENSOR_A_PORT   C
#define SENSOR_A_PIN    3
#define SENSOR_A_ADCMUX 3
#define SENSOR_A_DIDR0  3

// oil_temperature (PC2, A2, 220Ω from vcc to sensor, sensor to ground)
#define SENSOR_B_PORT   C
#define SENSOR_B_PIN    2
#define SENSOR_B_ADCMUX 2
#define SENSOR_B_DIDR0  2

// water temperature (PC0, A0, 220Ω from vcc to sensor, sensor to ground)
#define SENSOR_C_PORT   C
#define SENSOR_C_PIN    0
#define SENSOR_C_ADCMUX 0
#define SENSOR_C_DIDR0  0

// reset button (PC1, A1)
// #define BUTTON_A_PORT C
// #define BUTTON_A_PIN 1

// external thermostat trigger (PC1)
#define BUTTON_B_PORT C
#define BUTTON_B_PIN  1


// #define LED_A_PORT B
// #define LED_A_PIN 5

// #define LED_B_PORT B
// #define LED_B_PIN 5

// K1 - D10 - PB2
#define K1_PORT     B
#define K1_PIN      2
#define K1_INVERTED true

// K2 - D11 / PB3
#define K2_PORT     B
#define K2_PIN      3
#define K2_INVERTED true

// K3 - D12 / PB4
#define K3_PORT     B
#define K3_PIN      4
#define K3_INVERTED true

// K4 - D9 / PB1
#define K4_PORT     B
#define K4_PIN      1
#define K4_INVERTED true


#define DEFAULT_RELAY_CONFIG \
    { \
        [RELAY_EXT1] = 1, \
        [RELAY_EXT2] = 2, \
        [RELAY_EXT3] = 3, \
        [RELAY_EXT4] = 4, \
    }

#define LED_ON(n)     IO_PIN_LOW(n)
#define LED_OFF(n)    IO_PIN_HIGH(n)
#define LED_TOGGLE(n) IO_PIN_TOGGLE(n)

#endif /* __WOB_HW_UNO_H__ */
