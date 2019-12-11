#ifndef __WOB_HW_PROMICRO_H__
#define __WOB_HW_PROMICRO_H__

#include "io.h"

// UART
#define UART_RX_PORT D
#define UART_RX_PIN 2

#define UART_TX_PORT D
#define UART_TX_PIN 3


// RFTX module
//#define TX_DATA_PORT D
//#define TX_DATA_PIN 3

// RXB6 module

#define RX_DATA_PORT D
#define RX_DATA_PIN  4

// LCD
// PD0 (D3) - SCL
// PD1 (D2) - SDA

// flame sensor (PF4, PROMICRO A3, 10kΩ resistor from vcc to sensor, sensor to ground)
#define SENSOR_A_PORT F
#define SENSOR_A_PIN 4
#define SENSOR_A_ADCMUX 4
#define SENSOR_A_DIDR0 4

// oil_temperature (PF5, PROMICRO A2, 220Ω from vcc to sensor, sensor to ground)
#define SENSOR_B_PORT F
#define SENSOR_B_PIN 5
#define SENSOR_B_ADCMUX 5
#define SENSOR_B_DIDR0 5

// water temperature (PF7, PROMICRO A0, 220Ω from vcc to sensor, sensor to ground)
#define SENSOR_C_PORT F
#define SENSOR_C_PIN 7
#define SENSOR_C_ADCMUX 7
#define SENSOR_C_DIDR0 7

// external thermostat trigger (PB2, PROMICRO MOSI, D16)
#define BUTTON_B_PORT B
#define BUTTON_B_PIN 2

// rotary encoder button
#define BUTTON_R_PORT F
#define BUTTON_R_PIN 6

// rotary encoder A & B
#define ENCODER_A_PORT B
#define ENCODER_A_PIN 1
#define ENCODER_B_PORT B
#define ENCODER_B_PIN 3

#define LED_A_PORT B
#define LED_A_PIN 0

#define LED_B_PORT D
#define LED_B_PIN 5

// K1, PC6, PROMICRO D5
#define K1_PORT C
#define K1_PIN  6
#define K1_INVERTED true

// K2, PD7, PROMICRO D6
#define K2_PORT D
#define K2_PIN  7
#define K2_INVERTED true

// K3, PB5, PROMICRO D9
#define K3_PORT B
#define K3_PIN  5
#define K3_INVERTED true

// K4, PB4, PROMICRO D8
#define K4_PORT B
#define K4_PIN  4
#define K4_INVERTED true

// K5, PE6, PROMICRO D7
#define K5_PORT E
#define K5_PIN 6
#define K5_INVERTED true

// K6, PB6, PROMICRO D10
#define K6_PORT B
#define K6_PIN 6
#define K6_INVERTED true

#define DEFAULT_RELAY_CONFIG \
    { \
        [ RELAY_HEATER ] = 1, \
        [ RELAY_FAN ] = 2, \
        [ RELAY_AIR ] = 3, \
        [ RELAY_SPARK ] = 4, \
        [ RELAY_ZONE_EXT1 ] = 5, \
        [ RELAY_ZONE_EXT2 ] = 6, \
    }

#define LED_ON(n) IO_PIN_LOW(n)
#define LED_OFF(n) IO_PIN_HIGH(n)
#define LED_TOGGLE(n) IO_PIN_TOGGLE(n)

#endif /* __WOB_HW_PROMICRO_H__ */
