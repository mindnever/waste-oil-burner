#ifndef __WOB_HW_PROMINI_H__
#define __WOB_HW_PROMINI_H__

#include "io.h"

// UART
#define UART_RX_PORT D
#define UART_RX_PIN 0

#define UART_TX_PORT D
#define UART_TX_PIN 1

// RXB6 module

#define RX_DATA_PORT B
#define RX_DATA_PIN  0

// rotary encoder button
#define BUTTON_R_PORT B
#define BUTTON_R_PIN 5

// rotary encoder A & B
#define ENCODER_A_PORT B
#define ENCODER_A_PIN 1
#define ENCODER_B_PORT B
#define ENCODER_B_PIN 4

// LCD
// PD0 (D3) - SCL
// PD1 (D2) - SDA

#define LCD_RS_PORT C
#define LCD_RS_PIN  1

#define LCD_E_PORT C
#define LCD_E_PIN 0

#define LCD_DATA_PORT  PORTD
#define LCD_DATA_DIR   DDRD
#define LCD_DATA_SHIFT 4


// flame sensor (PC3, A3, 10kΩ resistor from vcc to sensor, sensor to ground)
#define SENSOR_A_PORT C
#define SENSOR_A_PIN 3
#define SENSOR_A_ADCMUX 3
#define SENSOR_A_DIDR0 3

// oil_temperature (ADC6, A6, 220Ω from vcc to sensor, sensor to ground)
#define SENSOR_B_ADCMUX 6

// water temperature (ADC7, A7, 220Ω from vcc to sensor, sensor to ground)
#define SENSOR_C_ADCMUX 7

// LM35 calibration helper
#define SENSOR_D_ADCMUX 4
#define SENSOR_D_PORT C
#define SENSOR_D_PIN 4
#define SENSOR_D_DIDR0 4

// reset button (PC1, A1)
//#define BUTTON_A_PORT C
//#define BUTTON_A_PIN 1

// external thermostat trigger (PC2, ADC2)
#define BUTTON_B_PORT C
#define BUTTON_B_PIN 2


//#define LED_A_PORT B
//#define LED_A_PIN 5

//#define LED_B_PORT B
//#define LED_B_PIN 5

// Kx - heater - D2 / PD2
//#define RELAY_HEATER_PORT D
//#define RELAY_HEATER_PIN  2

// K2 - fan    - D3 / PD3
#define RELAY_FAN_PORT D
#define RELAY_FAN_PIN  3

// K1 - air    - D2 / PD2
#define RELAY_AIR_PORT D
#define RELAY_AIR_PIN  2

// K3 - spark  - D10 / PB2
#define RELAY_SPARK_PORT B
#define RELAY_SPARK_PIN  2

// K4 - ZONE1  - MOSI / PB2
#define RELAY_ZONE_EXT1_PORT B
#define RELAY_ZONE_EXT1_PIN 3

// K6 - ZONE2  - MOSI / PB3
//#define RELAY_ZONE_EXT2_PORT B
//#define RELAY_ZONE_EXT2_PIN 3

// K7 - ZONE3  - D7 / PD7
//#define RELAY_ZONE_EXT3_PORT D
//#define RELAY_ZONE_EXT3_PIN 7

// K8 - ZONE4  - D9 / PB1
//#define RELAY_ZONE_EXT4_PORT B
//#define RELAY_ZONE_EXT4_PIN 1


#define RELAY_ON(n) IO_PIN_LOW(n)
#define RELAY_OFF(n) IO_PIN_HIGH(n)
#define RELAY_STATE(n) (!IO_PIN_READ(n))

#define LED_ON(n) IO_PIN_LOW(n)
#define LED_OFF(n) IO_PIN_HIGH(n)
#define LED_TOGGLE(n) IO_PIN_TOGGLE(n)




#endif /* __WOB_HW_PROMINI_H__ */
