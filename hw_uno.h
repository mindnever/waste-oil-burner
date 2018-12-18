#ifndef __WOB_HW_UNO_H__
#define __WOB_HW_UNO_H__

#include "io.h"

// UART
#define UART_RX_PORT D
#define UART_RX_PIN 0

#define UART_TX_PORT D
#define UART_TX_PIN 1

// RXB6 module

#define RX_DATA_PORT B
#define RX_DATA_PIN  0

// LCD
// PD0 (D3) - SCL
// PD1 (D2) - SDA

// flame sensor (PC3, A3, 10kΩ resistor from vcc to sensor, sensor to ground)
#define SENSOR_A_PORT C
#define SENSOR_A_PIN 3
#define SENSOR_A_ADCMUX 3
#define SENSOR_A_DIDR0 3

// oil_temperature (PC2, A2, 220Ω from vcc to sensor, sensor to ground)
#define SENSOR_B_PORT C
#define SENSOR_B_PIN 2
#define SENSOR_B_ADCMUX 2
#define SENSOR_B_DIDR0 2

// water temperature (PC0, A0, 220Ω from vcc to sensor, sensor to ground)
#define SENSOR_C_PORT C
#define SENSOR_C_PIN 0
#define SENSOR_C_ADCMUX 0
#define SENSOR_C_DIDR0 0

// reset button (PC1, A1)
#define BUTTON_A_PORT C
#define BUTTON_A_PIN 1

// external thermostat trigger (PB4, MISO)
#define BUTTON_B_PORT B
#define BUTTON_B_PIN 4


//#define LED_A_PORT B
//#define LED_A_PIN 5

#define LED_B_PORT B
#define LED_B_PIN 5

// K1 - heater - D2 / PD2
#define RELAY_HEATER_PORT D
#define RELAY_HEATER_PIN  2

// K2 - fan    - D3 / PD3
#define RELAY_FAN_PORT D
#define RELAY_FAN_PIN  3

// K3 - air    - D4 / PD4
#define RELAY_AIR_PORT D
#define RELAY_AIR_PIN  4

// K4 - spark  - D5 / PD5
#define RELAY_SPARK_PORT D
#define RELAY_SPARK_PIN  5

// K5 - ZONE1  - D6 / PD6
#define RELAY_ZONE_EXT1_PORT D
#define RELAY_ZONE_EXT1_PIN 6

// K6 - ZONE2  - MOSI / PB3
#define RELAY_ZONE_EXT2_PORT B
#define RELAY_ZONE_EXT2_PIN 3

// K7 - ZONE3  - D7 / PD7
#define RELAY_ZONE_EXT3_PORT D
#define RELAY_ZONE_EXT3_PIN 7

// K8 - ZONE4  - D9 / PB1
#define RELAY_ZONE_EXT4_PORT B
#define RELAY_ZONE_EXT4_PIN 1


#define RELAY_ON(n) IO_PIN_LOW(n)
#define RELAY_OFF(n) IO_PIN_HIGH(n)
#define RELAY_STATE(n) (!IO_PIN_READ(n))

#define LED_ON(n) IO_PIN_LOW(n)
#define LED_OFF(n) IO_PIN_HIGH(n)
#define LED_TOGGLE(n) IO_PIN_TOGGLE(n)

#endif /* __WOB_HW_UNO_H__ */
