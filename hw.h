#ifndef _32U4_HW_H_
#define _32U4_HW_H_

#include "io.h"

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

// reset button (PF6, PROMICRO A1)
#define BUTTON_A_PORT F
#define BUTTON_A_PIN 6

// external thermostat trigger (PB2, PROMICRO MOSI, D16)
#define BUTTON_B_PORT B
#define BUTTON_B_PIN 2


#define LED_A_PORT B
#define LED_A_PIN 0

#define LED_B_PORT D
#define LED_B_PIN 5

// Arduino 4, 5, 6, 7
// PROMICRO:
// PD4, PC6, PD7, PE6

// K1 - heater - PD4
// K2 - fan    - PC6
// K3 - air    - PD7
// K4 - spark  - PE6

#if 0
// K1, PD4, PROMICRO D4  (conflict with RF-RX)
# define RELAY_HEATER_PORT D
# define RELAY_HEATER_PIN  4
#else
// K1, PB4, PROMICRO D8
# define RELAY_HEATER_PORT B
# define RELAY_HEATER_PIN  4

#endif

// K2, PC6, PROMICRO D5
#define RELAY_FAN_PORT C
#define RELAY_FAN_PIN  6

// K3, PD7, PROMICRO D6
#define RELAY_AIR_PORT D
#define RELAY_AIR_PIN  7

// K4, PE6, PROMICRO D7
#define RELAY_SPARK_PORT E
#define RELAY_SPARK_PIN  6

#define RELAY_ON(n) IO_PIN_LOW(n)
#define RELAY_OFF(n) IO_PIN_HIGH(n)


#define LED_ON(n) IO_PIN_LOW(n)
#define LED_OFF(n) IO_PIN_HIGH(n)
#define LED_TOGGLE(n) IO_PIN_TOGGLE(n)

#endif /* _32U4_HW_H_ */
