#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>

#include "usb_descriptors.h"
#include "vcp.h"

#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>


#define SENSOR_A_PORT F
#define SENSOR_A_PIN 4
#define SENSOR_A_ADCMUX 4
#define SENSOR_A_DIDR0 4

#define SENSOR_B_PORT F
#define SENSOR_B_PIN 5
#define SENSOR_B_ADCMUX 5
#define SENSOR_B_DIDR0 5

#define BUTTON_A_PORT F
#define BUTTON_A_PIN 6

#define BUTTON_B_PORT F
#define BUTTON_B_PIN 7

#define LED_A_PORT B
#define LED_A_PIN 0
#define LED_B_PORT D
#define LED_B_PIN 5

#define RELAY_FAN_PORT D
#define RELAY_FAN_PIN 4
#define RELAY_AIR_PORT C
#define RELAY_AIR_PIN  6
#define RELAY_SPARK_PORT D
#define RELAY_SPARK_PIN  7
#define RELAY_AUX_PORT E
#define RELAY_AUX_PIN 6


// Arduino A3
#define IS_BURNING() (sensor_a < 50000)

// Arduino A2
#define TEMP_SENSOR SENSOR_B

#define TICK_MS 10

#define DELAY_AIR     (3000 / TICK_MS)
#define TIMEOUT_SPARK (4000 / TICK_MS)
#define DELAY_STATUS (500/TICK_MS)
#define TICK_FAST (50/TICK_MS)
#define TICK_SLOW (500/TICK_MS)

#define BUTTON_DEBOUNCE (100/TICK_MS)

#define _CONCAT(a,b) a ## b
#define _CONCAT3(a,b,c) a ## b ## c

//#define IO_PIN(n) _CONCAT3(IO_, n, _PIN)
//#define IO_PORTNAME(n) _CONCAT3(IO_, n, _PORT)

#define IO_PIN(n) _CONCAT(n, _PIN)
#define IO_PORTNAME(n) _CONCAT(n, _PORT)
#define IO_ADCMUX(n) _CONCAT(n, _ADCMUX)
#define IO_DIDR0(n) _CONCAT(n, _DIDR0)

#define IO_PORT_OUT(p) _CONCAT(PORT, p)
#define IO_PORT_IN(p) _CONCAT(PIN, p)
#define IO_DDR(p)  _CONCAT(DDR, p)

#define IO_PIN_HIGH( n )  { IO_PORT_OUT( IO_PORTNAME( n ) ) |= _BV( IO_PIN( n ) ); }

#define IO_PIN_LOW( n )   { IO_PORT_OUT( IO_PORTNAME( n ) ) &= ~_BV( IO_PIN( n ) ); }
#define IO_PIN_READ( n ) ( IO_PORT_IN( IO_PORTNAME( n ) ) & _BV( IO_PIN( n ) ) )

#define IO_DIR_IN( n ) { IO_DDR( IO_PORTNAME( n ) ) &= ~_BV( IO_PIN( n ) ); }
#define IO_DIR_OUT( n ) { IO_DDR( IO_PORTNAME( n ) ) |= _BV( IO_PIN( n ) ); }

#define RELAY_ON(n) IO_PIN_LOW(n)
#define RELAY_OFF(n) IO_PIN_HIGH(n)

#define LED_ON(n) IO_PIN_LOW(n)
#define LED_OFF(n) IO_PIN_HIGH(n)

typedef enum
{
  state_init,
  state_wait,
  state_fan,
  state_air,
  state_spark,
  state_burn,
  state_fault,
  state_sensor_a_test,
} state_t;


typedef enum
{
  blink_off,
  blink_slow,
  blink_fast,
  blink_on
} blink_t;

uint8_t led_a = blink_off;
uint8_t led_b = blink_off;

uint8_t led_state(blink_t b, uint8_t slow, uint8_t fast)
{
  if(b == blink_fast) { b = fast ? blink_on : blink_off; }
  if(b == blink_slow) { b = slow ? blink_on : blink_off; }
  
  return b == blink_on;
}

void handle_led()
{
  static uint8_t fast = 0;
  static uint8_t slow = 0;
  static uint16_t tick_f = 0, tick_s = 0;

  if(++tick_f > TICK_FAST) {
    fast = !fast;
    tick_f = 0;
  }
  
  if(++tick_s > TICK_SLOW) {
    slow = !slow;
    tick_s = 0;
  }

  if(led_state( led_a, slow, fast)) {
    LED_ON( LED_A );
  } else {
    LED_OFF( LED_A );
  }
  
  if(led_state( led_b, slow, fast)) {
    LED_ON( LED_B );
  } else {
    LED_OFF( LED_B );
  }
}

static void adc_mux(uint8_t mux)
{
  ADMUX = _BV( REFS0 ) | _BV( ADLAR ) | mux;
}

static void init_sensors()
{
// setup ADC, ADMUX input to ADC6, Vref = AVcc, start , 125khz clock
// free running mode
  adc_mux( IO_ADCMUX( SENSOR_A ) );
  
  ADCSRA = _BV( ADEN ) | _BV( ADSC ) | _BV( ADPS0 ) | _BV( ADPS1 ) | _BV( ADPS2 ) | _BV( ADATE );

  DIDR0 = IO_DIDR0( SENSOR_A ) | IO_DIDR0( SENSOR_B ); // disable digital input
}

int main(void)
{
  wdt_enable(WDTO_1S);
  
  USB_Init();
  
  IO_DIR_OUT( LED_A );
  IO_DIR_OUT( LED_B );
  
  IO_DIR_OUT( RELAY_AUX );
  IO_DIR_OUT( RELAY_FAN );
  IO_DIR_OUT( RELAY_AIR );
  IO_DIR_OUT( RELAY_SPARK );

  IO_DIR_IN( SENSOR_A );
  IO_DIR_IN( SENSOR_B );
  
  IO_DIR_IN( BUTTON_A );
  IO_DIR_IN( BUTTON_B );

  IO_PIN_HIGH( SENSOR_A ); // pullup
  IO_PIN_HIGH( SENSOR_B ); // pullup

  IO_PIN_HIGH( BUTTON_A ); // pullup
  IO_PIN_HIGH( BUTTON_B ); // pullup

  RELAY_OFF( RELAY_AUX );
  RELAY_OFF( RELAY_FAN );
  RELAY_OFF( RELAY_AIR );
  RELAY_OFF( RELAY_SPARK );
  
  LED_ON( LED_A );
  LED_ON( LED_B );

  init_sensors();
  
  unsigned int timer = 0;
  unsigned int status = 0;

  state_t state = state_init;

  // state = state_sensor_a_test;

  int button_a = 0;
  int button_b = 0;
  
  uint16_t sensor_a = 0;
  uint16_t sensor_b = 0;
  
  int sensor = 0;
    
  GlobalInterruptEnable();

  VCP_Printf("Booting\r\n");

  for(;;)
  {
    wdt_reset();
    
    USB_USBTask();
    VCP_Task();
    
    handle_led();
    
    switch(sensor) {
      default:
        sensor = 0;
      case 0:
        adc_mux( IO_ADCMUX( SENSOR_A ) );
        break;
      case 1:
        adc_mux( IO_ADCMUX( SENSOR_B ) );
        break;
    }
    
    _delay_ms(TICK_MS);
    
    switch(sensor) {
      case 0:
        sensor_a = ADC;
        break;
      case 1:
        sensor_b = ADC;
        break;
    }
    
    ++sensor;
    
    ++status;
    if(status > DELAY_STATUS) {
      VCP_Printf("State:%d, Sen_A:%u, Sen_B:%u\r\n", state, sensor_a, sensor_b);
      status = 0;
    }
    

    
    if( !IO_PIN_READ( BUTTON_A ) ) {
      if(button_a < BUTTON_DEBOUNCE) {
        ++button_a;
      }
    } else {
      button_a = 0;
    }
    
    if( !IO_PIN_READ( BUTTON_B ) ) {
      if(button_b < BUTTON_DEBOUNCE) {
        ++button_b;
      }
    } else {
      button_b = 0;
    }
    
    if(button_a == BUTTON_DEBOUNCE) {
      state = state_init;
    }
    
    switch(state)
    {
      case state_init:
        led_a = blink_off;
        led_b = blink_slow;
        state = state_wait;
        
        // turn off relays
        RELAY_OFF( RELAY_AUX );
        RELAY_OFF( RELAY_FAN );
        RELAY_OFF( RELAY_AIR );
        RELAY_OFF( RELAY_SPARK );
        
      break;
    
      case state_wait:
        {
           // assert oil temperature
           RELAY_ON( RELAY_FAN );
           state = state_fan;
           timer = 0;
        }
      break;

      case state_fan:
        if( ++timer  > DELAY_AIR )
        {
          RELAY_ON( RELAY_AIR );
          state = state_air;
        }
        break;

      case state_air:
        state = state_spark;
        RELAY_ON( RELAY_SPARK );
        led_a = blink_fast;
        timer = 0;
        break;

      case state_spark:
        if( ++timer > TIMEOUT_SPARK )
        {
          state = state_fault;
        }
        else if( IS_BURNING() )
        {
          RELAY_OFF( RELAY_SPARK );
          
          led_a = blink_off;
          led_b = blink_on;
          
          state = state_burn;
        }
        break;
      case state_burn:
        if( !IS_BURNING() )
        {
          state = state_init;
        }
        break;
      case state_fault:
        RELAY_OFF( RELAY_AUX );
        RELAY_OFF( RELAY_FAN );
        RELAY_OFF( RELAY_AIR );
        RELAY_OFF( RELAY_SPARK );

        led_a = blink_slow;
        led_b = blink_slow;

        break;

      case state_sensor_a_test:
        {
          if(IO_PIN_READ( SENSOR_A ))
          {
            led_a = blink_fast;
          }
          else
          {
            led_a = blink_off;
          }
          
        }
        break;
    }
    
  }


}


