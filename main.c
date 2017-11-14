#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>

#include "usb_descriptors.h"
#include "vcp.h"
#include "twi.h"
#include "lcd.h"

#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

#define SYM_DEG "\xdf"

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

#define BUTTON_B_PORT B
#define BUTTON_B_PIN 2

#define SENSOR_C_PORT F
#define SENSOR_C_PIN 7
#define SENSOR_C_ADCMUX 7
#define SENSOR_C_DIDR0 7

#define LED_A_PORT B
#define LED_A_PIN 0
#define LED_B_PORT D
#define LED_B_PIN 5

// Arduino 4, 5, 6, 7
// PROMICRO:
// PD4, PC6, PD7, PE6

// K1 - aux - PD4
// K2 - fan - PC6
// K3 - air - PD7
// K4 - spark - PE6


#define RELAY_FAN_PORT C
#define RELAY_FAN_PIN 6
#define RELAY_AIR_PORT D
#define RELAY_AIR_PIN  7
#define RELAY_SPARK_PORT E
#define RELAY_SPARK_PIN  6
#define RELAY_HEATER_PORT D
#define RELAY_HEATER_PIN 4


// Arduino A3
#define IS_BURNING() (sensor_a < 61000)

// Arduino A2
#define TEMP_SENSOR(x) (-0.001958311*(float)(x) + 129.5639)


#define TARGET_OIL_TEMPERATURE 90.0
#define TARGET_WATER_TEMPERATURE 60.0

#define TEMP_HYST 2

#define IS_OIL_HOT() (oil_temperature > target_oil_temperature)
#define IS_OIL_COLD() (oil_temperature < (target_oil_temperature - TEMP_HYST))

#define IS_WATER_HOT() (water_temperature > target_water_temperature)
#define IS_WATER_COLD() (water_temperature < (target_water_temperature - TEMP_HYST))

#define IS_PRESSED(b) ((b) == BUTTON_DEBOUNCE)

#define TICK_MS 10

#define TIMEOUT_FAN   (10000L / TICK_MS)
#define TIMEOUT_AIR   (1000L / TICK_MS)
#define TIMEOUT_SPARK (4000 / TICK_MS)
#define TIMEOUT_FLAME (4000 / TICK_MS)
#define DELAY_STATUS (500/TICK_MS)

#define IGNITION_RETRY 3

#define TICK_FAST (50/TICK_MS)
#define TICK_SLOW (500/TICK_MS)

#define FLAME_SENSOR_DELAY (3000/TICK_MS)

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
  state_detect_flame,
  state_burn,
  state_fault,
} state_t;

static const char *state_name[] = {
  "Init",
  "Wait",
  "Fan",
  "Air",
  "Spark",
  "Flame?",
  "Burn",
  "Fault"
};

typedef enum
{
  blink_off,
  blink_slow,
  blink_fast,
  blink_on
} blink_t;

static float target_oil_temperature = TARGET_OIL_TEMPERATURE;
static float target_water_temperature = TARGET_WATER_TEMPERATURE;

static uint8_t led_a = blink_off;
static uint8_t led_b = blink_off;

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
#ifdef SENSOR_A_PORT
  IO_DIR_IN( SENSOR_A );
  IO_PIN_HIGH( SENSOR_A ); // pullup
#endif

#ifdef SENSOR_B_PORT
  IO_DIR_IN( SENSOR_B );
  IO_PIN_HIGH( SENSOR_B ); // pullup
#endif

#ifdef SENSOR_C_PORT
  IO_DIR_IN( SENSOR_C );
  IO_PIN_HIGH( SENSOR_C ); // pullup
#endif

// setup ADC, ADMUX input to ADC6, Vref = AVcc, start , 125khz clock
// free running mode
  adc_mux( IO_ADCMUX( SENSOR_A ) );
  
  ADCSRA = _BV( ADEN ) | _BV( ADSC ) | _BV( ADPS0 ) | _BV( ADPS1 ) | _BV( ADPS2 ) | _BV( ADATE );

  DIDR0 =
#ifdef SENSOR_A_PORT
  _BV( IO_DIDR0( SENSOR_A ) ) |
#endif
#ifdef SENSOR_B_PORT
  _BV( IO_DIDR0( SENSOR_B ) ) |
#endif
#ifdef SENSOR_C_PORT
  _BV( IO_DIDR0( SENSOR_C ) ) |
#endif
  0;
}

static void IO_Init()
{
  IO_DIR_OUT( LED_A );
  IO_DIR_OUT( LED_B );
  
  IO_DIR_OUT( RELAY_HEATER );
  IO_DIR_OUT( RELAY_FAN );
  IO_DIR_OUT( RELAY_AIR );
  IO_DIR_OUT( RELAY_SPARK );

#ifdef BUTTON_A_PORT  
  IO_DIR_IN( BUTTON_A );
  IO_PIN_HIGH( BUTTON_A ); // pullup
#endif
#ifdef BUTTON_B_PORT
  IO_DIR_IN( BUTTON_B );
  IO_PIN_HIGH( BUTTON_B ); // pullup
#endif
}

int main(void)
{
  wdt_enable(WDTO_1S);
  
  USB_Init();
  
  IO_Init();
  

  RELAY_OFF( RELAY_HEATER );
  RELAY_OFF( RELAY_FAN );
  RELAY_OFF( RELAY_AIR );
  RELAY_OFF( RELAY_SPARK );
  
  LED_ON( LED_A );
  LED_ON( LED_B );

  init_sensors();
  
  twi_init();
  lcd_init();
  
  uint32_t timer = 0;
  uint32_t burning = 0;
  uint32_t status = 0;
  uint8_t ignition_count = 0;

  state_t state = state_init;

#ifdef BUTTON_A_PORT
  int button_a = 0;
#endif
#ifdef BUTTON_B_PORT
  int button_b = 0;
#endif
  
  uint16_t sensor_a = 0;
  uint16_t sensor_b = 0;
  uint16_t sensor_c = 0;
  
  int sensor = 0;
  float oil_temperature = 0;
  float water_temperature = 0;
    
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
      case 2:
        adc_mux( IO_ADCMUX( SENSOR_C ) );
    }
    
    _delay_ms(TICK_MS);
    
    switch(sensor) {
      case 0:
        sensor_a = ADC;
        break;
      case 1:
        sensor_b = ADC;
        break;
      case 2:
        sensor_c = ADC;
        break;
    }
    
    ++sensor;
    
    ++status;
    if(status > DELAY_STATUS) {
      oil_temperature = TEMP_SENSOR(sensor_b);
      water_temperature = TEMP_SENSOR(sensor_c);
      
      VCP_Printf("S:%d, A:%u, B:%u, C:%u, tO:%d, tW: %d, F:%d I:%d  longer and long and even longer text, than usb buffer.. whtf\r\n", state, sensor_a, sensor_b, sensor_c, (int)oil_temperature, (int)water_temperature, (int)burning, (int)ignition_count);
      status = 0;
      lcd_move(0, 0);
      lcd_printf("F:% 5u %-6s", sensor_a, state_name[state]);
      lcd_move(0, 1);
      lcd_printf("tO:%3d" SYM_DEG "C tW:%3d" SYM_DEG "C", (int)oil_temperature, (int)water_temperature);
    }
    

#ifdef BUTTON_A_PORT    
    if( !IO_PIN_READ( BUTTON_A ) ) {
      if(button_a < BUTTON_DEBOUNCE) {
        ++button_a;
      }
    } else {
      button_a = 0;
    }
#endif

#ifdef BUTTON_B_PORT    
    if( !IO_PIN_READ( BUTTON_B ) ) {
      if(button_b < BUTTON_DEBOUNCE) {
        ++button_b;
      }
    } else {
      button_b = 0;
    }
#endif    
    
    // flame state machine
    switch(state)
    {
      case state_init:
        led_a = blink_off;
        led_b = blink_slow;
        state = state_wait;
        timer = 0;
        
        // turn off relays
        RELAY_OFF( RELAY_FAN );
        RELAY_OFF( RELAY_AIR );
        RELAY_OFF( RELAY_SPARK );
        
      break;
    
      case state_wait:
        if(IS_OIL_HOT() && IS_WATER_COLD()) {
           RELAY_ON( RELAY_FAN );
           state = state_fan;
           timer = 0;
           ++ignition_count;
        }
      break;

      case state_fan:
        if( ++timer  > TIMEOUT_FAN )
        {
          RELAY_ON( RELAY_AIR );
          state = state_air;
          timer = 0;
        }
        break;

      case state_air:
        if( ++timer > TIMEOUT_AIR ) {
            state = state_spark;
            RELAY_ON( RELAY_SPARK );
            led_a = blink_fast;
            timer = 0;
        }
        break;

      case state_spark:
        if( ++timer > TIMEOUT_SPARK )
        {
          RELAY_OFF( RELAY_SPARK );
          state = state_detect_flame;
          led_a = blink_off;
          led_b = blink_fast;
          timer = 0;
        }
        break;
      case state_detect_flame:
        if( ++timer > TIMEOUT_FLAME )
        {
          if(ignition_count > IGNITION_RETRY) {
            state = state_fault;
          } else {
            state = state_init;
          }
          timer = 0;
        }
        else if( IS_BURNING() )
        {
          led_a = blink_off;
          led_b = blink_slow;
          
          state = state_burn;
          timer = 0;
          ignition_count = 0;
        }
        break;
      case state_burn:
        if( IS_BURNING() ) {
          burning = FLAME_SENSOR_DELAY;
        } else if(burning) {
          --burning;
        }
        
        if(burning == 0) {
          state = state_init;
        }

//        if( IS_OIL_COLD() ) {
//          state = state_init;
//        }

        if( IS_WATER_HOT() ) {
          state = state_init;
        }

        break;
      case state_fault:
        RELAY_OFF( RELAY_FAN );
        RELAY_OFF( RELAY_AIR );
        RELAY_OFF( RELAY_SPARK );

        led_a = blink_slow;
        led_b = blink_slow;

        break;

    }
    
    // heater state machine
    switch(state)
    {
      case state_init:
      case state_fault:
        RELAY_OFF( RELAY_HEATER );
        break;
      default:
        if(IS_OIL_HOT()) {
          RELAY_OFF( RELAY_HEATER );
        }
        if(IS_OIL_COLD()) {
          RELAY_ON( RELAY_HEATER );
        }
        break;
    }
    
#ifdef BUTTON_A_PORT
    if(IS_PRESSED(button_a)) {
      state = state_init;
    }
#endif
#ifdef BUTTON_B_PORT
    if(IS_PRESSED(button_b)) {
      state = state_fault;
    }
#endif
    
  }


}
