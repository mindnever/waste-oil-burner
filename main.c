#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>

#include "usb_descriptors.h"
#include "vcp.h"
#include "usb.h"
#include "twi.h"
#include "lcd.h"

#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

#include "microrl/src/microrl.h"

#define SYM_DEG "\xdf"


// LCD
// PD0 (D3) - SCL
// PD1 (D2)- SDA


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


// K1, PD4, PROMICRO D4
#define RELAY_HEATER_PORT D
#define RELAY_HEATER_PIN  4

// K2, PC6, PROMICRO D5
#define RELAY_FAN_PORT C
#define RELAY_FAN_PIN  6

// K3, PD7, PROMICRO D6
#define RELAY_AIR_PORT D
#define RELAY_AIR_PIN  7

// K4, PE6, PROMICRO D7
#define RELAY_SPARK_PORT E
#define RELAY_SPARK_PIN  6


//////
#define IS_BURNING() (sensor_a < 61000)
#define TEMP_SENSOR(x) (-0.001958311*(float)(x) + 129.5639)


#define TARGET_OIL_TEMPERATURE 90.0
#define TARGET_WATER_TEMPERATURE 60.0

#define OIL_TEMP_HYST 1
#define WATER_TEMP_HYST 15

#define IS_OIL_HOT() (oil_temperature > config.target_oil_temperature)
#define IS_OIL_COLD() (oil_temperature < (config.target_oil_temperature - config.oil_temp_hyst))

#define IS_WATER_HOT() (water_temperature > config.target_water_temperature)
#define IS_WATER_COLD() (water_temperature < (config.target_water_temperature - config.water_temp_hyst))

#define IS_PRESSED(b) ((b) == BUTTON_DEBOUNCE)

#define TICK_MS 10

#define TIMEOUT_FAN   (10000L / TICK_MS)
#define TIMEOUT_AIR   (150L / TICK_MS)
#define TIMEOUT_SPARK (6000 / TICK_MS)
#define TIMEOUT_FLAME (4000 / TICK_MS)
#define DELAY_STATUS (500/TICK_MS)

#define LCD_REINIT_COUNT 10

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
    state_preheat,
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
    "Prheat",
    "Fan",
    "Air",
    "Spark",
    "Flame?",
    "Burn",
    "Fault",
};

typedef enum
{
    blink_off,
    blink_slow,
    blink_fast,
    blink_on
} blink_t;

struct {
    float target_oil_temperature;
    float target_water_temperature;
    float oil_temp_hyst;
    float water_temp_hyst;
} config = {
    .target_oil_temperature = TARGET_OIL_TEMPERATURE,
    .target_water_temperature = TARGET_WATER_TEMPERATURE,

    .oil_temp_hyst = OIL_TEMP_HYST,
    .water_temp_hyst = WATER_TEMP_HYST,
};


static uint8_t led_a = blink_off;
static uint8_t led_b = blink_off;

static uint8_t monitor_mode = 0;

static microrl_t mrl;

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

static int CLI_Execute(int argc, const char * const *argv)
{
    if(!strcasecmp(argv[0], "monitor")) {
        monitor_mode = 1;
        VCP_Printf("Monitor mode on\r\n");
    } else if(!strcasecmp(argv[0], "config")) {
        VCP_Printf("target_oil_temperature=%.1f\r\n", config.target_oil_temperature);
        VCP_Printf("target_water_temperature=%.1f\r\n", config.target_water_temperature);
        VCP_Printf("oil_temp_hyst=%.1f\r\n", config.oil_temp_hyst);
        VCP_Printf("water_temp_hyst=%.1f\r\n", config.water_temp_hyst);
    }
    return 0;
}

static char ** CLI_GetCompletion(int argc, const char * const *argv)
{
    static char *tok = 0;
    return &tok;
}

static void CLI_Task()
{
    uint8_t cmdBuf[64];

    uint16_t r = VCP_Read(cmdBuf, sizeof(cmdBuf) - 1);

    if (r > 0) {
        if(monitor_mode) {
            monitor_mode = false;
            VCP_Printf("Monitor mode off\r\n");
        }
        for(uint16_t i = 0; i < r; ++i) {
            microrl_insert_char(&mrl, cmdBuf[i]);
        }
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

    microrl_init(&mrl, VCP_Puts);
    microrl_set_execute_callback(&mrl, CLI_Execute);
    microrl_set_complete_callback(&mrl, CLI_GetCompletion);
    
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
    uint32_t lcd_reinit = 0;
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
        
        USB_Task();
        
        CLI_Task();
        
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
            if(lcd_reinit == 0) {
                lcd_bus_error = 1; // force LCD bus error
                lcd_reinit = LCD_REINIT_COUNT;
            }
            
            --lcd_reinit;
            
            oil_temperature = TEMP_SENSOR(sensor_b);
            water_temperature = TEMP_SENSOR(sensor_c);
            
            if(lcd_bus_error) {
                lcd_init();
            }
            
            if(monitor_mode) { // TODO: output JSON for MQTT
                VCP_Printf("State:%d [%s], s_A:%u, s_B:%u, s_C:%u, t_Oil:%.1f, t_Water:%.1f, Flame:%d IgnCount:%d\r\n", state, state_name[state], sensor_a, sensor_b, sensor_c, oil_temperature, water_temperature, (int)burning, (int)ignition_count);
            }
            status = 0;

            char ab[3] = {
#ifdef BUTTON_A_PORT
                IS_PRESSED(button_a) ? 'A' : ' ',
#else
                ' ',
#endif
#ifdef BUTTON_B_PORT
                IS_PRESSED(button_b) ? 'B' : ' ',
#else
                ' ',
#endif
                0
            };
            
            lcd_move(0, 0);
            lcd_printf("F:% 2u %s %-6s", sensor_a/1000, ab, state_name[state]);
            lcd_move(0, 1);
            lcd_printf("O:%3d" SYM_DEG "C W:%3d" SYM_DEG "C", (int)oil_temperature, (int)water_temperature);
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
                
            case state_wait: // This state is waiting for water to cool down and button_b (external thermostat?) to be pressed.
                if(IS_WATER_COLD() && IS_PRESSED(button_b)) {
                    state = state_preheat;
                }
                break;
                
            case state_preheat: // waiting for oil to be *not* cold (hot oil trigger is for oil heating state machine)
                if(!IS_OIL_COLD()) {
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
            case state_wait:
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
        
        // safety:
        
#ifdef BUTTON_A_PORT
        // soft reset
        if(IS_PRESSED(button_a)) {
            state = state_init;
        }
#endif
#ifdef BUTTON_B_PORT
        // external thermostat
        if(!IS_PRESSED(button_b)) {
            state = state_init;
        }
#endif
        // water hot
        if(IS_WATER_HOT()) {
            state = state_init;
        }
        
    }
    
    
}


void EVENT_VCP_SetLineEncoding(CDC_LineEncoding_t *LineEncoding)
{
}


void EVENT_VCP_DataReceived()
{
}

void EVENT_VCP_SetControlLineState(uint16_t State)
{
  if(State & CDC_CONTROL_LINE_OUT_RTS)
  {

  }
  else
  {

  }
}


