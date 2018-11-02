#include <util/delay.h>
#include <avr/wdt.h>

#include "hw.h"

#include "usb_descriptors.h"
#include "vcp.h"
#include "usb.h"
#include "twi.h"
#include "lcd.h"
#include "led.h"
#include "rx.h"

#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

#include "microrl/src/microrl.h"

#include <stdlib.h>

#define SYM_DEG "\xdf"



//////

#define ANALOG_SENSOR_DEFAULT_GAIN -0.001958311
#define ANALOG_SENSOR_DEFAULT_OFFSET 129.5639

#define IS_BURNING() (sensor_a < 61000)
#define TEMP_SENSOR(x) (ANALOG_SENSOR_DEFAULT_GAIN*(float)(x) + ANALOG_SENSOR_DEFAULT_OFFSET)


#define TARGET_OIL_TEMPERATURE 90.0
#define TARGET_WATER_TEMPERATURE 60.0

#define OIL_TEMP_HYST 1
#define WATER_TEMP_HYST 15

#define IS_OIL_HOT() (oil_temperature > config.target_oil_temperature)
#define IS_OIL_COLD() (oil_temperature < (config.target_oil_temperature - config.oil_temp_hyst))

#define IS_WATER_HOT() (water_temperature > config.target_water_temperature)
#define IS_WATER_COLD() (water_temperature < (config.target_water_temperature - config.water_temp_hyst))

#define IS_PRESSED(b) ((b) == BUTTON_DEBOUNCE)

#define TIMEOUT_FAN   (10000L / TICK_MS)
#define TIMEOUT_AIR   (150L / TICK_MS)
#define TIMEOUT_SPARK (6000 / TICK_MS)
#define TIMEOUT_FLAME (4000 / TICK_MS)
#define DELAY_STATUS (500/TICK_MS)

#define LCD_REINIT_COUNT 10

#define IGNITION_RETRY 3

#define FLAME_SENSOR_DELAY (3000/TICK_MS)
#define BUTTON_DEBOUNCE (100/TICK_MS)
#define FORCE_HID_REPORTS (1000/TICK_MS)


typedef enum
{
    state_init, // 0
    state_wait, // 1
    state_preheat, // 2
    state_fan, // 3
    state_air, // 4
    state_spark, // 5
    state_detect_flame, // 6
    state_burn, // 7
    state_fault, // 8
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

static void on_rfrx_sensor_data(struct RfRx_SensorData *data);

static uint16_t force_hid_reports_counter;
static uint8_t force_hid_reports;

static void do_hid_report_03();
static void do_hid_report_04();

static uint8_t monitor_mode;
static microrl_t mrl;

#ifdef BUTTON_A_PORT
static int button_a;
#endif
#ifdef BUTTON_B_PORT
static int button_b;
#endif
    
static uint16_t sensor_a;
static uint16_t sensor_b;
static uint16_t sensor_c;
static uint32_t burning;
static float oil_temperature;
static float water_temperature;
static state_t state = state_init;

typedef enum {
    SENSOR_ANALOG1,
    SENSOR_ANALOG2,
    SENSOR_BINARY, /* button_b */
    SENSOR_RFRX,
} sensor_type_t;

struct ThermalZone {
    int16_t SetPoint;
    int16_t Current;
    uint8_t Flags; // use hid.h: HID_WOB_Report_04_t Flags values ( WOB_REPORT_FLAGS_CONTROL_ENABLED, WOB_REPORT_FLAGS_OUTPUT_ACTIVE)
    
    sensor_type_t _sensor_type;

    union {
        struct {
            uint16_t sensor_id;
        } rfrx;
        struct {
            float gain;
            float offset;
        } analog;
    } _config;
    
    uint16_t _hysteresis;
};

static struct ThermalZone *Zones_GetZone(uint8_t id);

#define NUM_EXT_ZONES 4

static struct ThermalZone Zones[ NUM_EXT_ZONES ];

static void Zones_Init()
{
    Zones[ 0 ].SetPoint = 200;
    Zones[ 0 ].Flags = WOB_REPORT_FLAGS_CONTROL_ENABLED;
    Zones[ 0 ]._sensor_type = SENSOR_BINARY;
}

static const char *zone_names[] = {
    "water",
    "oil",
    "external_1",
    "external_2",
    "external_3",
    "external_4",
};

static void Zones_SetCurrent(sensor_type_t type, uint16_t id, int16_t current)
{
    for(uint8_t i = 0; i < _WOB_REPORT_ZONE_COUNT; ++i) {
        struct ThermalZone *zone = Zones_GetZone(i);
        if(!zone) { continue; }// how can this be? cannot..
        
        if((zone->_sensor_type == type)
        && ((type != SENSOR_RFRX) || (id == zone->_config.rfrx.sensor_id))) {
            zone->Current = current;
        }
    }
}

static void Zones_Update()
{
    for(uint8_t i = 0; i < _WOB_REPORT_ZONE_COUNT; ++i) {
        struct ThermalZone *zone = Zones_GetZone(i);
        if(!zone) { continue; }// how can this be? cannot..
        if(!(zone->Flags & WOB_REPORT_FLAGS_CONTROL_ENABLED)) {
            zone->Flags &= ~WOB_REPORT_FLAGS_OUTPUT_ACTIVE;
            continue;
        }
        if(zone->Current < (zone->SetPoint - zone->_hysteresis)) {
            zone->Flags |= WOB_REPORT_FLAGS_OUTPUT_ACTIVE;
        } else {
            zone->Flags &= ~WOB_REPORT_FLAGS_OUTPUT_ACTIVE;
        }
    }
}

static void Zones_DumpZone(uint8_t id, struct ThermalZone *zone)
{
    VCP_Printf_P(PSTR("Zone %u (%s) is %s\r\n"), id + 1, zone_names[id], (zone->Flags & WOB_REPORT_FLAGS_CONTROL_ENABLED) ? "ENABLED" : "DISABLED");
    VCP_Printf_P(PSTR("      Output %s\r\n"), (zone->Flags & WOB_REPORT_FLAGS_OUTPUT_ACTIVE) ? "ACTIVE" : "INACTIVE");
    VCP_Printf_P(PSTR("    SetPoint %.1f\r\n"), (float)zone->SetPoint / 10);
    VCP_Printf_P(PSTR("     Current %.1f\r\n"), (float)zone->Current / 10);
    VCP_Printf_P(PSTR("  Hysteresis %.1f\r\n"), (float)zone->_hysteresis / 10);
    VCP_Printf_P(PSTR("      Sensor "));

    switch(zone->_sensor_type) {
        case SENSOR_ANALOG1:
        case SENSOR_ANALOG2:
            VCP_Printf_P(PSTR("Analog gain=%f offset=%f\r\n"), zone->_config.analog.gain, zone->_config.analog.offset);
            break;
        case SENSOR_BINARY:
            VCP_Printf_P(PSTR("Binary (BUTTON_B)\r\n"));
            break;
        case SENSOR_RFRX:
            VCP_Printf_P(PSTR("RFRX sensor_id=%u\r\n"), zone->_config.rfrx.sensor_id);
            break;
        default:
            VCP_Printf_P(PSTR("unknown?\r\n"));
    }
}

struct ThermalZone *Zones_GetZone(uint8_t id)
{
    static struct ThermalZone fake;
    
    switch(id) {
        case WOB_REPORT_ZONE_INTERNAL_WATER:
            fake.SetPoint = config.target_water_temperature * 10;
            fake.Current = water_temperature * 10;
            fake.Flags = 0;

            if(IS_PRESSED(button_b)) {
                fake.Flags |= WOB_REPORT_FLAGS_CONTROL_ENABLED;
            }
            
            if(state == state_preheat
            || state == state_fan
            || state == state_air
            || state == state_spark
            || state == state_detect_flame
            || state == state_burn) {
                fake.Flags |= WOB_REPORT_FLAGS_OUTPUT_ACTIVE;
            }

            fake._sensor_type = SENSOR_ANALOG1;
            fake._hysteresis = WATER_TEMP_HYST * 10;
            fake._config.analog.gain = ANALOG_SENSOR_DEFAULT_GAIN;
            fake._config.analog.offset = ANALOG_SENSOR_DEFAULT_OFFSET;
            
            return &fake;
        case WOB_REPORT_ZONE_INTERNAL_OIL:
            fake.SetPoint = config.target_oil_temperature * 10;
            fake.Current = oil_temperature * 10;
            fake.Flags = 0;

            if(state != state_init
            && state != state_wait
            && state != state_fault) {
                fake.Flags |= WOB_REPORT_FLAGS_CONTROL_ENABLED;
            }
            
            if(RELAY_STATE(RELAY_HEATER)) {
                fake.Flags |= WOB_REPORT_FLAGS_OUTPUT_ACTIVE;
            }
            
            fake._sensor_type = SENSOR_ANALOG2;
            fake._hysteresis = OIL_TEMP_HYST * 10;
            fake._config.analog.gain = ANALOG_SENSOR_DEFAULT_GAIN;
            fake._config.analog.offset = ANALOG_SENSOR_DEFAULT_OFFSET;
            
            return &fake;
        case WOB_REPORT_ZONE_EXTERNAL1:
        case WOB_REPORT_ZONE_EXTERNAL2:
        case WOB_REPORT_ZONE_EXTERNAL3:
        case WOB_REPORT_ZONE_EXTERNAL4:
            return &Zones[ id - WOB_REPORT_ZONE_EXTERNAL1 ];
            break;
    }

    return 0;
}

static void Zones_Dump()
{
    for(int i = 0; i < _WOB_REPORT_ZONE_COUNT; ++i) {
        Zones_DumpZone( i, Zones_GetZone( i ) );
    }
}

static void Zones_ZoneCommand(struct ThermalZone *zone, int argc, const char * const *argv)
{
    if(!strcasecmp(argv[0], "enable")) {
        zone->Flags |= WOB_REPORT_FLAGS_CONTROL_ENABLED;
    } else if(!strcasecmp(argv[0], "disable")) {
        zone->Flags &= ~WOB_REPORT_FLAGS_CONTROL_ENABLED;
    } else if(!strcasecmp(argv[0], "setpoint")) {
        if(argc > 1) {
            zone->SetPoint = atof(argv[1]) * 10;
        }
    } else if(!strcasecmp(argv[0], "sensor")) {
        if(argc > 1) {
            if(!strcasecmp(argv[1], "analog1")) {
                zone->_sensor_type = SENSOR_ANALOG1;
                zone->_config.analog.gain = ANALOG_SENSOR_DEFAULT_GAIN;
                zone->_config.analog.offset = ANALOG_SENSOR_DEFAULT_OFFSET;
            } else if(!strcasecmp(argv[1], "analog2")) {
                zone->_sensor_type = SENSOR_ANALOG2;
                zone->_config.analog.gain = ANALOG_SENSOR_DEFAULT_GAIN;
                zone->_config.analog.offset = ANALOG_SENSOR_DEFAULT_OFFSET;
            } else if(!strcasecmp(argv[1], "binary")) {
                zone->_sensor_type = SENSOR_BINARY;
            } else if(!strcasecmp(argv[1], "rfrx")) {
                if(argc > 2) {
                    zone->_sensor_type = SENSOR_RFRX;
                    zone->_config.rfrx.sensor_id = atoi(argv[2]);
                }
            } else {
                VCP_Printf_P(PSTR("unknown zone sensor type '%s'\r\n"), argv[1]);
            }
        }
    } else if(!strcasecmp(argv[0], "hysteresis")) {
        if(argc > 1) {
            zone->_hysteresis = atof(argv[1]) * 10;
        }
    } else {
        VCP_Printf_P(PSTR("unknown zone command '%s'\r\n"), argv[0]);
    }
}

static int CLI_Execute(int argc, const char * const *argv)
{
    VCP_Printf_P(PSTR("\r\n"));

    if(!strcasecmp(argv[0], "monitor")) {
        monitor_mode = 1;
        VCP_Printf_P(PSTR("Monitor mode on\r\n"));
    } else if(!strcasecmp(argv[0], "config")) {
        VCP_Printf_P(PSTR("target_oil_temperature=%.1f\r\n"), config.target_oil_temperature);
        VCP_Printf_P(PSTR("target_water_temperature=%.1f\r\n"), config.target_water_temperature);
        VCP_Printf_P(PSTR("oil_temp_hyst=%.1f\r\n"), config.oil_temp_hyst);
        VCP_Printf_P(PSTR("water_temp_hyst=%.1f\r\n"), config.water_temp_hyst);
    } else if(!strcasecmp(argv[0], "zone")) {
        if(argc > 1) {
            if(!strcasecmp(argv[1], "print")) {
                Zones_Dump();
            } else  {
                if(argc > 2) {
                    struct ThermalZone *zone;
                    if((zone = Zones_GetZone(atoi(argv[1]) - 1))) {

                        Zones_ZoneCommand(zone, argc - 2, argv + 2);
                        
                    } else {
                        VCP_Printf_P(PSTR("zone %s does not exist\r\n"), argv[1]);
                    }
                } else {
                    VCP_Printf_P(PSTR("zone #nr and subcommand required\r\n"));
                }
            }
        } else {
            VCP_Printf_P(PSTR("%s op required: [ print | enable | disable ]\r\n"), argv[0]);
        }
    } else {
        VCP_Printf_P(PSTR("unknown command '%s'\r\n"), argv[0]);
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
    uint8_t cmdBuf[65];

    uint16_t r = VCP_Read(cmdBuf, sizeof(cmdBuf) - 2);

    if (r > 0) {
        if(monitor_mode) {
            monitor_mode = false;
            VCP_Printf_P(PSTR("Monitor mode off\r\n"));
        }
        cmdBuf[r] = 0;
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
    
    Zones_Init();
    
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
    RfRx_Init();
    
    uint32_t timer = 0;

    uint32_t status = 0;
    uint32_t lcd_reinit = 0;
    uint8_t ignition_count = 0;
    
    
    
    int sensor = 0;
    
    GlobalInterruptEnable();
    
    VCP_Printf_P(PSTR("Booting\r\n"));
    
    for(;;)
    {
        wdt_reset();
        
        USB_Task();        
        CLI_Task();

        RfRx_Task( on_rfrx_sensor_data );
        
        handle_led();
        
        ++force_hid_reports_counter;

        if(force_hid_reports_counter > FORCE_HID_REPORTS) {
            force_hid_reports = 1;
            force_hid_reports_counter = 0;
        }

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
                VCP_Printf_P(PSTR("State:%d [%s], s_A:%u, s_B:%u, s_C:%u, t_Oil:%.1f, t_Water:%.1f, Flame:%d IgnCount:%d\r\n"), state, state_name[state], sensor_a, sensor_b, sensor_c, oil_temperature, water_temperature, (int)burning, (int)ignition_count);
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
        
        Zones_SetCurrent(SENSOR_BINARY, 0, IS_PRESSED(button_b) ? 100 : 250);
        
        Zones_Update();
        
        do_hid_report_03();
        do_hid_report_04();
        
        force_hid_reports = 0;
    }
}


void do_hid_report_03()
{
// HID stuff, report 03, general state

    HID_WOB_Report_03_t report;
    static HID_WOB_Report_03_t prev_report = { 0 };
    
    report.State = state;
    report.Flame = sensor_a/1000;
    report.OilTemperature = cpu_to_le16( oil_temperature * 10 );
    report.WaterTemperature = cpu_to_le16( water_temperature * 10);
    
    report.Inputs = 0;
    if(IS_PRESSED(button_a)) {
        report.Inputs |= WOB_REPORT_INPUT_BUTTON_A;
    }
    if(IS_PRESSED(button_b)) {
        report.Inputs |= WOB_REPORT_INPUT_BUTTON_B;
    }
    if(burning > 0) {
        report.Inputs |= WOB_REPORT_INPUT_BURNING;
    }
    
    report.Outputs = 0;
    if(RELAY_STATE(RELAY_HEATER)) {
        report.Outputs |= WOB_REPORT_OUTPUT_HEATER;
    }
    if(RELAY_STATE(RELAY_AIR)) {
        report.Outputs |= WOB_REPORT_OUTPUT_AIR;
    }
    if(RELAY_STATE(RELAY_FAN)) {
        report.Outputs |= WOB_REPORT_OUTPUT_FAN;
    }
    if(RELAY_STATE(RELAY_SPARK)) {
        report.Outputs |= WOB_REPORT_OUTPUT_SPARK;
    }
    
    if(force_hid_reports || memcmp(&report, &prev_report, sizeof(report))) {
        if(HID_Report(0x03, &report, sizeof(report))) {
            memcpy(&prev_report, &report, sizeof(report));
        }
    }
}

void send_hid_report_04_if_changed(HID_WOB_Report_04_t *report)
{
    static HID_WOB_Report_04_t prev_report[_WOB_REPORT_ZONE_COUNT] = { 0 };

    if(report->Zone >= _WOB_REPORT_ZONE_COUNT) {
        return;
    }
    if(force_hid_reports || memcmp(report, &prev_report[report->Zone], sizeof(*report))) {
        if(HID_Report(0x04, report, sizeof(*report))) {
            memcpy(&prev_report[report->Zone], report, sizeof(prev_report[report->Zone]));
        }
    }
}

void do_hid_report_04()
{
    HID_WOB_Report_04_t report;

    for(uint8_t i = 0; i < _WOB_REPORT_ZONE_COUNT; ++i) {
        struct ThermalZone *zone = Zones_GetZone(i);

        if(!zone) {
            continue;
        }
        
        report.Zone = i;
        report.SetPoint = cpu_to_le16( zone->SetPoint );
        report.Current = cpu_to_le16( zone->Current );
        report.Flags = zone->Flags;
        
        send_hid_report_04_if_changed(&report);
    }
}

void on_rfrx_sensor_data(struct RfRx_SensorData *data)
{
    uint16_t sensor_id = (data->channel << 8) | data->sensor_id;

    int t_int = data->temp / 10;
    unsigned t_frac = abs(data->temp % 10);
    
    char raw[11];
    static const char *hex = "0123456789abcdef";
    
    for(int j = 0, i = 0; i < 5; ++i) { // 5 raw bytes
      uint8_t d = data->_raw[i];
      raw[j++] = hex[d >> 4];
      raw[j++] = hex[d & 0x0f];
    }
    
    raw[10] = 0;

    HID_RfRx_Report_02_t report;
    report.SensorGUID = cpu_to_le16( (data->channel << 8) | data->sensor_id );
    report.Temperature = cpu_to_le16( data->temp );
    report.Humidity = data->humidity;
    report.Battery = data->battery ? 100 : 10;
    
    HID_Report(0x02, &report, sizeof(report));

    if(monitor_mode) {
        VCP_Printf_P(PSTR("{\"signal\":\"%u/%u\",\"batt\":%u,\"sensor_id\":%u,\"temperature\":%d.%u,\"t_%u\":\"%d.%u\",\"humidity\": %u, \"raw\": \"%s\"}\r\n"), data->_matching, data->_samples, data->battery, sensor_id , t_int, t_frac,  sensor_id, t_int, t_frac, data->humidity, raw);
    }

    Zones_SetCurrent(SENSOR_RFRX, report.SensorGUID, report.Temperature);
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


