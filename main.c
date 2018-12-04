#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <string.h>

#include "boot.h"

#include "hw.h"

#if defined(USE_USB_VCP) || defined(USE_USB_HID)
# include "usb_descriptors.h"
//# include <LUFA/Drivers/Board/LEDs.h>
# include <LUFA/Drivers/USB/USB.h>
# include <LUFA/Platform/Platform.h>
# include "usb.h"
#endif

#include "vcp.h"
#include "hid.h"

#include "twi.h"
#include "lcd.h"
#include "led.h"
#include "rx.h"
#include "zones.h"
#include "flame.h"
#include "adc.h"
#include "eeconfig.h"

#include "microrl/src/microrl.h"

#include <stdlib.h>

#ifndef cpu_to_le16
# define cpu_to_le16(x)           (x)
#endif

#define SYM_DEG "\xdf"



//////

#define DEFAULT_TARGET_OIL_TEMPERATURE 90.0
#define DEFAULT_TARGET_WATER_TEMPERATURE 60.0

#define DEFAULT_OIL_TEMP_HYST 1
#define DEFAULT_WATER_TEMP_HYST 15

#define IS_PRESSED(b) ((b) == BUTTON_DEBOUNCE)

#define DELAY_STATUS (500/TICK_MS)

#define LCD_REINIT (30000/TICK_MS)
#define UI_IDLE_TIMEOUT (5000/TICK_MS)

#define BUTTON_DEBOUNCE (30/TICK_MS)
#define FORCE_HID_REPORTS (1000/TICK_MS)



static const char *state_name[] = {
    "Idle",
    "Wait",
    "Prheat",
    "Fan",
    "Air",
    "Spark",
    "Flame?",
    "Burn",
    "Fault",
};

typedef enum {
    UI_MODE_STATUS = 0,
    UI_MODE_SET_OIL,
    UI_MODE_SET_WATER,
    UI_MODE_SET_EXT1,
    UI_MODE_SET_EXT2,
    UI_MODE_SET_EXT3,
    UI_MODE_SET_EXT4,
    _UI_MODE_MAX,
} ui_mode_t;

static ui_mode_t ui_mode = UI_MODE_STATUS;
static uint8_t ui_refresh = 0;

static uint32_t uptime;

static void on_rfrx_sensor_data(struct RfRx_SensorData *data);

static uint16_t force_hid_reports_counter;
static uint8_t force_hid_reports;

static void do_hid_report_03();
static void do_hid_report_04();

static int8_t encoder_event;
static uint8_t monitor_mode;
static microrl_t mrl;

#ifdef BUTTON_A_PORT
static int button_a;
#endif
#ifdef BUTTON_B_PORT
static int button_b;
#endif
#ifdef BUTTON_R_PORT
static int button_r;
#endif    

static void CLI_Dfu()
{
    if(pgm_read_word_near( BOOTLOADER_MAGIC_SIGNATURE_START ) == BOOTLOADER_MAGIC_SIGNATURE) {
        BootloaderAPI_GoDFU();
    }
}

static void CLI_Info()
{
    uint16_t bootloader_signature = pgm_read_word_near( BOOTLOADER_MAGIC_SIGNATURE_START );
    VCP_Printf_P(PSTR("bootloader_signature: %02x\r\n"), bootloader_signature);
    if(bootloader_signature != BOOTLOADER_MAGIC_SIGNATURE) {
        VCP_Printf_P(PSTR("Unknown bootloader type\r\n"));
        return;
    }
    uint16_t bootloader_class = pgm_read_word_near( BOOTLOADER_CLASS_SIGNATURE_START );
    VCP_Printf_P(PSTR("bootloader_class: %02x\r\n"), bootloader_class);
    VCP_Printf_P(PSTR("lock: %u\r\n"), BootloaderAPI_ReadLock());
}


static int CLI_Execute(int argc, const char * const *argv)
{
    VCP_Printf_P(PSTR("\r\n"));

    if(!strcasecmp(argv[0], "monitor")) {
        monitor_mode = 1;
        VCP_Printf_P(PSTR("Monitor mode on\r\n"));
    } else if(!strcasecmp(argv[0], "uptime")) {
        VCP_Printf_P(PSTR("%lu\r\n"), uptime);
    } else if(!strcasecmp(argv[0], "info")) {
        CLI_Info();
    } else if(!strcasecmp(argv[0], "dfu")) {
        CLI_Dfu();
    } else if(!strcasecmp(argv[0], "analog")) {
        if(argc > 1) {
            if(!strcasecmp(argv[1], "print")) {
                // print analog calibration
                VCP_Printf_P(PSTR("analog 1 gain %f offset %f\r\n"), ADC_Config.Calibration[0].gain, ADC_Config.Calibration[0].offset);
                VCP_Printf_P(PSTR("analog 2 gain %f offset %f\r\n"), ADC_Config.Calibration[1].gain, ADC_Config.Calibration[1].offset);
            } else {
                if(argc > 3) {
                    unsigned index = atoi(argv[1]);
                    if(index == 1 || index == 2) {
                        --index;
                        
                        if(!strcasecmp(argv[2], "gain")) {
                            ADC_Config.Calibration[index].gain = atof(argv[3]);
                        } else if(!strcasecmp(argv[2], "offset")) {
                            ADC_Config.Calibration[index].offset = atof(argv[3]);
                        } else {
                            VCP_Printf_P(PSTR("Unknown analog sensor parameter '%s'\r\n"), argv[2]);
                        }
                        
                        EEConfig_Save();
                        
                    } else {
                        VCP_Printf_P(PSTR("Sensors 1 & 2 are supported only\r\n"));
                    }
                    
                } else {
                    
                }
            }
        }
    } else if(!strcasecmp(argv[0], "zone")) {
        if(argc > 1) {
            if(!strcasecmp(argv[1], "print")) {
                Zones_Dump();
            } else  {
                if(argc > 2) {
                    ThermalZone *zone;
                    if((zone = Zones_GetZone(atoi(argv[1]) - 1))) {

                        Zones_ZoneCLI(zone, argc - 2, argv + 2);
                        
                        EEConfig_Save();
                        
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


static void IO_Init()
{
#ifdef LED_A_PORT
    IO_DIR_OUT( LED_A );
#endif
#ifdef LED_B_PORT
    IO_DIR_OUT( LED_B );
#endif

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

#ifdef HAVE_ENCODER
    IO_DIR_IN( BUTTON_R );
    IO_PIN_HIGH( BUTTON_R ); // pullup

    IO_DIR_IN( ENCODER_A );
    IO_PIN_HIGH( ENCODER_A );

    IO_DIR_IN( ENCODER_B );
    IO_PIN_HIGH( ENCODER_B );

    PCMSK0 = _BV( ENCODER_A_PIN ) | _BV( ENCODER_B_PIN );

    /* enable pin change interupts */
    PCICR |= ( 1<<PCIE0 );

#endif
}

static void Init_ThermalZones(void)
{
    ThermalZone *zone;
    
    if( ( zone = Zones_GetZone( ZONE_ID_WATER ) ) ) {
        zone->Config.SetPoint = DEFAULT_TARGET_WATER_TEMPERATURE * 10;
        zone->Current = 0;
        zone->Flags = 0;
        zone->Config.SensorType = SENSOR_ANALOG2;
        zone->Config.Hysteresis = DEFAULT_WATER_TEMP_HYST * 10;
    }
    
    if( ( zone = Zones_GetZone( ZONE_ID_OIL) ) ) {
        zone->Config.SetPoint = DEFAULT_TARGET_OIL_TEMPERATURE * 10;
        zone->Current = 0;
        zone->Flags = 0;
        zone->Config.SensorType = SENSOR_ANALOG1;
        zone->Config.Hysteresis = DEFAULT_OIL_TEMP_HYST * 10;
    }
    
    if( ( zone = Zones_GetZone( ZONE_ID_EXT1 ) ) ) {
        zone->Config.SetPoint = 22 * 10;
        zone->Current = 0;
        zone->Flags = WOB_REPORT_FLAGS_CONTROL_ENABLED;
        zone->Config.SensorType = SENSOR_BINARY;
        zone->Config.Hysteresis = 0.0;
    }
    
    Zones_Init();
}

static void Update_ZoneFlags(void)
{

    if( ( (Zones_GetZone( ZONE_ID_EXT1 )->Flags & WOB_REPORT_FLAGS_OUTPUT_ACTIVE)
        || (Zones_GetZone( ZONE_ID_EXT2 )->Flags & WOB_REPORT_FLAGS_OUTPUT_ACTIVE)
        || (Zones_GetZone( ZONE_ID_EXT3 )->Flags & WOB_REPORT_FLAGS_OUTPUT_ACTIVE)
        || (Zones_GetZone( ZONE_ID_EXT4 )->Flags & WOB_REPORT_FLAGS_OUTPUT_ACTIVE) ) ) {

        Zones_GetZone( ZONE_ID_WATER )->Flags |= WOB_REPORT_FLAGS_CONTROL_ENABLED;

    } else {

        Zones_GetZone( ZONE_ID_WATER )->Flags &= ~WOB_REPORT_FLAGS_CONTROL_ENABLED;

    }
}

static void Update_Outputs()
{
    if(Zones_GetZone(ZONE_ID_OIL)->Flags & WOB_REPORT_FLAGS_OUTPUT_ACTIVE) { RELAY_ON( RELAY_HEATER );    } else { RELAY_OFF( RELAY_HEATER );   }
    // ZONE_ID_WATER is internally connected only
#ifdef RELAY_ZONE_EXT1_PORT
    if(Zones_GetZone(ZONE_ID_EXT1)->Flags & WOB_REPORT_FLAGS_OUTPUT_ACTIVE) { RELAY_ON( RELAY_ZONE_EXT1) ; } else { RELAY_OFF( RELAY_ZONE_EXT1); }
#endif
#ifdef RELAY_ZONE_EXT2_PORT
    if(Zones_GetZone(ZONE_ID_EXT2)->Flags & WOB_REPORT_FLAGS_OUTPUT_ACTIVE) { RELAY_ON( RELAY_ZONE_EXT2) ; } else { RELAY_OFF( RELAY_ZONE_EXT2); }
#endif
#ifdef RELAY_ZONE_EXT3_PORT
    if(Zones_GetZone(ZONE_ID_EXT3)->Flags & WOB_REPORT_FLAGS_OUTPUT_ACTIVE) { RELAY_ON( RELAY_ZONE_EXT3) ; } else { RELAY_OFF( RELAY_ZONE_EXT3); }
#endif
#ifdef RELAY_ZONE_EXT4_PORT
    if(Zones_GetZone(ZONE_ID_EXT4)->Flags & WOB_REPORT_FLAGS_OUTPUT_ACTIVE) { RELAY_ON( RELAY_ZONE_EXT4) ; } else { RELAY_OFF( RELAY_ZONE_EXT4); }
#endif
}


#ifdef HAVE_ENCODER
/* https://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros/ */
/* encoder routine. Expects encoder with four state changes between detents */
/* and both pins open on detent */
ISR( PCINT0_vect )
{
  static uint8_t old_AB = 3;  //lookup table index
  static int8_t encval = 0;   //encoder value  
  static const int8_t enc_states [] PROGMEM = 
  {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};  //encoder lookup table
  /**/

  old_AB <<=2;  //remember previous state

  uint8_t AB = IO_PORT_IN( IO_PORTNAME( ENCODER_A ) );
  if(AB & _BV( IO_PIN( ENCODER_A ) )) {
      old_AB |= (1 << 0);
  }
  if(AB & _BV( IO_PIN( ENCODER_B ) )) {
      old_AB |= (1 << 1);
  }
  encval += pgm_read_byte(&(enc_states[( old_AB & 0x0f )]));
  /* post "Navigation forward/reverse" event */
  if( encval > 3 ) {  //four steps forward
    encoder_event = -1;
    encval = 0;
  }
  else if( encval < -3 ) {  //four steps backwards
    encoder_event = 1;
    encval = 0;
  }
}
#endif

static void Button_Task()
{
#ifdef BUTTON_A_PORT
    if( !IO_PIN_READ( BUTTON_A ) ) {
        if(button_a < BUTTON_DEBOUNCE) {
            ++button_a;
        }
    } else {
        button_a = 0;
    }

    if(IS_PRESSED(button_a)) {
        Flame_Init();
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

    Zones_SetCurrent(SENSOR_BINARY, 0, IS_PRESSED(button_b) ? 100 : 250);
#endif

#ifdef BUTTON_R_PORT
    if( !IO_PIN_READ( BUTTON_R ) ) {
        if(button_r < BUTTON_DEBOUNCE) {
            ++button_r;
        }
    } else {
        button_r = 0;
    }
#endif
}

typedef struct {
    int16_t Min;
    int16_t Max;
    int16_t Step;
    int16_t *Value;
    const char *Name;
} UI_Setpoint;

static void UI_Task()
{
    static uint8_t status = 0;
    static uint16_t lcd_reinit = 0;
    static uint16_t ui_idle = 0;

    static UI_Setpoint setpoint = { 
        .Min = 0,
        .Max = 1200,
        .Step = 1,
        .Name = 0,
    };

    if(lcd_reinit == 0) {
        lcd_bus_error = 1; // force LCD bus error
        lcd_reinit = LCD_REINIT;
    }
    
    --lcd_reinit;

    if(ui_idle == 0) {
        ui_mode = UI_MODE_STATUS;
        ui_refresh = 1;
        
        if(setpoint.Value) {
            EEConfig_Save();
        }
        
        setpoint.Name = 0;
        setpoint.Value = 0;
    } else {
        --ui_idle;
    }

#ifdef HAVE_ENCODER
    static uint8_t was_pressed = 0;

    if(IS_PRESSED( button_r ) && !was_pressed) {
        was_pressed = 1;
        // transition ui

        ++ui_mode;
        if(ui_mode == _UI_MODE_MAX) {
            ui_mode = 0;
        }
        ui_refresh = 1;
        ui_idle = UI_IDLE_TIMEOUT;

        switch(ui_mode) {
            case UI_MODE_STATUS:
                setpoint.Name = 0;
                setpoint.Value = 0;
                break;
            case UI_MODE_SET_OIL:
                setpoint.Name = "Oil";
                setpoint.Value = &(Zones_GetZone(ZONE_ID_OIL)->Config.SetPoint);
                break;
            case UI_MODE_SET_WATER:
                setpoint.Name = "Water"; 
                setpoint.Value = &(Zones_GetZone(ZONE_ID_WATER)->Config.SetPoint);
                break;
            case UI_MODE_SET_EXT1:
                setpoint.Name = "Ext1";
                setpoint.Value = &(Zones_GetZone(ZONE_ID_EXT1)->Config.SetPoint);;
                break;
            case UI_MODE_SET_EXT2:
                setpoint.Name = "Ext2";
                setpoint.Value = &(Zones_GetZone(ZONE_ID_EXT2)->Config.SetPoint);;
                break;
            case UI_MODE_SET_EXT3:
                setpoint.Name = "Ext3";
                setpoint.Value = &(Zones_GetZone(ZONE_ID_EXT3)->Config.SetPoint);
                break;
            case UI_MODE_SET_EXT4:
                setpoint.Name = "Ext4";
                setpoint.Value = &(Zones_GetZone(ZONE_ID_EXT4)->Config.SetPoint);;
                break;
            default:break;
        }
    }

    if(!IS_PRESSED( button_r )) {
        was_pressed = 0;
    }
    

    if(encoder_event) {

        ui_idle = UI_IDLE_TIMEOUT;

        if(setpoint.Value) {

            float nv = *setpoint.Value + (setpoint.Step * encoder_event);

            if(nv >= setpoint.Min && nv <= setpoint.Max) {
                *setpoint.Value = nv;
                ui_refresh = 1;
            }
        }

        encoder_event = 0;
    }
#endif

    ++status;
    if((status > DELAY_STATUS) || (ui_refresh)) {
        status = 0;
        ui_refresh = 0;

        
        if(lcd_bus_error) {
            lcd_init();
        }
        
        ThermalZone *oil = Zones_GetZone(ZONE_ID_OIL),
                  *water = Zones_GetZone(ZONE_ID_WATER);
        
        if(monitor_mode) { // TODO: output JSON for MQTT
            VCP_Printf_P(PSTR("State:%d [%s], s_A:%u, s_B:%u, s_C:%u, t_Oil:%.1f, t_Water:%.1f, Flame:%d IgnCount:%d\r\n"), FlameData.state, state_name[FlameData.state], FlameData.sensor, raw_adc[1], raw_adc[2], (float)oil->Current / 10, (float)water->Current / 10, (int)FlameData.burning, (int)FlameData.ignition_count);
        }


        char ab[] = {
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
#ifdef BUTTON_R_PORT
            IS_PRESSED(button_r) ? 'R' : ' ',
#else
            ' ',
#endif
            0
        };

        lcd_move(0, 0);
        lcd_printf_P(PSTR("F:% 2u %s %-6s"), FlameData.sensor/1000, ab, state_name[FlameData.state]);
        lcd_move(0, 1);
        
        switch(ui_mode) {
            case UI_MODE_STATUS:
                lcd_printf_P(PSTR("O:%3d" SYM_DEG "C  W:%3d" SYM_DEG "C"), (int)oil->Current / 10, (int)water->Current / 10);
                break;
            default:break;
        }

        if(setpoint.Name) {
            int16_t v = *setpoint.Value;
            lcd_printf_P(PSTR("%-8s %3d.%1u" SYM_DEG "C"), setpoint.Name, v / 10, v % 10);
        }

    }
}

int main(void)
{
    wdt_enable(WDTO_1S);
    
#if defined(USE_USB_VCP) || defined(USE_USB_HID)
    USB_Init();
#endif

    VCP_Init();

    IO_Init();
    
    Init_ThermalZones();
    
    microrl_init(&mrl, VCP_Puts);
    microrl_set_execute_callback(&mrl, CLI_Execute);
    microrl_set_complete_callback(&mrl, CLI_GetCompletion);
    
    RELAY_OFF( RELAY_HEATER );
    RELAY_OFF( RELAY_FAN );
    RELAY_OFF( RELAY_AIR );
    RELAY_OFF( RELAY_SPARK );
    
#ifdef LED_A_PORT
    LED_ON( LED_A );
#endif
#ifdef LED_B_PORT
    LED_ON( LED_B );
#endif
    
    ADC_Init();
    
    EEConfig_Load();
    
    twi_init();
    lcd_init();
    RfRx_Init();
    
    Flame_Init();

    
    sei();
    
    VCP_Printf_P(PSTR("Booting\r\n"));
    
    for(;;)
    {
        ++uptime;
        
        wdt_reset();
        
#if defined(USE_USB_VCP) || defined(USE_USB_HID)
        USB_Task();        
#endif

        CLI_Task();

        RfRx_Task( on_rfrx_sensor_data );
        
        handle_led();
        
        ++force_hid_reports_counter;

        if(force_hid_reports_counter > FORCE_HID_REPORTS) {
            force_hid_reports = 1;
            force_hid_reports_counter = 0;
        }
        
        ADC_Task();
        
        _delay_ms(TICK_MS);
        
        UI_Task();
        
        
        Button_Task();
        
        Flame_Task();
        

        Update_ZoneFlags();
        
        Zones_Update(); // run thermostat functions
        
        // Do zone outputs :O
        Update_Outputs();
        
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
    
    report.State = FlameData.state;
    report.Flame = FlameData.sensor/1000;
    report.OilTemperature = cpu_to_le16( Zones_GetZone( ZONE_ID_OIL)->Current );
    report.WaterTemperature = cpu_to_le16( Zones_GetZone( ZONE_ID_WATER )->Current );
    
    report.Inputs = 0;
    if(IS_PRESSED(button_a)) {
        report.Inputs |= WOB_REPORT_INPUT_BUTTON_A;
    }
    if(IS_PRESSED(button_b)) {
        report.Inputs |= WOB_REPORT_INPUT_BUTTON_B;
    }
    if(IS_PRESSED(button_r)) {
        report.Inputs |= WOB_REPORT_INPUT_BUTTON_R;
    }
    if(IS_BURNING()) {
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
    static HID_WOB_Report_04_t prev_report[_WOB_REPORT_ZONE_COUNT] = { { 0 } };

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
        ThermalZone *zone = Zones_GetZone(i);

        if(!zone) {
            continue;
        }
        
        report.Zone = i;
        report.SetPoint = cpu_to_le16( zone->Config.SetPoint );
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

#ifdef USE_USB_VCP

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


#endif /* USE_USB_VCP */
