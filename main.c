#include <util/delay.h>
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

#define LCD_REINIT_COUNT 10

#define BUTTON_DEBOUNCE (100/TICK_MS)
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

static uint32_t uptime;

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
#ifdef BUTTON_R_PORT
    IO_DIR_IN( BUTTON_R );
    IO_PIN_HIGH( BUTTON_R ); // pullup
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

    uint32_t status = 0;
    uint32_t lcd_reinit = 0;
    
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
        
        
        ++status;
        if(status > DELAY_STATUS) {
            if(lcd_reinit == 0) {
                lcd_bus_error = 1; // force LCD bus error
                lcd_reinit = LCD_REINIT_COUNT;
            }
            
            --lcd_reinit;
            
            if(lcd_bus_error) {
                lcd_init();
            }
            
            ThermalZone *oil = Zones_GetZone(ZONE_ID_OIL),
                      *water = Zones_GetZone(ZONE_ID_WATER);
            
            if(monitor_mode) { // TODO: output JSON for MQTT
                VCP_Printf_P(PSTR("State:%d [%s], s_A:%u, s_B:%u, s_C:%u, t_Oil:%.1f, t_Water:%.1f, Flame:%d IgnCount:%d\r\n"), FlameData.state, state_name[FlameData.state], FlameData.sensor, raw_adc[1], raw_adc[2], (float)oil->Current / 10, (float)water->Current / 10, (int)FlameData.burning, (int)FlameData.ignition_count);
            }
            status = 0;

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
            lcd_printf("F:% 2u %s %-6s", FlameData.sensor/1000, ab, state_name[FlameData.state]);
            lcd_move(0, 1);
            lcd_printf("O:%3d" SYM_DEG "C W:%3d" SYM_DEG "C", (int)oil->Current / 10, (int)water->Current / 10);
        }
        
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
