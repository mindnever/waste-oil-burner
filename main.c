#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <string.h>

#include "hw.h"

#if defined(USE_USB_VCP) || defined(USE_USB_HID)
# include "usb_descriptors.h"
// # include <LUFA/Drivers/Board/LEDs.h>
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
#include "tx.h"
#include "zones.h"
#include "flame.h"
#include "adc.h"
#include "eeconfig.h"
#include "relay.h"
#include "usart.h"
#include "mqtt.h"
#include "cli.h"

#include <stdlib.h>
#include <stdio.h>

#ifndef cpu_to_le16
# define cpu_to_le16(x) (x)
#endif

#define SYM_DEG                          "\x01"


//////

#define DEFAULT_TARGET_OIL_TEMPERATURE   90.0
#define DEFAULT_TARGET_WATER_TEMPERATURE 60.0

#define DEFAULT_OIL_TEMP_HYST            1
#define DEFAULT_WATER_TEMP_HYST          15

#define DELAY_STATUS                     (500 / TICK_MS)
#define CYCLE_TEMP                       (3000 / TICK_MS)

#define LCD_REINIT                       (30000 / TICK_MS)
#define UI_IDLE_TIMEOUT                  (5000 / TICK_MS)

#define HID_TIME                         (750 / TICK_MS)
#define HID_FORCE                        (5000 / TICK_MS)
#define SEQ_TICKS                        (1000 / TICK_MS)

// #define MASTER_ENABLE IS_PRESSED(button_b)
#define MASTER_ENABLE                    true

static const char idle_PSTR[] PROGMEM = "idle";
static const char wait_PSTR[] PROGMEM = "wait";
static const char prheat_PSTR[] PROGMEM = "prheat";
static const char fan_PSTR[] PROGMEM = "fan";
static const char air_PSTR[] PROGMEM = "air";
static const char spark_PSTR[] PROGMEM = "spark";
static const char flame_PSTR[] PROGMEM = "flame?";
static const char burn_PSTR[] PROGMEM = "burn";
static const char fault_PSTR[] PROGMEM = "fault";
static const char nooil_PSTR[] PROGMEM = "oil?";


static const char *state_name[] = {
    [state_idle]    = idle_PSTR,
    [state_wait]    = wait_PSTR,
    [state_preheat] = prheat_PSTR,
    [state_fan]     = fan_PSTR,
    [state_air]     = air_PSTR,
    [state_spark]   = spark_PSTR,
    [state_detect_flame] = flame_PSTR,
    [state_burn]    = burn_PSTR,
    [state_fault]   = fault_PSTR,
    [state_nooil]     = nooil_PSTR,
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


uint8_t mcusr_save = 0;

static uint16_t lcd_reinit = 0;

static ui_mode_t ui_mode   = UI_MODE_STATUS;
static uint8_t ui_refresh  = 0;

static void on_rfrx_sensor_data(struct RfRx_SensorData *data);
// #ifdef USE_MQTT
// static void on_mqtt_msg(const char *topic, const char *msg);
// #endif

static void do_hid_report_03();
static void do_hid_report_04();

bool force_hid_reports = false;

#ifdef HAVE_ENCODER
static int8_t encoder_event;
#endif

#ifdef BUTTON_A_PORT
uint16_t button_a;
#endif
#ifdef BUTTON_B_PORT
uint16_t button_b;
#endif
#ifdef BUTTON_R_PORT
uint16_t button_r;
#endif

static void IO_Init()
{
#ifdef LED_A_PORT
    IO_DIR_OUT(LED_A);
#endif
#ifdef LED_B_PORT
    IO_DIR_OUT(LED_B);
#endif

#ifdef BUTTON_A_PORT
    IO_DIR_IN(BUTTON_A);
    IO_PIN_HIGH(BUTTON_A); // pullup
#endif
#ifdef BUTTON_B_PORT
    IO_DIR_IN(BUTTON_B);
    IO_PIN_HIGH(BUTTON_B); // pullup
#endif

#ifdef HAVE_ENCODER
    IO_DIR_IN(BUTTON_R);
    IO_PIN_HIGH(BUTTON_R); // pullup

    IO_DIR_IN(ENCODER_A);
    IO_PIN_HIGH(ENCODER_A);

    IO_DIR_IN(ENCODER_B);
    IO_PIN_HIGH(ENCODER_B);

    PCMSK0 = _BV(ENCODER_A_PIN) | _BV(ENCODER_B_PIN);

    /* enable pin change interupts */
    PCICR |= (1 << PCIE0);

#endif
}

static void Init_ThermalZones(void)
{
    for (int i = 0; i < NUM_ZONES; ++i) {
        ThermalZone *zone = Zones_GetZone(i);

        memset(zone, 0, sizeof(*zone));

        switch (i) {
        case ZONE_ID_WATER:
            zone->Config.SetPoint   = DEFAULT_TARGET_WATER_TEMPERATURE * 10;
            zone->Config.SensorType = SENSOR_ANALOG;
            zone->Config.SensorID   = 2;
            zone->Config.Hysteresis = DEFAULT_WATER_TEMP_HYST * 10;
            break;
        case ZONE_ID_OIL:
            zone->Config.SetPoint   = DEFAULT_TARGET_OIL_TEMPERATURE * 10;
            zone->Config.SensorType = SENSOR_ANALOG;
            zone->Config.SensorID   = 1;
            zone->Config.Hysteresis = DEFAULT_OIL_TEMP_HYST * 10;
            zone->Config.Hysteresis = DEFAULT_OIL_TEMP_HYST * 10;
            zone->Config.Hysteresis = DEFAULT_OIL_TEMP_HYST * 10;
            break;
        }
    }
}

static void Update_ZoneFlags(void)
{
    ThermalZone *zone;

    Zones_GetZone(ZONE_ID_WATER)->Config.Enabled = false;

    for (enum ZoneID id = ZONE_ID_EXT1; id <= ZONE_ID_EXT4; ++id) {
        if ((zone = Zones_GetZone(id))) {
            Zones_GetZone(ZONE_ID_WATER)->Config.Enabled |= zone->Active;
        }
    }
}

#ifdef HAVE_ENCODER
/* https://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros/ */
/* encoder routine. Expects encoder with four state changes between detents */
/* and both pins open on detent */
ISR(PCINT0_vect) {
    static uint8_t old_AB = 3; // lookup table index
    static int8_t encval = 0; // encoder value
    static const int8_t enc_states[] PROGMEM =
    { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 }; // encoder lookup table

    /**/

    old_AB <<= 2; // remember previous state

    uint8_t AB = IO_PORT_IN(IO_PORTNAME(ENCODER_A));

    if (AB & _BV(IO_PIN(ENCODER_A))) {
        old_AB |= (1 << 0);
    }
    if (AB & _BV(IO_PIN(ENCODER_B))) {
        old_AB |= (1 << 1);
    }
    encval += pgm_read_byte(&(enc_states[(old_AB & 0x0f)]));
    /* post "Navigation forward/reverse" event */
    if (encval > 3) { // four steps forward
        encoder_event = -1;
        encval = 0;
    } else if (encval < -3) { // four steps backwards
        encoder_event = 1;
        encval = 0;
    }
}
#endif /* ifdef HAVE_ENCODER */

static void Button_Task()
{
#ifdef BUTTON_A_PORT
    if (!IO_PIN_READ(BUTTON_A)) {
        if (button_a < BUTTON_MAX) {
            ++button_a;
        }
    } else {
        button_a = 0;
    }

    if (IS_PRESSED(button_a)) {
        Flame_Reset();
    }
#endif

#ifdef BUTTON_B_PORT
    if (!IO_PIN_READ(BUTTON_B)) {
        if (button_b < BUTTON_MAX) {
            ++button_b;
        }
    } else {
        button_b = 0;
    }

    Zones_SetCurrent(SENSOR_BUTTON, 0, IS_PRESSED(button_b) ? 100 : 250);
#endif

#ifdef BUTTON_R_PORT
    if (!IO_PIN_READ(BUTTON_R)) {
        if (button_r < BUTTON_MAX) {
            ++button_r;
        }
    } else {
        button_r = 0;
    }

    if (IS_LONGPRESS(button_r)) {
        Flame_Reset();
    }
    if (button_r == BUTTON_RESTORE) {
        printf_P(PSTR("button_r: %u\n"), button_r);
        EEConfig_Restore();
    }
#endif
}

typedef struct {
    int16_t    Min;
    int16_t    Max;
    int16_t    Step;
    int16_t    *Value;
    const char *Name;
} UI_Setpoint;

static void UI_Task()
{
    static uint8_t status       = 0;

    static uint16_t ui_idle     = 0;
    static uint8_t current_zone = 0;
    static uint16_t cycle       = 0;

    static UI_Setpoint setpoint = {
        .Min  = 0,
        .Max  = 1200,
        .Step = 1,
        .Name = 0,
    };

    if (lcd_reinit == 0) {
        lcd_bus_error = 1; // force LCD bus error
        lcd_reinit    = LCD_REINIT;
    }

    --lcd_reinit;

    if (ui_idle == 0) {
        if (ui_mode != UI_MODE_STATUS) {
            ui_mode    = UI_MODE_STATUS;
            ui_refresh = 1;
        }

        if (setpoint.Value) {
            EEConfig_Save();
        }

        setpoint.Name  = 0;
        setpoint.Value = 0;
    } else {
        --ui_idle;
    }

#ifdef HAVE_ENCODER
    static uint8_t was_pressed = 0;

    if (IS_PRESSED(button_r) && !was_pressed) {
        was_pressed = 1;
        // transition ui

        ++ui_mode;
        if (ui_mode == _UI_MODE_MAX) {
            ui_mode = 0;
        }
        ui_refresh = 1;
        ui_idle    = UI_IDLE_TIMEOUT;

        switch (ui_mode) {
        case UI_MODE_STATUS:
            setpoint.Name  = 0;
            setpoint.Value = 0;
            break;
        case UI_MODE_SET_OIL:
            setpoint.Name  = PSTR("oil");
            setpoint.Value = &(Zones_GetZone(ZONE_ID_OIL)->Config.SetPoint);
            break;
        case UI_MODE_SET_WATER:
            setpoint.Name  = PSTR("water");
            setpoint.Value = &(Zones_GetZone(ZONE_ID_WATER)->Config.SetPoint);
            break;
        case UI_MODE_SET_EXT1:
            setpoint.Name  = PSTR("ext1");
            setpoint.Value = &(Zones_GetZone(ZONE_ID_EXT1)->Config.SetPoint);;
            break;
        case UI_MODE_SET_EXT2:
            setpoint.Name  = PSTR("ext2");
            setpoint.Value = &(Zones_GetZone(ZONE_ID_EXT2)->Config.SetPoint);;
            break;
        case UI_MODE_SET_EXT3:
            setpoint.Name  = PSTR("ext3");
            setpoint.Value = &(Zones_GetZone(ZONE_ID_EXT3)->Config.SetPoint);
            break;
        case UI_MODE_SET_EXT4:
            setpoint.Name  = PSTR("ext4");
            setpoint.Value = &(Zones_GetZone(ZONE_ID_EXT4)->Config.SetPoint);;
            break;
        default: break;
        }
    }

    if (!IS_PRESSED(button_r)) {
        was_pressed = 0;
    }


    if (encoder_event) {
        ui_idle = UI_IDLE_TIMEOUT;

        if (setpoint.Value) {
            float nv = *setpoint.Value + (setpoint.Step * encoder_event);

            if (nv >= setpoint.Min && nv <= setpoint.Max) {
                *setpoint.Value = nv;
                ui_refresh = 1;
            }
        }

        encoder_event = 0;
    }
#endif /* ifdef HAVE_ENCODER */

    ++cycle;
    ++status;
    if ((status > DELAY_STATUS) || (ui_refresh)) {
        status     = 0;
        ui_refresh = 0;


        if (lcd_bus_error) {
            lcd_init();
        }

// ThermalZone *oil = Zones_GetZone(ZONE_ID_OIL);

        static const char zn[] = {
            "OW1234"
        };
        static const enum ZoneID zi[] = {
            ZONE_ID_OIL, ZONE_ID_WATER, ZONE_ID_EXT1, ZONE_ID_EXT2, ZONE_ID_EXT3, ZONE_ID_EXT4
        };


        if (cycle > CYCLE_TEMP) {
            cycle = 0;

            for (int i = 0; i < 5; ++i) {
                ++current_zone;


                if (!zn[current_zone]) {
                    current_zone = 0;
                }

                if (Zones_GetZone(zi[current_zone])->Config.SensorType != SENSOR_NONE) {
                    break;
                }
            }
        }

        ThermalZone *zone = Zones_GetZone(zi[current_zone]);

        char ab[10] = {
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

#if 0
        mqtt_publish_P("debug", PSTR("{ \"state\":\"%d/%S\", \"adc0\":\"%u\", \"adc1\":\"%u\", \"adc2\":\"%u\", \"adc3\":\"%u\", \"flame\":\"%d\", \"ignCount\":\"%d\", \"buttons\":\"%s\" }"),
                       FlameData.state, state_name[FlameData.state],
                       raw_adc[0],
                       raw_adc[1],
                       raw_adc[2],
                       raw_adc[3],
                       (int)FlameData.burning, (int)FlameData.ignition_count, ab);
#endif

        lcd_move(0, 0);
        lcd_printf_P(PSTR("F:%02u %s %-6S"), FlameData.sensor / 1000, ab, state_name[FlameData.state]);
        lcd_move(0, 1);

        switch (ui_mode) {
        case UI_MODE_STATUS:
            snprintf_P(ab, sizeof(ab), PSTR("%.1f"), (float)zone->Current / 10);
// lcd_printf_P(PSTR("O:%3d" SYM_DEG "C  %c:%3d" SYM_DEG "C"), (int)oil->Current / 10, zn[current_zone], (int)zone->Current / 10);
            lcd_printf_P(PSTR("%c:%s" SYM_DEG "C (%.1f)   "), zn[current_zone], zone->Valid ? ab : "--.-", (float)zone->Config.SetPoint / 10);
            break;
        default: break;
        }

        if (setpoint.Name) {
            int16_t v = *setpoint.Value;
            lcd_printf_P(PSTR("%-8S %3d.%1u" SYM_DEG "C"), setpoint.Name, v / 10, v % 10);
        }
    }
}


int main(void)
{
    mcusr_save = MCUSR;
    MCUSR = 0;

    wdt_enable(WDTO_1S);

#if defined(USE_USB_VCP) || defined(USE_USB_HID)
    USB_Init();
#endif
    FILE *usart = USART_Init();
#ifdef USE_MQTT
    mqtt  = usart;
#endif
#if defined(USE_USB_VCP)
    stdin = stdout = VCP_Init();
#else
    stdin = stdout = usart;
#endif

    fdev_set_udata(stdout, (void *)true);

    CLI_Init();

    RfRx_Init(); // This is important as it shares timer with Sys_Idle
    IO_Init();

    Relay_Init();

    Init_ThermalZones();


#ifdef LED_A_PORT
    LED_ON(LED_A);
#endif
#ifdef LED_B_PORT
    LED_ON(LED_B);
#endif

    ADC_Init();


#ifdef LCD_DATA_PORT
    lcd_gpio_init();
#else
    twi_init();
#endif
    lcd_init();


// RfTx_Init();

    Flame_Init();

    EEConfig_Load();

    sei();

// int seq = 0, ping = 0;
    int hid_time  = 0;
    int hid_force = 0;

    state_t flame_state = FlameData.state;

    for (;;) {
        Sys_Idle();

        wdt_reset();

#if defined(USE_USB_VCP) || defined(USE_USB_HID)
        USB_Task();
#endif

        CLI_Task();


        handle_led();

        RfRx_Task(on_rfrx_sensor_data);


        ADC_Task();

        UI_Task();

        Button_Task();

        Flame_Task();

        if (flame_state != FlameData.state) {
#ifdef USE_MQTT
            mqtt_publish_P("flame/state", PSTR("%S"), state_name[FlameData.state]);
#endif
            CLI_notify_P(PSTR("FLAME"), FlameData.state == state_fault ? PSTR("state => %S => %S") : PSTR("state => %S"), state_name[FlameData.state], FlameData.fault_P);

            flame_state = FlameData.state;
        }


        Update_ZoneFlags();

        Zones_Update(MASTER_ENABLE); // run thermostat functions

#if defined(USE_USB_HID) || defined(USE_MQTT)
        if (hid_force++ > HID_FORCE) {
            force_hid_reports = true;
            hid_force = 0;
        }

        if (hid_time++ > HID_TIME) {
            do_hid_report_03();
            do_hid_report_04();

            force_hid_reports = false;
            hid_force = 0;
            hid_time = 0;
        }
#endif
#ifdef USE_MQTT
        mqtt_task(0);
#endif
    }
}
// #ifdef USE_MQTT
// static void on_mqtt_msg(const char *topic, const char *msg)
// {
// CLI_Erase();
// printf_P(PSTR("on_mqtt_msg: topic=%s msg=%s\n"), topic, msg);
// CLI_Redraw();
// }
// #endif

#if defined(USE_USB_HID) || defined(USE_MQTT)
void do_hid_report_03()
{
// HID stuff, report 03, general state

    HID_WOB_Report_03_t report;
    static HID_WOB_Report_03_t prev_report = { 0 };

    report.State = FlameData.state;
    report.Flame = FlameData.sensor / 1000;
    report.OilTemperature   = cpu_to_le16(Zones_GetZone(ZONE_ID_OIL)->Current);
    report.WaterTemperature = cpu_to_le16(Zones_GetZone(ZONE_ID_WATER)->Current);

    report.Inputs = 0;
#ifdef BUTTON_A_PORT
    if (IS_PRESSED(button_a)) {
        report.Inputs |= WOB_REPORT_INPUT_BUTTON_A;
    }
#endif
#ifdef BUTTON_B_PORT
    if (IS_PRESSED(button_b)) {
        report.Inputs |= WOB_REPORT_INPUT_BUTTON_B;
    }
#endif
#ifdef BUTTON_R_PORT
    if (IS_PRESSED(button_r)) {
        report.Inputs |= WOB_REPORT_INPUT_BUTTON_R;
    }
#endif
    if (IS_BURNING()) {
        report.Inputs |= WOB_REPORT_INPUT_BURNING;
    }

    report.Outputs = 0;
    if (Relay_State(RELAY_HEATER)) {
        report.Outputs |= WOB_REPORT_OUTPUT_HEATER;
    }
    if (Relay_State(RELAY_AIR)) {
        report.Outputs |= WOB_REPORT_OUTPUT_AIR;
    }
    if (Relay_State(RELAY_FAN)) {
        report.Outputs |= WOB_REPORT_OUTPUT_FAN;
    }
    if (Relay_State(RELAY_SPARK)) {
        report.Outputs |= WOB_REPORT_OUTPUT_SPARK;
    }
    if (Zones_GetZone(ZONE_ID_EXT1)->Active) {
        report.Outputs |= WOB_REPORT_OUTPUT_EXT1;
    }
    if (Zones_GetZone(ZONE_ID_EXT2)->Active) {
        report.Outputs |= WOB_REPORT_OUTPUT_EXT2;
    }
    if (Zones_GetZone(ZONE_ID_EXT3)->Active) {
        report.Outputs |= WOB_REPORT_OUTPUT_EXT3;
    }
    if (Zones_GetZone(ZONE_ID_EXT4)->Active) {
        report.Outputs |= WOB_REPORT_OUTPUT_EXT4;
    }

    if (force_hid_reports || memcmp(&report, &prev_report, sizeof(report))) {
#ifdef USE_MQTT
        mqtt_hid(0x03, (uint8_t *)&report, sizeof(report));
#endif
#if defined(USE_USB_HID)
        if (HID_Report(0x03, &report, sizeof(report))) {
            memcpy(&prev_report, &report, sizeof(report));
        }
#endif

        memcpy(&prev_report, &report, sizeof(report));
    }
}

void send_hid_report_04_if_changed(HID_WOB_Report_04_t *report)
{
    static HID_WOB_Report_04_t prev_report[_WOB_REPORT_ZONE_COUNT] = {
        { 0 }
    };

    if (report->Zone >= _WOB_REPORT_ZONE_COUNT) {
        return;
    }
    if (force_hid_reports || memcmp(report, &prev_report[report->Zone], sizeof(*report))) {
#ifdef USE_MQTT
        mqtt_hid(0x04, (uint8_t *)report, sizeof(*report));
#endif
#if defined(USE_USB_HID)
        if (HID_Report(0x04, report, sizeof(*report))) {
            memcpy(&prev_report[report->Zone], report, sizeof(prev_report[report->Zone]));
        }
#endif

        memcpy(&prev_report[report->Zone], report, sizeof(prev_report[report->Zone]));
    }
}

void do_hid_report_04()
{
    HID_WOB_Report_04_t report;

    for (uint8_t i = 0; i < _WOB_REPORT_ZONE_COUNT; ++i) {
        ThermalZone *zone = Zones_GetZone(i);

        if (!zone) {
            continue;
        }

        report.Zone     = i;
        report.SetPoint = cpu_to_le16(zone->Config.SetPoint);
        report.Current  = cpu_to_le16(zone->Current);
        report.Flags    = (zone->Active ? WOB_REPORT_FLAGS_OUTPUT_ACTIVE : 0) | (zone->Config.Enabled ? WOB_REPORT_FLAGS_CONTROL_ENABLED : 0);

        send_hid_report_04_if_changed(&report);
    }
}
#endif /* USE_USB_HID || USE_MQTT */

void on_rfrx_sensor_data(struct RfRx_SensorData *data)
{
// uint16_t sensor_id = (data->channel << 8) | data->sensor_id;

    HID_RfRx_Report_02_t report;

    report.SensorGUID  = cpu_to_le16((data->channel << 8) | data->sensor_id);
    report.Temperature = cpu_to_le16(data->temp);
    report.Humidity    = data->humidity;
    report.Battery     = data->battery ? 100 : 10;

#if defined(USE_USB_HID)
    HID_Report(0x02, &report, sizeof(report));
#endif // USE_USB_HID
#if defined(USE_MQTT)
    mqtt_hid(0x02, (uint8_t *)&report, sizeof(report));

    char topic[16];
    snprintf_P(topic, sizeof(topic), PSTR("rfrx/%u.%u"), data->sensor_id, data->channel);

    mqtt_publish_P(topic, PSTR("{\"id\":\"%u\",\"battery\":\"%u\",\"channel\":\"%u\", \"humidity\":\"%u\", \"temperature\":\"%.1f\" }"),
                   data->sensor_id, data->battery, data->channel, data->humidity, (float)data->temp / 10);
#endif

    Zones_SetCurrent(SENSOR_RFRX, report.SensorGUID, report.Temperature);

    CLI_notify_P(PSTR("RFRX"), PSTR("guid %u id %u chan %u temp %.1f"), report.SensorGUID, data->sensor_id, data->channel, (float)data->temp / 10);
}

#ifdef USE_USB_VCP

// void EVENT_VCP_SetLineEncoding(CDC_LineEncoding_t *LineEncoding)
// {
////    mqtt_publish_P("vcp/setlineencoding", PSTR("{\"baud\": \"%lu\" }"), LineEncoding->BaudRateBPS);
// }


// void EVENT_VCP_DataReceived()
// {
// }

// void EVENT_VCP_SetControlLineState(uint16_t State)
// {
//// ESP8266:
//// DTR -> RESET
//// RTS -> GPIO0
////
////  mqtt_publish_P("vcp/setcontrollinestate", PSTR("{\"state\": \"%04x\", \"dtr\":\"%u\", \"rts\":\"%u\" }"), State, State & CDC_CONTROL_LINE_OUT_DTR ? 1 : 0, State & CDC_CONTROL_LINE_OUT_RTS ? 1 : 0);
// }


#endif /* USE_USB_VCP */
