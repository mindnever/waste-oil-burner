#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "hw.h"
#include "hid.h"
#include "zones.h"
#include "flame.h"
#include "led.h"
#include "vcp.h"
#include "eeconfig.h"
#include "relay.h"

#define TIMEOUT_FAN        (FlameConfiguration.fan_time / TICK_MS)
#define TIMEOUT_AIR        (FlameConfiguration.air_time / TICK_MS)
#define TIMEOUT_SPARK      (FlameConfiguration.spark_time / TICK_MS)
#define TIMEOUT_FLAME      (FlameConfiguration.detect_time / TICK_MS)
//#define TIMEOUT_NOOIL      (FlameConfiguration.nooil_time / TICK_MS)
#define TIMEOUT_NOOIL ((5*60*1000L)/TICK_MS)
#define IGNITION_RETRY     (FlameConfiguration.retry_count)

#define FLAME_SENSOR_DELAY (FlameConfiguration.flame_time / TICK_MS)

static struct {
    bool fan;
    bool air;
    bool spark;
} relay_state[_state_count] = {
    [state_fan]   =        {
        .fan = true
    },
    [state_air]   =        {
        .fan = true,
        .air = true,
    },
    [state_spark] =        {
        .fan   = true,
        .air   = true,
        .spark = true,
    },
    [state_detect_flame] = {
        .fan = true,
        .air = true,
    },
    [state_burn] =         {
        .fan = true,
        .air = true,
    },
    [state_nooil] = {
        .fan = false // do not..
    }
};

static uint32_t timer = 0;

struct FlameData FlameData;
struct FlameConfiguration FlameConfiguration;

void Flame_Init(void)
{
    FlameData.fault_P = 0;

    Flame_Reset();

    FlameConfiguration.fan_time    = 10000;
    FlameConfiguration.air_time    = 150;
    FlameConfiguration.spark_time  = 1000;
    FlameConfiguration.detect_time = 5000;
    FlameConfiguration.flame_time  = 3000;
    FlameConfiguration.retry_count = 3;
    FlameConfiguration.flame_trig  = 61000;
    FlameConfiguration.manage_oil  = true;
    FlameConfiguration.flame_lpf   = 0.0;
}

void Flame_Reset(void)
{
    FlameData.state = state_idle;
    FlameData.ignition_count = 0;
}

void Flame_Task(void)
{
    ThermalZone *water = Zones_GetZone(ZONE_ID_WATER);
    ThermalZone *oil   = Zones_GetZone(ZONE_ID_OIL);

    if(NO_OIL()) {
        if((FlameData.state != state_nooil) && (FlameData.state != state_fault)) {
            timer = 0;
            FlameData.state = state_nooil;
        }
    } else if (!water->Active) {
        if (FlameData.state != state_fault) {
            FlameData.state = state_idle;
        }
    } else {
        if (FlameData.state == state_idle) {
            FlameData.state = state_preheat;
        }
    }


    // flame state machine
    switch (FlameData.state) {
    case state_fault:
        led_a = blink_slow;
        led_b = blink_slow;

        // turn off oil heater
        if (FlameConfiguration.manage_oil) {
            oil->Config.Enabled = false;
        }
        break;

    case state_idle:
        led_a = blink_off;
        led_b = blink_slow;

        timer = 0;

        // turn off oil heater
        if (FlameConfiguration.manage_oil) {
            oil->Config.Enabled = false;
        }
        break;

    case state_wait:
    case state_preheat: // waiting for oil to be *not* cold (hot oil trigger is for oil heating state machine)

        if (FlameConfiguration.manage_oil) {
            oil->Config.Enabled = true;
        }

        if (!Zones_IsCold(ZONE_ID_OIL)) {
            FlameData.state = state_fan;
            timer = 0;
            ++FlameData.ignition_count;
        }
        break;

    case state_fan:
        if (++timer > TIMEOUT_FAN) {
            FlameData.state = state_air;
            timer = 0;
        }
        break;

    case state_air:
        if (++timer > TIMEOUT_AIR) {
            FlameData.state = state_spark;
            led_a = blink_fast;
            timer = 0;
        }
        break;

    case state_spark:
        if (++timer > TIMEOUT_SPARK) {
            FlameData.state = state_detect_flame;
            led_a = blink_off;
            led_b = blink_fast;
            timer = 0;
        }
        break;
    case state_detect_flame:
        if (++timer > TIMEOUT_FLAME) {
            if (FlameData.ignition_count > IGNITION_RETRY) {
                FlameData.state   = state_fault;
                FlameData.fault_P = PSTR("No flame");
            } else {
                FlameData.state = state_idle;
            }
            timer = 0;
        } else if (IS_BURNING()) {
            led_a = blink_off;
            led_b = blink_slow;

            FlameData.state = state_burn;
            timer = 0;
            FlameData.ignition_count = 0;
        }
        break;
    case state_burn:
        if (IS_BURNING()) {
            FlameData.burning = FLAME_SENSOR_DELAY;
        } else if (FlameData.burning) {
            --FlameData.burning;
        }

        if (FlameData.burning == 0) {
            FlameData.state = state_idle;
        }
        break;
    case state_nooil:
        // everything is off, except AIR
        if (FlameConfiguration.manage_oil) {
            oil->Config.Enabled = false;
        }
        if (++timer > TIMEOUT_NOOIL) {
            FlameData.state = state_fault;
            timer = 0;
        } else if(!NO_OIL()) {
            FlameData.state = state_idle;
        }

        break;
    }

    Relay_Set(RELAY_AIR, relay_state[FlameData.state].air);
    Relay_Set(RELAY_FAN, relay_state[FlameData.state].fan);
    Relay_Set(RELAY_SPARK, relay_state[FlameData.state].spark);
}

void Flame_CLI(int argc, const char *const *argv)
{
    bool unknown = false;

    if (argc == 0) {
        printf_P(PSTR("?? op\n"));
        return;
    }

    if (!strcasecmp_P(argv[0], PSTR("print"))) {
        printf_P(PSTR("flame fan_time %u\n"
                      "flame air_time %u\n"
                      "flame spark_time %u\n"
                      "flame detect_time %u\n"
                      "flame flame_time %u\n"
                      "flame flame_trig %u ; current %u\n"
                      "flame flame_lpf %.4f\n"
                      "flame retry_count %u\n"
                      "flame manage_oil %s\n"
                      ), FlameConfiguration.fan_time,
                 FlameConfiguration.air_time,
                 FlameConfiguration.spark_time,
                 FlameConfiguration.detect_time,
                 FlameConfiguration.flame_time,
                 FlameConfiguration.flame_trig,
                 FlameData.sensor,
                 FlameConfiguration.flame_lpf,
                 (uint16_t)FlameConfiguration.retry_count,
                 FlameConfiguration.manage_oil ? "yes" : "no"
                 );
    } else if (!strcasecmp_P(argv[0], PSTR("reset"))) {
        Flame_Reset();
    } else {
        if (argc > 1) {
            int val = atoi(argv[1]);
            if (!strcasecmp_P(argv[0], PSTR("fan_time"))) {
                FlameConfiguration.fan_time = val;
            } else if (!strcasecmp_P(argv[0], PSTR("air_time"))) {
                FlameConfiguration.air_time = val;
            } else if (!strcasecmp_P(argv[0], PSTR("spark_time"))) {
                FlameConfiguration.spark_time = val;
            } else if (!strcasecmp_P(argv[0], PSTR("detect_time"))) {
                FlameConfiguration.detect_time = val;
            } else if (!strcasecmp_P(argv[0], PSTR("flame_time"))) {
                FlameConfiguration.flame_time = val;
            } else if (!strcasecmp_P(argv[0], PSTR("flame_trig"))) {
                FlameConfiguration.flame_trig = val;
            } else if (!strcasecmp_P(argv[0], PSTR("flame_lpf"))) {
                FlameConfiguration.flame_lpf = atof(argv[1]);
            } else if (!strcasecmp_P(argv[0], PSTR("retry_count"))) {
                FlameConfiguration.retry_count = val;
            } else if (!strcasecmp_P(argv[0], PSTR("manage_oil"))) {
                if (!strcasecmp_P(argv[1], PSTR("yes"))) {
                    FlameConfiguration.manage_oil = true;
                } else if (!strcasecmp_P(argv[1], PSTR("no"))) {
                    FlameConfiguration.manage_oil = false;
                }
            } else {
                unknown = true;
            }
            EEConfig_Save();
        } else {
            unknown = true;
        }
    }

    if (unknown) {
        printf_P(PSTR("?? '%s'\n"), argv[0]);
    }
}
