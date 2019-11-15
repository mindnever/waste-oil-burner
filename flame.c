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

#define TIMEOUT_FAN   (FlameConfiguration.fan_time / TICK_MS)
#define TIMEOUT_AIR   (FlameConfiguration.air_time / TICK_MS)
#define TIMEOUT_SPARK (FlameConfiguration.spark_time / TICK_MS)
#define TIMEOUT_FLAME (FlameConfiguration.detect_time / TICK_MS)

#define IGNITION_RETRY (FlameConfiguration.retry_count)

#define FLAME_SENSOR_DELAY (FlameConfiguration.flame_time / TICK_MS)



static uint32_t timer = 0;

struct FlameData FlameData;
struct FlameConfiguration FlameConfiguration;

void Flame_Init(void)
{
    Flame_Reset();
    
    FlameConfiguration.fan_time = 10000;
    FlameConfiguration.air_time = 150;
    FlameConfiguration.spark_time = 6000;
    FlameConfiguration.detect_time = 4000;
    FlameConfiguration.flame_time = 3000;
    FlameConfiguration.retry_count = 3;
    FlameConfiguration.flame_trig = 61000;
}

void Flame_Reset(void)
{
    FlameData.state = state_idle;
    FlameData.ignition_count = 0;
}

void Flame_Task(void)
{
    ThermalZone *water = Zones_GetZone( ZONE_ID_WATER );
    ThermalZone *oil = Zones_GetZone( ZONE_ID_OIL );

    if( ! water->Active ) {

        if(FlameData.state != state_fault) {
            FlameData.state = state_idle;
        }
    } else {

        if(FlameData.state == state_idle) {
            FlameData.state = state_preheat;
        }
    }

    // flame state machine
    switch(FlameData.state)
    {	
        case state_fault:
            led_a = blink_slow;
            led_b = blink_slow;

            Relay_Off( RELAY_FAN );
            Relay_Off( RELAY_AIR );
            Relay_Off( RELAY_SPARK );

            // turn off oil heater
            oil->Config.Enabled = false;            
            break;

        case state_idle:
            led_a = blink_off;
            led_b = blink_slow;

            timer = 0;
            
            // turn off relays
            Relay_Off( RELAY_FAN );
            Relay_Off( RELAY_AIR );
            Relay_Off( RELAY_SPARK );
            
            // turn off oil heater
            oil->Config.Enabled = false;
            break;

        case state_wait:
        case state_preheat: // waiting for oil to be *not* cold (hot oil trigger is for oil heating state machine)

            oil->Config.Enabled = true;            
            
            if( !Zones_IsCold( ZONE_ID_OIL ) ) {
                Relay_On( RELAY_FAN );
                FlameData.state = state_fan;
                timer = 0;
                ++FlameData.ignition_count;
            }
            break;
            
        case state_fan:
            if( ++timer  > TIMEOUT_FAN ) {
                Relay_On( RELAY_AIR );
                FlameData.state = state_air;
                timer = 0;
            }
            break;
            
        case state_air:
            if( ++timer > TIMEOUT_AIR ) {
                FlameData.state = state_spark;
                Relay_On( RELAY_SPARK );
                led_a = blink_fast;
                timer = 0;
            }
            break;
            
        case state_spark:
            if( ++timer > TIMEOUT_SPARK )
            {
                Relay_Off( RELAY_SPARK );
                FlameData.state = state_detect_flame;
                led_a = blink_off;
                led_b = blink_fast;
                timer = 0;
            }
            break;
        case state_detect_flame:
            if( ++timer > TIMEOUT_FLAME ) {
                if(FlameData.ignition_count > IGNITION_RETRY) {
                    FlameData.state = state_fault;
                } else {
                    FlameData.state = state_idle;
                }
                timer = 0;
            }
            else if( IS_BURNING() ) {
                led_a = blink_off;
                led_b = blink_slow;
                
                FlameData.state = state_burn;
                timer = 0;
                FlameData.ignition_count = 0;
            }
            break;
        case state_burn:
            if( IS_BURNING() ) {
                FlameData.burning = FLAME_SENSOR_DELAY;
            } else if(FlameData.burning) {
                --FlameData.burning;
            }
            
            if(FlameData.burning == 0) {
                FlameData.state = state_idle;
            }
            break;
    }
}

void Flame_CLI(int argc, const char * const *argv)
{
    bool unknown = false;
    
    if(argc == 0) {
        printf_P(PSTR("flame command missing op\r\n"));
        return;
    }
    
    if(!strcasecmp(argv[0], "print")) {
        printf_P(PSTR("fan_time %u (ms)\r\n"
                          "air_time %u (ms)\r\n"
                          "spark_time %u (ms)\r\n"
                          "detect_time %u (ms)\r\n"
                          "flame_time %u (ms)\r\n"
                          "flame_trig %u\r\n"
                          "retry_count %u\r\n"
        ), FlameConfiguration.fan_time,
           FlameConfiguration.air_time,
           FlameConfiguration.spark_time,
           FlameConfiguration.detect_time,
           FlameConfiguration.flame_time,
           FlameConfiguration.flame_trig,
           (uint16_t)FlameConfiguration.retry_count
        );
    } else {
        if(argc > 1) {
            int val = atoi(argv[1]);
            if(!strcasecmp(argv[0], "fan_time")) {
                FlameConfiguration.fan_time = val;
            } else if(!strcasecmp(argv[0], "air_time")) {
                FlameConfiguration.air_time = val;
            } else if(!strcasecmp(argv[0], "spark_time")) {
                FlameConfiguration.spark_time = val;
            } else if(!strcasecmp(argv[0], "detect_time")) {
                FlameConfiguration.detect_time = val;
            } else if(!strcasecmp(argv[0], "flame_time")) {
                FlameConfiguration.flame_time = val;
            } else if(!strcasecmp(argv[0], "flame_trig")) {
                FlameConfiguration.flame_trig = val;
            } else if(!strcasecmp(argv[0], "retry_count")) {
                FlameConfiguration.retry_count = val;
            } else {
                unknown = true;
            }
            EEConfig_Save();
        } else {
            unknown = true;
        }
    }
    
    if(unknown) {
        printf_P(PSTR("unknown flame command '%s'\r\n"), argv[0]);
    }
}
