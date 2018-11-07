#include <stdint.h>

#include "hw.h"
#include "hid.h"
#include "zones.h"
#include "flame.h"
#include "led.h"

#define TIMEOUT_FAN   (10000L / TICK_MS)
#define TIMEOUT_AIR   (150L / TICK_MS)
#define TIMEOUT_SPARK (6000 / TICK_MS)
#define TIMEOUT_FLAME (4000 / TICK_MS)

#define IGNITION_RETRY 3

#define FLAME_SENSOR_DELAY (3000/TICK_MS)


static uint32_t timer = 0;

struct FlameData FlameData;

void Flame_Init(void)
{
    FlameData.state = state_idle;
    FlameData.ignition_count = 0;
}

void Flame_Task(void)
{
    ThermalZone *water = Zones_GetZone( ZONE_ID_WATER );
    ThermalZone *oil = Zones_GetZone( ZONE_ID_OIL );

    if( ! (water->Flags & WOB_REPORT_FLAGS_OUTPUT_ACTIVE ) ) {

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

            RELAY_OFF( RELAY_FAN );
            RELAY_OFF( RELAY_AIR );
            RELAY_OFF( RELAY_SPARK );
            
            // turn off oil heater
            oil->Flags &= ~WOB_REPORT_FLAGS_CONTROL_ENABLED;            
            break;

        case state_idle:
            led_a = blink_off;
            led_b = blink_slow;

            timer = 0;
            
            // turn off relays
            RELAY_OFF( RELAY_FAN );
            RELAY_OFF( RELAY_AIR );
            RELAY_OFF( RELAY_SPARK );
            
            // turn off oil heater
            oil->Flags &= ~WOB_REPORT_FLAGS_CONTROL_ENABLED;
            break;

        case state_wait:
        case state_preheat: // waiting for oil to be *not* cold (hot oil trigger is for oil heating state machine)
            
            oil->Flags |= WOB_REPORT_FLAGS_CONTROL_ENABLED;
            
            if( !Zones_IsCold( ZONE_ID_OIL ) ) {
                RELAY_ON( RELAY_FAN );
                FlameData.state = state_fan;
                timer = 0;
                ++FlameData.ignition_count;
            }
            break;
            
        case state_fan:
            if( ++timer  > TIMEOUT_FAN ) {
                RELAY_ON( RELAY_AIR );
                FlameData.state = state_air;
                timer = 0;
            }
            break;
            
        case state_air:
            if( ++timer > TIMEOUT_AIR ) {
                FlameData.state = state_spark;
                RELAY_ON( RELAY_SPARK );
                led_a = blink_fast;
                timer = 0;
            }
            break;
            
        case state_spark:
            if( ++timer > TIMEOUT_SPARK )
            {
                RELAY_OFF( RELAY_SPARK );
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
