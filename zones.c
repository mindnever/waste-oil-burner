#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "zones.h"
#include "hid.h"
#include "vcp.h"
#include "hw.h"

static ThermalZone Zones[ NUM_ZONES ];

static const char *zone_names[] = {
    "water",
    "oil",
    "external_1",
    "external_2",
    "external_3",
    "external_4",
};

#define CURRENT_VALID (10*60*1000L)/TICK_MS


void Zones_Init(void)
{
// read config from eeprom
}

void Zones_SetCurrent(sensor_type_t type, uint16_t id, int16_t current)
{
    for(uint8_t i = 0; i < _WOB_REPORT_ZONE_COUNT; ++i) {
        ThermalZone *zone = Zones_GetZone(i);
        if(!zone) { continue; }// how can this be? cannot..
        
        if(zone->Config.SensorType != type) {
            continue;
        }
        
        if(zone->Config.SensorID != id) {
            continue;
        }
        
        zone->Current = current;
        zone->Valid = CURRENT_VALID;
    }
}

void Zones_Update(uint8_t master_enable)
{
    for(uint8_t i = 0; i < _WOB_REPORT_ZONE_COUNT; ++i) {
        ThermalZone *zone = Zones_GetZone(i);
        if(!zone) { continue; }// how can this be? cannot..
        
        if(zone->Valid > 0) {
            --zone->Valid;
        }
        
        if(!master_enable || !zone->Config.Enabled || !zone->Valid) {
            zone->Active = false;
        } else if(zone->Current < (zone->Config.SetPoint - (int16_t)zone->Config.Hysteresis)) {
            zone->Active = true;
        } else if(zone->Current > zone->Config.SetPoint) {
            zone->Active = false;
        }
    }
}

void Zones_DumpZone(enum ZoneID id, ThermalZone *zone)
{
    printf_P(PSTR("zone %u (%s) %s %s setpoint %.1f current %.1f %shysteresis %.1f sensor "),
                                        id + 1,
                                        zone_names[id],
                                        zone->Config.Enabled ? "ENABLED" : "DISABLED",
                                        zone->Active ? "ACTIVE" : "INACTIVE",
                                        (float)zone->Config.SetPoint / 10,
                                        (float)zone->Current / 10,
                                        zone->Valid > 0 ? "valid " : "",
                                        (float)zone->Config.Hysteresis / 10);

    switch(zone->Config.SensorType) {
        case SENSOR_NONE:
            printf_P(PSTR("none"));
            break;
        case SENSOR_ANALOG:
            printf_P(PSTR("analog %u"), zone->Config.SensorID);
            break;
        case SENSOR_BINARY:
            printf_P(PSTR("binary (BUTTON_B)"));
            break;
        case SENSOR_RFRX:
            printf_P(PSTR("rfrx %u"), zone->Config.SensorID);
            break;
        default:
            printf_P(PSTR("unknown"));
    }
    
    printf_P(PSTR("\n"));
}

ThermalZone *Zones_GetZone(enum ZoneID id)
{
    if(id < NUM_ZONES) {
        return &Zones[id];
    }
    return 0;
}

void Zones_ZoneCLI(ThermalZone *zone, int argc, const char * const *argv)
{
    if(!strcasecmp_P(argv[0], PSTR("enable"))) {
        zone->Config.Enabled = true;
    } else if(!strcasecmp_P(argv[0], PSTR("disable"))) {
        zone->Config.Enabled = false;
    } else if(!strcasecmp_P(argv[0], PSTR("setpoint"))) {
        if(argc > 1) {
            zone->Config.SetPoint = atof(argv[1]) * 10;
        }
    } else if(!strcasecmp_P(argv[0], PSTR("sensor"))) {
        if(argc > 1) {
            if(!strcasecmp_P(argv[1], PSTR("none"))) {
                zone->Config.SensorType = SENSOR_NONE;
            } else if(!strcasecmp_P(argv[1], PSTR("analog"))) {
                if(argc > 2) {
                    zone->Config.SensorType = SENSOR_ANALOG;
                    zone->Config.SensorID = atoi(argv[2]);
                }
            } else if(!strcasecmp_P(argv[1], PSTR("binary"))) {
                zone->Config.SensorType = SENSOR_BINARY;
                zone->Config.SensorID = 0;
            } else if(!strcasecmp_P(argv[1], PSTR("rfrx"))) {
                if(argc > 2) {
                    zone->Config.SensorType = SENSOR_RFRX;
                    zone->Config.SensorID = atoi(argv[2]);
                }
            } else {
                printf_P(PSTR("??? '%s'\n"), argv[1]);
            }
        }
    } else if(!strcasecmp_P(argv[0], PSTR("hysteresis"))) {
        if(argc > 1) {
            zone->Config.Hysteresis = atof(argv[1]) * 10;
        }
    } else {
        printf_P(PSTR("??? '%s'\n"), argv[0]);
    }
}

void Zones_Dump()
{
    for(int i = 0; i < _WOB_REPORT_ZONE_COUNT; ++i) {
        Zones_DumpZone( i, Zones_GetZone( i ) );
    }
}

uint8_t Zones_IsCold(enum ZoneID id)
{
    ThermalZone *zone = Zones_GetZone(id);
    if(!zone) { return false; }

    return zone->Current < (zone->Config.SetPoint - zone->Config.Hysteresis);
}

uint8_t Zones_IsHot(enum ZoneID id)
{
    ThermalZone *zone = Zones_GetZone(id);
    if(!zone) { return false; }

    return zone->Current > zone->Config.SetPoint;
}
