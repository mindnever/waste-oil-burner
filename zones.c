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

void Zones_Init(void)
{
// read config from eeprom
}

void Zones_SetCurrent(sensor_type_t type, uint16_t id, int16_t current)
{
    uint16_t rfrx_id = id;
    
    for(uint8_t i = 0; i < _WOB_REPORT_ZONE_COUNT; ++i) {
        ThermalZone *zone = Zones_GetZone(i);
        if(!zone) { continue; }// how can this be? cannot..
        
        rfrx_id = id;
        
        if((zone->Config.SensorType == SENSOR_RFRX)
            && (zone->Config.SensorID <= 3)) { // check just channel ID, ignore sensor id
            rfrx_id = id >> 8;
        } else {
            rfrx_id = id;
        }
        
        
        if((zone->Config.SensorType == type)
        && ((type != SENSOR_RFRX) || (rfrx_id == zone->Config.SensorID))) {
            zone->Current = current;
        }
    }
}

void Zones_Update(uint8_t master_enable)
{
    for(uint8_t i = 0; i < _WOB_REPORT_ZONE_COUNT; ++i) {
        ThermalZone *zone = Zones_GetZone(i);
        if(!zone) { continue; }// how can this be? cannot..
        if(!master_enable || !zone->Config.Enabled) {
            zone->Active = false;
        } else if(zone->Current < (zone->Config.SetPoint - (int16_t)zone->Config.Hysteresis)) {
            zone->Active = true;;
        } else if(zone->Current > zone->Config.SetPoint) {
            zone->Active = false;
        }
    }
}

void Zones_DumpZone(enum ZoneID id, ThermalZone *zone)
{
    printf_P(PSTR("zone %u (%s) %s %s setpoint %.1f current %.1f hysteresis %.1f sensor "),
                                        id + 1,
                                        zone_names[id],
                                        zone->Config.Enabled ? "ENABLED" : "DISABLED",
                                        zone->Active ? "ACTIVE" : "INACTIVE",
                                        (float)zone->Config.SetPoint / 10,
                                        (float)zone->Current / 10,
                                        (float)zone->Config.Hysteresis / 10);

    switch(zone->Config.SensorType) {
        case SENSOR_NONE:
            printf_P(PSTR("none"));
            break;
        case SENSOR_ANALOG1:
            printf_P(PSTR("analog 1"));
            break;
        case SENSOR_ANALOG2:
            printf_P(PSTR("analog 2"));
            break;
        case SENSOR_ANALOG3:
            printf_P(PSTR("analog 3"));
            break;
        case SENSOR_BINARY:
            printf_P(PSTR("binary (BUTTON_B)"));
            break;
        case SENSOR_RFRX:
            printf_P(PSTR("RFRX sensor_id=%u"), zone->Config.SensorID);
            break;
        default:
            printf_P(PSTR("unknown"));
    }
    
    printf_P(PSTR("\r\n"));
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
    if(!strcasecmp(argv[0], "enable")) {
        zone->Config.Enabled = true;
    } else if(!strcasecmp(argv[0], "disable")) {
        zone->Config.Enabled = false;
    } else if(!strcasecmp(argv[0], "setpoint")) {
        if(argc > 1) {
            zone->Config.SetPoint = atof(argv[1]) * 10;
        }
    } else if(!strcasecmp(argv[0], "sensor")) {
        if(argc > 1) {
            if(!strcasecmp(argv[1], "none")) {
                zone->Config.SensorType = SENSOR_NONE;
            } else if(!strcasecmp(argv[1], "analog")) {
            
                if(argc > 2) {
                    switch(atoi(argv[2])) {
                        case 1:
                            zone->Config.SensorType = SENSOR_ANALOG1;
                            break;
                        case 2:
                            zone->Config.SensorType = SENSOR_ANALOG2;
                            break;
                        case 3:
                            zone->Config.SensorType = SENSOR_ANALOG2;
                            break;
                        default:
                            ;
                    }
                }
            } else if(!strcasecmp(argv[1], "binary")) {
                zone->Config.SensorType = SENSOR_BINARY;
            } else if(!strcasecmp(argv[1], "rfrx")) {
                if(argc > 2) {
                    zone->Config.SensorType = SENSOR_RFRX;
                    zone->Config.SensorID = atoi(argv[2]);
                }
            } else {
                printf_P(PSTR("unknown zone sensor type '%s'\r\n"), argv[1]);
            }
        }
    } else if(!strcasecmp(argv[0], "hysteresis")) {
        if(argc > 1) {
            zone->Config.Hysteresis = atof(argv[1]) * 10;
        }
    } else {
        printf_P(PSTR("unknown zone command '%s'\r\n"), argv[0]);
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
