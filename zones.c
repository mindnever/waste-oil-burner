#include <stdint.h>
#include <stdlib.h>

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
    for(uint8_t i = 0; i < _WOB_REPORT_ZONE_COUNT; ++i) {
        ThermalZone *zone = Zones_GetZone(i);
        if(!zone) { continue; }// how can this be? cannot..
        
        if((zone->_sensor_type == type)
        && ((type != SENSOR_RFRX) || (id == zone->_config.rfrx.sensor_id))) {
            zone->Current = current;
        }
    }
}

void Zones_Update()
{
    for(uint8_t i = 0; i < _WOB_REPORT_ZONE_COUNT; ++i) {
        ThermalZone *zone = Zones_GetZone(i);
        if(!zone) { continue; }// how can this be? cannot..
        if(!(zone->Flags & WOB_REPORT_FLAGS_CONTROL_ENABLED)) {
            zone->Flags &= ~WOB_REPORT_FLAGS_OUTPUT_ACTIVE;
            continue;
        }
        if(zone->Current < (zone->SetPoint - zone->_hysteresis)) {
            zone->Flags |= WOB_REPORT_FLAGS_OUTPUT_ACTIVE;
        } else if(zone->Current > zone->SetPoint) {
            zone->Flags &= ~WOB_REPORT_FLAGS_OUTPUT_ACTIVE;
        }
    }
}

void Zones_DumpZone(enum ZoneID id, ThermalZone *zone)
{
    VCP_Printf_P(PSTR("zone %u (%s) %s %s setpoint %.1f current %.1f hysteresis %.1f sensor "),
                                        id + 1,
                                        zone_names[id],
                                        (zone->Flags & WOB_REPORT_FLAGS_CONTROL_ENABLED) ? "ENABLED" : "DISABLED",
                                        (zone->Flags & WOB_REPORT_FLAGS_OUTPUT_ACTIVE) ? "ACTIVE" : "INACTIVE",
                                        (float)zone->SetPoint / 10,
                                        (float)zone->Current / 10,
                                        (float)zone->_hysteresis / 10);

    switch(zone->_sensor_type) {
        case SENSOR_NONE:
            VCP_Printf_P(PSTR("none"));
            break;
        case SENSOR_ANALOG1:
            VCP_Printf_P(PSTR("analog 1"));
            break;
        case SENSOR_ANALOG2:
            VCP_Printf_P(PSTR("analog 2"));
            break;
        case SENSOR_BINARY:
            VCP_Printf_P(PSTR("binary (BUTTON_B)"));
            break;
        case SENSOR_RFRX:
            VCP_Printf_P(PSTR("RFRX sensor_id=%u"), zone->_config.rfrx.sensor_id);
            break;
        default:
            VCP_Printf_P(PSTR("unknown"));
    }
    
    VCP_Printf_P(PSTR("\r\n"));
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
            } else if(!strcasecmp(argv[1], "analog2")) {
                zone->_sensor_type = SENSOR_ANALOG2;
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

    return zone->Current < (zone->SetPoint - zone->_hysteresis);
}

uint8_t Zones_IsHot(enum ZoneID id)
{
    ThermalZone *zone = Zones_GetZone(id);
    if(!zone) { return false; }

    return zone->Current > zone->SetPoint;
}
