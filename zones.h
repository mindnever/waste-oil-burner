#ifndef _WOB_ZONES_H_
#define _WOB_ZONES_H_

typedef enum {
    SENSOR_NONE,
    SENSOR_ANALOG,
    SENSOR_BUTTON, /* button_b */
    SENSOR_RFRX,
} sensor_type_t;

typedef struct {
    int16_t  SetPoint;
    uint16_t Hysteresis;

    uint8_t  SensorType;
    uint16_t SensorID; /* for RFRX */

    uint8_t  Enabled;
    uint8_t  Timeout; /* minutes, or 0 to disable */
} ThermalZoneConfiguration;

typedef struct {
    int16_t  Current;
    uint8_t  Active;
    uint32_t Valid;
    uint32_t Timeout;

    ThermalZoneConfiguration Config;
} ThermalZone;

enum ZoneID {
    ZONE_ID_WATER,
    ZONE_ID_OIL,
    ZONE_ID_EXT1,
    ZONE_ID_EXT2,
    ZONE_ID_EXT3,
    ZONE_ID_EXT4,
};

#define NUM_ZONES 6

ThermalZone *Zones_GetZone(enum ZoneID id);

void Zones_SetCurrent(sensor_type_t type, uint16_t id, int16_t current);
void Zones_Update(uint8_t master_enable);
void Zones_Dump();
void Zones_DumpZone(enum ZoneID id, ThermalZone *zone);
void Zones_CLI(int argc, const char *const *argv);
uint8_t Zones_IsCold(enum ZoneID id);
uint8_t Zones_IsHot(enum ZoneID id);

#endif /* _WOB_ZONES_H_ */
