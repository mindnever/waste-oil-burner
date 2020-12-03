#include <stdint.h>
#include <stdio.h>

#include "eeconfig.h"
#include "zones.h"
#include "adc.h"
#include "vcp.h"
#include "flame.h"
#include "relay.h"

#include <util/crc16.h>
#include <avr/eeprom.h>
#include <string.h>

#define EELAYOUT_VERSION     7
#define EELAYOUT_CONFIG_SLOT 0
#define EELAYOUT_BACKUP_SLOT 1


struct EELayout {
    uint8_t  Version;
    uint16_t CRC;
    ThermalZoneConfiguration  TZ_Config[NUM_ZONES];
    AnalogSensorConfiguration ADC_Config;
    struct FlameConfiguration F_Config;
    RelayConfig R_Config;
};

static EEMEM struct EELayout eevars[2];

static void EEConfig_Save_Slot(uint8_t slot);
static void EEConfig_Load_Slot(uint8_t slot);

void EEConfig_Load()
{
    EEConfig_Load_Slot(EELAYOUT_CONFIG_SLOT);
}

void EEConfig_Restore()
{
    EEConfig_Load_Slot(EELAYOUT_BACKUP_SLOT);
    EEConfig_Save_Slot(EELAYOUT_CONFIG_SLOT);
}

void EEConfig_Load_Slot(uint8_t slot)
{
    struct EELayout buffer;

    eeprom_read_block(&buffer, &eevars[slot], sizeof(buffer));

    if (buffer.Version != EELAYOUT_VERSION) {
        return;
    }

    uint16_t ecrc = buffer.CRC;

    buffer.CRC = 123;

    uint8_t *ptr = (uint8_t *)&buffer;
    uint16_t crc = 123;

    for (uint8_t i = 0; i < sizeof(buffer); ++i) {
        crc = _crc16_update(crc, *ptr++);
    }

    if (ecrc != crc) {
        fputs_P(PSTR("cfg: bad crc\n"), stdout);
        return;
    }

    for (uint8_t i = 0; i < NUM_ZONES; ++i) {
        ThermalZone *zone = Zones_GetZone(i);
        if (!zone) {
            continue;
        }
        memcpy(&zone->Config, &buffer.TZ_Config[i], sizeof(buffer.TZ_Config[i]));
    }

    memcpy(&ADC_Config, &buffer.ADC_Config, sizeof(ADC_Config));

    memcpy(&FlameConfiguration, &buffer.F_Config, sizeof(FlameConfiguration));
    memcpy(&RelayConfiguration, &buffer.R_Config, sizeof(RelayConfiguration));
}

void EEConfig_Save()
{
    EEConfig_Save_Slot(EELAYOUT_CONFIG_SLOT);
}

void EEConfig_Backup()
{
    EEConfig_Save_Slot(EELAYOUT_BACKUP_SLOT);
}

void EEConfig_Save_Slot(uint8_t slot)
{
    struct EELayout buffer;

    buffer.Version = EELAYOUT_VERSION;
    buffer.CRC     = 123;

    printf_P(PSTR("saving cfg...\n"));

    for (uint8_t i = 0; i < NUM_ZONES; ++i) {
        ThermalZone *zone = Zones_GetZone(i);
        if (!zone) {
            continue;
        }
        memcpy(&buffer.TZ_Config[i], &zone->Config, sizeof(buffer.TZ_Config[i]));
    }

    memcpy(&buffer.ADC_Config, &ADC_Config, sizeof(buffer.ADC_Config));
    memcpy(&buffer.F_Config, &FlameConfiguration, sizeof(buffer.F_Config));
    memcpy(&buffer.R_Config, &RelayConfiguration, sizeof(buffer.R_Config));

    uint8_t *ptr = (uint8_t *)&buffer;
    uint16_t crc = 123;

    for (uint8_t i = 0; i < sizeof(buffer); ++i) {
        crc = _crc16_update(crc, *ptr++);
    }

    buffer.CRC = crc;

    eeprom_write_block(&buffer, &eevars[slot], sizeof(buffer));
}

void EEConfig_Format(void)
{
    eeprom_write_byte(&eevars[EELAYOUT_CONFIG_SLOT].Version, 0);
}
