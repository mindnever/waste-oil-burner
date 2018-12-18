#include <stdint.h>

#include "eeconfig.h"
#include "zones.h"
#include "adc.h"
#include "vcp.h"

#include <util/crc16.h>
#include <avr/eeprom.h>
#include <string.h>

#define EELAYOUT_VERSION 1

struct EELayout {
    uint8_t  Version;
    uint16_t CRC;
    ThermalZoneConfiguration TZ_Config[NUM_ZONES];
    AnalogSensorCalibration ADC_Calibration[NUM_ANALOG_SENSORS];
};

static EEMEM struct EELayout eevars;

void EEConfig_Load(void)
{
    struct EELayout buffer;
    
    eeprom_read_block(&buffer, &eevars, sizeof(buffer));

    VCP_Printf_P(PSTR("EEConfig_Load() layout version %u @%u\r\n"), buffer.Version, &eevars);

    if(buffer.Version != EELAYOUT_VERSION) {
        VCP_Printf_P(PSTR("EEConfig_Load() layout version mismatch %u != %u\r\n"), buffer.Version, EELAYOUT_VERSION);
        return;
    }
    
    uint16_t ecrc = buffer.CRC;
    buffer.CRC = 123;

    uint8_t *ptr = (uint8_t *) &buffer;
    uint16_t crc = 123;
    
    for(uint8_t i = 0; i < sizeof(buffer); ++i) {
        crc = _crc16_update(crc, *ptr++);
    }
    
    if(ecrc != crc) {
        VCP_Printf_P(PSTR("EEConfig_Load() bad crc %u != %u\r\n"), ecrc, crc);
        return;
    }
    
    VCP_Printf_P(PSTR("EEConfig_Load() crc ok stored %u calculated %u\r\n"), ecrc, crc);
    
    for(uint8_t i = 0; i < NUM_ZONES; ++i) {
        ThermalZone *zone = Zones_GetZone(i);
        if(!zone) { continue; }
        memcpy(&zone->Config, &buffer.TZ_Config[i], sizeof(buffer.TZ_Config[i]));
    }
    
    for(uint8_t i = 0; i < NUM_ANALOG_SENSORS; ++i) {
        memcpy(&ADC_Config.Calibration, &buffer.ADC_Calibration, sizeof(buffer.ADC_Calibration));
    }


}

void EEConfig_Save(void)
{
    struct EELayout buffer;
    buffer.Version = EELAYOUT_VERSION;
    buffer.CRC = 123;

    for(uint8_t i = 0; i < NUM_ZONES; ++i) {
        ThermalZone *zone = Zones_GetZone(i);
        if(!zone) { continue; }
        memcpy(&buffer.TZ_Config[i], &zone->Config, sizeof(buffer.TZ_Config[i]));
    }
    
    for(uint8_t i = 0; i < NUM_ANALOG_SENSORS; ++i) {
        memcpy(&buffer.ADC_Calibration, &ADC_Config.Calibration, sizeof(buffer.ADC_Calibration));
    }
    
    uint8_t *ptr = (uint8_t *) &buffer;
    uint16_t crc = 123;
    
    for(uint8_t i = 0; i < sizeof(buffer); ++i) {
        crc = _crc16_update(crc, *ptr++);
    }
    
    buffer.CRC = crc;
    
    eeprom_write_block(&buffer, &eevars, sizeof(buffer));
}

void EEConfig_Format(void)
{
    eeprom_write_byte(&eevars.Version, 0);
}
