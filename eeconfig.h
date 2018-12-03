#ifndef _WOB_EECONFIG_H_
#define _WOB_EECONFIG_H_

#include "zones.h"
#include "adc.h"

struct EELayout {
    ThermalZoneConfig TZ_Config[NUM_ZONES];
    AnalogSensorCalibration ADC_Calibration[NUM_ANALOG_SENSORS];
};


EECONFIG_WRITE(TZ_Config[0], &Config);
EECONFIG_READ(TZ_Config[1], &Config);


#endif /* _WOB_EECONFIG_H_ */
