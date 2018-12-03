#ifndef _WOB_ADC_H_
#define _WOB_ADC_H_

#define NUM_ANALOG_SENSORS 2

typedef struct {
    float gain;
    float offset;
} AnalogSensorCalibration;

typedef struct {
    AnalogSensorCalibration Calibration[NUM_ANALOG_SENSORS];
} AnalogSensorConfiguration;

extern AnalogSensorConfiguration ADC_Config;
extern uint16_t raw_adc[3];


void ADC_Init(void);
void ADC_Task(void);

#endif /* _WOB_ADC_H_ */
