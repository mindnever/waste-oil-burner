#ifndef _WOB_ADC_H_
#define _WOB_ADC_H_

#define NUM_ANALOG_SENSORS 3

typedef struct {
    float gain;
    float offset;
    float filter;
} AnalogSensorCalibration;

typedef struct {
    AnalogSensorCalibration Calibration[NUM_ANALOG_SENSORS];
} AnalogSensorConfiguration;

extern AnalogSensorConfiguration ADC_Config;
extern uint16_t raw_adc[NUM_ANALOG_SENSORS + 1];


void ADC_Init(void);
void ADC_Task(void);
void ADC_CLI(int argc, const char *const *argv);

#endif /* _WOB_ADC_H_ */
