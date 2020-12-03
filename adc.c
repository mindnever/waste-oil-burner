#include <avr/pgmspace.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>

#include "adc.h"
#include "eeconfig.h"
#include "zones.h"
#include "flame.h"
#include "hw.h"

// analog sensor
#define DEFAULT_ANALOG_SENSOR_GAIN   -0.001958311
#define DEFAULT_ANALOG_SENSOR_OFFSET 129.5639
#define DEFAULT_ANALOG_SENSOR_FILTER 0

AnalogSensorConfiguration ADC_Config = {
    .Calibration    = {
        [0] = {
            .gain   = DEFAULT_ANALOG_SENSOR_GAIN,
            .offset = DEFAULT_ANALOG_SENSOR_OFFSET,
            .filter = DEFAULT_ANALOG_SENSOR_FILTER,
        },
        [1] = {
            .gain   = DEFAULT_ANALOG_SENSOR_GAIN,
            .offset = DEFAULT_ANALOG_SENSOR_OFFSET,
            .filter = DEFAULT_ANALOG_SENSOR_FILTER,
        },
        [2] = {
            .gain   = DEFAULT_ANALOG_SENSOR_GAIN,
            .offset = DEFAULT_ANALOG_SENSOR_OFFSET,
            .filter = DEFAULT_ANALOG_SENSOR_FILTER,
        },
    }
};

static const PROGMEM uint8_t mux_map[NUM_ANALOG_SENSORS + 1] = {
#ifdef SENSOR_A_ADCMUX
    [0] = SENSOR_A_ADCMUX,
#endif
#ifdef SENSOR_B_ADCMUX
    [1] = SENSOR_B_ADCMUX,
#endif
#ifdef SENSOR_C_ADCMUX
    [2] = SENSOR_C_ADCMUX,
#endif
#ifdef SENSOR_D_ADCMUX
    [3] = SENSOR_D_ADCMUX,
#endif
};

static void ADC_Mux(uint8_t sensor)
{
    ADMUX = _BV(REFS0) | _BV(ADLAR) | pgm_read_byte(&mux_map[sensor]);
}


void ADC_Init(void)
{
#ifdef SENSOR_A_PORT
    IO_DIR_IN(SENSOR_A);
    IO_PIN_LOW(SENSOR_A);
#endif

#ifdef SENSOR_B_PORT
    IO_DIR_IN(SENSOR_B);
    IO_PIN_LOW(SENSOR_B);
#endif

#ifdef SENSOR_C_PORT
    IO_DIR_IN(SENSOR_C);
    IO_PIN_LOW(SENSOR_C);
#endif

#ifdef SENSOR_D_PORT
    IO_DIR_IN(SENSOR_D);
    IO_PIN_LOW(SENSOR_D);
#endif

    // setup ADC, ADMUX input to ADC6, Vref = AVcc, start , 125khz clock
    // free running mode
    ADC_Mux(0);

    ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2) | _BV(ADATE);

    DIDR0  =
#ifdef SENSOR_A_DIDR0
        _BV(IO_DIDR0(SENSOR_A)) |
#endif
#ifdef SENSOR_B_DIDR0
        _BV(IO_DIDR0(SENSOR_B)) |
#endif
#ifdef SENSOR_C_DIDR0
        _BV(IO_DIDR0(SENSOR_C)) |
#endif
#ifdef SENSOR_D_DIDR0
        _BV(IO_DIDR0(SENSOR_D)) |
#endif
        0;
}

uint16_t raw_adc[NUM_ANALOG_SENSORS + 1];

void ADC_Task(void)
{
    static int sensor = 0;

    uint16_t value    = ADC;

    float filter = sensor ? ADC_Config.Calibration[sensor - 1].filter : FlameConfiguration.flame_lpf;

    raw_adc[sensor] = (1.0f - filter) * value + filter * raw_adc[sensor];

    if (sensor == 0) {
        FlameData.sensor = raw_adc[sensor]; // flame
    } else {
        Zones_SetCurrent(SENSOR_ANALOG, sensor, (ADC_Config.Calibration[sensor - 1].gain * raw_adc[sensor] + ADC_Config.Calibration[sensor - 1].offset) * 10);
    }

    ++sensor;
    if (sensor > NUM_ANALOG_SENSORS) {
        sensor = 0;
    }

    ADC_Mux(sensor);
}

void ADC_CLI(int argc, const char *const *argv)
{
    if (argc == 0) {
        return;
    }

    if (!strcasecmp_P(argv[0], PSTR("print"))) {
        // print analog calibration
        for (unsigned index = 0; index < NUM_ANALOG_SENSORS; ++index) {
            printf_P(PSTR("analog %u gain %f offset %f lpf %f ; raw %u\n"), index + 1, ADC_Config.Calibration[index].gain, ADC_Config.Calibration[index].offset, ADC_Config.Calibration[index].filter, raw_adc[index + 1]);
        }
    } else {
        unsigned index = atoi(argv[0]) - 1;
        if (index > 0 && index < NUM_ANALOG_SENSORS) {
            --argc;

            for (int i = 1; i < argc; i += 2) {
                float val = atof(argv[i + 1]);
                if (!strcasecmp_P(argv[i], PSTR("gain"))) {
                    ADC_Config.Calibration[index].gain = val;
                } else if (!strcasecmp_P(argv[i], PSTR("offset"))) {
                    ADC_Config.Calibration[index].offset = val;
                } else if (!strcasecmp_P(argv[i], PSTR("lpf"))) {
                    ADC_Config.Calibration[index].filter = val;
                } else {
                    printf_P(PSTR("?? '%s'\n"), argv[i]);
                }
            }

            EEConfig_Save();
        } else {
            printf_P(PSTR("?? id\n"));
        }
    }
}
