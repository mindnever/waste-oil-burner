#include <avr/io.h>

#include "adc.h"

#include "zones.h"
#include "flame.h"
#include "hw.h"

// analog sensor
#define DEFAULT_ANALOG_SENSOR_GAIN -0.001958311
#define DEFAULT_ANALOG_SENSOR_OFFSET 129.5639


AnalogSensorConfiguration ADC_Config = {
    .Calibration = {
        [ 0 ] = {
            .gain = DEFAULT_ANALOG_SENSOR_GAIN,
            .offset = DEFAULT_ANALOG_SENSOR_OFFSET,
        },
        [ 1 ] = {
            .gain = DEFAULT_ANALOG_SENSOR_GAIN,
            .offset = DEFAULT_ANALOG_SENSOR_OFFSET,
        },
    }
};

static void ADC_Mux(uint8_t mux)
{
    ADMUX = _BV( REFS0 ) | _BV( ADLAR ) | mux;
}


void ADC_Init(void)
{
#ifdef SENSOR_A_PORT
    IO_DIR_IN( SENSOR_A );
    IO_PIN_HIGH( SENSOR_A ); // pullup
#endif
    
#ifdef SENSOR_B_PORT
    IO_DIR_IN( SENSOR_B );
    IO_PIN_HIGH( SENSOR_B ); // pullup
#endif
    
#ifdef SENSOR_C_PORT
    IO_DIR_IN( SENSOR_C );
    IO_PIN_HIGH( SENSOR_C ); // pullup
#endif
    
    // setup ADC, ADMUX input to ADC6, Vref = AVcc, start , 125khz clock
    // free running mode
    ADC_Mux( IO_ADCMUX( SENSOR_A ) );
    
    ADCSRA = _BV( ADEN ) | _BV( ADSC ) | _BV( ADPS0 ) | _BV( ADPS1 ) | _BV( ADPS2 ) | _BV( ADATE );
    
    DIDR0 =
#ifdef SENSOR_A_PORT
    _BV( IO_DIDR0( SENSOR_A ) ) |
#endif
#ifdef SENSOR_B_PORT
    _BV( IO_DIDR0( SENSOR_B ) ) |
#endif
#ifdef SENSOR_C_PORT
    _BV( IO_DIDR0( SENSOR_C ) ) |
#endif
    0;

}

uint16_t raw_adc[3];

void ADC_Task(void)
{
    static int sensor = 0;
    
    uint16_t value = ADC;
    
    raw_adc[sensor] = value;

    switch(sensor) {
        case 0:
            FlameData.sensor = value; // flame
            ADC_Mux( IO_ADCMUX( SENSOR_B ) ); // next
            break;
        case 1:
            Zones_SetCurrent(SENSOR_ANALOG1, 0, (ADC_Config.Calibration[0].gain * value + ADC_Config.Calibration[0].offset) * 10);
            ADC_Mux( IO_ADCMUX( SENSOR_C ) ); // next
            break;
        case 2:
            Zones_SetCurrent(SENSOR_ANALOG2, 0, (ADC_Config.Calibration[1].gain * value + ADC_Config.Calibration[1].offset) * 10);
            ADC_Mux( IO_ADCMUX( SENSOR_A ) ); // next
            break;
    }

    ++sensor;
    if(sensor > 2) {
        sensor = 0;
    }
}
