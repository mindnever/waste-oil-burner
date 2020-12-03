#ifndef _WOB_FLAME_H_
#define _WOB_FLAME_H_

typedef enum {
    state_idle, // 0
    state_wait, // 1
    state_preheat, // 2
    state_fan, // 3
    state_air, // 4
    state_spark, // 5
    state_detect_flame, // 6
    state_burn, // 7
    state_fault, // 8
} state_t;

#define _state_count 9

extern struct FlameData {
    state_t    state;
    uint8_t    ignition_count;
    uint16_t   sensor;
    uint32_t   burning;
    const char *fault_P;
} FlameData;

extern struct FlameConfiguration {
    uint16_t fan_time;
    uint16_t air_time;
    uint16_t spark_time;
    uint16_t detect_time; // detect flame
    uint16_t flame_time; // FLAME_SENSOR_DELAY
    uint16_t flame_trig; // flame trigger (def 61000)
    uint8_t  retry_count : 4;
    uint8_t  manage_oil : 1;
    float    flame_lpf;
} FlameConfiguration;

#define IS_BURNING() (FlameData.sensor < FlameConfiguration.flame_trig)


void Flame_Task(void);
void Flame_Init(void);
void Flame_Reset(void);
void Flame_CLI(int argc, const char *const *argv);

#endif /* _WOB_FLAME_H_ */
