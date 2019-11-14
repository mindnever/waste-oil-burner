#ifndef _WOB_FLAME_H_
#define _WOB_FLAME_H_

typedef enum
{
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


extern struct FlameData {
    state_t state;
    uint8_t ignition_count;
    uint16_t sensor;
    uint32_t burning;
} FlameData;

extern struct FlameConfiguration {
    uint16_t fan_time;
    uint16_t air_time;
    uint16_t spark_time;
    uint16_t detect_time; // detect flame
    uint16_t flame_time; // FLAME_SENSOR_DELAY
    uint8_t retry_count;
} FlameConfiguration;

#define IS_BURNING() (FlameData.sensor < 61000)


void Flame_Task(void);
void Flame_Init(void);
void Flame_Reset(void);

#endif /* _WOB_FLAME_H_ */
