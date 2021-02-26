#ifndef _RF_RX_RX_H_
#define _RF_RX_RX_H_

struct RfRx_SensorData {
	int16_t temp;
	uint8_t channel;
	uint8_t sensor_id;
	uint8_t battery;
	uint8_t humidity;

	uint8_t *_raw;
	uint8_t _samples;
	uint8_t _matching;
};

typedef void (*RfRx_Callback)(struct RfRx_SensorData *data);

void RfRx_Init(void);
void RfRx_Task(RfRx_Callback cb);


void Sys_Idle(void);

extern uint16_t sys_idle_ticks;
extern uint16_t sys_busy_ticks;
extern uint32_t sys_millis;

#endif /* _RF_RX_RX_H_ */
