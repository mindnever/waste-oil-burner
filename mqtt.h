#ifndef _WOB_MQTT_H_
#define _WOB_MQTT_H_

#define JSMN_HEADER
#include "jsmn.h"

extern FILE *mqtt;

typedef void (* mqtt_msg_callback_t)(const char *topic, const char *msg);

void mqtt_task(mqtt_msg_callback_t on_msg);
void mqtt_idle(void);

void mqtt_subscribe(const char *topic);
void mqtt_publish(const char *topic, const char *fmt, ...);
void mqtt_publish_P(const char *topic, const char *fmt, ...);
void mqtt_cli(int argc, const char * const *argv);
void mqtt_hid(int id, uint8_t *buffer, uint8_t size);

#endif /* _WOB_MQTT_H_ */
