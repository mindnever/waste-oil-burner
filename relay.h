#ifndef _WOB_RELAY_H_
#define _WOB_RELAY_H_

#include <stdint.h>
#include <stdbool.h>

enum RelayID {
    _RELAY_FIRST,
    
    RELAY_HEATER = _RELAY_FIRST,
    RELAY_FAN,
    RELAY_AIR,
    RELAY_SPARK,
    RELAY_ZONE_EXT1,
    RELAY_ZONE_EXT2,
    RELAY_ZONE_EXT3,
    RELAY_ZONE_EXT4,

    _NRELAYS,
};

typedef struct {
    int8_t K[_NRELAYS];
} RelayConfig;

extern RelayConfig RelayConfiguration;

void Relay_Init(void);

const char *Relay_Name(enum RelayID id);
void Relay_On(enum RelayID id);
void Relay_Off(enum RelayID id);
bool Relay_State(enum RelayID id);

void Relay_CLI(int argc, const char * const *argv);

#endif /* _WOB_RELAY_H_ */
