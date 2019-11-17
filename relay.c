#include "hw.h"
#include "relay.h"
#include "eeconfig.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <avr/pgmspace.h>

RelayConfig RelayConfiguration = {
    .K = DEFAULT_RELAY_CONFIG
};

static const char *names[] = {
    [RELAY_HEATER] = "heater",
    [RELAY_FAN] = "fan",
    [RELAY_AIR] = "air",
    [RELAY_SPARK] = "spark",
    [RELAY_ZONE_EXT1] = "zone_ext1",
    [RELAY_ZONE_EXT2] = "zone_ext2",
    [RELAY_ZONE_EXT3] = "zone_ext3",
    [RELAY_ZONE_EXT4] = "zone_ext4",
};

static uint8_t state;

static void k_set(uint8_t k, bool v)
{
    switch(k) {
#ifdef K1_PORT
        case 1:
            if(v ^ K1_INVERTED) {
                IO_PIN_HIGH(K1);
            } else {
                IO_PIN_LOW(K1);
            }
            break;
#endif /* K1_PORT */

#ifdef K2_PORT
        case 2:
            if(v ^ K2_INVERTED) {
                IO_PIN_HIGH(K2);
            } else {
                IO_PIN_LOW(K2);
            }
            break;
#endif /* K2_PORT */

#ifdef K3_PORT
        case 3:
            if(v ^ K3_INVERTED) {
                IO_PIN_HIGH(K3);
            } else {
                IO_PIN_LOW(K3);
            }
            break;
#endif /* K3_PORT */

#ifdef K4_PORT
        case 4:
            if(v ^ K4_INVERTED) {
                IO_PIN_HIGH(K4);
            } else {
                IO_PIN_LOW(K4);
            }
            break;
#endif /* K4_PORT */

#ifdef K5_PORT
        case 5:
            if(v ^ K5_INVERTED) {
                IO_PIN_HIGH(K5);
            } else {
                IO_PIN_LOW(K5);
            }
            break;
#endif /* K5_PORT */

#ifdef K6_PORT
        case 6:
            if(v ^ K6_INVERTED) {
                IO_PIN_HIGH(K6);
            } else {
                IO_PIN_LOW(K6);
            }
            break;
#endif /* K6_PORT */

#ifdef K7_PORT
        case 7:
            if(v ^ K7_INVERTED) {
                IO_PIN_HIGH(K7);
            } else {
                IO_PIN_LOW(K7);
            }
            break;
#endif /* K7_PORT */

#ifdef K8_PORT
        case 8:
            if(v ^ K8_INVERTED) {
                IO_PIN_HIGH(K8);
            } else {
                IO_PIN_LOW(K8);
            }
            break;
#endif /* K8_PORT */

    }
}

void Relay_Init()
{
    for(int i = 1; i <=8; ++i) {
        k_set(i, false);
    }

#ifdef K1_PORT
    IO_DIR_OUT(K1);
#endif /* K1_PORT */

#ifdef K2_PORT
    IO_DIR_OUT(K2);
#endif /* K2_PORT */

#ifdef K3_PORT
    IO_DIR_OUT(K3);
#endif /* K3_PORT */

#ifdef K4_PORT
    IO_DIR_OUT(K4);
#endif /* K4_PORT */

#ifdef K5_PORT
    IO_DIR_OUT(K5);
#endif /* K5_PORT */

#ifdef K6_PORT
    IO_DIR_OUT(K6);
#endif /* K6_PORT */

#ifdef K7_PORT
    IO_DIR_OUT(K7);
#endif /* K7_PORT */

#ifdef K8_PORT
    IO_DIR_OUT(K8);
#endif /* K8_PORT */
}

void Relay_On(enum RelayID id)
{
    state |= _BV(id);
    
    k_set(RelayConfiguration.K[id], true);
}

void Relay_Off(enum RelayID id)
{
    state &= ~_BV(id);

    k_set(RelayConfiguration.K[id], false);
}

bool Relay_State(enum RelayID id)
{
    return !!(state & _BV(id));
}

const char *Relay_Name(enum RelayID id)
{
    return names[id];
}

void Relay_CLI(int argc, const char * const *argv)
{
    if(argc == 0) {
        printf_P(PSTR("relay command missing op\r\n"));
        return;
    }

    if(!strcasecmp(argv[0], "print")) {
        for(enum RelayID id = _RELAY_FIRST; id < _NRELAYS; ++id) {
            printf_P(RelayConfiguration.K[id] ? PSTR("%10s - %3s - k%u\r\n") : PSTR("%10s - %3s - none\r\n"), names[id], Relay_State(id) ? "on" : "off", RelayConfiguration.K[id]);
        }
    } else {
        for(enum RelayID id = _RELAY_FIRST; id < _NRELAYS; ++id) {
            if(!strcasecmp(argv[0], names[id])) {
                if(argc > 1) {
                    if(!strcasecmp(argv[1], "none")) {
                        RelayConfiguration.K[id] = 0;
                    } else if((argv[1][0] == 'k' || argv[1][0] == 'K') && isdigit(argv[1][1])) {
                        RelayConfiguration.K[id] = atoi(argv[1] + 1);
                    } else {
                        printf_P(PSTR("unknown relay output '%s'\r\n"), argv[1]);
                    }

                    // sanity check
                    for(enum RelayID other = _RELAY_FIRST; other < _NRELAYS; ++other) {
                        if((other != id) && (RelayConfiguration.K[other] == RelayConfiguration.K[id])) {
                            RelayConfiguration.K[other] = 0;
                        }
                    }                    
                }
                break;
            }
        }
        // apply states, turn off unassigned relays
        for(int i = 1; i <=8; ++i) {
            bool val = false;

            for(enum RelayID id = _RELAY_FIRST; id < _NRELAYS; ++id) {
                if(RelayConfiguration.K[id] == i) {
                    val = Relay_State(id);
                    break;
                }
            }
            
            k_set(i, val);
        }
        
        
        EEConfig_Save();
    }
}