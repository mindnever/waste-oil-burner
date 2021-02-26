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

static const char heater_PSTR[] PROGMEM = "heater";
static const char fan_PSTR[] PROGMEM = "fan";
static const char air_PSTR[] PROGMEM = "air";
static const char spark_PSTR[] PROGMEM = "spark";
static const char ext1_PSTR[] PROGMEM = "ext1";
static const char ext2_PSTR[] PROGMEM = "ext2";
static const char ext3_PSTR[] PROGMEM = "ext3";
static const char ext4_PSTR[] PROGMEM = "ext4";

static const char *names[] = {
	[RELAY_HEATER] = heater_PSTR,
	[RELAY_FAN]    = fan_PSTR,
	[RELAY_AIR]    = air_PSTR,
	[RELAY_SPARK]  = spark_PSTR,
	[RELAY_EXT1]   = ext1_PSTR,
	[RELAY_EXT2]   = ext2_PSTR,
	[RELAY_EXT3]   = ext3_PSTR,
	[RELAY_EXT4]   = ext4_PSTR,
};

static uint8_t state;

static void k_set(uint8_t k, bool v)
{
	switch (k) {
#ifdef K1_PORT
	case 1:
		if (v ^ K1_INVERTED) {
			IO_PIN_HIGH(K1);
		} else {
			IO_PIN_LOW(K1);
		}
		break;
#endif /* K1_PORT */

#ifdef K2_PORT
	case 2:
		if (v ^ K2_INVERTED) {
			IO_PIN_HIGH(K2);
		} else {
			IO_PIN_LOW(K2);
		}
		break;
#endif /* K2_PORT */

#ifdef K3_PORT
	case 3:
		if (v ^ K3_INVERTED) {
			IO_PIN_HIGH(K3);
		} else {
			IO_PIN_LOW(K3);
		}
		break;
#endif /* K3_PORT */

#ifdef K4_PORT
	case 4:
		if (v ^ K4_INVERTED) {
			IO_PIN_HIGH(K4);
		} else {
			IO_PIN_LOW(K4);
		}
		break;
#endif /* K4_PORT */

#ifdef K5_PORT
	case 5:
		if (v ^ K5_INVERTED) {
			IO_PIN_HIGH(K5);
		} else {
			IO_PIN_LOW(K5);
		}
		break;
#endif /* K5_PORT */

#ifdef K6_PORT
	case 6:
		if (v ^ K6_INVERTED) {
			IO_PIN_HIGH(K6);
		} else {
			IO_PIN_LOW(K6);
		}
		break;
#endif /* K6_PORT */

#ifdef K7_PORT
	case 7:
		if (v ^ K7_INVERTED) {
			IO_PIN_HIGH(K7);
		} else {
			IO_PIN_LOW(K7);
		}
		break;
#endif /* K7_PORT */

#ifdef K8_PORT
	case 8:
		if (v ^ K8_INVERTED) {
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
	for (int i = 1; i <= 8; ++i) {
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

void Relay_Set(enum RelayID id, bool s)
{
	if (s) {
		state |= _BV(id);
	} else {
		state &= ~_BV(id);
	}

	k_set(RelayConfiguration.K[id], s);
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

// const char *Relay_Name(enum RelayID id)
// {
// return names[id];
// }

void Relay_CLI(int argc, const char *const *argv)
{
	if (argc == 0) {
		printf_P(PSTR("?? op\n"));
		return;
	}

	if (!strcasecmp_P(argv[0], PSTR("print"))) {
		for (enum RelayID id = _RELAY_FIRST; id < _NRELAYS; ++id) {
			char k[] = { 'k', '0' + RelayConfiguration.K[id], 0 };
			static const char *none = "none";

			printf_P(PSTR("relay %-6S %-4s ; %S\n"), names[id], RelayConfiguration.K[id] ? k : none, Relay_State(id) ? PSTR("on") : PSTR("off"));
		}
	} else {
		for (enum RelayID id = _RELAY_FIRST; id < _NRELAYS; ++id) {
			if (!strcasecmp_P(argv[0], names[id])) {
				if (argc > 1) {
					if (!strcasecmp_P(argv[1], PSTR("none"))) {
						RelayConfiguration.K[id] = 0;
					} else if ((argv[1][0] == 'k' || argv[1][0] == 'K') && isdigit(argv[1][1])) {
						RelayConfiguration.K[id] = atoi(argv[1] + 1);
					} else {
						printf_P(PSTR("?? '%s'\n"), argv[1]);
					}

					// sanity check
					for (enum RelayID other = _RELAY_FIRST; other < _NRELAYS; ++other) {
						if ((other != id) && (RelayConfiguration.K[other] == RelayConfiguration.K[id])) {
							RelayConfiguration.K[other] = 0;
						}
					}
				}
				break;
			}
		}
		// apply states, turn off unassigned relays
		for (int i = 1; i <= 8; ++i) {
			bool val = false;

			for (enum RelayID id = _RELAY_FIRST; id < _NRELAYS; ++id) {
				if (RelayConfiguration.K[id] == i) {
					val = Relay_State(id);
					break;
				}
			}

			k_set(i, val);
		}


		EEConfig_Save();
	}
}
