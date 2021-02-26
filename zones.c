#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "eeconfig.h"
#include "zones.h"
#include "hid.h"
#include "vcp.h"
#include "hw.h"
#include "flame.h"
#include "cli.h"
#include "relay.h"

static ThermalZone Zones[NUM_ZONES];

static const char water_PSTR[] PROGMEM = "water";
static const char oil_PSTR[] PROGMEM = "oil";
static const char ext1_PSTR[] PROGMEM = "ext1";
static const char ext2_PSTR[] PROGMEM = "ext2";
static const char ext3_PSTR[] PROGMEM = "ext3";
static const char ext4_PSTR[] PROGMEM = "ext4";

static const char *zone_names[] = {
	[ZONE_ID_WATER] = water_PSTR,
	[ZONE_ID_OIL]   = oil_PSTR,
	[ZONE_ID_EXT1]  = ext1_PSTR,
	[ZONE_ID_EXT2]  = ext2_PSTR,
	[ZONE_ID_EXT3]  = ext3_PSTR,
	[ZONE_ID_EXT4]  = ext4_PSTR,
};

static const char none_PSTR[] PROGMEM = "none";
static const char analog_PSTR[] PROGMEM = "analog %u";
static const char button_PSTR[] PROGMEM = "button";
static const char rfrx_PSTR[] PROGMEM = "rfrx %u";

static const char *sensor_names[] = {
	[SENSOR_NONE]   = none_PSTR,
	[SENSOR_ANALOG] = analog_PSTR,
	[SENSOR_BUTTON] = button_PSTR,
	[SENSOR_RFRX]   = rfrx_PSTR,
};

#define CURRENT_VALID (10 * 60 * 1000L) / TICK_MS

static void Zones_ZoneCLI(ThermalZone *zone, int argc, const char *const *argv);

void Zones_SetCurrent(sensor_type_t type, uint16_t id, int16_t current)
{
	for (uint8_t i = 0; i < NUM_ZONES; ++i) {
		ThermalZone *zone = &Zones[i];

		if (zone->Config.SensorType != type) {
			continue;
		}

		if (zone->Config.SensorID != id) {
			continue;
		}

		zone->Current = current;
		zone->Valid   = CURRENT_VALID;
	}
}

void Zones_Update(uint8_t master_enable)
{
	for (uint8_t i = 0; i < NUM_ZONES; ++i) {
		ThermalZone *zone = &Zones[i];

		if (zone->Valid > 0) {
			--zone->Valid;
		}

		if (!master_enable || !zone->Config.Enabled || !zone->Valid) {
			zone->Active = false;
		} else if (zone->Current < (zone->Config.SetPoint - (int16_t)zone->Config.Hysteresis)) {
			zone->Active = true;
		} else if (zone->Current > zone->Config.SetPoint) {
			zone->Active = false;
		}

		if (zone->Active) {
			// dec timeout
			if (zone->Config.Timeout) {
				if (zone->Timeout) {
					--zone->Timeout;
				} else {
					CLI_notify_P(zone_names[i], PSTR("TIMEOUT"));
					FlameData.state   = state_fault;
					FlameData.fault_P = PSTR("Timeout");
				}
			}
		} else {
			// clear timeout
			zone->Timeout = (uint32_t)zone->Config.Timeout * (60L * 1000 / TICK_MS);
		}
	}


	Relay_Set(RELAY_HEATER, Zones_GetZone(ZONE_ID_OIL)->Active);
	// ZONE_ID_WATER is internally connected to burner fsm.
	Relay_Set(RELAY_EXT1, Zones_GetZone(ZONE_ID_EXT1)->Active);
	Relay_Set(RELAY_EXT2, Zones_GetZone(ZONE_ID_EXT2)->Active);
	Relay_Set(RELAY_EXT3, Zones_GetZone(ZONE_ID_EXT3)->Active);
	Relay_Set(RELAY_EXT4, Zones_GetZone(ZONE_ID_EXT4)->Active);
}

void Zones_DumpZone(enum ZoneID id, ThermalZone *zone)
{
	printf_P(PSTR("zone %5S %S setpoint %5.1f hysteresis %5.1f timeout %2u sensor "),
	         zone_names[id],
	         zone->Config.Enabled ? PSTR("enable ") : PSTR("disable"),
	         (float)zone->Config.SetPoint / 10,
	         (float)zone->Config.Hysteresis / 10,
	         zone->Config.Timeout);

	printf_P(sensor_names[zone->Config.SensorType], zone->Config.SensorID);

	// state, current, valid, timeout
	fputs_P(PSTR(" ;"), stdout);

	if (zone->Active) {
		fputs_P(PSTR(" active"), stdout);
	}
	if (zone->Valid > 0) {
		fputs_P(PSTR(" valid"), stdout);
	}
	printf_P(PSTR(" current %5.1f"), (float)zone->Current / 10);

	if (zone->Config.Timeout && zone->Active) {
		printf_P(PSTR(" timeout in %u mins"), (unsigned)(zone->Timeout * TICK_MS / 1000 / 60));
	}

	fputc('\n', stdout);
}

ThermalZone *Zones_GetZone(enum ZoneID id)
{
	if (id < NUM_ZONES) {
		return &Zones[id];
	}
	return 0;
}

void Zones_CLI(int argc, const char *const *argv)
{
	if (argc == 0) {
		printf_P(PSTR("?? op\n"));
		return;
	}

	if (!strcasecmp_P(argv[0], PSTR("print"))) {
		Zones_Dump();
	} else if (argc > 1) {
		ThermalZone *zone = 0;

		for (int i = 0; i < NUM_ZONES; ++i) {
			if (!strcasecmp_P(argv[0], zone_names[i])) {
				zone = &Zones[i];
			}
		}
		if (zone) {
			Zones_ZoneCLI(zone, argc - 1, argv + 1);
			EEConfig_Save();
		} else {
			printf_P(PSTR("?? '%s'\n"), argv[0]);
		}
	} else {
		printf_P(PSTR("?? zone\n"));
	}
}

void Zones_ZoneCLI(ThermalZone *zone, int argc, const char *const *argv)
{
	for (int i = 0; i < argc; ++i) {
		if (!strcasecmp_P(argv[i], PSTR("enable"))) {
			zone->Config.Enabled = true;
		} else if (!strcasecmp_P(argv[i], PSTR("disable"))) {
			zone->Config.Enabled = false;
		} else if (!strcasecmp_P(argv[i], PSTR("setpoint"))) {
			++i;
			if (i < argc) {
				zone->Config.SetPoint = atof(argv[i]) * 10;
			}
		} else if (!strcasecmp_P(argv[i], PSTR("timeout"))) {
			++i;
			if (i < argc) {
				zone->Config.Timeout = atoi(argv[i]);
			}
		} else if (!strcasecmp_P(argv[i], PSTR("sensor"))) {
			++i;

			if (i < argc) {
				if (!strcasecmp_P(argv[i], PSTR("none"))) {
					zone->Config.SensorType = SENSOR_NONE;
				} else if (!strcasecmp_P(argv[i], PSTR("analog"))) {
					zone->Config.SensorType = SENSOR_ANALOG;
				} else if (!strcasecmp_P(argv[i], PSTR("button"))) {
					zone->Config.SensorType = SENSOR_BUTTON;
				} else if (!strcasecmp_P(argv[i], PSTR("rfrx"))) {
					zone->Config.SensorType = SENSOR_RFRX;
				} else {
					printf_P(PSTR("?? '%s'\n"), argv[i]);
				}

				++i;
				if (i < argc) {
					zone->Config.SensorID = atoi(argv[i]);
				}
			}
		} else if (!strcasecmp_P(argv[i], PSTR("hysteresis"))) {
			++i;
			if (i < argc) {
				zone->Config.Hysteresis = atof(argv[i]) * 10;
			}
		} else {
			printf_P(PSTR("?? '%s'\n"), argv[i]);
			break;
		}
	}
}

void Zones_Dump()
{
	for (int i = 0; i < _WOB_REPORT_ZONE_COUNT; ++i) {
		Zones_DumpZone(i, Zones_GetZone(i));
	}
}

uint8_t Zones_IsCold(enum ZoneID id)
{
	ThermalZone *zone = Zones_GetZone(id);

	return zone->Current < (zone->Config.SetPoint - zone->Config.Hysteresis);
}

uint8_t Zones_IsHot(enum ZoneID id)
{
	ThermalZone *zone = Zones_GetZone(id);

	return zone->Current > zone->Config.SetPoint;
}
