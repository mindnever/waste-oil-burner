#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <avr/pgmspace.h>

#include "eeconfig.h"
#include "zones.h"
#include "relay.h"
#include "mqtt.h"
#include "cli.h"
#include "hw.h"
#include "hid.h"

FILE *mqtt;

static char rx_buf[128];
static uint8_t rx_pos = 0;


static ThermalZone Zones[ NUM_ZONES ];
static bool Relays[ _NRELAYS ];

static void mqtt_recv(mqtt_msg_callback_t on_msg);

static void mqtt_publish_va(const char *topic, bool fmt_pgm, const char *fmt, va_list ap)
{
    fputs_P(PSTR("pub:"), mqtt);
    fputs(topic, mqtt);
    fputc(':', mqtt);
    
    if(fmt_pgm) {
      vfprintf_P(mqtt, fmt, ap);
    } else {
      vfprintf(mqtt, fmt, ap);
    }

    fputs("\r\n", mqtt);
}

void mqtt_publish(const char *topic, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    
    mqtt_publish_va(topic, false, fmt, ap);
    va_end(ap);
}

void mqtt_publish_P(const char *topic, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    mqtt_publish_va(topic, true, fmt, ap);
    va_end(ap);
}


static void fputjs(const char *val, FILE *fp)
{
  fputc('"', fp);
  char c;
  while((c = *val++)) {
    bool esc = false;
    switch(c) {
      case '\r':
        c = 'r';
        esc = true;
        break;
      case '\n':
        c = 'n';
        esc = true;
        break;
      case '\t':
        c = 't';
        esc = true;
        break;
      case '"':
      case '\\':
        esc = true;
        break;
    }
    if(esc) {
      fputc('\\', fp);
    }
    fputc(c, fp);
  }
  fputc('"', fp);
}

void mqtt_subscribe(const char *topic)
{
  const char *argv[2] = {
    "subscribe", topic
  };
  
  mqtt_cli(2, argv);
}

void mqtt_cli(int argc, const char * const *argv)
{
    if(argc == 0) {
        return;
    }

#if 0    
    if(!strcasecmp_P(argv[0], PSTR("debug"))) {
      printf_P(PSTR("rx_pos = %u\n"), rx_pos);
      printf_P(PSTR("rx_buf = "));
      fwrite(rx_buf, rx_pos, 1, stdout);
      printf_P(PSTR("\n"));
      return;
    }
#endif

    fputs_P(PSTR("cli:["), mqtt);
    for(int i = 0; i < argc; ++i) {
      if(i) {
        fputc(',', mqtt);
      }
      fputjs(argv[i], mqtt);
    }
    fputs_P(PSTR("]\n"), mqtt);
}

static void zpub_P(int zone, const char *tpart, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    char topic[32];
    snprintf_P(topic, sizeof(topic), PSTR("zone/%d/%s/state"), zone, tpart);
    
    mqtt_publish_va(topic, true, fmt, ap);
    va_end(ap);
}

static void mqtt_recv(mqtt_msg_callback_t on_msg)
{

  int c;

  // mqtt should support following topics
  // <id>/zone/+/setpoint/set
  // <id>/zone/+/hysteresis/set
  // <id>/zone/+/enabled/set

  while((c = fgetc(mqtt)) >= 0) {
    if(c == '\r') {
      continue;
    }
    if(c == '\n') {
      // process
      bool dump = true;
      if(rx_pos > 4) {
          if(!memcmp(rx_buf, "cli:", 4)) {
            CLI_Erase();
            fwrite(&rx_buf[4], rx_pos - 4, 1, stdout);
            fputs_P(PSTR("\n"), stdout);
            CLI_Redraw();
            dump = false;
          } else if(!memcmp(rx_buf, "msg:", 4)) {
            char *msg = strchr(&rx_buf[4], ':');
            if(msg && on_msg) {
              *msg++ = 0;
              on_msg(&rx_buf[4], msg);
            }
            dump = false;
          }
      }
      if(dump) {
          CLI_Erase();
          fwrite(rx_buf, rx_pos, 1, stdout);
          fputc('\n', stdout);
          CLI_Redraw();
      }

      rx_pos = 0;
    } else if(rx_pos == sizeof(rx_buf)) {

    } else {
      rx_buf[rx_pos++] = c;
    }
  }


}

void mqtt_idle(void)
{
  mqtt_recv(0);
}

void mqtt_task(mqtt_msg_callback_t on_msg)
{
  mqtt_recv(on_msg);
  
#if 0
  // check zones

  for(uint8_t i = 0; i < NUM_ZONES; ++i) {
    ThermalZone *zone = Zones_GetZone(i);
    if(!zone) { continue; } // ???
    
    // check Current, Active, Config.SetPoint, Config.Hysteresis, Config.Enabled

    if(zone->Current != Zones[i].Current) {
      zpub_P(i, "current", PSTR("%.1f"), (float)zone->Current / 10);
    }
    if(zone->Active != Zones[i].Active) {
      zpub_P(i, "active", PSTR("%s"), zone->Active ? "On" : "Off");
    }
    if(zone->Config.Enabled != Zones[i].Config.Enabled) {
      zpub_P(i, "enabled", PSTR("%s"), zone->Config.Enabled ? "On" : "Off");
    }
    if(zone->Config.SetPoint != Zones[i].Config.SetPoint) {
      zpub_P(i, "setpoint", PSTR("%.1f"), (float)zone->Config.SetPoint / 10);
    }
    if(zone->Config.Hysteresis != Zones[i].Config.Hysteresis) {
      zpub_P(i, "hysteresis", PSTR("%.1f"), (float)zone->Config.Hysteresis / 10);
    }
    
    memcpy(&Zones[i], zone, sizeof(Zones[i]));
  }
  
  for(enum RelayID i = _RELAY_FIRST; i < _NRELAYS; ++i) {
    bool s = Relay_State(i);
    if(s != Relays[i]) {
      char topic[16];
      snprintf_P(topic, sizeof(topic), PSTR("relay/%s/state"), Relay_Name(i));
      
      mqtt_publish_P(topic, PSTR("%s"), s ? "On" : "Off");
      
      Relays[i] = s;
    }
  }
#endif

}

void mqtt_hid(int id, uint8_t *buffer, uint8_t size)
{
    static const char *hex_chars = "0123456789ABCDEF";
    char topic[16];
    char hex[(sizeof(HID_Report_Storage_t) * 2) + 1];

    snprintf_P(topic, sizeof(topic), PSTR("hid/%u"), id);

    uint8_t dp = 0;
    for(uint8_t i = 0; i < size; ++i) {
      uint8_t d = *buffer++;
      hex[dp++] = hex_chars[d >> 4];
      hex[dp++] = hex_chars[d & 0x0f];
    }

    hex[dp] = 0;

    mqtt_publish_P(topic, PSTR("%s"), hex);
}
