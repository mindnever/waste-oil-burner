#include <avr/pgmspace.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "zones.h"
#include "mqtt.h"
#include "flame.h"
#include "relay.h"
#include "adc.h"
#include "eeconfig.h"
#include "rx.h"
#include "cli.h"
#include "boot.h"
#include "usart.h"
#include "microrl/src/microrl.h"

static microrl_t mrl;

extern uint8_t _end;
extern uint8_t __stack;

#define STACK_CANARY 0xc5

void StackPaint(void) __attribute__((naked)) __attribute__((section(".init1")));

void StackPaint(void)
{
#if 0
    uint8_t *p = &_end;

    while (p <= &__stack) {
        *p = STACK_CANARY;
        p++;
    }
#else
    __asm volatile ("    ldi r30,lo8(_end)\n"
                    "    ldi r31,hi8(_end)\n"
                    "    ldi r24,lo8(0xc5)\n" /* STACK_CANARY = 0xc5 */
                    "    ldi r25,hi8(__stack)\n"
                    "    rjmp .cmp\n"
                    ".loop:\n"
                    "    st Z+,r24\n"
                    ".cmp:\n"
                    "    cpi r30,lo8(__stack)\n"
                    "    cpc r31,r25\n"
                    "    brlo .loop\n"
                    "    breq .loop" ::);
#endif
}

uint16_t StackCount(void)
{
    const uint8_t *p = &_end;
    uint16_t c = 0;

    while (*p == STACK_CANARY && p <= &__stack) {
        p++;
        c++;
    }

    return c;
}


static void CLI_Dfu()
{
    if (pgm_read_word_near(BOOTLOADER_MAGIC_SIGNATURE_START) == BOOTLOADER_MAGIC_SIGNATURE) {
        BootloaderAPI_GoDFU();
    }
}

extern uint8_t mcusr_save;

// MCUSR: USBRF, JTRF, WDRF, BORF, EXTRF, PORF

static void CLI_Info()
{
#if 0
    char mcusr_str[32];

    mcusr_str[0] = 0;
#ifdef USBRF
    if (mcusr_save & _BV(USBRF)) {
        strcat_P(mcusr_str, PSTR(" USB"));
    }
#endif
    if (mcusr_save & _BV(WDRF)) {
        strcat_P(mcusr_str, PSTR(" WDR"));
    }
#ifdef JTRF
    if (mcusr_save & _BV(JTRF)) {
        strcat_P(mcusr_str, PSTR(" JTAG"));
    }
#endif
    if (mcusr_save & _BV(BORF)) {
        strcat_P(mcusr_str, PSTR(" BOR"));
    }
    if (mcusr_save & _BV(EXTRF)) {
        strcat_P(mcusr_str, PSTR(" EXT"));
    }
    if (mcusr_save & _BV(PORF)) {
        strcat_P(mcusr_str, PSTR(" POR"));
    }

    printf_P(PSTR("Up: %lu\nStack: %u\nmcusr: (%02x) %s\n"), sys_millis / 1000, StackCount(), mcusr_save, mcusr_str);
#endif /* if 0 */

    uint16_t bootloader_signature = pgm_read_word_near(BOOTLOADER_MAGIC_SIGNATURE_START);
    printf_P(PSTR("bl_sig: %02x\n"), bootloader_signature);
    if (bootloader_signature != BOOTLOADER_MAGIC_SIGNATURE) {
// printf_P(PSTR("Unknown bootloader\n"));
        return;
    }
    uint16_t bootloader_class = pgm_read_word_near(BOOTLOADER_CLASS_SIGNATURE_START);
    printf_P(PSTR("bl_class: %02x\nlock: %u\n"), bootloader_class, BootloaderAPI_ReadLock());
}

static void CLI_Puts(const char *str)
{
    fputs(str, stdout);
}

static int CLI_Execute(int argc, const char *const *argv)
{
    printf_P(PSTR("\n"));

    if (!strcasecmp_P(argv[0], PSTR("eeprom"))) {
        if (argc > 1) {
            if (!strcasecmp_P(argv[1], PSTR("format"))) {
                EEConfig_Format();
            } else if (!strcasecmp_P(argv[1], PSTR("backup"))) {
                EEConfig_Backup();
            } else if (!strcasecmp_P(argv[1], PSTR("restore"))) {
                EEConfig_Restore();
            } else if (!strcasecmp_P(argv[1], PSTR("dump"))) {
                const char *argv0 = "print";
                ADC_CLI(1, &argv0);
                Flame_CLI(1, &argv0);
                Relay_CLI(1, &argv0);
                Zones_CLI(1, &argv0);
                mqtt_cli(1, &argv0);
            }
        }
#if defined(USE_USB_HID)
    } else if (!strcasecmp_P(argv[0], PSTR("hid"))) {
        if (argc > 1) {
            if (!strcasecmp_P(argv[1], PSTR("on"))) {
                hid_enabled = 1;
            } else if (!strcasecmp_P(argv[1], PSTR("off"))) {
                hid_enabled = 0;
            }
        }
        printf_P(PSTR("HID reporting is %s\n"), hid_enabled ? "on" : "off");
#endif
    } else if (!strcasecmp_P(argv[0], PSTR("info"))) {
        CLI_Info();
// } else if(!strcasecmp_P(argv[0], PSTR("uart")) || !strcasecmp_P(argv[0], PSTR("usart"))) {
// USART_CLI(argc - 1, argv + 1);
    } else if (!strcasecmp_P(argv[0], PSTR("dfu"))) {
        CLI_Dfu();
    } else if (!strcasecmp_P(argv[0], PSTR("analog"))) {
        ADC_CLI(argc - 1, argv + 1);
    } else if (!strcasecmp_P(argv[0], PSTR("flame"))) {
        Flame_CLI(argc - 1, argv + 1);
    } else if (!strcasecmp_P(argv[0], PSTR("relay"))) {
        Relay_CLI(argc - 1, argv + 1);
#ifdef USE_MQTT
    } else if (!strcasecmp_P(argv[0], PSTR("mqtt"))) {
        mqtt_cli(argc - 1, argv + 1);
#endif
    } else if (!strcasecmp_P(argv[0], PSTR("zone"))) {
        Zones_CLI(argc - 1, argv + 1);
    } else {
        printf_P(PSTR("?? '%s'\n"), argv[0]);
    }
    return 0;
}

void CLI_Redraw(void)
{
    microrl_insert_char(&mrl, KEY_DC2);
}

void CLI_Task(void)
{
    uint8_t cmdBuf[65];

    uint16_t r = fread(cmdBuf, 1, sizeof(cmdBuf) - 2, stdin);

    if (r > 0) {
        cmdBuf[r] = 0;
        for (uint16_t i = 0; i < r; ++i) {
            microrl_insert_char(&mrl, cmdBuf[i]);
        }
    }
}

void CLI_Init(void)
{
    microrl_init(&mrl, CLI_Puts);
    microrl_set_execute_callback(&mrl, CLI_Execute);
}

void CLI_Erase(void)
{
    fputs_P(PSTR("\r\x1b[K"), stdout);
}


void CLI_notify_P(const char *tag, const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);

    CLI_Erase();

    printf_P(PSTR("<<< %S >>> "), tag);

    vfprintf_P(stdout, fmt, ap);

    fputc('\n', stdout);

    CLI_Redraw();

    va_end(ap);
}
