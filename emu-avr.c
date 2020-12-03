#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <poll.h>
#include <IOKit/serial/ioss.h>

#include "microrl/src/microrl.h"

FILE *serial;

static microrl_t mrl;

static void set_raw(int fd)
{
    struct termios theTermios;

    tcgetattr(fd, &theTermios);

    cfmakeraw(&theTermios);

    theTermios.c_cflag |= CS8 | CREAD | CLOCAL; // turn on READ

    tcsetattr(fd, TCSANOW, &theTermios);
}

static void set_baud(int fd, unsigned baud_rate)
{
    // OSX

    if (ioctl(fd, IOSSIOSPEED, &baud_rate) < 0) {
        perror("IOSSIOSPEED");
    }
}

static char *unescape(char *dst, int dst_len, const char *src, int start, int end)
{
    bool esc = false;

    while (start < end) {
        char c = src[start++];
        if (esc) {
            switch (c) {
            case 'r':
                c = '\r';
                break;
            case 'n':
                c = '\n';
                break;
            case 't':
                c = '\t';
                break;
            }
            esc = false;
        } else if (c == '\\') {
            esc = true;
            continue;
        }

        if (dst_len > 1) {
            *dst++ = c;
            --dst_len;
        }
    }

    *dst = 0;
    return dst;
}

static void CLI_Puts(const char *str)
{
    fputs(str, stdout);
    fflush(stdout);
}

static void fputjs(const char *val, FILE *fp)
{
    fputc('"', fp);
    char c;

    while ((c = *val++)) {
        bool esc = false;
        switch (c) {
        case '\r':
            c   = 'r';
            esc = true;
            break;
        case '\n':
            c   = 'n';
            esc = true;
            break;
        case '\t':
            c   = 't';
            esc = true;
            break;
        case '"':
        case '\\':
            esc = true;
            break;
        }
        if (esc) {
            fputc('\\', fp);
        }
        fputc(c, fp);
    }
    fputc('"', fp);
}


static int CLI_Execute(int argc, const char *const *argv)
{
    if (!strcasecmp(argv[0], "mqtt")) {
        fputs("cli:[", serial);
        for (int i = 1; i < argc; ++i) {
            if (i > 1) {
                fputc(',', serial);
            }
            fputjs(argv[i], serial);
        }
        fputs("]\n", serial);
    } else if (!strcasecmp(argv[0], "exit")) {
        exit(0);
    } else {
        printf("?? '%s'", argv[0]);
    }

    printf("\n");

    return 0;
}

static char **CLI_GetCompletion(int argc, const char *const *argv)
{
    static char *tok = 0;

    return &tok;
}

void CLI_Redraw(void)
{
    microrl_insert_char(&mrl, KEY_DC2);
}

void CLI_Task(void)
{
    uint8_t cmdBuf[65];

    int r = read(STDIN_FILENO, cmdBuf, sizeof(cmdBuf));

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
    microrl_set_complete_callback(&mrl, CLI_GetCompletion);
}

void CLI_Erase(void)
{
    fputs("\r\x1b[K", stdout);
}


static void mqtt_recv()
{
    static char buf[128];
    static uint8_t pos = 0;

    char c;

    // mqtt should support following topics
    // <id>/zone/+/setpoint/set
    // <id>/zone/+/hysteresis/set
    // <id>/zone/+/enabled/set

    while (read(fileno(serial), &c, 1) > 0) {
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            // process
            bool dump = true;

            if (pos > 4) {
                if (!memcmp(buf, "cli:", 4)) {
                    CLI_Erase();
                    fwrite(&buf[4], pos - 4, 1, stdout);
                    fputs("\n", stdout);
                    CLI_Redraw();
                    dump = false;
                } else if (!memcmp(buf, "msg:", 4)) {
                    char *msg = strchr(&buf[4], ':');
                    dump = true;
                    if (msg) {}
                }
            }

            if (dump) {
                CLI_Erase();
                fwrite(buf, pos, 1, stdout);
                fputs("\n", stdout);
                CLI_Redraw();
            }

            pos = 0;
        } else if (pos == sizeof(buf)) {
            printf("mqtt_task: buffer overflow\r\n");
        } else {
            buf[pos++] = c;
        }
    }
}


int main(int argc, char * *argv)
{
    const char *port = "/dev/cu.SLAB_USBtoUART";

    if (argc > 1) {
        port = argv[1];
    }

    printf("Using port %s\n", port);

    int fd = open(port, O_RDWR);

    if (fd < 0) {
        perror(port);
        exit(1);
    }

    serial = fdopen(fd, "w+");
    if (!serial) {
        perror("fdopen");
        exit(1);
    }

    set_raw(fd);
    set_baud(fd, 74880);

    set_raw(STDIN_FILENO);

    long ap = 1;

    ioctl(STDIN_FILENO, FIONBIO, &ap);
    ioctl(fd, FIONBIO, &ap);

    CLI_Init();


    struct pollfd pfd[2] = {
        { .fd = fd,           .events = POLLIN },
        { .fd = STDIN_FILENO, .events = POLLIN }
    };

    while (true) {
        poll(pfd, 2, 1000);
        CLI_Task();
        mqtt_recv();
    }
}
