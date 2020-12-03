#include <avr/io.h>

#include "hw.h"
#include "led.h"

#define TICK_FAST (50 / TICK_MS)
#define TICK_SLOW (500 / TICK_MS)

uint8_t led_a = blink_off;
uint8_t led_b = blink_off;

uint8_t led_state(blink_t b, uint8_t slow, uint8_t fast)
{
    if (b == blink_fast) {
        b = fast ? blink_on : blink_off;
    }
    if (b == blink_slow) {
        b = slow ? blink_on : blink_off;
    }

    return b == blink_on;
}

void handle_led(void)
{
    static uint8_t fast = 0;
    static uint8_t slow = 0;
    static uint16_t pulse_a = 0;
    static uint16_t pulse_b = 0;
    static uint16_t tick_f = 0, tick_s = 0;

    if (led_a == blink_pulse) {
        pulse_a = TICK_SLOW;
        led_a   = blink_off;
    }

    if (led_b == blink_pulse) {
        pulse_b = TICK_SLOW;
        led_b   = blink_off;
    }

    if (pulse_a > 0) {
        --pulse_a;
    }
    if (pulse_b > 0) {
        --pulse_b;
    }

    if (++tick_f > TICK_FAST) {
        fast   = !fast;
        tick_f = 0;
    }

    if (++tick_s > TICK_SLOW) {
        slow   = !slow;
        tick_s = 0;
    }
#ifdef LED_A_PORT
    if (led_state((pulse_a > 0) ? blink_on : led_a, slow, fast)) {
        LED_ON(LED_A);
    } else {
        LED_OFF(LED_A);
    }
#endif
#ifdef LED_B_PORT
    if (led_state((pulse_b > 0) ? blink_on : led_b, slow, fast)) {
        LED_ON(LED_B);
    } else {
        LED_OFF(LED_B);
    }
#endif
}
