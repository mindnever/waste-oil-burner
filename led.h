#ifndef _WOB_LED_H_
#define _WOB_LED_H_

typedef enum
{
    blink_off,
    blink_slow,
    blink_fast,
    blink_on,
    blink_pulse
} blink_t;

extern uint8_t led_a;
extern uint8_t led_b;

void handle_led();


#endif /* _WOB_LED_H_ */
