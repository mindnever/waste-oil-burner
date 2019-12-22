#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <stdio.h>

#include "led.h"
#include "hw.h"

#define TIME_SYNC 4000
#define TIME_END  6000
#define TIME_SHORT 900
#define TIME_LONG 1900

#define TIME_LOW 540

#if (F_CPU == 16000000)
# define OCR_VAL(x) ((x)*2)
#elif (F_CPU == 8000000)
# define OCR_VAL(x) (x)
#else
# error Unsupported F_CPU
#endif


void RfTx_Init()
{
#ifdef TX_DATA_PORT
    IO_DIR_OUT(TX_DATA);
    IO_PIN_LOW(TX_DATA);
#endif    
    // assume RfRx_Init() was also done and TIMER1 is setup correctly
}

#ifdef TX_DATA_PORT
static uint8_t tx_buffer[16];
static uint8_t tx_len;
static uint8_t tx_burst;
static uint8_t tx_pos;
static uint8_t tx_bit; // 0xf0 = sync

#define BIT_FIRST 7
#define BIT_NEXT 0xff
#define BIT_SYNC 0xfe

ISR (TIMER1_COMPB_vect)
{
    led_a = blink_on;

    if(tx_len == 0) {
        TIMSK1 &= ~_BV(OCIE1C); // disable self
        return;
    }

    if(tx_bit == BIT_NEXT) {
       tx_bit = BIT_FIRST;
       ++tx_pos; 
    }
    
    if(tx_pos == tx_len) {
        tx_pos = 0;
        tx_bit = BIT_SYNC;
        --tx_burst;
    }
    
    if(tx_burst == 0) {
        TIMSK1 &= ~_BV(OCIE1C); // disable self
        return;
    }
    
    if(IO_PORT_OUT( IO_PORTNAME( TX_DATA ) ) & _BV( IO_PIN( TX_DATA ) ) ) {
        // if TX_DATA was high, make it low and set timeout to required value            
        IO_PIN_LOW( TX_DATA );

        if(tx_bit == BIT_SYNC) {
            OCR1C += OCR_VAL( TIME_SYNC );
            tx_bit = BIT_FIRST;
        } else {
            if(tx_buffer[tx_pos] & (1 << tx_bit)) {
                OCR1C += OCR_VAL( TIME_LONG );
            } else {
                OCR1C += OCR_VAL( TIME_SHORT );
            }
            --tx_bit;
        }
    } else {
        // if TX_DATA was already low, make it high and set timeout to TIME_LOW
        OCR1C += OCR_VAL( TIME_LOW );
        IO_PIN_HIGH( TX_DATA );
    }
}
#endif

void RfTx_Transmit(const uint8_t *buffer, uint8_t len, uint8_t repeat)
{
#ifdef TX_DATA_PORT
    if(tx_burst == 0) {
        if(len > sizeof(tx_buffer)) { len = sizeof(tx_buffer); }
        memcpy(tx_buffer, buffer, len);
        tx_bit = BIT_SYNC;
        tx_len = len;
        tx_burst = repeat;
        OCR1C = TCNT1; // force irq?
        TCCR1C |= _BV(FOC1C);
        TIMSK1 |= _BV(OCIE1C); // enable interrupt
    }
#else
    (void)buffer;
    (void)len;
    (void)repeat;
#endif
}

