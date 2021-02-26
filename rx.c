#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "rx.h"
#include "hw.h"
#include "usb.h"
#include "vcp.h"
#include "led.h"
#include "mqtt.h"

#define TIMER1_PRESCALER_8X         _BV(CS11)
#define TIMER1_CAPTURE_FALLING_EDGE (0)
#define TIMER1_CAPTURE_RISING_EDGE  _BV(ICES1)

#define TIMER1_TIMEOUT_MS           5
#define F_TIMER1                    (F_CPU / 8)

#define TIMER1_MS_TICKS             (F_TIMER1 / 1000)
#define TIMER1_TIMEOUT_TICKS        (TIMER1_TIMEOUT_MS * TIMER1_MS_TICKS)

#define IDLE_TICKS                  (TICK_MS * TIMER1_MS_TICKS)

#if TIMER1_TIMEOUT_TICKS > 65534
# error TIMER1_TIMEOUT_TICKS too big
#endif

#if IDLE_TICKS > 65534
# error IDLE_TICKS too big
#endif

#define RX_SAMPLES 400

typedef enum {
	S_UNDEF,
	S_SYNC,
	S_LONG,
	S_SHORT,
	S_END,
	S_VLP,
} sample_t;


static sample_t rx_buffer[RX_SAMPLES];
static uint16_t rx_current;

static uint8_t timer1_comp;

static uint8_t RfRx_decode(RfRx_Callback cb);
// static void RfRx_dump();

uint16_t sys_idle_ticks = IDLE_TICKS;
uint16_t sys_busy_ticks = 0;
uint32_t sys_millis;

void Sys_Idle()
{
	sys_millis    += TICK_MS;
	sys_busy_ticks = OCF1B - TCNT1;
	while (!(TIFR1 & _BV(OCF1B))) {
#ifdef USE_MQTT
		mqtt_task(0);
#endif
	}
	OCR1B += IDLE_TICKS;
	TIFR1 |= _BV(OCF1B);
}

void RfRx_Init()
{
	IO_DIR_IN(RX_DATA); // input
	IO_PIN_HIGH(RX_DATA); // internal pullup

	// set prescaler
	// enable capture interrupt

	TCCR1A = 0; // Normal port operation, OCnA/OCnB/OCnC disconnected
	TCCR1B = TIMER1_CAPTURE_FALLING_EDGE | TIMER1_PRESCALER_8X; // falling edge detect (RF RX DATA is active low). Prescaler 8x
	// (we need to measure 500uS -  9000uS intervals, with 16Mhz F_CPU, so 8x prescaler would give 16mS max interval)

	TIMSK1 = _BV(ICIE1); // enable input capture interrupt

	OCR1B  = TCNT1 + IDLE_TICKS;
}

void RfRx_Task(RfRx_Callback cb)
{
	if (timer1_comp) {
		if ((rx_current > 0) && RfRx_decode(cb)) {
			led_b = blink_pulse;
		}

		timer1_comp = 0;
		rx_current  = 0;
	}
}

ISR(TIMER1_COMPA_vect) {
	// timeout!
	timer1_comp = 1;

	// reset edge
	TCCR1B &= ~_BV(ICES1); // falling edge again.
	TIMSK1 &= ~_BV(OCIE1A); // disable self
}

ISR(TIMER1_CAPT_vect) {
	static uint16_t prev;
	uint16_t capture = ICR1;

	// disable irq, clear pending
	TIMSK1 &= ~_BV(OCIE1A);
	TIFR1   = _BV(OCF1A);

	if (timer1_comp) {
		rx_current  = 0;
		timer1_comp = 0;
	}

	OCR1A   = capture + TIMER1_TIMEOUT_TICKS;

	// enable irq
	TIMSK1 |= _BV(OCIE1A);

#if (F_CPU == 16000000)
	uint16_t len = (capture - prev) / 2;
#elif (F_CPU == 8000000)
	uint16_t len = (capture - prev);
#else
# error Unsupported F_CPU
#endif


	if (TCCR1B & TIMER1_CAPTURE_RISING_EDGE) {
		// look for sync.
		sample_t sample = S_UNDEF;

		if (len > 5000) {
			sample = S_END;
		} else if ((len > 3500) && (len < 4500)) {
			sample = S_SYNC;
		} else if ((len > 800) && (len < 1100)) {
			sample = S_SHORT;
		} else if ((len > 1800) && (len < 2000)) {
			sample = S_LONG;
		}


		if (((rx_current == 0) && (sample == S_SYNC)) // sync
		    || ((rx_current > 0) && (rx_current < RX_SAMPLES))) {
			rx_buffer[rx_current] = sample;
			++rx_current;
		}
	}

	prev    = capture;
	// toggle edge
	TCCR1B ^= _BV(ICES1);
}

/* Working routine for checking the crc, lots of magic but it works */

static uint8_t rp[] = { 0xb8, 0x80, 0xea, 0xfe, 0x80 };
// static uint8_t rp[] = {0xea, 0x8f, 0x6a, 0xfa, 0x50};

int rubicson_crc_check(uint8_t *packet)
{
	uint8_t crc, w;
	uint8_t diff[9];
	int i, ret;

	// diff against ref packet

	diff[0] = rp[0] ^ packet[0];
	diff[1] = rp[1] ^ packet[1];
	diff[2] = rp[2] ^ packet[2];
	diff[3] = rp[3] ^ packet[3];
	diff[4] = rp[4] ^ packet[4];

// fprintf(stdout, "%02x %02x %02x %02x %02x\n",rp[0],rp[1],rp[2],rp[3],rp[4]);
// fprintf(stdout, "%02x %02x %02x %02x %02x\n",bb[1][0],bb[1][1],bb[1][2],bb[1][3],bb[1][4]);
// fprintf(stdout, "%02x %02x %02x %02x %02x\n",diff[0],diff[1],diff[2],diff[3],diff[4]);

	for (crc = 0, w = 0xf1, i = 0; i < 7; i++) {
		uint8_t c = diff[i / 2];
		unsigned digit = (i & 1) ? c & 0xF : (c & 0xF0) >> 4;
		unsigned j;
		for (j = 4; j-- > 0;) {
			if ((digit >> j) & 1) {
				crc ^= w;
			}
			w = (w >> 1) ^ ((w & 1) ? 0x98 : 0);
		}
	}
	if (crc == (((diff[3] << 4) & 0xF0) | (diff[4] >> 4))) {
// printf ("\ncrc ok: %x\n", crc);
		ret = 1;
	} else {
// printf ("\ncrc fail: %x\n", crc);
		ret = 0;
	}

	return ret;
}

typedef struct {
	uint8_t data[5];
} packet_t;

#define BURST_LEN 10

static bool rubicson_decode(packet_t *packet, struct RfRx_SensorData *data)
{
/*
   ID ID ID CC         8-bit ID, the two least significant might encode the channel
   BF CC               4 bits of flags:
                         B  =1 battery good
                         F  =1 forced transmission
                         CC =  channel, zero based
   TT TT TT TT TT TT   12 bits signed integer, temp in Celsius/10
   11 11               const / reserved
   HH HH HH HH         8 bits, either
                        - humidity (or 1111 xxxx if not available); or
                        - a CRC, e.g. Rubicson, algorithm in source code linked above
 */
	if ((packet->data[3] & 0xf0) != 0xf0) {
		return false;
	}

	data->temp      = (int16_t)((uint16_t)(packet->data[1] << 12) | (packet->data[2] << 4));
	data->temp      = data->temp >> 4;

	data->channel   = ((packet->data[1] & 0x30) >> 4) + 1;
	data->battery   = (packet->data[1] & 0x80);
	data->sensor_id = packet->data[0];

	data->humidity  = ((packet->data[3] & 0x0f) << 4) | (packet->data[4] >> 4);

	return true;
}

static uint8_t RfRx_decode(RfRx_Callback cb)
{
	packet_t packet;

	packet_t history[BURST_LEN];
	uint8_t hits[BURST_LEN];
	uint8_t hcurrent = 0;

	int8_t bit = -1;
	uint8_t decoded  = 0;

	struct RfRx_SensorData data;

	memset(&hits, 0, sizeof(hits));

	for (int i = 0; i < rx_current; ++i) {
		sample_t sample = rx_buffer[i];

		if (bit == 36) {
			uint8_t match = 0;

			for (uint8_t j = 0; j < hcurrent; ++j) {
				match = !memcmp(&packet, &history[j], sizeof(packet));

				if (match) {
					hits[j]++;
					break;
				}
			}

			if (!match && (hcurrent < BURST_LEN)) {
				memcpy(&history[hcurrent], &packet, sizeof(packet));
				hits[hcurrent] = 1;
				++hcurrent;
			}

			++decoded;
		}

		if (sample == S_SYNC) { // sync
			bit = 0;
			memset(&packet, 0, sizeof(packet));
			continue;
		}

		if (bit < 0) { // sync not found
			continue;
		}

		if (sample == S_SHORT) {
			// 1
			++bit;
			continue;
		}

		if (sample == S_LONG) {
			packet.data[bit / 8] |= 1 << (7 - (bit % 8));

			++bit;
			continue;
		}

		bit = -1;
	}

	uint8_t maxj = 0;

	for (uint8_t j = 0; j < hcurrent; ++j) {
		if (hits[j] > hits[maxj]) {
			maxj = j;
		}
	}

	if (maxj < hcurrent) {
		if (rubicson_decode(&history[maxj], &data) && cb) {
			data._raw      = history[maxj].data;
			data._samples  = decoded;
			data._matching = hits[maxj];

			cb(&data);
		}
	}

	return decoded;
}

#if 0
static void RfRx_dump()
{
	printf("Got %d:", rx_current);
	for (int i = 0; i < rx_current && i < 50; ++i) {
		printf("%u ", rx_buffer[i]);
	}
	printf("\n");
}
#endif
