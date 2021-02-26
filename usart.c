#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "hw.h"
#include "hid.h"
#include "usart.h"
#include "fifo.h"

#if defined UBRR
#   define  ODDBG_UBRR     UBRR
#elif defined UBRRL
#   define  ODDBG_UBRR     UBRRL
#elif defined UBRR0
#   define  ODDBG_UBRR     UBRR0
#elif defined UBRR1
#   define  ODDBG_UBRR     UBRR1
#elif defined UBRR0L
#   define  ODDBG_UBRR     UBRR0L
#elif defined UBRR1L
#   define  ODDBG_UBRR     UBRR1L
#endif

#if defined UCR
#   define  ODDBG_UCR      UCR
#elif defined UCSRB
#   define  ODDBG_UCR      UCSRB
#elif defined UCSR0B
#   define  ODDBG_UCR      UCSR0B
#elif defined UCSR1B
#   define  ODDBG_UCR      UCSR1B
#endif

#if defined TXEN
#   define  ODDBG_TXEN     TXEN
#elif defined TXEN0
#   define  ODDBG_TXEN     TXEN0
#elif defined TXEN1
#   define  ODDBG_TXEN     TXEN1
#endif

#if defined RXEN
#   define  ODDBG_RXEN     RXEN
#elif defined RXEN0
#   define  ODDBG_RXEN     RXEN0
#elif defined RXEN1
#   define  ODDBG_RXEN     RXEN1
#endif

#if defined RXCIE
#   define  ODDBG_RXCIE    RXCIE
#elif defined RXCIE0
#   define  ODDBG_RXCIE    RXCIE0
#elif defined RXCIE1
#   define  ODDBG_RXCIE    RXCIE1
#endif

#if defined USR
#   define  ODDBG_USR      USR
#elif defined UCSRA
#   define  ODDBG_USR      UCSRA
#elif defined UCSR0A
#   define  ODDBG_USR      UCSR0A
#elif defined UCSR1A
#   define  ODDBG_USR      UCSR1A
#endif

#if defined UDRE
#   define  ODDBG_UDRE     UDRE
#elif defined UDRE0
#   define  ODDBG_UDRE     UDRE0
#elif defined UDRE1
#   define  ODDBG_UDRE     UDRE1
#endif

#if defined UDR
#   define  ODDBG_UDR      UDR
#elif defined UDR0
#   define  ODDBG_UDR      UDR0
#elif defined UDR1
#   define  ODDBG_UDR      UDR1
#endif

#if defined USART_RX_vect_num
#   define ODDBG_RX_vect   USART_RX_vect
#elif defined USART1_RX_vect_num
#   define ODDBG_RX_vect   USART1_RX_vect
#endif

#define BAUD2DIV(baud) (((F_CPU / 16) / baud) - 1)

#define USART_RX_FIFO_SIZE 254

static struct {
	fifo_t fifo;
	uint8_t _alloc[USART_RX_FIFO_SIZE];
} rx = {
	.fifo     = {
		.in   = 0,
		.out  = 0,
		.size = USART_RX_FIFO_SIZE,
	}
};

ISR(ODDBG_RX_vect) {
	uint8_t d = ODDBG_UDR;

	fifo_write(&rx.fifo, &d, 1);
}

static int usart_putchar(char c, FILE *stream)
{
	if (fdev_get_udata(stream) && (c == '\n')) {
		usart_putchar('\r', stream);
	}

	loop_until_bit_is_set(ODDBG_USR, ODDBG_UDRE);
	ODDBG_UDR = c;
	return 0;
}

static int usart_getchar(FILE *stream)
{
// ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		uint8_t buf;

		if (fifo_read(&rx.fifo, &buf, 1) == 1) {
			return buf;
		}
	}
	return _FDEV_ERR;
}

void USART_CLI(int argc, const char *const *argv)
{
	printf_P(PSTR("rx.fifo.overflow = %u\n"), rx.fifo.overflow);
	printf_P(PSTR("rx.fifo avail_read = %u\n"), fifo_avail_read(&rx.fifo));
	printf_P(PSTR("rx.fifo avail_write = %u\n"), fifo_avail_write(&rx.fifo));
}

FILE *USART_Init(void)
{
	ODDBG_UCR |= _BV(ODDBG_TXEN) | _BV(ODDBG_RXEN) | _BV(ODDBG_RXCIE);
	ODDBG_UBRR = BAUD2DIV(74880);

	IO_DIR_OUT(UART_TX);
	IO_DIR_IN(UART_RX);

	static FILE myfp = FDEV_SETUP_STREAM(usart_putchar, usart_getchar, _FDEV_SETUP_RW);

	return &myfp;
}

#if 0
static int usart_write_sync(const void *p, uint16_t len)
{
	int ret = len;
	const uint8_t *chrs = (const uint8_t *)p;

	while (len-- > 0) {
		while (!(ODDBG_USR & (1 << ODDBG_UDRE))) {
			; /* wait for data register empty */
		}
		ODDBG_UDR = *chrs++;
	}

	return ret;
}
#endif
