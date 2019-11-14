#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include <avr/interrupt.h>

#include "hw.h"
#include "hid.h"
#include "vcp.h"

#if defined UBRR
#   define  ODDBG_UBRR  UBRR
#elif defined UBRRL
#   define  ODDBG_UBRR  UBRRL
#elif defined UBRR0
#   define  ODDBG_UBRR  UBRR0
#elif defined UBRR0L
#   define  ODDBG_UBRR  UBRR0L
#endif

#if defined UCR
#   define  ODDBG_UCR   UCR
#elif defined UCSRB
#   define  ODDBG_UCR   UCSRB
#elif defined UCSR0B
#   define  ODDBG_UCR   UCSR0B
#endif

#if defined TXEN
#   define  ODDBG_TXEN  TXEN
#else
#   define  ODDBG_TXEN  TXEN0
#endif

#if defined RXEN
#   define  ODDBG_RXEN  RXEN
#else
#   define  ODDBG_RXEN  RXEN0
#endif

#if defined RXCIE
#   define  ODDBG_RXCIE  RXCIE
#else
#   define  ODDBG_RXCIE  RXCIE0
#endif

#if defined USR
#   define  ODDBG_USR   USR
#elif defined UCSRA
#   define  ODDBG_USR   UCSRA
#elif defined UCSR0A
#   define  ODDBG_USR   UCSR0A
#endif

#if defined UDRE
#   define  ODDBG_UDRE  UDRE
#else
#   define  ODDBG_UDRE  UDRE0
#endif

#if defined UDR
#   define  ODDBG_UDR   UDR
#elif defined UDR0
#   define  ODDBG_UDR   UDR0
#endif

#define BAUD2DIV(baud)  (((F_CPU/16)/baud)-1)

#define VCP_RX_FIFO_SIZE 64

typedef struct {
  uint16_t in, out;
  uint16_t size;
  uint8_t buffer[0];
} vcp_fifo_t;

static struct {
  vcp_fifo_t fifo;
  uint8_t _alloc[VCP_RX_FIFO_SIZE];
} rx = { 
  .fifo = {
    .in = 0,
    .out = 0,
    .size = VCP_RX_FIFO_SIZE,
  }
};

static int fifo_read(vcp_fifo_t *fifo, uint8_t *buf, int len)
{
  int i;
  
  for(i = 0; (i < len) && (fifo->in != fifo->out); ++i) {
    buf[i] = fifo->buffer[fifo->out++];
    if(fifo->out == fifo->size) {
      fifo->out = 0;
    }
  }
  
  return i;
}

static int fifo_write(vcp_fifo_t *fifo, const uint8_t *buf, int len)
{
  int i;
  for(i = 0; i < len; ++i) {
    fifo->buffer[fifo->in++] = buf[i];
    if(fifo->in == fifo->size) {
      fifo->in = 0;
    }
  }
  return i;
}

ISR (USART_RX_vect)
{
  uint8_t d = ODDBG_UDR;
  fifo_write(&rx.fifo, &d, 1);
}

void  VCP_Init(void)
{
    ODDBG_UCR |= _BV(ODDBG_TXEN) | _BV(ODDBG_RXEN) | _BV(ODDBG_RXCIE);
    ODDBG_UBRR = BAUD2DIV(38400);
    
    IO_DIR_OUT( UART_TX );
    IO_DIR_IN( UART_RX );
}

static int uart_write_sync(const void *p, uint16_t len)
{
    int ret = len;
    const uint8_t *chrs = (const uint8_t *)p;
    
    while(len-- > 0) {
        while(!(ODDBG_USR & (1 << ODDBG_UDRE)));    /* wait for data register empty */
        ODDBG_UDR = *chrs++;
    }
    
    return ret;
}

bool HID_Report(uint8_t reportId, const void *reportData, uint8_t reportSize)
{
    char tmp[2];
    static const char *hex = "0123456789abcdef";
    
    VCP_Printf_P(PSTR("R:%u:"), reportId);

    for(const uint8_t *p = (const uint8_t *) reportData; reportSize > 0; --reportSize, ++p) {
      tmp[0] = hex[*p >> 4];
      tmp[1] = hex[*p & 0x0f];
      uart_write_sync(tmp, 2);
    }
    
    uart_write_sync("\r\n", 2);
    
    return true;
}

int VCP_Write(uint8_t *buf, int len)
{
  return uart_write_sync(buf, len);
}

int VCP_Read(uint8_t *buf, int len)
{
  cli();
  int r = fifo_read(&rx.fifo, buf, len);
  sei();
  
  return r;
}

void VCP_Puts(const char *str)
{
  uart_write_sync(str, strlen(str));
}

void VCP_Printf(const char *fmt, ...)
{
  va_list ap;
  char tmp[128];
  
  va_start(ap, fmt);
  
  vsnprintf(tmp, sizeof(tmp) - 1, fmt, ap);
  
  tmp[sizeof(tmp) - 1] = 0;
  
  uart_write_sync(tmp, strlen(tmp));

  va_end(ap);
}

void VCP_Printf_P(const char *fmt, ...)
{
  va_list ap;
  char tmp[128];
  
  va_start(ap, fmt);
  
  vsnprintf_P(tmp, sizeof(tmp) - 1, fmt, ap);
  
  tmp[sizeof(tmp) - 1] = 0;
  
  uart_write_sync(tmp, strlen(tmp));

  va_end(ap);
}
