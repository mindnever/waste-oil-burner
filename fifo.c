#include "fifo.h"

int fifo_write(fifo_t *fifo, const uint8_t *buf, int len)
{
  int i;

  for(i = 0; i < len; ++i) {
    if(fifo->used == fifo->size) { // full!
      fifo->overflow++;
      continue;
    }

    fifo->buffer[fifo->in++] = buf[i];

    if(fifo->in == fifo->size) {
      fifo->in = 0;
    }

    ++fifo->used;
  }
  return i;
}

int fifo_read(fifo_t *fifo, uint8_t *buf, int len)
{
  int i;
  
  for(i = 0; (i < len) && (fifo->used > 0); ++i) {
    buf[i] = fifo->buffer[fifo->out++];
    if(fifo->out == fifo->size) {
      fifo->out = 0;
    }
    --fifo->used;
  }
  
  return i;
}

int fifo_avail_read(fifo_t *fifo)
{
  return fifo->used;
}

int fifo_avail_write(fifo_t *fifo)
{
  return fifo->size - fifo->used;
}

