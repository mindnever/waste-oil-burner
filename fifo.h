#ifndef _WOB_FIFO_H_
#define _WOB_FIFO_H_

#include <stdint.h>

typedef struct {
    uint8_t in, out;
    uint8_t size;
    uint8_t used;
    uint8_t overflow;
    uint8_t buffer[0];
} fifo_t;

int fifo_write(fifo_t *fifo, const uint8_t *buf, int len);
int fifo_read(fifo_t *fifo, uint8_t *buf, int len);
int fifo_avail_read(fifo_t *fifo);
int fifo_avail_write(fifo_t *fifo);

#endif /* _WOB_FIFO_H_ */
