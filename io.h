#ifndef _RF_SENSORS_COMMON_IO_H_
#define _RF_SENSORS_COMMON_IO_H_

#include <avr/io.h>

#define _CONCAT(a, b)     a ## b
#define _CONCAT3(a, b, c) a ## b ## c

#define IO_PIN(n)         _CONCAT(n, _PIN)
#define IO_PORTNAME(n)    _CONCAT(n, _PORT)
#define IO_ADCMUX(n)      _CONCAT(n, _ADCMUX)
#define IO_DIDR0(n)       _CONCAT(n, _DIDR0)

#define IO_PORT_OUT(p)    _CONCAT(PORT, p)
#define IO_PORT_IN(p)     _CONCAT(PIN, p)
#define IO_DDR(p)         _CONCAT(DDR, p)

#define IO_PIN_HIGH(n)    { IO_PORT_OUT(IO_PORTNAME(n)) |= _BV(IO_PIN(n)); }
#define IO_PIN_LOW(n)     { IO_PORT_OUT(IO_PORTNAME(n)) &= ~_BV(IO_PIN(n)); }
#define IO_PIN_TOGGLE(n)  { IO_PORT_OUT(IO_PORTNAME(n)) ^= ~_BV(IO_PIN(n)); }

#define IO_PIN_READ(n)    (IO_PORT_IN(IO_PORTNAME(n)) & _BV(IO_PIN(n)))

#define IO_DIR_IN(n)      { IO_DDR(IO_PORTNAME(n)) &= ~_BV(IO_PIN(n)); }
#define IO_DIR_OUT(n)     { IO_DDR(IO_PORTNAME(n)) |= _BV(IO_PIN(n)); }


#endif /* _RF_SENSORS_COMMON_IO_H_ */
