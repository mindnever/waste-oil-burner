#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <util/twi.h>		/* Note [1] */

#if (BOARD == BOARD_PROMICRO)

#define TWI_PRE    1        // my TWI prescaler value 

#define TWI_PRE_1   ( 0 )
#define TWI_PRE_4   ( _BV( TWPS0 ) )
#define TWI_PRE_16   ( _BV( TWPS1 ) )
#define TWI_PRE_64   ( _BV( TWPS1 ) | _BV( TWPS0 ) )

#define TWI_FREQ    100000   // my TWI bit rate

#define TWI_PORT PORTD
#define TWI_SCL_PIN PD0
#define TWI_SDA_PIN PD1

#else

#endif

/*
 * Maximal number of iterations to wait for a device to respond for a
 * selection.  Should be large enough to allow for a pending write to
 * complete, but low enough to properly abort an infinite loop in case
 * a slave is broken or not present at all.  With 100 kHz TWI clock,
 * transfering the start condition and SLA+R/W packet takes about 10
 * µs.  The longest write period is supposed to not exceed ~ 10 ms.
 * Thus, normal operation should not require more than 100 iterations
 * to get the device to respond to a selection.
 */
//#define MAX_ITER	200
#define MAX_ITER 1

#define TWI_TIMEOUT 1000

/*
 * Saved TWI status register, for error messages only.  We need to
 * save it in a variable, since the datasheet only guarantees the TWSR
 * register to have valid contents while the TWINT bit in TWCR is set.
 */
static uint8_t twst;



void twi_init()
{
#if F_CPU < 3600000UL
  TWBR = 10;			/* smallest TWBR value, see note [5] */
#else
  TWBR = (F_CPU / TWI_FREQ - 16) / TWI_PRE / 2;
#endif

#if TWI_PRE == 1
  TWSR = TWI_PRE_1;
#elif TWI_PRE == 4
  TWSR = TWI_PRE_4;
#elif TWI_PRE == 16
  TWSR = TWI_PRE_16;
#elif TWI_PRE == 64
  TWSR = TWI_PRE_64;
#else
# error invalid TWI_PRE value
#endif

// enable internal pullups
  TWI_PORT |= _BV( TWI_SCL_PIN ) | _BV( TWI_SDA_PIN );
}

#define TWI_WAIT { int timeout = 0; while (((TWCR & _BV(TWINT)) == 0) && (++timeout < TWI_TIMEOUT)); if(!(TWCR & _BV(TWINT))) { return -1; } }

int twi_write_bytes(uint8_t base, uint8_t addr, int len, const uint8_t *buf)
{
  uint8_t n = 0;
  int rv = 0;

  restart:
  if (n++ >= MAX_ITER)
    return -1;
  begin:

  /* Note [15] */
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); /* send start condition */

  TWI_WAIT;  /* wait for transmission */

  switch ((twst = TW_STATUS))
    {
    case TW_REP_START:		/* OK, but should not happen */
    case TW_START:
      break;

    case TW_MT_ARB_LOST:
      goto begin;

    default:
      return -1;		/* error: not in start condition */
				/* NB: do /not/ send stop condition */
    }

  /* send SLA+W */
  TWDR = base | TW_WRITE;
  TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */

  TWI_WAIT;  /* wait for transmission */

  switch ((twst = TW_STATUS))
    {
    case TW_MT_SLA_ACK:
      break;

    case TW_MT_SLA_NACK:	/* nack during select: device busy writing */
      goto restart;

    case TW_MT_ARB_LOST:	/* re-arbitrate */
      goto begin;

    default:
      goto error;		/* must send stop condition */
    }

  TWDR = addr;		/* low 8 bits of addr */
  TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */

  TWI_WAIT;  /* wait for transmission */

  switch ((twst = TW_STATUS))
    {
    case TW_MT_DATA_ACK:
      break;

    case TW_MT_DATA_NACK:
      goto quit;

    case TW_MT_ARB_LOST:
      goto begin;

    default:
      goto error;		/* must send stop condition */
    }

  for (; len > 0; len--)
    {
      TWDR = *buf++;
      TWCR = _BV(TWINT) | _BV(TWEN); /* start transmission */

      TWI_WAIT;  /* wait for transmission */

      switch ((twst = TW_STATUS))
	{
	case TW_MT_DATA_NACK:
	  goto error;		/* device write protected -- Note [16] */

	case TW_MT_DATA_ACK:
	  rv++;
	  break;

	default:
	  goto error;
	}
    }
  quit:
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); /* send stop condition */

  return rv;

  error:
  rv = -1;
  goto quit;
}


int twi_read_bytes(uint8_t base, uint8_t addr, int len, uint8_t *buf)
{
  uint8_t twcr, n = 0;
  int rv = 0;

  restart:
  if (n++ >= MAX_ITER)
    return -1;
  begin:

  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); /* send start condition */
  
  TWI_WAIT;

  switch ((twst = TW_STATUS))
    {
    case TW_REP_START:		/* OK, but should not happen */
    case TW_START:
      break;

    case TW_MT_ARB_LOST:	/* Note [9] */
      goto begin;

    default:
      return -1;		/* error: not in start condition */
				/* NB: do /not/ send stop condition */
    }

  /* Note [10] */
  /* send SLA+W */
  TWDR = base | TW_WRITE;
  TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
  
  TWI_WAIT;

  switch ((twst = TW_STATUS))
    {
    case TW_MT_SLA_ACK:
      break;

    case TW_MT_SLA_NACK:	/* nack during select: device busy writing */
				/* Note [11] */
      goto restart;

    case TW_MT_ARB_LOST:	/* re-arbitrate */
      goto begin;

    default:
      goto error;		/* must send stop condition */
    }

  TWDR = addr;		/* low 8 bits of addr */
  TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
  
  TWI_WAIT;

  switch ((twst = TW_STATUS))
    {
    case TW_MT_DATA_ACK:
      break;

    case TW_MT_DATA_NACK:
      goto quit;

    case TW_MT_ARB_LOST:
      goto begin;

    default:
      goto error;		/* must send stop condition */
    }

  /*
   * Note [12]
   * Next cycle(s): master receiver mode
   */
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); /* send (rep.) start condition */
  
  TWI_WAIT;

  switch ((twst = TW_STATUS))
    {
    case TW_START:		/* OK, but should not happen */
    case TW_REP_START:
      break;

    case TW_MT_ARB_LOST:
      goto begin;

    default:
      goto error;
    }

  /* send SLA+R */
  TWDR = base | TW_READ;
  TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
  
  TWI_WAIT;

  switch ((twst = TW_STATUS))
    {
    case TW_MR_SLA_ACK:
      break;

    case TW_MR_SLA_NACK:
      goto quit;

    case TW_MR_ARB_LOST:
      goto begin;

    default:
      goto error;
    }

  for (twcr = _BV(TWINT) | _BV(TWEN) | _BV(TWEA) /* Note [13] */;
       len > 0;
       len--)
    {
      if (len == 1)
	twcr = _BV(TWINT) | _BV(TWEN); /* send NAK this time */
      TWCR = twcr;		/* clear int to start transmission */

      TWI_WAIT;

      switch ((twst = TW_STATUS))
	{
	case TW_MR_DATA_NACK:
	  len = 0;		/* force end of loop */
	  /* FALLTHROUGH */
	case TW_MR_DATA_ACK:
	  *buf++ = TWDR;
	  rv++;
	  break;

	default:
	  goto error;
	}
    }
  quit:
  /* Note [14] */
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); /* send stop condition */

  return rv;

  error:
  rv = -1;
  goto quit;

}
