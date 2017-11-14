#ifndef _AVR_PROJECT_DS1307_H_
#define _AVR_PROJECT_DS1307_H_

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

void twi_init();

#define TWI_SLA_DS1307    0xD0
#define TWI_SLA_PCF8583_0 0xA0
#define TWI_SLA_PCF8583_1 0xA2
#define TWI_SLA_PCF8563   0xA2
#define TWI_SLA_MPU6050_0 0xD0
#define TWI_SLA_MPU6050_1 0xD2

int twi_read_bytes(uint8_t base, uint8_t addr, int len, uint8_t *buf);
int twi_write_bytes(uint8_t base, uint8_t addr, int len, const uint8_t *buf);

#ifdef __cplusplus
 } /* extern "C" */
#endif

#endif /* _AVR_PROJECT_TWI_H_ */
