#ifndef _WOB_TX_H_
#define _WOB_TX_H_

void RfTx_Init(void);
void RfTx_Transmit(const uint8_t *buffer, uint8_t len, uint8_t repeat);

#endif /* _WOB_TX_H_ */
