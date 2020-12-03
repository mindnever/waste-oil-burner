#ifndef _WOB_USART_H_
#define _WOB_USART_H_

#include <stdio.h>

FILE *USART_Init(void);

void USART_CLI(int argc, const char *const *argv);

#endif /* _WOB_USART_H_ */
