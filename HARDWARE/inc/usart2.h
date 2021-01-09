#ifndef   _USART_H
#define   _USART_H
#include "stdint.h"



void usart2_init(uint32_t bound);
void usart2_send(uint8_t *data,uint8_t len);

#endif

