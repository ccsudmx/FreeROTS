#ifndef  _USART3_H
#define  _USART3_H
#include "stm32f10x.h"
#define USART3_RX_SIZE 1024
void usart3_init(u32 bound);
void Usart3_Send(const uint8_t* buf, uint16_t len);
#endif
