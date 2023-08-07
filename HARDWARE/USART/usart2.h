#ifndef _USART2_H
#define _USART2_H
#include "stm32f10x.h"
#define USART2_RX_SIZE 1024
#define USART2_TX_SIZE 1024
void Usart2_Send(const uint8_t* buf, uint16_t len);
void Usart2_Init(void);
#endif

