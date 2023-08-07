#ifndef _UART4_H
#define _UART4_H
#include "stm32f10x.h"
#define UART4_RX_SIZE 100
void Uart4_Init(void);
void Uart4_Send(const uint8_t* buf, uint16_t len);
#endif


