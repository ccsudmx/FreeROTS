#ifndef _BLUE_H
#define _BLUE_H
#include "stm32f10x.h"
void BLT_USART_Config(void);
void printf_blue(char *buff,int len);
#define UART5_BIFF_SIZE 1024
typedef struct 
{
volatile    uint16_t datanum;
uint8_t     uart_buff[UART5_BIFF_SIZE];		
uint8_t     receive_data_flag;
}ReceiveData;
#endif



