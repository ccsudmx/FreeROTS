#include "blue.h"
#include "stm32f10x.h"
#include "stdio.h"
static void HC05_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;  
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


}
void BLT_USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD, ENABLE);
 
	//TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
//为什么推挽复用功能 因为通过片内外设USART控制高or低电平不是ODR寄存器
//可以查数据手册每个外设应该引脚模式配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	    
    //RX
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOD, &GPIO_InitStructure);
	  //我的模块不配置38400会出现乱码
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
	USART_Init(UART5, &USART_InitStructure); 
	
	
	HC05_NVIC_Configuration();
	
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	
	USART_ITConfig (UART5, USART_IT_IDLE, ENABLE ); 
 
	USART_Cmd(UART5, ENABLE);
	USART_ClearFlag(UART5, USART_FLAG_TC);
}


 
ReceiveData UART5_ReceiveData;
 
void UART5_IRQHandler(void)
{
    uint8_t ucCh; 
    if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
    {
      ucCh = USART_ReceiveData(UART5);
      if(UART5_ReceiveData.datanum < UART5_BIFF_SIZE)
      {
        if((ucCh != 0x0a) && (ucCh != 0x0d))
        {
          UART5_ReceiveData.uart_buff[UART5_ReceiveData.datanum] = ucCh;                 
          UART5_ReceiveData.datanum++;
          
        }
      }
    }
	if(USART_GetITStatus(UART5, USART_IT_IDLE ) == SET )                                         
    {
        UART5_ReceiveData.receive_data_flag = 1;
       
        USART_ReceiveData(UART5);                                                              
    }	
 
}
void printf_blue(char *buff,int len)
 {
  

  
    for(int i=0;i<len;i++)
    {
    
    
    while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
	  USART_SendData(UART5,buff[i]); 
    
    }




   }
