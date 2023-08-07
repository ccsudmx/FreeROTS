#include "FreeRTOS.h"
#include "semphr.h"
#include "delay.h"
#include "sys.h"
#include "bsp_uart.h"
#include "app_fifo.h"


uint8_t Uart1_Rx_Buffer[BSP_UART1_RX_SIZE];
FIFO_Type Uart1_Rx_Fifo;
static SemaphoreHandle_t Uart1TxSem;
static SemaphoreHandle_t Uart1TxWaitSem;




void Uart_1_Init(void)
{
//GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

  //Usart1 NVIC ����
 	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = 115200;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART1, &USART_InitStructure); //��ʼ������1
    USART_ITConfig(USART1, USART_IT_IDLE , ENABLE);//�����ж�
    USART_ITConfig(USART1, USART_IT_TC , ENABLE);//��������ж�
    USART_DMACmd(USART1, USART_DMAReq_Tx|USART_DMAReq_Rx, ENABLE);//ʹ�ܴ���DMA�շ�
    USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		

	DMA_DeInit(DMA1_Channel5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Uart1_Rx_Buffer;  //FIFO��bufferָ��ָ��
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //����ΪԴͷ
	DMA_InitStructure.DMA_BufferSize = BSP_UART1_RX_SIZE;  //FIF0��buffer��С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //DMAѭ������д��
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	/* Enable USART1 DMA TX request */

	DMA_Cmd (DMA1_Channel5,ENABLE); 

	DMA_DeInit(DMA1_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;  //�������޸ĵ�ַ����ʹ�ܸ�ͨ��
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //����ΪĿ��
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);

	//����FIFO��ʼ��
	Fifo_Init(&Uart1_Rx_Fifo,Uart1_Rx_Buffer,BSP_UART1_RX_SIZE);

	//�����ź�����ʼ��
	Uart1TxSem = xSemaphoreCreateCounting(1,1); 
	Uart1TxWaitSem = xSemaphoreCreateCounting(1,0); 

}


void Uart_1_Send(const uint8_t* buf, uint16_t len)
{
	if(len > BSP_UART1_TX_SIZE || len == 0)
	{
		return;
	}

	xSemaphoreTake(Uart1TxSem, 500);

	DMA_Cmd(DMA1_Channel4, DISABLE);

	DMA1_Channel4->CNDTR = len;//DMA_InitStructure->DMA_BufferSize; ������Ϊ��

	DMA1_Channel4->CMAR = (uint32_t)buf;//DMA_InitStructure->DMA_MemoryBaseAddr;
	/* Enable USART1 DMA TX request */
	DMA_Cmd (DMA1_Channel4,ENABLE);
	xSemaphoreTake(Uart1TxWaitSem, 500);
	xSemaphoreGive (Uart1TxSem);
}



uint16_t Uart_1_Select(uint16_t delay)
{
	uint16_t count = 0;
	uint16_t len = 0;
//	CPU_SR_ALLOC();
	
	while(count < delay)
	{
		delay_ms(10);
		count+=10;
		portDISABLE_INTERRUPTS();
		Uart1_Rx_Fifo.in = BSP_UART1_RX_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5);
		portENABLE_INTERRUPTS();
		len = Fifo_Status(&Uart1_Rx_Fifo);
		if(len != 0)
			return len;
	}
	return len;
}

uint16_t Uart_1_Get(uint8_t *buffer, uint16_t len)
{
	return Fifo_Get(&Uart1_Rx_Fifo,buffer, len);
}

void Uart_1_Clean(void)
{
	DMA_Cmd(DMA1_Channel5, DISABLE);
	Uart1_Rx_Fifo.in = 0;
	Uart1_Rx_Fifo.out = 0;
	DMA_SetCurrDataCounter(DMA1_Channel5, 0);
	DMA_Cmd(DMA1_Channel5, ENABLE);
}

void Uart_Init(void)
{
	Uart_1_Init( );

}


void Uart_Send(USART_TypeDef* uart, const uint8_t* buffer, uint16_t len)
{
	if(uart == USART1)
	{
		Uart_1_Send( buffer, len);
	}

}

uint16_t Uart_Get(USART_TypeDef* uart, uint8_t *buffer, uint16_t len)
{
	uint16_t lenght = 0;
	if(uart == USART1)
	{
		lenght = Uart_1_Get( buffer, len);
	}
	
	return lenght;
}

uint16_t Uart_Select(USART_TypeDef* uart, uint16_t delay)
{
	uint16_t lenght = 0;
	if(uart == USART1)
	{
		lenght = Uart_1_Select( delay);
	}
	
	return lenght;	
}

void Uart_Clean(USART_TypeDef* uart)
{
	if(uart == USART1)
	{
		Uart_1_Clean();
	}

}

extern SemaphoreHandle_t xSemaphore;
//�ж�
void USART1_IRQHandler(void)
{
	uint32_t temp = 0;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(USART_GetITStatus(USART1,USART_IT_IDLE)!= RESET)//�����ж�
	{
//		DMA_Cmd(DMA1_Channel5, DISABLE);
		Uart1_Rx_Fifo.in = BSP_UART1_RX_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5);
		if(xSemaphore != NULL)
			xSemaphoreGiveFromISR(xSemaphore,&xHigherPriorityTaskWoken);
		temp = USART1->SR; //����������IDLEλ
		temp = USART1->DR; //�ȶ�USART_SR,Ȼ���USART_DR
		USART_ClearITPendingBit(USART1,USART_IT_IDLE);
	}
	else if(USART_GetITStatus(USART1,USART_IT_TC)!= RESET)//��������ж�
	{
		USART_ClearITPendingBit(USART1,USART_IT_TC);
		DMA_Cmd(DMA1_Channel4, DISABLE);
		if(Uart1TxWaitSem != NULL)
			xSemaphoreGiveFromISR (Uart1TxWaitSem, &xHigherPriorityTaskWoken);
 
	}	
}



