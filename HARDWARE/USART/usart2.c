#include "usart2.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32f10x.h"
#include "bsp_uart.h"
#include "app_fifo.h"
FIFO_Type Usart2_Rx_Fifo;
uint8_t Usart2_Rx_Buffer[USART2_RX_SIZE];
static SemaphoreHandle_t Usart2TxSem;
static SemaphoreHandle_t Usart2TxWaitSem;
void Usart2_Init(void)
{
//GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); //����2ʱ��ʹ��

 	USART_DeInit(USART2);                           //��λ����2
   //USART2_TX   PA2-TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;      //PA2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);          //��ʼ��PA2
   
    //USART2_RX	  PA3-RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);          //��ʼ��PA3
		
    
    
	USART_InitStructure.USART_BaudRate = 115200;                     //������һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;          //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;             //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure); //��ʼ������2

    USART_ITConfig(USART2, USART_IT_IDLE , ENABLE);//�����ж�
    USART_ITConfig(USART2, USART_IT_TC , ENABLE);//��������ж�
    USART_DMACmd(USART2, USART_DMAReq_Tx|USART_DMAReq_Rx, ENABLE);//ʹ�ܴ���DMA�շ�
    USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���2 
    
    
     //Usart1 NVIC ����
 	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
    

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		

	DMA_DeInit(DMA1_Channel6);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Usart2_Rx_Buffer;  //FIFO��bufferָ��ָ��
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //����ΪԴͷ
	DMA_InitStructure.DMA_BufferSize = USART2_RX_SIZE;  //FIF0��buffer��С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //DMAѭ������д��
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);
	/* Enable USART1 DMA TX request */

	DMA_Cmd (DMA1_Channel6,ENABLE); 

	DMA_DeInit(DMA1_Channel7);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);
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
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);

	//����FIFO��ʼ��
	Fifo_Init(&Usart2_Rx_Fifo,Usart2_Rx_Buffer,USART2_RX_SIZE);

	//�����ź�����ʼ��
	Usart2TxSem = xSemaphoreCreateCounting(1,1); 
	Usart2TxWaitSem = xSemaphoreCreateCounting(1,0); 

}
void Usart2_Send(const uint8_t* buf, uint16_t len)
{
	if(len > USART2_TX_SIZE || len == 0)
	{
		return;
	}

	xSemaphoreTake(Usart2TxSem, 500);

	DMA_Cmd(DMA1_Channel7, DISABLE);

	DMA1_Channel7->CNDTR = len;//DMA_InitStructure->DMA_BufferSize; ������Ϊ��

	DMA1_Channel7->CMAR = (uint32_t)buf;//DMA_InitStructure->DMA_MemoryBaseAddr;
	/* Enable USART1 DMA TX request */
	DMA_Cmd (DMA1_Channel7,ENABLE);
	xSemaphoreTake(Usart2TxWaitSem, 500);
	xSemaphoreGive (Usart2TxSem);
}
extern SemaphoreHandle_t xSemaphore_2;
void USART2_IRQHandler(void)
{
	volatile uint32_t temp = 0;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(USART_GetITStatus(USART2,USART_IT_IDLE)!= RESET)//�����ж�
	{
//		DMA_Cmd(DMA1_Channel5, DISABLE);
		Usart2_Rx_Fifo.in = USART2_RX_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);
		if(xSemaphore_2 != NULL)
			xSemaphoreGiveFromISR(xSemaphore_2,&xHigherPriorityTaskWoken);
		temp = USART2->SR; //����������IDLEλ
		temp = USART2->DR; //�ȶ�USART_SR,Ȼ���USART_DR
		USART_ClearITPendingBit(USART2,USART_IT_IDLE);
	}
	else if(USART_GetITStatus(USART2,USART_IT_TC)!= RESET)//��������ж�
	{
		USART_ClearITPendingBit(USART2,USART_IT_TC);
		DMA_Cmd(DMA1_Channel7, DISABLE);
		if(Usart2TxWaitSem != NULL)
			xSemaphoreGiveFromISR (Usart2TxWaitSem, &xHigherPriorityTaskWoken);
 
	}	
}

