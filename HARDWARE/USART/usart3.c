#include "stm32f10x.h"
#include "usart3.h"
#include "app_fifo.h"
#include "FreeRTOS.h"
#include "semphr.h"
uint8_t Usart3_Rx_Buffer[USART3_RX_SIZE];
FIFO_Type Usart3_Rx_Fifo;
static SemaphoreHandle_t Usart3TxSem;
static SemaphoreHandle_t Usart3TxWaitSem;

void usart3_init(u32 bound)
{  
   USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); //����2ʱ��ʹ��

 	USART_DeInit(USART3);                           //��λ����2
   //USART3_TX   PB10-TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;      //PA2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOB, &GPIO_InitStructure);          //��ʼ��PA2
   
    //USART3_RX	  PB11-RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);          //��ʼ��PA3
	
	USART_InitStructure.USART_BaudRate = bound;                     //������һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;          //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;             //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	
	USART_Init(USART3, &USART_InitStructure); //��ʼ������2
    USART_ITConfig(USART3, USART_IT_IDLE , ENABLE);//�����ж�
    USART_ITConfig(USART3, USART_IT_TC , ENABLE);//��������ж�
    USART_DMACmd(USART3, USART_DMAReq_Tx|USART_DMAReq_Rx, ENABLE);//ʹ�ܴ���DMA�շ�
 	USART_Cmd(USART3, ENABLE);                  //ʹ�ܴ��� 
	
	//ʹ�ܽ����ж�
   
	
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=6 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		

	DMA_DeInit(DMA1_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Usart3_Rx_Buffer;  //FIFO��bufferָ��ָ��
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //����ΪԴͷ
	DMA_InitStructure.DMA_BufferSize = USART3_RX_SIZE;  //FIF0��buffer��С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //DMAѭ������д��
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	/* Enable USART1 DMA TX request */

	DMA_Cmd (DMA1_Channel3,ENABLE); 

	DMA_DeInit(DMA1_Channel2);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART3->DR);
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
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);
    Fifo_Init(&Usart3_Rx_Fifo,Usart3_Rx_Buffer,USART3_RX_SIZE);
    Usart3TxSem = xSemaphoreCreateCounting(1,1); 
	Usart3TxWaitSem = xSemaphoreCreateCounting(1,0); 

}

void Usart3_Send(const uint8_t* buf, uint16_t len)
{
	if(len > 1024 || len == 0)
	{
		return;
	}

	xSemaphoreTake(Usart3TxSem, 500);

	DMA_Cmd(DMA1_Channel2, DISABLE);

	DMA1_Channel2->CNDTR = len;//DMA_InitStructure->DMA_BufferSize; ������Ϊ��

	DMA1_Channel2->CMAR = (uint32_t)buf;//DMA_InitStructure->DMA_MemoryBaseAddr;
	/* Enable USART1 DMA TX request */
	DMA_Cmd (DMA1_Channel2,ENABLE);
	xSemaphoreTake(Usart3TxWaitSem, portMAX_DELAY);
	xSemaphoreGive (Usart3TxSem);
}
extern SemaphoreHandle_t xSemaphore_3;
void USART3_IRQHandler(void)
{
	uint32_t temp = 0;
    static int init=0;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(USART_GetITStatus(USART3,USART_IT_IDLE)!= RESET)//�����ж�
	{ 
        
     if(init==0)
        {
          init++;
         temp = USART3->SR; //����������IDLEλ
		 temp = USART3->DR; //�ȶ�USART_SR,Ȼ���USART_DR
		 USART_ClearITPendingBit(USART3,USART_IT_IDLE);
            
        }
		//DMA_Cmd(DMA1_Channel5, DISABLE);DMA_GetCurrDataCounter(DMA1_Channel3)
	else 
    {
        Usart3_Rx_Fifo.in = USART3_RX_SIZE - DMA_GetCurrDataCounter(DMA1_Channel3);
       // ��ȡ��ǰDMAyͨ��x������ʣ�����ݵ�Ԫ������ 
       // printf("COUNT=%d,Usart3_Rx_Fifo.in=%d,init=%d",DMA_GetCurrDataCounter(DMA1_Channel3),Usart3_Rx_Fifo.in,init);
		if(xSemaphore_3 != NULL)
			xSemaphoreGiveFromISR(xSemaphore_3,&xHigherPriorityTaskWoken);
		temp = USART3->SR; //����������IDLEλ
		temp = USART3->DR; //�ȶ�USART_SR,Ȼ���USART_DR
		USART_ClearITPendingBit(USART3,USART_IT_IDLE);
    }
	}
	else if(USART_GetITStatus(USART3,USART_IT_TC)!= RESET)//��������ж�
	{
		USART_ClearITPendingBit(USART3,USART_IT_TC);
		DMA_Cmd(DMA1_Channel2, DISABLE);
		if(Usart3TxWaitSem != NULL)
			xSemaphoreGiveFromISR (Usart3TxWaitSem, &xHigherPriorityTaskWoken);
 
	}	
}

