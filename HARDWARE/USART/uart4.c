#include "uart4.h"
#include "stm32f10x.h"
#include "app_fifo.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
 FIFO_Type Uart4_Rx_Fifo;
uint8_t Uart4_Rx_Buffer1[UART4_RX_SIZE];
static SemaphoreHandle_t Uart4TxSem;
static SemaphoreHandle_t Uart4TxWaitSem;
void Uart4_Init(void)
{
//GPIO�?口�?�置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	//使能UART4，GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
   //UART4_TX   PC.10 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOC, &GPIO_InitStructure); //初�?�化PC10
   
	//UART4_RX	  PC.11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//�?空输�?
	GPIO_Init(GPIOC, &GPIO_InitStructure);  //初�?�化PC11

  //UART4 NVIC 配置
 	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5 ;//抢占优先�?3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存�?
  
   //USART 初�?�化设置

	USART_InitStructure.USART_BaudRate = 115200;//串口波特�?
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长�?8位数�?格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一�?停�??�?
	USART_InitStructure.USART_Parity = USART_Parity_No;//无�?�偶校验�?
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数�?流控�?
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

    USART_Init(UART4, &USART_InitStructure); //初�?�化串口1
    USART_ITConfig(UART4, USART_IT_IDLE , ENABLE);//空闲�?�?
    USART_ITConfig(UART4, USART_IT_TC , ENABLE);//传输完成�?�?
    USART_DMACmd(UART4, USART_DMAReq_Tx|USART_DMAReq_Rx, ENABLE);//使能串口DMA收发
    USART_Cmd(UART4, ENABLE);                    //使能串口1 

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);		

	DMA_DeInit(DMA2_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&UART4->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Uart4_Rx_Buffer1;  //FIFO的buffer指针指向
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //外�?�为源头
	DMA_InitStructure.DMA_BufferSize = UART4_RX_SIZE;  //FIF0的buffer大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //DMA�?�?覆盖写入
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA2_Channel3, &DMA_InitStructure);
    DMA_ClearFlag(DMA2_FLAG_GL3);
	DMA_Cmd (DMA2_Channel3,ENABLE); 

	DMA_DeInit(DMA2_Channel5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&UART4->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;  //传输再修改地址，并使能该通道
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //外�?�为�?�?
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;    
	DMA_ClearFlag(DMA2_FLAG_GL5);
	DMA_Init(DMA2_Channel5, &DMA_InitStructure);

	//接收FIFO初�?�化
	Fifo_Init(&Uart4_Rx_Fifo,Uart4_Rx_Buffer1,UART4_RX_SIZE);

	//发送信号量初�?�化
	Uart4TxSem = xSemaphoreCreateCounting(1,1); 
	Uart4TxWaitSem = xSemaphoreCreateCounting(1,0); 

}
void Uart4_Send(const uint8_t* buf, uint16_t len)
{
	if(len > UART4_RX_SIZE || len == 0)
	{
		return;
	}
  //  printf("send=%s,%d",buf,DMA_GetCurrDataCounter(DMA2_Channel3));

	xSemaphoreTake(Uart4TxSem, 500);

	DMA_Cmd(DMA2_Channel5, DISABLE);

	DMA2_Channel5->CNDTR = len;//DMA_InitStructure->DMA_BufferSize; 不能�?为零

	DMA2_Channel5->CMAR = (uint32_t)buf;//DMA_InitStructure->DMA_MemoryBaseAddr;
	/* Enable UART4 DMA TX request */
	DMA_Cmd (DMA2_Channel5,ENABLE);
	xSemaphoreTake(Uart4TxWaitSem, 500);
	xSemaphoreGive (Uart4TxSem);
}
extern  SemaphoreHandle_t  xSemaphore_4;
void UART4_IRQHandler(void)
{
	volatile uint32_t temp = 0;
    uint32_t ulReturn;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
   ulReturn=taskENTER_CRITICAL_FROM_ISR();
	if(USART_GetITStatus(UART4,USART_IT_IDLE)!= RESET)//空闲�?�?
	{   
       // printf("sadsd\t");
//		DMA_Cmd(DMA1_Channel5, DISABLE);
		Uart4_Rx_Fifo.in = UART4_RX_SIZE - DMA_GetCurrDataCounter(DMA2_Channel3);
       // printf("Count is %d",DMA_GetCurrDataCounter(DMA2_Channel3));
		if(xSemaphore_4 != NULL)
			xSemaphoreGiveFromISR(xSemaphore_4,&xHigherPriorityTaskWoken);
      // portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		temp = UART4->SR; //�?件序列清�?IDLE�?
		temp = UART4->DR; //先�?�USART_SR,然后读USART_DR
		USART_ClearITPendingBit(UART4,USART_IT_IDLE);
	}
	else if(USART_GetITStatus(UART4,USART_IT_TC)!= RESET)//发送完成中�?
	{
		USART_ClearITPendingBit(UART4,USART_IT_TC);
       
		DMA_Cmd(DMA2_Channel5, DISABLE);
		if(Uart4TxWaitSem != NULL)
			xSemaphoreGiveFromISR (Uart4TxWaitSem, &xHigherPriorityTaskWoken);
 
	}
      taskEXIT_CRITICAL_FROM_ISR(ulReturn);	
}
