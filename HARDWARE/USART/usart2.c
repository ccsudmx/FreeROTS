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
//GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); //串口2时钟使能

 	USART_DeInit(USART2);                           //复位串口2
   //USART2_TX   PA2-TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;      //PA2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);          //初始化PA2
   
    //USART2_RX	  PA3-RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);          //初始化PA3
		
    
    
	USART_InitStructure.USART_BaudRate = 115200;                     //波特率一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;          //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;             //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART2, &USART_InitStructure); //初始化串口2

    USART_ITConfig(USART2, USART_IT_IDLE , ENABLE);//空闲中断
    USART_ITConfig(USART2, USART_IT_TC , ENABLE);//传输完成中断
    USART_DMACmd(USART2, USART_DMAReq_Tx|USART_DMAReq_Rx, ENABLE);//使能串口DMA收发
    USART_Cmd(USART2, ENABLE);                    //使能串口2 
    
    
     //Usart1 NVIC 配置
 	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
    

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		

	DMA_DeInit(DMA1_Channel6);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Usart2_Rx_Buffer;  //FIFO的buffer指针指向
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //外设为源头
	DMA_InitStructure.DMA_BufferSize = USART2_RX_SIZE;  //FIF0的buffer大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //DMA循环覆盖写入
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);
	/* Enable USART1 DMA TX request */

	DMA_Cmd (DMA1_Channel6,ENABLE); 

	DMA_DeInit(DMA1_Channel7);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;  //传输再修改地址，并使能该通道
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //外设为目标
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);

	//接收FIFO初始化
	Fifo_Init(&Usart2_Rx_Fifo,Usart2_Rx_Buffer,USART2_RX_SIZE);

	//发送信号量初始化
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

	DMA1_Channel7->CNDTR = len;//DMA_InitStructure->DMA_BufferSize; 不能置为零

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
	if(USART_GetITStatus(USART2,USART_IT_IDLE)!= RESET)//空闲中断
	{
//		DMA_Cmd(DMA1_Channel5, DISABLE);
		Usart2_Rx_Fifo.in = USART2_RX_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);
		if(xSemaphore_2 != NULL)
			xSemaphoreGiveFromISR(xSemaphore_2,&xHigherPriorityTaskWoken);
		temp = USART2->SR; //软件序列清除IDLE位
		temp = USART2->DR; //先读USART_SR,然后读USART_DR
		USART_ClearITPendingBit(USART2,USART_IT_IDLE);
	}
	else if(USART_GetITStatus(USART2,USART_IT_TC)!= RESET)//发送完成中断
	{
		USART_ClearITPendingBit(USART2,USART_IT_TC);
		DMA_Cmd(DMA1_Channel7, DISABLE);
		if(Usart2TxWaitSem != NULL)
			xSemaphoreGiveFromISR (Usart2TxWaitSem, &xHigherPriorityTaskWoken);
 
	}	
}

