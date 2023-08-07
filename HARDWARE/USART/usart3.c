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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); //串口2时钟使能

 	USART_DeInit(USART3);                           //复位串口2
   //USART3_TX   PB10-TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;      //PA2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);          //初始化PA2
   
    //USART3_RX	  PB11-RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //上拉输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);          //初始化PA3
	
	USART_InitStructure.USART_BaudRate = bound;                     //波特率一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;          //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;             //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	
	USART_Init(USART3, &USART_InitStructure); //初始化串口2
    USART_ITConfig(USART3, USART_IT_IDLE , ENABLE);//空闲中断
    USART_ITConfig(USART3, USART_IT_TC , ENABLE);//传输完成中断
    USART_DMACmd(USART3, USART_DMAReq_Tx|USART_DMAReq_Rx, ENABLE);//使能串口DMA收发
 	USART_Cmd(USART3, ENABLE);                  //使能串口 
	
	//使能接收中断
   
	
	//设置中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=6 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		

	DMA_DeInit(DMA1_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Usart3_Rx_Buffer;  //FIFO的buffer指针指向
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //外设为源头
	DMA_InitStructure.DMA_BufferSize = USART3_RX_SIZE;  //FIF0的buffer大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //DMA循环覆盖写入
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	/* Enable USART1 DMA TX request */

	DMA_Cmd (DMA1_Channel3,ENABLE); 

	DMA_DeInit(DMA1_Channel2);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART3->DR);
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

	DMA1_Channel2->CNDTR = len;//DMA_InitStructure->DMA_BufferSize; 不能置为零

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
	if(USART_GetITStatus(USART3,USART_IT_IDLE)!= RESET)//空闲中断
	{ 
        
     if(init==0)
        {
          init++;
         temp = USART3->SR; //软件序列清除IDLE位
		 temp = USART3->DR; //先读USART_SR,然后读USART_DR
		 USART_ClearITPendingBit(USART3,USART_IT_IDLE);
            
        }
		//DMA_Cmd(DMA1_Channel5, DISABLE);DMA_GetCurrDataCounter(DMA1_Channel3)
	else 
    {
        Usart3_Rx_Fifo.in = USART3_RX_SIZE - DMA_GetCurrDataCounter(DMA1_Channel3);
       // 获取当前DMAy通道x传输中剩余数据单元的数量 
       // printf("COUNT=%d,Usart3_Rx_Fifo.in=%d,init=%d",DMA_GetCurrDataCounter(DMA1_Channel3),Usart3_Rx_Fifo.in,init);
		if(xSemaphore_3 != NULL)
			xSemaphoreGiveFromISR(xSemaphore_3,&xHigherPriorityTaskWoken);
		temp = USART3->SR; //软件序列清除IDLE位
		temp = USART3->DR; //先读USART_SR,然后读USART_DR
		USART_ClearITPendingBit(USART3,USART_IT_IDLE);
    }
	}
	else if(USART_GetITStatus(USART3,USART_IT_TC)!= RESET)//发送完成中断
	{
		USART_ClearITPendingBit(USART3,USART_IT_TC);
		DMA_Cmd(DMA1_Channel2, DISABLE);
		if(Usart3TxWaitSem != NULL)
			xSemaphoreGiveFromISR (Usart3TxWaitSem, &xHigherPriorityTaskWoken);
 
	}	
}

