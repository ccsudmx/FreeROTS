#include "stm32f10x.h"
#include "lora.h"
#include "stdio.h"
#include "delay.h"
#include "usart2.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "app_fifo.h"
#include "usart2.h"
#include "string.h"
extern SemaphoreHandle_t xSemaphore_2;
extern FIFO_Type Usart2_Rx_Fifo;
_LoRa_CFG LoRa_CFG=
{
	.addr = LORA_ADDR,       //�豸��ַ
	.power = LORA_POWER,     //���书��
	.chn = LORA_CHN,         //�ŵ�
	.wlrate = LORA_RATE,     //��������
	.wltime = LORA_WLTIME,   //˯��ʱ��
	.mode = LORA_MODE,       //����ģʽ
	.mode_sta = LORA_STA,    //����״̬
	.bps = LORA_TTLBPS ,     //����������
	.parity = LORA_TTLPAR    //У��λ����
};
u8 LoRa_Init(void)
{
    printf("300");
	 uint8_t Receive_Buff[USART2_RX_SIZE];
	 GPIO_InitTypeDef  GPIO_InitStructure;
		
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��PA�˿�ʱ��
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//ʹ�ܸ��ù���ʱ��

     GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//��ֹJTAG,�Ӷ�PA15��������ͨIOʹ��,����PA15��������ͨIO!!!	
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	    		 //LORA_MD0
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	 GPIO_Init(GPIOA, &GPIO_InitStructure);	  				 //������� ��IO���ٶ�Ϊ50MHz
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				 //LORA_AUX
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		     //��������
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.4
	
	 	 
	 LORA_MD0=0;
	 LORA_AUX=0;
	
	 while(LORA_AUX)//ȷ��LORAģ���ڿ���״̬��(LORA_AUX=0)
	 {
		 printf("lora error!!\n"); 	
		 delay_ms(500);
	 }
	 
	 LORA_MD0=1;//����ATģʽ
	 delay_ms(40);   
	 Usart2_Send((const uint8_t *)"ATE0\r\n",6);
     xSemaphoreTake(xSemaphore_2, portMAX_DELAY);
     delay_ms(30);
     Fifo_Get(&Usart2_Rx_Fifo,Receive_Buff,USART2_RX_SIZE);
     if(strstr((char *)Receive_Buff,"OK")!=NULL)
     {
     
         printf("Lora Init Succseeful\r\n");
        // return 1;
     
     }
    	
	LORA_MD0=1;
	delay_ms(40);

	
	LoRa_CFG.addr = 0x5410;
	LoRa_CFG.chn = 0x10;
	LoRa_CFG.power = LORA_PW_20Bbm;
	LoRa_CFG.wlrate = LORA_RATE_19K2;
	LoRa_CFG.wltime = LORA_WLTIME_1S;
	LoRa_CFG.mode = LORA_MODE_GEN;
	LoRa_CFG.mode_sta = LORA_STA_Tran;//LORA_STA_Dire
	LoRa_CFG.bps = LORA_TTLBPS_115200;
	LoRa_CFG.parity = LORA_TTLPAR_8N1;	
	LoRa_Set();
     
    return 0;
     
           

}
void LoRa_Set(void)
{
	u8 Send_Buff[20];
    u8 Receive_Buff[50];
	u8 lora_addrh,lora_addrl=0;
	while(LORA_AUX);
	LORA_MD0=1; 
	delay_ms(40);
	
	lora_addrh =  (LoRa_CFG.addr>>8)&0xff;
	lora_addrl = LoRa_CFG.addr&0xff;
	//配置地址
	sprintf((char*)Send_Buff,"AT+ADDR=%02x,%02x\r\n",lora_addrh,lora_addrl);
	Usart2_Send(Send_Buff,strlen((char *)Send_Buff));
	xSemaphoreTake(xSemaphore_2, portMAX_DELAY);
    delay_ms(100);
	Fifo_Get(&Usart2_Rx_Fifo,Receive_Buff,50);
   if(strstr((char *)Receive_Buff,"OK") ==NULL)
   {  
       printf("%s:1\r\n",Receive_Buff);
       return ;
       
   }
    memset(Send_Buff,0,20);
    memset(Receive_Buff,0,50);

	sprintf((char*)Send_Buff,"AT+WLRATE=%d,%d\r\n",LoRa_CFG.chn,LoRa_CFG.wlrate);//�����ŵ��Ϳ�������
	Usart2_Send(Send_Buff,strlen((char *)Send_Buff));
	xSemaphoreTake(xSemaphore_2, portMAX_DELAY);
      delay_ms(50);
    Fifo_Get(&Usart2_Rx_Fifo,Receive_Buff,50);
   if(strstr((char *)Receive_Buff,"OK") ==NULL)
   {  
       printf("%s:2\r\n",Receive_Buff);
       return ;
       
   }
    memset(Send_Buff,0,20);
    memset(Receive_Buff,0,50);

	sprintf((char*)Send_Buff,"AT+TPOWER=%d\r\n",LoRa_CFG.power);//���÷��书��
	Usart2_Send(Send_Buff,strlen((char *)Send_Buff));
	xSemaphoreTake(xSemaphore_2, portMAX_DELAY);
      delay_ms(50);
   Fifo_Get(&Usart2_Rx_Fifo,Receive_Buff,50);
   if(strstr((char *)Receive_Buff,"OK") ==NULL)
   {  
       printf("%s:3\r\n",Receive_Buff);
       return ;
       
   }
    memset(Send_Buff,0,20);
    memset(Receive_Buff,0,50);

	sprintf((char*)Send_Buff,"AT+CWMODE=%d\r\n",LoRa_CFG.mode);//���ù���ģʽ
	Usart2_Send(Send_Buff,strlen((char *)Send_Buff));
	xSemaphoreTake(xSemaphore_2, portMAX_DELAY);
      delay_ms(50);
   Fifo_Get(&Usart2_Rx_Fifo,Receive_Buff,50);
   if(strstr((char *)Receive_Buff,"OK") ==NULL)
   {  
       printf("%s:4\r\n",Receive_Buff);
       return ;
       
   }
    memset(Send_Buff,0,20);
    memset(Receive_Buff,0,50);

	sprintf((char*)Send_Buff,"AT+TMODE=%d\r\n",LoRa_CFG.mode_sta);//���÷���״̬
	Usart2_Send(Send_Buff,strlen((char *)Send_Buff));
	xSemaphoreTake(xSemaphore_2, portMAX_DELAY);
      delay_ms(50);
   Fifo_Get(&Usart2_Rx_Fifo,Receive_Buff,50);
   if(strstr((char *)Receive_Buff,"OK") ==NULL)
   {  
       printf("%s:5\r\n",Receive_Buff);
       return ;
       
   }
    memset(Send_Buff,0,20);
    memset(Receive_Buff,0,50);

	sprintf((char*)Send_Buff,"AT+WLTIME=%d\r\n",LoRa_CFG.wltime);//����˯��ʱ��
	Usart2_Send(Send_Buff,strlen((char *)Send_Buff));
	xSemaphoreTake(xSemaphore_2, portMAX_DELAY);
      delay_ms(50);
	Fifo_Get(&Usart2_Rx_Fifo,Receive_Buff,50);
   if(strstr((char *)Receive_Buff,"OK") ==NULL)
   {  
       printf("%s:6\r\n",Receive_Buff);
       return ;
       
   }
    memset(Send_Buff,0,20);
    memset(Receive_Buff,0,50);

	sprintf((char*)Send_Buff,"AT+UART=%d,%d\r\n",LoRa_CFG.bps,LoRa_CFG.parity);//���ô��ڲ����ʡ�����У��λ
	Usart2_Send(Send_Buff,strlen((char *)Send_Buff));
	xSemaphoreTake(xSemaphore_2, portMAX_DELAY);
      delay_ms(50);
   Fifo_Get(&Usart2_Rx_Fifo,Receive_Buff,50);
   if(strstr((char *)Receive_Buff,"OK") ==NULL)
   {  
       printf("%s:7\r\n",Receive_Buff);
       return ;
       
   }
    memset(Send_Buff,0,20);
    memset(Receive_Buff,0,50);

	LORA_MD0=0;
	delay_ms(40);
	while(LORA_AUX);
	printf("Lora set succseeful!\r\n");
	
}

