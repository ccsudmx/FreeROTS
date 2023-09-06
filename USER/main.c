#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "bsp_uart.h"
#include "stdio.h"
#include "string.h"
#include "usart2.h"
#include "usart3.h"
#include "uart4.h"
#include "rfid.h"
#include "wifi.h"
#include "app_fifo.h"
#include "lora.h"
#include "my_json.h"
#include "timers.h"
#include "blue.h"

//wifi句柄以及任务
TaskHandle_t Wifi_Handler;
void Wifi_task(void *pvParameters);
//lora句柄以及任务
TaskHandle_t Lora_Handler;
void Lora_task(void *pvParameters);
//Rfid发送读卡命令
TaskHandle_t sendRfidCmd_Handler;
void sendRfidCmd_task(void *pvParameters);
//读取Rfid卡
TaskHandle_t Rfid_Handler;
void Rfid_task(void *pvParameters);
// 按键扫描
TaskHandle_t Key_Handler;
void Key_task(void *pvParameters);
// 蓝牙模块
TaskHandle_t HC05_Handler;
void HC05_task(void *pvParameters);
//互斥量
SemaphoreHandle_t xMutex;
//信号量句柄
SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t xSemaphore_2;
SemaphoreHandle_t xSemaphore_3;
SemaphoreHandle_t  xSemaphore_4;
SemaphoreHandle_t  xSemaphore_5;
 char Send_id[40];
// Json Fan=
//{
//	.name="FAN1",
//	.status=0,
//	.value=0,
//	.len=4
//};
int main(void)
{  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	delay_init();	    				
    Usart1_Init();
    xMutex = xSemaphoreCreateMutex( );
    xTaskCreate((TaskFunction_t )Wifi_task,             
               (const char*    )"Wifi_task",           
               (uint16_t       )2056,        
               (void*          )NULL,                  
               (UBaseType_t    )4,        
               (TaskHandle_t*  )&Wifi_Handler); 
               
//    xTaskCreate((TaskFunction_t )Key_task,             
//               (const char*    )"Key_task",           
//               (uint16_t       )1024,        
//               (void*          )NULL,                  
//               (UBaseType_t    )3,        
//               (TaskHandle_t*  )&Key_Handler); 
               
   xTaskCreate((TaskFunction_t )HC05_task,             
               (const char*    )"HC05_task",           
               (uint16_t       )1024,        
               (void*          )NULL,                  
               (UBaseType_t    )3,        
               (TaskHandle_t*  )&HC05_Handler); 
  
    vTaskStartScheduler();   
    while(1);               
}



extern FIFO_Type Uart4_Rx_Fifo;
void Wifi_task(void *pvParameters)
{
 
    
    uint8_t Uart4_Read_Buff[UART4_RX_SIZE];
    xSemaphore_4 = xSemaphoreCreateCounting( 1, 0 );
    xSemaphore_5 = xSemaphoreCreateCounting( 1, 0 );
    Uart4_Init(); 
    Connect_MQTT();
   
    xTaskCreate((TaskFunction_t )Lora_task,             
               (const char*    )"Lora_task",           
               (uint16_t       )1024,        
               (void*          )NULL,                  
               (UBaseType_t    )3,        
               (TaskHandle_t*  )&Lora_Handler);
        
    
     
    while(1)
    {  
       
       xSemaphoreTake(xSemaphore_4, portMAX_DELAY);
        
       memset(Uart4_Read_Buff,0,UART4_RX_SIZE);
       xSemaphoreTake(xMutex, portMAX_DELAY);        
       Fifo_Get(&Uart4_Rx_Fifo,Uart4_Read_Buff,UART4_RX_SIZE);
       
        if(strstr((const char *)Uart4_Read_Buff, (const char *)"+MQTTSUBRECV:0") != NULL){
            printf("RX=%s\r\n",(char *)Uart4_Read_Buff);
      
            Usart2_Send(Uart4_Read_Buff,UART4_RX_SIZE);
           // MQTT_JSON((char *)Uart4_Read_Buff,&Fan);
            
          
        }
       xSemaphoreGive(xMutex);
     
   
    }


}

extern FIFO_Type Usart2_Rx_Fifo;
void Lora_task(void *pvParameters)
{
  uint8_t Usart2_RX[100];
  uint8_t Send_msg[150];
   // char *buff;
   xSemaphore_2=xSemaphoreCreateCounting( 1, 0 );
   Usart2_Init();
   LoRa_Init();
               
  xTaskCreate((TaskFunction_t )Rfid_task,             
               (const char*    )"Rfid_task",           
               (uint16_t       )1024,        
               (void*          )NULL,                  
               (UBaseType_t    )4,        
               (TaskHandle_t*  )&Rfid_Handler);
               
  xTaskCreate((TaskFunction_t )sendRfidCmd_task,             
               (const char*    )"sendRfidCmd_task",           
               (uint16_t       )256,        
               (void*          )NULL,                  
               (UBaseType_t    )3,        
               (TaskHandle_t*  )&sendRfidCmd_Handler);
    
  
    while(1)
    { 
         memset(Usart2_RX,0,100);
         xSemaphoreTake(xSemaphore_2, portMAX_DELAY);
          delay_ms(100);
         Fifo_Get(&Usart2_Rx_Fifo,Usart2_RX,USART2_RX_SIZE);
           //    Usart2_RX[strlen((char *)Usart2_RX)-1]=0;


        sprintf((char *)Send_msg,"%s,%s}",(char *)Usart2_RX,Send_id);
        //printf("%s",Send_id);
        // xSemaphoreTake(xMutex, portMAX_DELAY);
         ESP8266_MQTT_Pub((char *)Send_msg,strlen((char *)Send_msg),1);
         // xSemaphoreGive(xMutex);
         
        
    }

}


void Rfid_task(void *pvParameters)
{
   
    usart3_init(9600);
    char Id_num[20];

    xSemaphore_3 = xSemaphoreCreateCounting( 1, 0 );
    while(1)
    {
        
      xSemaphoreTake(xSemaphore_3, portMAX_DELAY);
      ReadId(Id_num);
      sprintf(Send_id,"ID=\"1\",card_number=\"%s\"",Id_num);
        printf("%s",Send_id);
      Usart2_Send((unsigned char *)Id_num,UART4_RX_SIZE);
    
   }


}
void sendRfidCmd_task(void *pvParameters)
{
    


  while(1)
  {
  
      RFID_CMD();
   
   
  }


}
void Key_task(void *pvParameters)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // GPIOA时钟
    GPIO_InitTypeDef GPIO_InitStructure;
                        
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;      //PD2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//复用推挽输出
    GPIO_Init(GPIOD, &GPIO_InitStructure);          
    
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPU;
   //s1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
	GPIO_Init(GPIOD, &GPIO_InitStructure); 	
   //s2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; 
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
    
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
    //拉低	
    GPIO_ResetBits(GPIOD,GPIO_Pin_2);
    
    //拉高
     GPIO_SetBits(GPIOD,GPIO_Pin_5);
   //  printf("enter key!");
    while(1)
    {
    
      if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3)==0)
      {
         
         vTaskDelay(20);
          if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3)==0)
          {   
              while(!GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3));
              
              //printf("sadsadsd");
             GPIO_ResetBits(GPIOD,GPIO_Pin_5);
              
          }
      
      }
      if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4)==0)
      {
         
         vTaskDelay(20);
          if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4)==0)
          {   
              while(!GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4));             
         
             GPIO_ResetBits(GPIOD,GPIO_Pin_5);
              
          }
      
      }
    
    
    }
    
    
}
extern ReceiveData UART5_ReceiveData;
void HC05_task(void *pvParameters)
{

    BLT_USART_Config();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // GPIOA时钟
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;      //PD2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//复用推挽输出
    GPIO_Init(GPIOD, &GPIO_InitStructure); 
    
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOD, &GPIO_InitStructure); 
    
    GPIO_ResetBits(GPIOD,GPIO_Pin_9);
    GPIO_SetBits(GPIOD,GPIO_Pin_5);
    printf("\nenter HC05\n");
    while(1)
    {
    
    if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_9)==1)
    {
       
     //  printf("hc05 succsuccl!");
      // GPIO_ResetBits(GPIOD,GPIO_Pin_5);
    
    
    }
    if(UART5_ReceiveData.receive_data_flag==1)
    {
        printf_blue((char *)UART5_ReceiveData.uart_buff,UART5_ReceiveData.datanum);
        UART5_ReceiveData.receive_data_flag=0;
        UART5_ReceiveData.datanum=0;
    
    }
    vTaskDelay(20);
     
    
    }
    

}




