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

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	delay_init();	    				
    Uart_Init();
    xTaskCreate((TaskFunction_t )Wifi_task,             
               (const char*    )"Wifi_task",           
               (uint16_t       )2056,        
               (void*          )NULL,                  
               (UBaseType_t    )3,        
               (TaskHandle_t*  )&Wifi_Handler); 
  
    vTaskStartScheduler();   
    while(1);               
}


SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t xSemaphore_2;
SemaphoreHandle_t xSemaphore_3;
SemaphoreHandle_t  xSemaphore_4;

extern FIFO_Type Uart4_Rx_Fifo;
void Wifi_task(void *pvParameters)
{
 
    
    uint8_t Uart4_Read_Buff[UART4_RX_SIZE];
    xSemaphore_4 = xSemaphoreCreateCounting( 1, 0 );
    Uart4_Init(); 
    printf("1");    
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
       Fifo_Get(&Uart4_Rx_Fifo,Uart4_Read_Buff,UART4_RX_SIZE);
        if(strstr((const char *)Uart4_Read_Buff, (const char *)"+MQTTSUBRECV:0") != NULL){
            printf("%s\n",Uart4_Read_Buff);
            printf("hello,world");
        }
        else 
        {
            Uart_1_Send((uint8_t *)"error!",6);
        }
   
    }


}

void Lora_task(void *pvParameters)
{
    
   printf("2");
   xSemaphore_2=xSemaphoreCreateCounting( 1, 0 );
   Usart2_Init();
   LoRa_Init();
    
  xTaskCreate((TaskFunction_t )Rfid_task,             
               (const char*    )"Rfid_task",           
               (uint16_t       )1024,        
               (void*          )NULL,                  
               (UBaseType_t    )3,        
               (TaskHandle_t*  )&Rfid_Handler);
               
  xTaskCreate((TaskFunction_t )sendRfidCmd_task,             
               (const char*    )"sendRfidCmd_task",           
               (uint16_t       )256,        
               (void*          )NULL,                  
               (UBaseType_t    )3,        
               (TaskHandle_t*  )&sendRfidCmd_Handler);
         
   
    while(1)
    { 
        
         xSemaphoreTake(xSemaphore_2, portMAX_DELAY);
         Uart_1_Send((uint8_t *)"loraRE!",6);
        
    }

}


void Rfid_task(void *pvParameters)
{
   
    usart3_init(9600);
    xSemaphore_3 = xSemaphoreCreateCounting( 1, 0 );
    while(1)
    {
        
      xSemaphoreTake(xSemaphore_3, portMAX_DELAY);
      ReadId();
    
    
   }


}
void sendRfidCmd_task(void *pvParameters)
{  


  while(1)
  {
  
      RFID_CMD();

   
  }


}


