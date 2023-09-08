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
#include "stmflash.h"
#define FLASH_SAVE_ADDR  0X08070000		//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)
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
 int send_volume=0;
 int mode=0;
 int TH=2;
  int test=0;
int main(void)
{  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	delay_init();	    				
    Usart1_Init();
    usart3_init(9600);
    STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)&TH,1);
    TH=3;
    STMFLASH_Read(FLASH_SAVE_ADDR,(u16*)&test,1);
    printf("%d",test);
    xMutex = xSemaphoreCreateMutex( );

    xTaskCreate((TaskFunction_t )Wifi_task,             
               (const char*    )"Wifi_task",           
               (uint16_t       )2056,        
               (void*          )NULL,                  
               (UBaseType_t    )4,        
               (TaskHandle_t*  )&Wifi_Handler); 

       xTaskCreate((TaskFunction_t )Key_task,             
               (const char*    )"Key_task",           
               (uint16_t       )512,        
               (void*          )NULL,                  
               (UBaseType_t    )3,        
               (TaskHandle_t*  )&Key_Handler);    
               
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
            if(strstr((const char *)Uart4_Read_Buff,(const char *)"Mode"))
            {
               Json_mode((char *)Uart4_Read_Buff,&mode);
                printf("%d",mode);
             
            }
            else if(strstr((const char *)Uart4_Read_Buff,(const char *)"TH"))
            {
               Json_mode((char *)Uart4_Read_Buff,&TH);
               printf("%d",TH);
                          
             
            }
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
  printf("enter  lora");
   // char *buff;
   xSemaphore_2=xSemaphoreCreateCounting( 1, 0 );
   Usart2_Init();
   LoRa_Init();
  
  xTaskCreate((TaskFunction_t )Rfid_task,             
               (const char*    )"Rfid_task",           
               (uint16_t       )512,        
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


        sprintf((char *)Send_msg,"%s,%s,\"water\"=\"%d\"}",(char *)Usart2_RX,Send_id,send_volume);
        //printf("%s",Send_id);
        // xSemaphoreTake(xMutex, portMAX_DELAY);
         ESP8266_MQTT_Pub((char *)Send_msg,strlen((char *)Send_msg),1);
         // xSemaphoreGive(xMutex);
         
        
    }

}


void Rfid_task(void *pvParameters)
{
   

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
    if(mode)
    {
      RFID_CMD();
    }
   
  }


}

void Key_task(void *pvParameters)
{    
    
    uint8_t flag_stop=0;
    uint8_t flag_key=0;
    int volume=0;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOG, ENABLE); // GPIOA时钟
    GPIO_InitTypeDef GPIO_InitStructure;
                        
    //row1--PD5
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
    GPIO_Init(GPIOD, &GPIO_InitStructure);  
    
    //PD6--led灯
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
     GPIO_Init(GPIOD, &GPIO_InitStructure);    
     //PG1--led灯
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
     GPIO_Init(GPIOG, &GPIO_InitStructure);   
    
    //上拉
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPU;
   //C1  PD3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
	GPIO_Init(GPIOD, &GPIO_InitStructure); 	
   //C2  PD4
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; 
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
    
 
    //拉低	
    GPIO_ResetBits(GPIOD,GPIO_Pin_7);
    GPIO_ResetBits(GPIOD,GPIO_Pin_6);
    GPIO_ResetBits(GPIOG,GPIO_Pin_1);
    
     printf("enter key!");
     printf("enter key!\n");
    while(1)
    {
      if(mode)
      {
      if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3)==0)
      {
         
         vTaskDelay(20);
          if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3)==0)
          {   
              while(!GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3));
              
             flag_stop++;
              if(flag_key==0)
              {
                   GPIO_SetBits(GPIOD,GPIO_Pin_6);
                   flag_key=1;
              }
              else if(flag_key==1)
              {    
                  GPIO_ResetBits(GPIOD,GPIO_Pin_6);
                  flag_key=0;
                  flag_stop=0;
              }
              else if(flag_key==2) 
              {
                GPIO_ResetBits(GPIOG,GPIO_Pin_1);
                GPIO_SetBits(GPIOD,GPIO_Pin_6);
                  flag_key=1;
              }
             
            
              
          }
      
      }
      if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4)==0)
      {
         
         vTaskDelay(20);
          if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4)==0)
          {   
              while(!GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4));             
              flag_stop++;
              if(flag_key==0)
              {
               GPIO_SetBits(GPIOG,GPIO_Pin_1);
                flag_key=2;
              
              }
              else if(flag_key==2)
              {  
                  flag_key=0;
                  GPIO_ResetBits(GPIOG,GPIO_Pin_1);
                  flag_stop=0;
              }
              else if(flag_key==1)
              {
               GPIO_ResetBits(GPIOD,GPIO_Pin_6);   
               GPIO_SetBits(GPIOG,GPIO_Pin_1);
                  flag_key=2;
                
              }
           
             
              
          }
      
      }
     if(flag_stop==0)
     {  
         if(volume!=0)
         {
        send_volume=volume;
         }

        volume=0;
        flag_key=0;
        
     
     }
    if(flag_stop>0)
     {
        volume++;
         if(volume>=6000)
         {    
             //关闭灯
             GPIO_ResetBits(GPIOD,GPIO_Pin_6);
             send_volume=6000;           
             flag_stop=0;
         }
        else vTaskDelay(100);
         
     
     }
    
    
    }
}
    
}
extern ReceiveData UART5_ReceiveData;
void HC05_task(void *pvParameters)
{
    char HC05_Buff[40];
    BLT_USART_Config();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // GPIOA时钟
    GPIO_InitTypeDef GPIO_InitStructure;
    
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;      //PD2
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//复用推挽输出
//    GPIO_Init(GPIOD, &GPIO_InitStructure); 
//    
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOD, &GPIO_InitStructure); 
    
    GPIO_ResetBits(GPIOD,GPIO_Pin_9);
 
    printf("\nenter HC05\n");
    while(1)
    {
    
    if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_9)==1)
    {
      sprintf(HC05_Buff,"ID=1,V1=2.3,V2=3.4,warning!");
      printf_blue(HC05_Buff,strlen(HC05_Buff));
      vTaskDelay(2000);
    
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




