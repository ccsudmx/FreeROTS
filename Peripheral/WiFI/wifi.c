#include "wifi.h"
#include "stm32f10x.h"
#include "uart4.h"
#include "string.h"
#include "stdio.h"
#include "delay.h"
#include "app_fifo.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "delay.h"
extern FIFO_Type Uart4_Rx_Fifo;
uint8_t Uart4_Rx_Buff[AT_RX_LEN];
uint8_t MQTTSendErrorTimes=0;
void Send_AtCmd(uint8_t *string,uint8_t len)
{   
    uint8_t Send[AT_RX_LEN] ;   
    sprintf((char *)Send,"%s\r\n",string);  
   // printf("��������3~");
    Uart4_Send(Send,len+2);
 
}
extern xSemaphoreHandle xSemaphore_4;
extern xSemaphoreHandle xSemaphore_5;

uint8_t ESP8266_SetStation(void)
{  
  //   printf("��������2~");  
   memset(Uart4_Rx_Buff,0,AT_RX_LEN);
  Send_AtCmd((uint8_t*)AT_CWMODE,strlen(AT_CWMODE));
  //printf("��������4~"); 
    xSemaphoreTake(xSemaphore_4, portMAX_DELAY);
    // printf("��������5~"); 
  delay_ms(10);
    // printf("��������6~"); 
  Fifo_Get(&Uart4_Rx_Fifo,Uart4_Rx_Buff,AT_RX_LEN);
  //  printf("Debug-SetStation:%s",Uart4_Rx_Buff);
    
  if(strstr((const char *)Uart4_Rx_Buff,(const char *)"OK")==NULL)
  	{    
          return 3;

  	}
  	return 0;

}
int8_t ESP8266_Connect_WIFI(char * wifi,char *pwd)
{
    uint8_t Send_AT[AT_TX_LEN];
    memset(Uart4_Rx_Buff,0,AT_RX_LEN);
    memset(Send_AT,0x00,AT_TX_LEN);
    sprintf((char *)Send_AT,"AT+CWJAP=\"%s\",\"%s\"",wifi,pwd);
    Send_AtCmd((uint8_t *)Send_AT,strlen((const char *)Send_AT));
      xSemaphoreTake(xSemaphore_4, portMAX_DELAY);
    delay_ms(1000);
    Fifo_Get(&Uart4_Rx_Fifo,Uart4_Rx_Buff,AT_RX_LEN);
        printf("Debug-Connect-WIFI:%s",Uart4_Rx_Buff);
    if(strstr((const char *)Uart4_Rx_Buff,(const char *)"OK")==NULL)
   	return -1;
    return 0;


}
int8_t ESP8266_Connect_IP(char * IpAddr,uint16_t port)
{

   uint8_t Connect_Ip[AT_TX_LEN];
   memset(Uart4_Rx_Buff,0,AT_RX_LEN);
   memset(Connect_Ip,0x00,AT_TX_LEN);
   sprintf((char *)Connect_Ip,"AT+MQTTUSERCFG=0,1,\"M3\",\"\",\"\",0,0,\"\"");
   Send_AtCmd((uint8_t *)Connect_Ip,strlen((const char *)Connect_Ip));
      xSemaphoreTake(xSemaphore_4, portMAX_DELAY);
   delay_ms(1500);
   Fifo_Get(&Uart4_Rx_Fifo,Uart4_Rx_Buff,AT_RX_LEN);
   printf("Debug-Connect-USER: %s", (char *)Uart4_Rx_Buff);
   if(strstr((const char *)Uart4_Rx_Buff, (const char *)"OK") == NULL)
   {
       memset(Connect_Ip, 0x00, AT_TX_LEN);
       memset(Uart4_Rx_Buff,0,AT_RX_LEN);
       sprintf((char *)Connect_Ip,"AT+MQTTCLEAN=0");
       Send_AtCmd((uint8_t *)Connect_Ip,strlen((const char *)Connect_Ip));
       return 4;

   }
   memset(Uart4_Rx_Buff,0,AT_RX_LEN);
   memset(Connect_Ip, 0x00, AT_TX_LEN);
   sprintf((char *)Connect_Ip,"AT+MQTTCONN=0,\"%s\",%d,0",IpAddr, port);
   Send_AtCmd((uint8_t *)Connect_Ip,strlen((const char *)Connect_Ip));
   xSemaphoreTake(xSemaphore_4, portMAX_DELAY);
    delay_ms(1500);
   Fifo_Get(&Uart4_Rx_Fifo,Uart4_Rx_Buff,AT_RX_LEN);
  printf("Debug-Connect-IP: %s", (char *)Uart4_Rx_Buff);
  if(strstr((const char *)Uart4_Rx_Buff, (const char *)"OK") == NULL)
	{
		return 3;
	}
   return 0;

}
int8_t ESP8266_MQTT_SUB(char *topic)
{

    uint8_t Connect_MQTT[AT_TX_LEN];
    memset(Uart4_Rx_Buff,0,AT_RX_LEN);
    memset(Connect_MQTT,0x00,AT_TX_LEN);
    sprintf((char *)Connect_MQTT,"AT+MQTTSUB=0,\"%s\",1", topic);
    Send_AtCmd((uint8_t *)Connect_MQTT,strlen((char *)Connect_MQTT));
    xSemaphoreTake(xSemaphore_4, portMAX_DELAY);
    delay_ms(100);
    Fifo_Get(&Uart4_Rx_Fifo,Uart4_Rx_Buff,AT_RX_LEN);
    printf("Debug-Connect-MQTT_Sub: \"%s\"",Uart4_Rx_Buff);
  if(strstr((const char *)Uart4_Rx_Buff, (const char *)"OK") == NULL)
	{
		return -4;
	}
	return 0;

}
extern SemaphoreHandle_t xMutex;
int8_t ESP8266_MQTT_Pub(char *IpBuf, uint8_t len, uint8_t qos)
{
	uint8_t IpSend[AT_TX_LEN];
	memset(IpSend, 0x00, AT_TX_LEN);//��ջ���
    memset(Uart4_Rx_Buff,0,AT_RX_LEN);
	/************����ΪMQTT_Binary����ģʽ************/
	sprintf((char *)IpSend,"AT+MQTTPUBRAW=0,\"M3\",%d,%d,0", len, qos);
    
	//1.�������� 2.�ȴ���Ϣ�յ� 3.�ȴ���Ϣͨ��DMA���䵽Uart4_Rx_Buff������
     //  taskENTER_CRITICAL(); 
    xSemaphoreTake(xMutex, portMAX_DELAY);
    
	Send_AtCmd((uint8_t *)IpSend,strlen((const char *)IpSend));
    xSemaphoreTake(xSemaphore_5, portMAX_DELAY);
     delay_ms(100);
    Fifo_Get(&Uart4_Rx_Fifo,Uart4_Rx_Buff,AT_RX_LEN);
    
    xSemaphoreGive(xMutex);
    
	if(strstr((const char *)Uart4_Rx_Buff, (const char *)"OK") == NULL)
	{
		printf("MQTT_Pub Fail:%s\r\n", Uart4_Rx_Buff);
		MQTTSendErrorTimes++;
		return -1;
	}
	
	 memset(Uart4_Rx_Buff,0,AT_RX_LEN);
	
	//�Զ�����������ʽ�����ַ���
    //taskENTER_CRITICAL(); 
    xSemaphoreTake(xMutex, portMAX_DELAY);
	 Send_AtCmd((uint8_t *)IpBuf, len);
     xSemaphoreTake(xSemaphore_5, portMAX_DELAY);
      delay_ms(100);
      
      Fifo_Get(&Uart4_Rx_Fifo,Uart4_Rx_Buff,AT_RX_LEN);
	xSemaphoreGive(xMutex);
	if(strstr((const char *)Uart4_Rx_Buff, (const char *)"OK") == NULL)
	{
		MQTTSendErrorTimes++;
		return -2;
	}
     printf("MQTT_Pub: %s\r\n", IpBuf);
    MQTTSendErrorTimes=0;
	return 0;
}
uint8_t Connect_MQTT(void)
{
  int error=0;
  int i=0;
     // printf("��������~");  
    for(i=0;i<3;i++)
    { 
        // printf("��������1~");  
     if(ESP8266_SetStation()==0)break;
    
    }
    if(i==3)error= -1;
    
    
    for(i=0;i<3;i++)
    {
       if(ESP8266_Connect_WIFI( (char *)MQTT_WIFI, (char *)MQTT_PWD)==0)break;
    }
    if(i==3)error=-2;
    
    
    for(i=0;i<3;i++)
    {
      if(ESP8266_Connect_IP((char * )MQTT_IP,(uint16_t)MQTT_PORT)==0)break;
    
    }
    if(i==3)error= -3;
    
     for(i=0;i<3;i++)
    {
       if(ESP8266_MQTT_SUB((char *)"setM3")==0)break;
    }
    if(i==3)error= -4;
   
 switch(error)
    {
     
     case  0:printf("�������Ӿ�����!!!\r\n");break;
        
     case -1:printf("����Ϊվ��ģʽʧ��!\r\n");break;
      
     case -2:printf("����wifiʧ��!\r\n"); break; 
     
     case -3 :printf("����IPʧ��!\r\n");break;
     
     case -4 :printf("����mqttʧ��!\r\n");break;
    
     default :break;
    
    
    
    }
    return error;
    
    

}

