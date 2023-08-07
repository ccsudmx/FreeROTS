#include "wifi.h"
#include "stm32f10x.h"
#include "uart4.h"
#include "string.h"
#include "stdio.h"
#include "delay.h"
#include "app_fifo.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern FIFO_Type Uart4_Rx_Fifo;
uint8_t Uart4_Rx_Buff[AT_RX_LEN];
void Send_AtCmd(uint8_t *string,uint8_t len)
{   
    uint8_t Send[AT_RX_LEN] ;   
    sprintf((char *)Send,"%s\r\n",string);
    
    Uart4_Send(Send,len+2);
   
    
    //printf("%s",Send);
  //  Uart4_Send((uint8_t*)"\r\n",2);
 
}
extern xSemaphoreHandle xSemaphore_4;
uint8_t ESP8266_SetStation(void)
{
   memset(Uart4_Rx_Buff,0,AT_RX_LEN);
  Send_AtCmd((uint8_t*)AT_CWMODE,strlen(AT_CWMODE));
    xSemaphoreTake(xSemaphore_4, portMAX_DELAY);
  delay_ms(10);
  Fifo_Get(&Uart4_Rx_Fifo,Uart4_Rx_Buff,AT_RX_LEN);
    printf("Debug-SetStation:%s",Uart4_Rx_Buff);
    
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
   delay_ms(2000);
   Fifo_Get(&Uart4_Rx_Fifo,Uart4_Rx_Buff,AT_RX_LEN);
   printf("Debug-Connect-USER: %s", (char *)Uart4_Rx_Buff);
   if(strstr((const char *)Uart4_Rx_Buff, (const char *)"OK") == NULL)
   {
       memset(Connect_Ip, 0x00, AT_TX_LEN);
       sprintf((char *)Connect_Ip,"AT+MQTTCLEAN=0");
       Send_AtCmd((uint8_t *)Connect_Ip,strlen((const char *)Connect_Ip));
       return 4;

   }
   memset(Uart4_Rx_Buff,0,AT_RX_LEN);
   memset(Connect_Ip, 0x00, AT_TX_LEN);
   sprintf((char *)Connect_Ip,"AT+MQTTCONN=0,\"%s\",%d,0",IpAddr, port);
   Send_AtCmd((uint8_t *)Connect_Ip,strlen((const char *)Connect_Ip));
   delay_ms(1000);
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
    delay_ms(1000);
    Fifo_Get(&Uart4_Rx_Fifo,Uart4_Rx_Buff,AT_RX_LEN);
    printf("Debug-Connect-MQTT_Sub: \"%s\"",Uart4_Rx_Buff);
  if(strstr((const char *)Uart4_Rx_Buff, (const char *)"OK") == NULL)
	{
		return -4;
	}
	return 0;

}
void Connect_MQTT(void)
{

  int i=0;
    for(i=0;i<3;i++)
    {
     if(ESP8266_SetStation()==0)break;
    
    }
    if(i==3)printf("Failed-SetStation!\n");
    for(i=0;i<3;i++)
    {
       if(ESP8266_Connect_WIFI( (char *)MQTT_WIFI, (char *)MQTT_PWD)==0)break;
    }
       if(i==3)printf("Failed-Connect-WIFI!\n");
    for(i=0;i<3;i++)
    {
      if(ESP8266_Connect_IP((char * )MQTT_IP,(uint16_t)MQTT_PORT)==0)break;
    
    }
      if(i==3)printf("Failed-Connect-IP!\n");
         for(i=0;i<3;i++)
    {
       if(ESP8266_MQTT_SUB((char *)"setM3")==0)break;
    }
      if(i==3)printf("Failed-MQTT-SUB!\n");

}

