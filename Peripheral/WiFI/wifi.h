#ifndef _WIFI_H
#define _WIFI_H
#include "stm32f10x.h"
#define AT_CWMODE "AT+CWMODE=1"
#define AT_TX_LEN	128
#define AT_RX_LEN   256
#define MQTT_WIFI  "QFKJ"
#define MQTT_PWD   "QFKJ666888"
#define MQTT_IP    "192.168.30.219"
#define MQTT_PORT   1883
void Connect_MQTT(void) ; 
int8_t ESP8266_MQTT_Pub(char *IpBuf, uint8_t len, uint8_t qos);
#endif

