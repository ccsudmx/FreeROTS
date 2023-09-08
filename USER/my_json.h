#ifndef _MY_JSON_H
#define _MY_JSON_H

#include "stdio.h"
#include "string.h" 

 typedef struct
 {
 	char *name;
    int status;
 	int value;
    int len;
 	
 }Json; 
 
 void MQTT_JSON(char *a,Json * Fan);
 void Json_mode(char *a,int *c);
#endif
