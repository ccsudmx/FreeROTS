#include "my_json.h"
#include "usart2.h"
void MQTT_JSON(char *a,Json * Fan)
{
// char Send[100];
 char *b=a;
 while(*b!=',')b++;
 while(*b!=':')	
 {  
     b++;
 }
 b++;
 b++;
 if(strstr((char *)a,Fan->name)!=NULL)
 { 	
   if(*b=='1')Fan->status=1;
   else Fan->status=0;
       
 }
 else return ;
 while(*b!=':')b++;
 b++;b++;
 if(*b=='0')Fan->value=0;
 Fan->value=(*b-48)*10;
 b++;
 Fan->value+=(*b-48);
 if(Fan->value==10&&(*++b=='0'))Fan->value=100;
// printf("%d,%d,%s",Fan->status,Fan->value,Fan->name);
// sprintf(Send,"{%s:%d:%d}",Fan->name,Fan->status,Fan->value);
// printf("%s",Send);
// Usart2_Send((uint8_t *)Send,strlen(Send));
 
}

void Json_mode(char * a,int *c)
{

 char *b=a;
 while(*b!=',')b++;
 while(*b!=':')	
 {  
     b++;
 }
 b++;
 b++;
 *c=(*b)-48;

}


