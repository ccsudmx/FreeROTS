#ifndef	_APP_FIFO_H
#define	_APP_FIFO_H
#include "stm32f10x.h"


/**************** 用户任务声明 *******************/

typedef struct {
	uint8_t* buffer;	//FIFO的缓存数组
	uint16_t in;			//缓存数组的下标，in指向要存入数据的地址
	uint16_t out;			//缓存数组的下标，out指向要读出数据的地址
	uint16_t size;		//FIFO缓存数组长度
}FIFO_Type;

void Fifo_Init(FIFO_Type* fifo, uint8_t* buffer, uint16_t size);

uint16_t Fifo_Get(FIFO_Type* fifo, uint8_t* buffer, uint16_t len);

uint16_t Fifo_Set(FIFO_Type* fifo, uint8_t* buffer, uint16_t len);

uint16_t Fifo_Status(FIFO_Type* fifo);

#endif	//_APP_H_

