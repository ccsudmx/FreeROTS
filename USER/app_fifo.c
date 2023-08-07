
#include "sys.h"
#include "app_fifo.h"


void Fifo_Init(FIFO_Type* fifo, uint8_t* buffer, uint16_t size)
{
	fifo->buffer = buffer;
	fifo->in = 0 ;
	fifo->out = 0;
	fifo->size = size;
}

uint16_t Fifo_Get(FIFO_Type* fifo, uint8_t* buffer, uint16_t len)
{
	uint16_t lenght;
	uint16_t in = fifo->in;	//防止处理过程中，中断修改fifo->in
	uint16_t i;
	lenght = (in + fifo->size - fifo->out)%fifo->size;
	if(lenght > len)
		lenght = len;
	for(i = 0; i < lenght; i++)
	{
		buffer[i] = fifo->buffer[(fifo->out + i)%fifo->size];
	}
	fifo->out = (fifo->out + lenght)%fifo->size;
	return lenght;
}

uint16_t Fifo_Set(FIFO_Type* fifo, uint8_t* buffer, uint16_t len)
{
	uint16_t lenght;
	uint16_t i;
	lenght = (fifo->out + fifo->size - fifo->in)%fifo->size;
	if(lenght > len)
		lenght = len;
	for(i = 0; i < lenght; i++)
	{
		fifo->buffer[(fifo->in + i)%fifo->size] = buffer[i];
	}
	fifo->in = (fifo->in + lenght)%fifo->size;
	return lenght;
}

uint16_t Fifo_Status(FIFO_Type* fifo)
{
	uint16_t lenght;
	lenght = (fifo->in + fifo->size - fifo->out)%fifo->size;
	return lenght;
}

