/*
 * queue.c
 *
 *  Created on: Oct 12, 2025
 *      Author: Nestor Kalambay
 */

#include <ringBuffer.h>
#include <string.h>

void ringBufferInsertByte(ringBuffer * rnBuf, uint8_t data)
{
	uint16_t i = (unsigned int) (rnBuf->head + 1) % QUEUE_BUFFER_SIZE;
	if(i != rnBuf->tail)
	{
		rnBuf->buffer[rnBuf->head] = data;
		rnBuf->head = i;
	}
}

void ringBufferInsertString(ringBuffer * rnBuf, char * ptr)
{
	while(*ptr) ringBufferInsertByte(rnBuf, *ptr++);
}
void ringBufferInsertMult(ringBuffer * rnBuf, uint8_t * data, uint16_t length)
{
	for(uint16_t i = 0; i < length; i++) ringBufferInsertByte(rnBuf, *data++);
}
void ringBufferReadByte(ringBuffer * rnBuf, uint8_t * data)
{
	if(rnBuf->head == rnBuf->tail) *data = 0;
	else
	{
		*data = rnBuf->buffer[rnBuf->tail];
		rnBuf->tail = (rnBuf->tail + 1) % QUEUE_BUFFER_SIZE;
	}
}
void ringBufferReadMult(ringBuffer * rnBuf, uint8_t * data, uint16_t length)
{
	for(uint16_t i = 0; i < length; i++) ringBufferReadByte(rnBuf, data++);
}
void ringBufferInit(ringBuffer * rnBuf)
{
	rnBuf->head = 0;
	rnBuf->tail = 0;
	memset(rnBuf->buffer, 0, sizeof(rnBuf->buffer) / sizeof(uint8_t));
}

uint16_t ringBufferDataAvailable(ringBuffer * rnBuf)
{
	return ((uint16_t) (QUEUE_BUFFER_SIZE + rnBuf->head - rnBuf->tail)) % QUEUE_BUFFER_SIZE;
}
