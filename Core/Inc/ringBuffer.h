/*
 * queue.h
 *
 *  Created on: Oct 12, 2025
 *      Author: Nestor Kalambay
 */

#ifndef INC_RINGBUFFER_H_
#define INC_RINGBUFFER_H_

#include <stdio.h>
#include <stdint.h>

#define QUEUE_BUFFER_SIZE	5000

typedef struct _ringBuffer
{
	uint8_t buffer[QUEUE_BUFFER_SIZE];
	uint16_t head;
	uint16_t tail;
}ringBuffer;

void ringBufferInsertByte(ringBuffer * rnBuf, uint8_t data);
void ringBufferInsertMult(ringBuffer * rnBuf, uint8_t * data, uint16_t length);
void ringBufferReadByte(ringBuffer * rnBuf, uint8_t * data);
void ringBufferReadMult(ringBuffer * rnBuf, uint8_t * data, uint16_t length);
void ringBufferInit(ringBuffer * rnBuf);
uint16_t ringBufferDataAvailable(ringBuffer * rnBuf);
void ringBufferInsertString(ringBuffer * rnBuf, char * ptr);

#endif /* INC_RINGBUFFER_H_ */
