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

typedef struct _ft_ringBuffer
{
	uint8_t buffer[QUEUE_BUFFER_SIZE];
	uint16_t head;
	uint16_t tail;
}ft_ringBuffer;

typedef enum _tf_ringBuffer_status
{
	ft_ring_buffer_init_success 		= 0,
	ft_ring_buffer_write_success		= 1,
	ft_ring_buffer_write_fail			= 2,
	ft_ring_buffer_read_success			= 3,
	ft_ring_buffer_read_fail			= 4,
	ft_ring_buffer_data_available		= 5,
	ft_ring_buffer_no_data_available	= 6
}tf_ringBuffer_status;

void tf_ringbuffer_init(ft_ringBuffer * rnBuf);
tf_ringBuffer_status tf_ringbuffer_insertByte(ft_ringBuffer * rnBuf, uint8_t data);
tf_ringBuffer_status tf_ringbuffer_insertMult(ft_ringBuffer * rnBuf, uint8_t * data, uint16_t length);
tf_ringBuffer_status tf_ringbuffer_readByte(ft_ringBuffer * rnBuf, uint8_t * data);
tf_ringBuffer_status tf_ringbuffer_readMultiBytes(ft_ringBuffer * rnBuf, uint8_t * data, uint16_t length);
tf_ringBuffer_status tf_ringbuffer_dataIsAvailable(ft_ringBuffer * rnBuf);
tf_ringBuffer_status tf_ringbuffer_puts(ft_ringBuffer * rnBuf, char * ptr);

#endif /* INC_RINGBUFFER_H_ */
