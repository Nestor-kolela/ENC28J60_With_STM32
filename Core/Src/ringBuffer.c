/*
 * queue.c
 *
 *  Created on: Oct 12, 2025
 *      Author: Nestor Kalambay
 */

#include <ringBuffer.h>
#include <string.h>

tf_ringBuffer_status tf_ringbuffer_insertByte(ft_ringBuffer * rnBuf, uint8_t data)
{
	uint16_t i = (uint16_t) (rnBuf->head + 1) % QUEUE_BUFFER_SIZE;
	if(i != rnBuf->tail)
	{
		rnBuf->buffer[rnBuf->head] = data;
		rnBuf->head = i;
		return ft_ring_buffer_write_success;
	}else
	{
		return ft_ring_buffer_write_fail;
	}
}

tf_ringBuffer_status tf_ringbuffer_puts(ft_ringBuffer * rnBuf, char * ptr)
{
	while(*ptr)
	{
		tf_ringBuffer_status stat;
		stat = tf_ringbuffer_insertByte(rnBuf, *ptr++);
		if(stat != ft_ring_buffer_write_success) return ft_ring_buffer_write_fail;
	}

	//At the end we can append a null for the string.
	*ptr = 0;
	return ft_ring_buffer_write_success;
}

tf_ringBuffer_status tf_ringbuffer_insertMultipleBytes(ft_ringBuffer * rnBuf, uint8_t * data, uint16_t length)
{
	for(uint16_t i = 0; i < length; i++)
	{
		tf_ringBuffer_status stat;
		stat = tf_ringbuffer_insertByte(rnBuf, *data++);
		if(stat != ft_ring_buffer_write_success) return ft_ring_buffer_write_fail;
	}
	return ft_ring_buffer_write_success;
}

tf_ringBuffer_status tf_ringbuffer_readByte(ft_ringBuffer * rnBuf, uint8_t * data)
{
	if(rnBuf->head == rnBuf->tail)
	{
		return ft_ring_buffer_read_fail;
	}
	else
	{
		*data = rnBuf->buffer[rnBuf->tail];
		rnBuf->tail = (rnBuf->tail + 1) % QUEUE_BUFFER_SIZE;
		return ft_ring_buffer_read_success;
	}
}

tf_ringBuffer_status tf_ringbuffer_readMultiBytes(ft_ringBuffer * rnBuf, uint8_t * data, uint16_t length)
{
	for(uint16_t i = 0; i < length; i++)
	{
		tf_ringBuffer_status stat;
		stat = tf_ringbuffer_readByte(rnBuf, data++);
		if(stat != ft_ring_buffer_read_success) return ft_ring_buffer_read_fail;
	}
	return ft_ring_buffer_read_success;
}
void tf_ringbuffer_init(ft_ringBuffer * rnBuf)
{
	rnBuf->head = 0;
	rnBuf->tail = 0;
	memset(rnBuf->buffer, 0, sizeof(rnBuf->buffer));
}

tf_ringBuffer_status tf_ringbuffer_dataIsAvailable(ft_ringBuffer * rnBuf)
{
	uint16_t value = ((QUEUE_BUFFER_SIZE + rnBuf->head - rnBuf->tail) % QUEUE_BUFFER_SIZE);
	return value > 0 ? ft_ring_buffer_data_available : ft_ring_buffer_no_data_available;
}
