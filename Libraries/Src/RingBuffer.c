// RingBuffer.c
//

#include <stdint.h>
#include <stdio.h>
#include <sys/param.h>
#include "RingBuffer.h"


void RB_Init(RingBuffer* rb, uint8_t * data, uint16_t size)
{
	rb->data = data;
	rb->size = size;

	rb->head = rb->tail = 0;
}

int32_t RB_GetBufferCount(RingBuffer* rb)
{
	return rb->size;
}

int32_t RB_GetDataCount(RingBuffer* rb)
{
	return (rb->head - rb->tail) & (rb->size - 1);
}

int32_t RB_IsFull(RingBuffer* rb)
{
	return ((rb->head + 1) & (rb->size - 1)) == rb->tail ? 1 : 0;
}

int32_t RB_IsEmpty(RingBuffer* rb)
{
	return (rb->head == rb->tail) ? 1 : 0;
}

int32_t RB_Push(RingBuffer* rb, uint8_t data)
{
	if (RB_IsFull(rb))
		return -1;

	rb->data[rb->head] = data;
	rb->head = (rb->head + 1) & (rb->size -1);

	return 0;
}

int32_t RB_PushFront(RingBuffer* rb, uint8_t data)
{
	if (RB_IsFull(rb))
		return -1;

	rb->tail = (rb->tail == 0) ? (rb->size - 1) : (rb->tail - 1);
	rb->data[rb->tail] = data;

	return 0;
}

int32_t RB_Pop(RingBuffer* rb)
{
	if (RB_IsEmpty(rb))
		return -1;

	int32_t ch = rb->data[rb->tail];
	rb->tail = (rb->tail + 1) & (rb->size -1);

	return ch;

}

int32_t RB_Peek(RingBuffer* rb)
{
	if (RB_IsEmpty(rb))
		return -1;

	return rb->data[rb->tail];

}

int32_t RB_Flush(RingBuffer* rb, int32_t size)
{
	int32_t tot = RB_GetDataCount(rb);
	int32_t len = MIN(tot, size);
	rb->tail = (rb->tail + len) & (rb->size -1);

	return tot - len;
}

void RB_Empty(RingBuffer* rb)
{
	rb->head = rb->tail = 0;
}

int32_t RB_GetConsecutiveCount(RingBuffer* rb)
{
	if (rb->tail <= rb->head)
		return rb->head - rb->tail;

	return rb->size - rb->tail;
}

uint8_t *RB_GetConsecutiveData(RingBuffer* rb, int32_t *pLen)
{
	int32_t len = RB_GetConsecutiveCount(rb);
	if (len <= 0)
		return 0;

	if (pLen)
		*pLen = len;

	return &rb->data[rb->tail];
}
