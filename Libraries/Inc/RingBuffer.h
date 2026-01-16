// RingBuffer.h
//

#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__


#ifdef __cplusplus
extern "C" {
#endif



////////////////////////////////////////////////////////////////////////////////////////////////
//

typedef struct _RingBuffer
{
	uint8_t*	data;
	uint16_t	size;

	uint16_t	head;
	uint16_t	tail;
	uint16_t	reserve;

} RingBuffer;


////////////////////////////////////////////////////////////////////////////////////////////////
//

void    RB_Init(RingBuffer* rb, uint8_t * data, uint16_t size);

int32_t	RB_GetBufferCount(RingBuffer* rb);
int32_t	RB_GetDataCount(RingBuffer* rb);

int32_t	RB_IsFull(RingBuffer* rb);
int32_t	RB_IsEmpty(RingBuffer* rb);

int32_t	RB_Push(RingBuffer* rb, uint8_t data);
int32_t	RB_PushFront(RingBuffer* rb, uint8_t data);
int32_t	RB_Pop(RingBuffer* rb);
int32_t	RB_Peek(RingBuffer* rb);

int32_t RB_Flush(RingBuffer* rb, int32_t size);
void	RB_Empty(RingBuffer* rb);

int32_t RB_GetConsecutiveCount(RingBuffer* rb);
uint8_t *RB_GetConsecutiveData(RingBuffer* rb, int32_t *pLen);


#ifdef __cplusplus
}
#endif


#endif // __RINGBUFFER_H__
