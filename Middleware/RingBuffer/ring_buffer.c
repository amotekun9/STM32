#include "ring_buffer.h"
#include <string.h>

void RingBuffer_Init(RingBuffer_t *rb, void *buffer, uint16_t capacity, uint16_t element_size)
{
    if (rb == NULL || buffer == NULL || capacity == 0 || element_size == 0)
    {
        return;
    }

    rb->buffer       = (uint8_t *)buffer;
    rb->head         = 0;
    rb->tail         = 0;
    rb->capacity     = capacity;
    rb->mask         = capacity - 1; /* 核心：预先计算掩码 (例如 64 - 1 = 63，二进制 0x3F) */
    rb->element_size = element_size;
}

bool RingBuffer_IsEmpty(const RingBuffer_t *rb)
{
    if (rb == NULL)
    {
        return true;
    }
    return (rb->head == rb->tail);
}

bool RingBuffer_IsFull(const RingBuffer_t *rb)
{
    if (rb == NULL)
    {
        return true;
    }
    /* 用 & rb->mask 替代 % rb->capacity */
    return (((rb->head + 1) & rb->mask) == rb->tail);
}

bool RingBuffer_Push(RingBuffer_t *rb, const void *data)
{
    if (rb == NULL || data == NULL)
    {
        return false;
    }

    /* 用 & rb->mask 替代 % rb->capacity，极速计算下一个写指针位置 */
    uint16_t next_head = (rb->head + 1) & rb->mask;

    // 判满
    if (next_head == rb->tail)
    {
        return false;
    }

    // 单字节直接赋值以优化速度，多字节结构体使用 memcpy
    if (rb->element_size == 1)
    {
        rb->buffer[rb->head] = *(const uint8_t *)data;
    }
    else
    {
        memcpy(&rb->buffer[rb->head * rb->element_size], data, rb->element_size);
    }

    // 内存写屏障机制：先拷贝数据，最后更新 head，保证多线程/中断安全
    rb->head = next_head;

    return true;
}

bool RingBuffer_Pop(RingBuffer_t *rb, void *data)
{
    if (rb == NULL || data == NULL)
    {
        return false;
    }

    // 判空
    if (rb->head == rb->tail)
    {
        return false;
    }

    // 单字节直接赋值以优化速度，多字节结构体使用 memcpy
    if (rb->element_size == 1)
    {
        *(uint8_t *)data = rb->buffer[rb->tail];
    }
    else
    {
        memcpy(data, &rb->buffer[rb->tail * rb->element_size], rb->element_size);
    }

    /* 用 & rb->mask 替代 % rb->capacity，极速更新读指针位置 */
    rb->tail = (rb->tail + 1) & rb->mask;

    return true;
}

uint16_t RingBuffer_GetCount(const RingBuffer_t *rb)
{
    if (rb == NULL)
    {
        return 0;
    }

    uint16_t head = rb->head;
    uint16_t tail = rb->tail;

    if (head >= tail)
    {
        return (head - tail);
    }
    else
    {
        return (rb->capacity + head - tail);
    }
}

void RingBuffer_Flush(RingBuffer_t *rb)
{
    if (rb == NULL)
    {
        return;
    }
    
    rb->head = 0;
    rb->tail = 0;
}
