#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
  * @brief 通用环形缓冲区结构体定义
  */
typedef struct {
    uint8_t  *buffer;        /*!< 缓冲区内存首地址 */
    volatile uint16_t head;  /*!< 写指针 (写者/中断/DMA 维护) */
    volatile uint16_t tail;  /*!< 读指针 (读者/主循环 维护) */
    uint16_t capacity;       /*!< 缓冲区可容纳的最大元素数量 (必须是 2 的幂) */
    uint16_t mask;           /*!< 掩码 (capacity - 1)，用于替代 % 运算 */
    uint16_t element_size;   /*!< 单个元素的大小 (Bytes) */
} RingBuffer_t;

void RingBuffer_Init(RingBuffer_t *rb, void *buffer, uint16_t capacity, uint16_t element_size);
bool RingBuffer_Push(RingBuffer_t *rb, const void *data);
bool RingBuffer_Pop(RingBuffer_t *rb, void *data);
bool RingBuffer_IsEmpty(const RingBuffer_t *rb);
bool RingBuffer_IsFull(const RingBuffer_t *rb);
uint16_t RingBuffer_GetCount(const RingBuffer_t *rb);
void RingBuffer_Flush(RingBuffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* __RING_BUFFER_H */
