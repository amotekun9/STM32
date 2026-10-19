#ifndef __RS485_H
#define __RS485_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "ring_buffer.h"  /* 通用无锁环形队列 */
#include <stdbool.h>
#include <stdint.h>
#include "bsp_delay.h"


/*********************** 缓冲区容量配置 ***********************/
#define RS485_RX_DMA_BUF_SIZE   256U   /* DMA 物理直写缓冲区大小 */
#define RS485_APP_RX_BUF_SIZE   512U   /* 通用环形队列容量 */

/*********************** RS485 引脚与串口定义 **********************/
#define RS485_RE_GPIO_PORT      GPIOA
#define RS485_RE_GPIO_PIN       GPIO_PIN_1

#define RS485_UX                USART2
#define RS485_HANDLE            huart2

#define MAX485_UX                USART3
#define MAX485_HANDLE            huart3

extern UART_HandleTypeDef       RS485_HANDLE;
extern UART_HandleTypeDef       MAX485_HANDLE;

/*********************** RS485 实例句柄结构体 ***********************/
typedef struct {
    UART_HandleTypeDef *huart;          /* 串口句柄 (如 &huart2, &huart3) */
    GPIO_TypeDef       *re_port;        /* RE/DE 控制引脚端口 */
    uint16_t           re_pin;          /* RE/DE 控制引脚编号 */
    
    uint8_t            *dma_rx_buf;     /* DMA 物理直写缓冲区 */
    uint16_t           dma_buf_size;    /* DMA 缓冲区大小 */
    
    RingBuffer_t       ring_buf;        /* 软件环形队列句柄 */
    uint8_t            *app_rx_buf;     /* 软件环形队列物理内存 */
    uint16_t           app_buf_size;    /* 软件环形队列容量 */
    
    uint16_t           last_dma_head;   /* DMA 上一次写入位置 */
    volatile bool      tx_busy;         /* 发送忙标志 */
    volatile uint32_t  last_rx_tick;    /* 最后一次接收字节时间戳 (用于 3ms 静默断帧) */
} RS485_Dev_t;
	

/*********************** RS485 收发方向控制 ***********************/
#define RS485_RE(x)             do { \
                                    (x) ? HAL_GPIO_WritePin(RS485_RE_GPIO_PORT, RS485_RE_GPIO_PIN, GPIO_PIN_SET) : \
                                          HAL_GPIO_WritePin(RS485_RE_GPIO_PORT, RS485_RE_GPIO_PIN, GPIO_PIN_RESET); \
                                } while(0)

/*********************** 对外应用层 API (传入设备句柄) ***********************/
RS485_Dev_t* RS485_GetDevice(UART_HandleTypeDef *huart);
bool RS485_IsFrameReady(RS485_Dev_t *dev);
void RS485_Dev_Init(RS485_Dev_t *dev);	/* 初始化 RS485 */
void RS485_All_Init(void);	/* 一次性初始化所有 RS485 实例 */
void RS485_Poll(RS485_Dev_t *dev);	/* 主循环轮询与 DMA 搬运看门狗 */
bool rs485_ReadByte(RS485_Dev_t *dev, uint8_t *pByte);	/* 从环形队列读取 1 字节 */
uint16_t rs485_ReadArray(RS485_Dev_t *dev, uint8_t *pData, uint16_t max_len);/* 批量从环形队列读取整帧 */
uint16_t rs485_GetRxCount(RS485_Dev_t *dev);	/* 获取队列可读字节数 */
void rs485_FlushRx(RS485_Dev_t *dev);	/* 清空接收环形队列 */
bool rs485_SendAsync(RS485_Dev_t *dev, const uint8_t *pData, uint16_t len);/* DMA 异步发送 */                                

#ifdef __cplusplus
}
#endif

#endif /* __RS485_H */
