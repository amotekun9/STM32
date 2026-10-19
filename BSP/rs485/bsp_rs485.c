#include "bsp_rs485.h"

/* --- 静态分配私有缓冲区与实例 (安全、不占用栈空间、生命周期永久) --- */
static uint8_t vfd_dma_buf[RS485_RX_DMA_BUF_SIZE];
static uint8_t vfd_app_buf[RS485_APP_RX_BUF_SIZE];
static RS485_Dev_t g_vfd_dev = {
    .huart        = &huart2,
    .re_port      = GPIOA,
    .re_pin       = GPIO_PIN_1,
    .dma_rx_buf   = vfd_dma_buf,
    .dma_buf_size = sizeof(vfd_dma_buf),
    .app_rx_buf   = vfd_app_buf,
    .app_buf_size = sizeof(vfd_app_buf)
};

static uint8_t hmi_dma_buf[RS485_RX_DMA_BUF_SIZE];
static uint8_t hmi_app_buf[RS485_APP_RX_BUF_SIZE];
static RS485_Dev_t g_hmi_dev = {
    .huart        = &huart3,
    .re_port      = GPIOE,
    .re_pin       = GPIO_PIN_15,
    .dma_rx_buf   = hmi_dma_buf,
    .dma_buf_size = sizeof(hmi_dma_buf),
    .app_rx_buf   = hmi_app_buf,
    .app_buf_size = sizeof(hmi_app_buf)
};

/* 提供给外部安全获取句柄的函数 */
RS485_Dev_t* RS485_GetDevice(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) return &g_vfd_dev;
    if (huart->Instance == USART3) return &g_hmi_dev;
    return NULL;
}

/* 内部静态控制宏：切换收发模式 */
static inline void RS485_SetRE(RS485_Dev_t *dev, bool transmit)
{
    if (transmit)
	{
        HAL_GPIO_WritePin(dev->re_port, dev->re_pin, GPIO_PIN_SET);
    }
	else
	{
        HAL_GPIO_WritePin(dev->re_port, dev->re_pin, GPIO_PIN_RESET);
    }
}

/* 判断总线是否完全空闲/一帧结束的 API */
bool RS485_IsFrameReady(RS485_Dev_t *dev)
{
    if (RingBuffer_GetCount(&dev->ring_buf) > 0)
    {
        if ((HAL_GetTick() - dev->last_rx_tick) >= 3) /* 3ms 静默断帧判定 */
        {
            return true;
        }
    }
    return false;
}

/**
  * @brief ：初始化 RS485 硬件与 DMA 接收
  */
void RS485_Dev_Init(RS485_Dev_t *dev)
{
    RS485_SetRE(dev, false); /* 默认接收 */
    dev->last_dma_head = 0;
    dev->tx_busy = false;
	
	/* 1. 初始化通用无锁环形队列 */
    RingBuffer_Init(&dev->ring_buf, dev->app_rx_buf, dev->app_buf_size, sizeof(uint8_t));
	/* 2. 启动 DMA 循环接收 (DMA Rx Mode 必须为 Circular) */
    HAL_UART_Receive_DMA(dev->huart, dev->dma_rx_buf, dev->dma_buf_size);
}

/* 一次性初始化所有实例 */
void RS485_All_Init(void)
{
    RS485_Dev_Init(&g_vfd_dev);
    RS485_Dev_Init(&g_hmi_dev);
}

/**
  * @brief ：将 DMA 数据同步至 RingBuffer
  */
void RS485_Poll(RS485_Dev_t *dev)
{
    uint16_t current_dma_head;
	
	if (dev == NULL || dev->huart == NULL || dev->huart->hdmarx == NULL)
    {
        return;
    }
	
    /* 1. 计算当前 DMA 写入偏移量 */
    current_dma_head = dev->dma_buf_size - __HAL_DMA_GET_COUNTER(dev->huart->hdmarx);

    /* 2. 检查是否有新数据写入（当前指针与上一次记录的指针不一致） */
    if (current_dma_head != dev->last_dma_head)
    {
    	/* 只要检测到有新字节进来，立即刷新最后接收时间戳*/
        dev->last_rx_tick = HAL_GetTick();

		/* 3. 分情况将新数据压入软件环形队列，情况 A：未发生回卷（线性增长）
         * 数据范围从 g_last_dma_head 一直到 current_dma_head*/
        if (current_dma_head > dev->last_dma_head)
        {
            for (uint16_t i = dev->last_dma_head; i < current_dma_head; i++)
            {
                (void)RingBuffer_Push(&dev->ring_buf, &dev->dma_rx_buf[i]);
            }
        }
        else
        {
        /* 情况 B：发生回卷（Wrap-around）
         * 因为使用的是 DMA 循环模式，写指针到达缓冲区末尾后会跳回 0 继续写。
         * 需要分两段来搬运数据：*/
         	/* 第一段：从上一次的位置一直搬到缓冲区末尾 */
            for (uint16_t i = dev->last_dma_head; i < dev->dma_buf_size; i++)
            {
                (void)RingBuffer_Push(&dev->ring_buf, &dev->dma_rx_buf[i]);
            }
			/* 第二段：从缓冲区头部 0 一直到当前的写指针位置 */
            for (uint16_t i = 0; i < current_dma_head; i++)
            {
                (void)RingBuffer_Push(&dev->ring_buf, &dev->dma_rx_buf[i]);
            }
        }
		/* 4. 更新历史指针，为下一轮轮询做准备 */
        dev->last_dma_head = current_dma_head;
    }

    /* 5. 硬件错误检测与自愈 (SR 寄存器 ORE/NE/FE 联合处理)*/
    uint32_t sr = dev->huart->Instance->SR;
    if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) != RESET)
    {
        volatile uint32_t dummy = dev->huart->Instance->SR;
        dummy = dev->huart->Instance->DR;
        (void)dummy;

        __HAL_UART_CLEAR_OREFLAG(dev->huart);
        __HAL_UART_CLEAR_NEFLAG(dev->huart);
        __HAL_UART_CLEAR_FEFLAG(dev->huart);
		
		/* 如果 DMA 接收因为错误被 HAL 库关闭了，强行重启 DMA */
        if (dev->huart->RxState != HAL_UART_STATE_BUSY_RX)
        {
            HAL_UART_Receive_DMA(dev->huart, dev->dma_rx_buf, dev->dma_buf_size);
        }
    }

	// 检查 DMA 句柄状态兜底
    if (dev->huart->RxState == HAL_UART_STATE_READY)
    {
        HAL_UART_Receive_DMA(dev->huart, dev->dma_rx_buf, dev->dma_buf_size);
    }
}

/**
  * @brief ：从环形队列读取 1 个字节
  */
bool rs485_ReadByte(RS485_Dev_t *dev, uint8_t *pByte)
{
	if (dev == NULL || pByte == NULL)
	{
		return false;
	}
	return RingBuffer_Pop(&dev->ring_buf, pByte);
}

/**
  * @brief ：批量拉取数据
  */
uint16_t rs485_ReadArray(RS485_Dev_t *dev, uint8_t *pData, uint16_t max_len)
{
    uint16_t count = 0;
    while (count < max_len && RingBuffer_Pop(&dev->ring_buf, &pData[count]))
    {
        count++;
    }
    return count;
}

/**
  * @brief ：获取环形队列中可读的字节数
  */
uint16_t rs485_GetRxCount(RS485_Dev_t *dev)
{
    return RingBuffer_GetCount(&dev->ring_buf);
}

/**
  * @brief ：清空接收队列
  */
void rs485_FlushRx(RS485_Dev_t *dev)
{
    RingBuffer_Flush(&dev->ring_buf);
}

/**
  * @brief：异步非阻塞发送 API
  */
bool rs485_SendAsync(RS485_Dev_t *dev, const uint8_t *pData, uint16_t len)
{
    if (pData == NULL || len == 0 || dev->tx_busy)
    {
        return false;
    }

    dev->tx_busy = true;
    RS485_SetRE(dev, true);	// 切换为发送模式
    delay_us(3);	// 硬件建立时间

	/* 启动 DMA 发送 (HAL 库会在发送完成后自动回调上面的 HAL_UART_TxCpltCallback) */
    if (HAL_UART_Transmit_DMA(dev->huart, (uint8_t *)pData, len) != HAL_OK)
    {
        RS485_SetRE(dev, false);
        dev->tx_busy = false;
        return false;
    }
    return true;
}

/**
  * @brief ：HAL 库原生的发送完成回调函数 (完全无需修改 stm32f1xx_it.c)
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	RS485_Dev_t *dev = RS485_GetDevice(huart);

	if (dev != NULL)
    {
        delay_us(3);	// 保持停止位
        RS485_SetRE(dev, false); // 切回接收模式
        dev->tx_busy = false; // 解除发送忙锁定
    }
}

/**
  * @brief ：HAL 库原生的硬件错误回调函数 (防卡死自愈)
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	RS485_Dev_t *dev = RS485_GetDevice(huart);

	if (dev != NULL)
	{
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_NEFLAG(huart);
		__HAL_UART_CLEAR_FEFLAG(huart);

		HAL_UART_Receive_DMA(dev->huart, dev->dma_rx_buf, dev->dma_buf_size);
	}
}

