#include "tpc_slave.h"

static uint8_t tpc_rx_buf[128];

/**
  * @brief  处理昆仑通态 HMI（USART3）侧的请求数据
  * @param  dev: HMI RS485 设备句柄
  */
void App_Process_TPC_Request(RS485_Dev_t *dev)
{
    /* 检查 TPC 总线是否满足 3ms 静默断帧条件 */
    if (RS485_IsFrameReady(dev))
    {
        uint16_t count = rs485_GetRxCount(dev);
        if (count > 0 && count <= sizeof(tpc_rx_buf))
        {
            uint16_t rx_len = rs485_ReadArray(dev, tpc_rx_buf, count);

            /* TODO: 在这里解析昆仑通态发来的 Modbus 从机请求，并组包通过 rs485_SendAsync 回复 */
        }
    }
}

