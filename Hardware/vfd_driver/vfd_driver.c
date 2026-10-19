#include "vfd_driver.h"

/* 变频器专属的接收解析缓冲区 (静态私有，安全隔离) */
static uint8_t vfd_rx_buf[128];

void App_Process_VFD_Response(RS485_Dev_t *dev)
{
    ModbusFrame_t vfd_frame;

    /* 检查变频器总线是否满足 3ms 静默断帧条件 */
    if (RS485_IsFrameReady(dev))
    {
        uint16_t count = rs485_GetRxCount(dev);
        if (count > 0 && count <= sizeof(vfd_rx_buf))
        {
            uint16_t rx_len = rs485_ReadArray(dev, vfd_rx_buf, count);
			
            /* 调用通用 Modbus 协议层解析响应 */
            if (Modbus_Master_ParseResponse(vfd_rx_buf, rx_len, &vfd_frame))
            {
                if (!vfd_frame.is_error && vfd_frame.func_code == MODBUS_FC_READ_HOLDING_REG)
                {
                    /* TODO: 在这里直接更新变频器状态数据供全局或上层调用 */
                }
            }
        }
    }
}

/**
  * @brief  控制变频器运行动作 (正转/反转/停机/复位)
  * @param  slave_addr: 变频器站号 (通常为 1)
  * @param  cmd: 命令码 (如 VFD_CMD_FORWARD_RUN 正转)
  */
bool VFD_ControlRun(RS485_Dev_t *dev, uint8_t slave_addr, uint16_t cmd)
{
    static uint8_t s_vfd_tx_buf[8]; // 使用 static 存储在静态区，保证 DMA 传输期间内存稳定
    /* 写地址 0x2000，写入控制命令 */
    uint16_t len = Modbus_Master_BuildWriteReg(s_vfd_tx_buf, slave_addr, VFD_ADDR_CONTROL_CMD, cmd);
    return rs485_SendAsync(dev, s_vfd_tx_buf, len); // 结合底层 RS485 非阻塞发送
}

/**
  * @brief  设定变频器目标频率 / 通讯设定值
  * @param  slave_addr: 变频器站号
  * @param  freq_val: 设定值 (-10000 ~ 10000 对应 -100.00% ~ 100.00%)
  */
bool VFD_SetFrequency(RS485_Dev_t *dev, uint8_t slave_addr, int16_t freq_val)
{
    static uint8_t s_vfd_tx_buf[8];
    /* 写地址 0x1000，写入目标频率 */
    uint16_t len = Modbus_Master_BuildWriteReg(s_vfd_tx_buf, slave_addr, VFD_ADDR_SET_FREQ, (uint16_t)freq_val);
    return rs485_SendAsync(dev, s_vfd_tx_buf, len);
}

/**
  * @brief  读取变频器当前运行状态
  * @param  slave_addr: 变频器站号
  */
bool VFD_ReadStatus(RS485_Dev_t *dev, uint8_t slave_addr)
{
    static uint8_t s_vfd_tx_buf[8];
    /* 读地址 0x3000，读取 1 个寄存器 */
    uint16_t len = Modbus_Master_BuildReadReg(s_vfd_tx_buf, slave_addr, VFD_ADDR_RUN_STATUS, 1);
    return rs485_SendAsync(dev, s_vfd_tx_buf, len);
}
