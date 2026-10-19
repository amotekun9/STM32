/************************************************
 * @file    tpc_slave.h
 * @brief   Modbus RTU从站，适配固定版本rs485.h(MAX485 USART3)
 * @author  MO wenzhi
 * @version V2.0
 * @date    2026.10.17
 * @note    使用rs485.h原生API，不新增任何驱动函数
 ***********************************************/
#include "tpc_slave.h"
#include "bsp_rs485.h"
#include <string.h>

// Modbus RTU 3.5字符静默超时(ms)，9600=5ms，19200=3ms
#define MODBUS_FRAME_INTERVAL_MS    1

static uint8_t  modbus_tx_buf[MODBUS_TX_BUF_SIZE];
uint16_t modbus_holding_reg[MODBUS_HOLD_REG_COUNT] = {0};

// Modbus CRC16 查表
static const uint16_t crc16_table[] = {
0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,0xC601,0x06C0,0x0780,0xC741,0x0500,0xC5C1,0xC481,0x0440,
0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,0xCFC1,0xCE81,0x0E40,0x0A00,0xCAC1,0xCB81,0x0B40,0xC901,0x09C0,0x0880,0xC841,
0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,0x1E00,0xDEC1,0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,
0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,
0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,0xF281,0x3240,0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,
0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,
0x2800,0xE8C1,0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,
0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,
0xA001,0x60C0,0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,0x6600,0xA6C1,0xA781,0x6740,0xA501,0x65C0,0x6480,0xA441,
0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,0xAA01,0x6AC0,0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,
0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,
0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,0xB681,0x7640,0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,0x7080,0xB041,
0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,0x5280,0x9241,0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,
0x9C01,0x5CC0,0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,0x5A00,0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,
0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,0x4A40,0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,
0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,0x8201,0x42C0,0x4380,0x8341,0x4100,0x81C1,0x8081,0x4040
};

static uint16_t modbus_crc16(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while(len--)
    {
        uint8_t idx = crc ^ (*buf++);
        crc >>= 8;
        crc ^= crc16_table[idx];
    }
    return crc;
}

/**
 * @brief Modbus从机轮询任务，放在main的while(1)循环
 * @note  依赖rs485驱动内置环形缓存，调用RS485_Poll刷新DMA
 */
void modbus_slave_task(void)
{
    RS485_Dev_t *dev = RS485_GetDevice(&MAX485_HANDLE);
    if(dev == NULL)
    {
        return;
    }

    uint8_t modbus_rx_buf[MODBUS_RX_BUF_SIZE];
    uint16_t crc_calc, crc_recv;
    uint8_t  slave_addr, func;
    uint16_t reg_start, reg_cnt;
    uint16_t i;

    // 刷新485 DMA与环形队列
    RS485_Poll(dev);

    // 判断是否收到完整帧（驱动内部自带静默断帧检测）
    if(RS485_IsFrameReady(dev))
    {
        // 从驱动环形队列读出整帧
        uint16_t rx_index = rs485_ReadArray(dev, modbus_rx_buf, MODBUS_RX_BUF_SIZE);
        if(rx_index < 4)
        {
            rs485_FlushRx(dev);
            return;
        }

        slave_addr = modbus_rx_buf[0];
        func = modbus_rx_buf[1];
        crc_recv  = (modbus_rx_buf[rx_index - 1] << 8) | modbus_rx_buf[rx_index - 2];
        crc_calc  = modbus_crc16(modbus_rx_buf, rx_index - 2);

        // CRC校验 + 从站地址匹配
        if(crc_calc == crc_recv && slave_addr == MODBUS_SLAVE_ADDR)
        {
            memset(modbus_tx_buf, 0, MODBUS_TX_BUF_SIZE);
            uint8_t tx_len = 0;
            switch(func)
            {
                // 03 读保持寄存器
                case 0x03:
                {
                    reg_start = (modbus_rx_buf[2] << 8) | modbus_rx_buf[3];
                    reg_cnt   = (modbus_rx_buf[4] << 8) | modbus_rx_buf[5];
                    if(reg_start + reg_cnt > MODBUS_HOLD_REG_COUNT || reg_cnt ==0 || reg_cnt>125)
                    {
                        modbus_tx_buf[tx_len++] = slave_addr;
                        modbus_tx_buf[tx_len++] = func | 0x80;
                        modbus_tx_buf[tx_len++] = 0x02; //异常02 非法地址
                        break;
                    }
                    modbus_tx_buf[tx_len++] = slave_addr;
                    modbus_tx_buf[tx_len++] = func;
                    modbus_tx_buf[tx_len++] = reg_cnt * 2;
                    for(i = 0; i < reg_cnt; i++)
                    {
                        modbus_tx_buf[tx_len++] = (modbus_holding_reg[reg_start + i] >> 8) & 0xFF;
                        modbus_tx_buf[tx_len++] = modbus_holding_reg[reg_start + i] & 0xFF;
                    }
                    break;
                }
                // 06 写单个保持寄存器
                case 0x06:
                {
                    reg_start = (modbus_rx_buf[2] << 8) | modbus_rx_buf[3];
                    uint16_t write_val = (modbus_rx_buf[4] << 8) | modbus_rx_buf[5];
                    if(reg_start >= MODBUS_HOLD_REG_COUNT)
                    {
                        modbus_tx_buf[tx_len++] = slave_addr;
                        modbus_tx_buf[tx_len++] = func | 0x80;
                        modbus_tx_buf[tx_len++] = 0x02;
                        break;
                    }
                    modbus_holding_reg[reg_start] = write_val;
                    memcpy(modbus_tx_buf, modbus_rx_buf, 6);
                    tx_len = 6;
                    break;
                }
                // 10 写多个保持寄存器
                case 0x10:
                {
                    reg_start = (modbus_rx_buf[2] << 8) | modbus_rx_buf[3];
                    reg_cnt   = (modbus_rx_buf[4] << 8) | modbus_rx_buf[5];
                    uint8_t byte_len = modbus_rx_buf[6];
                    if( (reg_start + reg_cnt) > MODBUS_HOLD_REG_COUNT || byte_len != reg_cnt*2 || reg_cnt ==0 || reg_cnt>123 )
                    {
                        modbus_tx_buf[tx_len++] = slave_addr;
                        modbus_tx_buf[tx_len++] = func | 0x80;
                        modbus_tx_buf[tx_len++] = 0x02;
                        break;
                    }
                    for(i=0; i<reg_cnt; i++)
                    {
                        uint8_t high = modbus_rx_buf[7 + i*2];
                        uint8_t low  = modbus_rx_buf[7 + i*2 + 1];
                        modbus_holding_reg[reg_start + i] = (high << 8) | low;
                    }
                    modbus_tx_buf[tx_len++] = slave_addr;
                    modbus_tx_buf[tx_len++] = func;
                    modbus_tx_buf[tx_len++] = modbus_rx_buf[2];
                    modbus_tx_buf[tx_len++] = modbus_rx_buf[3];
                    modbus_tx_buf[tx_len++] = modbus_rx_buf[4];
                    modbus_tx_buf[tx_len++] = modbus_rx_buf[5];
                    break;
                }
                // 不支持功能码
                default:
                {
                    modbus_tx_buf[tx_len++] = slave_addr;
                    modbus_tx_buf[tx_len++] = func | 0x80;
                    modbus_tx_buf[tx_len++] = 0x01; //异常01 非法功能码
                    break;
                }
            }

            // 发送应答帧，使用原生rs485_SendAsync
            if(tx_len > 0)
            {
                uint16_t crc_tx = modbus_crc16(modbus_tx_buf, tx_len);
                modbus_tx_buf[tx_len++] = crc_tx & 0x00FF;
                modbus_tx_buf[tx_len++] = (crc_tx >> 8) & 0x00FF;
                rs485_SendAsync(dev, modbus_tx_buf, tx_len);
            }
        }
        // 清空接收环形缓冲区，准备下一帧
        rs485_FlushRx(dev);
    }
}

void modbus_slave_init(void)
{
    memset(modbus_holding_reg, 0, sizeof(modbus_holding_reg));
}




