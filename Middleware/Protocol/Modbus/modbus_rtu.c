#include "modbus_rtu.h"

/**
  * @brief  查表/算表法计算 Modbus CRC16 (低字节在前，高字节在后)
  * @note   严格依据手册 156 页给出的算法实现
  */
uint16_t Modbus_CRC16(const uint8_t *pBuffer, uint16_t length)
{
    uint16_t crc_value = 0xFFFF; // 预置 16 位寄存器为 0xFFFF

    while (length--)
    {
        crc_value ^= *pBuffer++; // 8位数据与 CRC 低8位相异或
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc_value & 0x0001) // 检查最低位是否为 1
            {
                crc_value = (crc_value >> 1) ^ 0xA001; // 右移并与 0xA001 异或
            }
            else
            {
                crc_value = (crc_value >> 1); // 直接右移
            }
        }
    }
    return crc_value; // 返回 CRC 结果
}

/**
  * @brief  构建 0x03 读保持寄存器命令帧 (主站发送)
  * @param  pTxBuf: 目标打包缓冲区 (至少 8 字节)
  * @param  slave_addr: 从机地址 (1~247)
  * @param  reg_addr: 寄存器起始地址
  * @param  reg_num: 读取的寄存器个数 (例如 1)
  * @retval 打包后的总字节长度 (标准读命令固定 8 字节)
  */
uint16_t Modbus_Master_BuildReadReg(uint8_t *pTxBuf, uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_num)
{
    if (pTxBuf == NULL)
	{
		return 0;
    }

    pTxBuf[0] = slave_addr;                            /* 从机地址 */
    pTxBuf[1] = MODBUS_FC_READ_HOLDING_REG;            /* 功能码 0x03 */
    pTxBuf[2] = (uint8_t)(reg_addr >> 8);              /* 寄存器地址高字节 */
    pTxBuf[3] = (uint8_t)(reg_addr & 0xFF);            /* 寄存器地址低字节 */
    pTxBuf[4] = (uint8_t)(reg_num >> 8);               /* 寄存器个数高字节 */
    pTxBuf[5] = (uint8_t)(reg_num & 0xFF);             /* 寄存器个数低字节 */

    uint16_t crc = Modbus_CRC16(pTxBuf, 6);
    pTxBuf[6] = (uint8_t)(crc & 0xFF);                 /* CRC 低字节在前 */
    pTxBuf[7] = (uint8_t)(crc >> 8);                   /* CRC 高字节在后 */

    return 8;
}

/**
  * @brief  构建 0x06 写单个寄存器命令帧 (主站发送)
  * @param  pTxBuf: 目标打包缓冲区 (至少 8 字节)
  * @param  slave_addr: 从机地址 (1~247)
  * @param  reg_addr: 目标寄存器地址
  * @param  reg_val: 写入的 16 位数值
  * @retval 打包后的总字节长度 (标准写命令固定 8 字节)
  */
uint16_t Modbus_Master_BuildWriteReg(uint8_t *pTxBuf, uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_val)
{
    if (pTxBuf == NULL)
	{
		return 0;
    }

    pTxBuf[0] = slave_addr;                            /* 从机地址 */
    pTxBuf[1] = MODBUS_FC_WRITE_SINGLE_REG;           /* 功能码 0x06 */
    pTxBuf[2] = (uint8_t)(reg_addr >> 8);              /* 寄存器地址高字节 */
    pTxBuf[3] = (uint8_t)(reg_addr & 0xFF);            /* 寄存器地址低字节 */
    pTxBuf[4] = (uint8_t)(reg_val >> 8);               /* 写入数据高字节 */
    pTxBuf[5] = (uint8_t)(reg_val & 0xFF);             /* 写入数据低字节 */

    uint16_t crc = Modbus_CRC16(pTxBuf, 6);
    pTxBuf[6] = (uint8_t)(crc & 0xFF);                 /* CRC 低字节 */
    pTxBuf[7] = (uint8_t)(crc >> 8);                   /* CRC 高字节 */

    return 8;
}

/**
  * @brief  解析变频器/从机返回的应答帧
  * @param  pRxBuf: 接收缓冲区首地址
  * @param  rx_len: 实际收到的数据帧长度
  * @param  pFrame: 解析输出结果结构体
  * @retval true: CRC校验成功且解析完成 | false: 帧校验错或格式有误
  */
bool Modbus_Master_ParseResponse(const uint8_t *pRxBuf, uint16_t rx_len, ModbusFrame_t *pFrame)
{
    if (pRxBuf == NULL || pFrame == NULL || rx_len < 5)
    {
        return false;
    }

    /* 1. 强效 CRC 校验拦截 */
    uint16_t cal_crc = Modbus_CRC16(pRxBuf, rx_len - 2);
    uint16_t recv_crc = pRxBuf[rx_len - 2] | (pRxBuf[rx_len - 1] << 8);

    if (cal_crc != recv_crc)
    {
        return false; // CRC 校验不匹配，弃帧
    }

    /* 2. 基础信息提取 */
    pFrame->slave_addr = pRxBuf[0];      /* 从机地址 */
    pFrame->func_code  = pRxBuf[1];      /* 功能码 */
    pFrame->is_error   = false;
    pFrame->error_code = 0;

    /* 3. 判断是否为从机异常应答 (功能码最高位置 1，例如 0x83 或 0x86) */
    if (pFrame->func_code & 0x80)
    {
        pFrame->is_error   = true;
        pFrame->error_code = pRxBuf[2];  /* 错误代码 (01:指令码错 02:地址错 03:数据错 04:无法处理) */
        return true;
    }

    /* 4. 正常响应帧解析 */
    switch (pFrame->func_code)
    {
        case MODBUS_FC_READ_HOLDING_REG: /* 0x03 读响应 */
        {
			/* 1. 解析双字节的数据字节数 (2n) */
            uint16_t byte_count = pRxBuf[2] <<8 | pRxBuf[3]; /* 字节数 (2n) */
			uint16_t reg_num = byte_count / 2; // 计算寄存器个数 n
			
			/* 2. 严格校验总长度：目标地址(1) + 功能码(1) + 字节数字段(2) + 数据区(byte_count) + CRC校验(2) */
            if (rx_len != (4 + byte_count + 2)) 
			{
				return false;
			}
			
        	/* 3. 校验字节数合法性：不能为 0，必须为偶数，且不能超过结构体缓冲区最大限制 */
			if ((byte_count == 0) || (byte_count % 2 != 0) || (reg_num > MAX_MODBUS_REG_COUNT))
            {
                return false;
            }

			/* 4. 循环提取所有寄存器的 16 位数值 (大端模式：高字节在前，低字节在后) */
			for (uint8_t i = 0; i < reg_num; i++)
			{
				/* 数据区从索引 4 开始，每个寄存器占 2 个字节 */
				uint16_t hi = pRxBuf[4 + (i * 2)];
				uint16_t lo = pRxBuf[5 + (i * 2)];
				pFrame->reg_value[i] = (uint16_t)((hi << 8) | lo);
			}

            break;
        }

        case MODBUS_FC_WRITE_SINGLE_REG: /* 0x06 写响应 (原样返回) */
        {
            if (rx_len < 8)
			{
				return false;
			}
            pFrame->reg_addr  = (uint16_t)(pRxBuf[2] << 8) | pRxBuf[3]; /* 确认写入的寄存器地址 */
            pFrame->reg_value[0] = (uint16_t)(pRxBuf[4] << 8) | pRxBuf[5]; /* 确认写入的数值 */
            break;
        }

        default:
            return false;
    }

    return true;
}


