#include "biss_coder.h"

//只保留角度值16bit
uint16_t ReadAngle(uint32_t read_biss_frame)
{
	uint16_t read_angle = 0;
    read_angle = (uint16_t)(read_biss_frame >> 8);

    return read_angle;
}

//极限速率读取，只保留角度值16bit
uint32_t ReadAngle_UltraFast(void)
{
    uint32_t val = 0;
    uint8_t timeout_cnt = 0;

    // 1. 触发起始下降沿
    BISS_CLK_LOW();
    delay_ns(BISS_CLK_HALF_PERIOD_NS);

    // 2. 28 位高速数据采样循环
    for(uint8_t i = 0; i < 28; i++)
    {
        val <<= 1;

        BISS_CLK_HIGH();
        delay_ns(BISS_CLK_HALF_PERIOD_NS); 

        if( BISS_DAT_STATUS() )
        {
            val |= 0x01;
        }

        BISS_CLK_LOW();
        delay_ns(BISS_CLK_HALF_PERIOD_NS); 
    }

    // 3. 恢复时钟线为空闲高电平
    BISS_CLK_HIGH();

    // 4. 纳秒级动态死区检测 (Timeout < 10us)
    // 每 500ns 轮询一次，Data 拉高立刻退出，超时上限设为 12us 防止死循环
    while((!BISS_DAT_STATUS()) && (timeout_cnt < 200))
    {
        delay_ns(60);
        timeout_cnt++;
    }

    // 5. 提取 16 位有效角度 (移除末尾 8 位 Error, Warn, CRC)
    val >>= 8;
    val &= 0x0000FFFF;

    return val;
}

/*极限速率读取一帧数据，共28bit:
1bit Idle(高电平),1bit Ack(低),1bit Start(高),1bit CDS(低),16bit Position,1bit Error,1bit Warn,6bit CRC
*/
uint32_t ReadBissFrame_UltraFast(void)
{
	uint32_t val = 0;
	uint8_t timeout_cnt = 0;

	// 1. 触发起始下降沿
	BISS_CLK_LOW();
	delay_ns(BISS_CLK_HALF_PERIOD_NS);
	// 2. 28 位高速数据采样循环
	for(uint8_t i = 0; i < 28; i++)
	{
	    val <<= 1;

	    BISS_CLK_HIGH();
	    delay_ns(BISS_CLK_HALF_PERIOD_NS); 
	    if( BISS_DAT_STATUS() )
	    {
	        val |= 0x01;
	    }

	    BISS_CLK_LOW();
	    delay_ns(BISS_CLK_HALF_PERIOD_NS); 
	}

	// 3. 恢复时钟线为空闲高电平
	BISS_CLK_HIGH();

	// 4. 纳秒级动态死区检测 (Timeout < 10us)
	// 每 500ns 轮询一次，Data 拉高立刻退出，超时上限设为 12us 防止死循环
	while((!BISS_DAT_STATUS()) && (timeout_cnt < 200))
	{
	    delay_ns(60);
	    timeout_cnt++;
	}

	return val;
}

/**
 * @brief  计算 BiSS-C 协议的 CRC6
 * @param  data_18bit: 提取出的 18 bit 核心数据 (ST + Error + Warn)
 * @retval 6 bit 的 CRC 计算结果
 */
uint8_t BiSSC_CalcCRC6(uint32_t read_biss_frame) 
{
    uint8_t crc = 0x00; // 初始余数为 0
	read_biss_frame= read_biss_frame >> 6;  //先将收到的校验位移除，对剩下的数据进行校验核对
			
    // 依次处理 18 bit 数据，高位 (MSB) 在前
    for (int8_t i = 17; i >= 0; i--)
	{
	    uint8_t bit = (read_biss_frame >> i) & 0x01; // 提取当前数据位
	    uint8_t crc_msb = (crc >> 5) & 0x01;    // 提取当前 CRC 的最高位 (bit 5)

	    crc <<= 1; // CRC 寄存器左移 1 位

	    // 如果数据位与 CRC 最高位异或为 1，则执行多项式异或
	    if (crc_msb ^ bit)
		{
	        // 多项式 X^6 + X^1 + 1 (二进制: 1000011)
	        // 因为最高位在移位时已经溢出处理，所以这里只与低 6 位 0x03 (000011) 异或
	        crc ^= 0x03; 
	    }
    }
    
    // 最终结果按位取反，并通过 0x3F 掩码保留低 6 bit
    return (~crc) & 0x3F;
}

/**
 * @brief  校验一帧完整的 BiSS-C 数据
 * @param  raw_frame: 接收到的 32 bit 完整帧 (例如 0x0A9A47E7)
 * @retval true: 校验通过 / false: 校验失败
 */
bool BiSSC_VerifyFrame(uint32_t read_biss_frame) 
{
	// 1. 提取帧尾自带的 6 bit CRC
	uint8_t received_crc = (uint8_t)read_biss_frame & 0x3F;
	// 2. 计算本地 CRC
	uint8_t calculated_crc = BiSSC_CalcCRC6(read_biss_frame);
	// 3. 对比结果并返回
	return (received_crc == calculated_crc);
}

