#ifndef _BISS_CODER_H
#define _BISS_CODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "stm32f1xx_hal.h"
#include "bsp_delay.h"
#include "bsp_usart.h"

#define BISS_CLK_HIGH()      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define BISS_CLK_LOW()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
#define BISS_DAT_STATUS()  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)

// ---------------- 极限速率参数配置 ----------------
// 半周期 100ns 对应 5MHz 时钟频率 (28bit 数据只需 5.6us)
// 如果信号受干扰，可微调为 200ns ~ 250ns (对应 2.5MHz ~ 2MHz)
#define BISS_CLK_HALF_PERIOD_NS   50


uint16_t ReadAngle(uint32_t read_biss_frame);//只保留角度值16bit
uint32_t ReadAngle_UltraFast(void);		//极限速率读取，只保留角度值
uint32_t ReadBissFrame_UltraFast(void);		//极限速率读取，保留角度值以及Error、Warn、CRC
uint8_t BiSSC_CalcCRC6(uint32_t read_biss_frame);		//计算 BiSS-C 协议的 CRC6
bool BiSSC_VerifyFrame(uint32_t read_biss_frame);		//校验一帧完整的 BiSS-C 数据

#ifdef __cplusplus
}
#endif

#endif /* _BISS_CODER_H */ 

