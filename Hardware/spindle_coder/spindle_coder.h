/************************************************
 * @file    bsp_encoder.c
 * @brief   编码器BSP底层驱动,AB相正交4倍频采集,TIM7定时测速
 * @author  MO wenzhi
 * @version V1.0
 * @date    2026.10.09
 * @note    适配百纳BN5058系列编码,PC10:A PC11:B
 ***********************************************/
 
#ifndef _SPINDLE_CODER_H
#define _SPINDLE_CODER_H
#include "scissor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "spindle_coder.h"
	
#define ENCODER_LINE     360U   //增量式光电编码器线数
#define ENCODER_PUL_PER_ROUND  (ENCODER_LINE * 4U)  //4倍频


extern int32_t encoder_cnt;
extern float encoder_angle;     //角度 单位°
extern float encoder_rpm;       //转速 单位 r/min
extern uint8_t encoder_sample_flag;
extern uint8_t encoder_z_flag;


void Spl_Encoder_Init(void);
int32_t Spl_Encoder_GetCount(void);
void Spl_Encoder_ResetCount(void);
void Spl_Encoder_CalcSpeedAndAngle(uint32_t sample_ms); //sample_ms 采样周期,单位ms
void Spl_Encoder_Timer_ISR(void);
	
	

#ifdef __cplusplus
}
#endif

#endif /* __encoder_H */ 

