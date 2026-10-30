/************************************************
 * @file    bsp_encoder.c
 * @brief   编码器BSP底层驱动,AB相正交4倍频采集,TIM7定时测速
 * @author  MO wenzhi
 * @version V1.0
 * @date    2026.10.09
 * @note    适配百纳BN5058系列编码,PC10:A PC11:B
 ***********************************************/
 
#include "stm32f1xx_hal.h"
#include "scissor_driver.h"
#include "tpc_slave.h"
#include <stdlib.h>
#include "bsp_delay.h"

#ifndef __SPINDLE_CODER_H
#define __SPINDLE_CODER_H
#include "app_main.h"

#define CODER_FILTER_K     4        // 编码器速度一阶滤波系数，数值越大滤波越强、响应越慢
#define ENC_PULSE_PER_ROUND 360     // 编码器单圈原始脉冲数（四倍频前）
#define RPM_SWITCH_THRESHOLD  50      // 切换阈值 50rpm
#define CODER_SAMPLE_MS     30U    // 采样周期 1ms

/**
 * @brief 编码器数据结构体
 */
typedef struct
{
    int32_t pos;         // 单圈累计脉冲位置
    int32_t speed_raw;   // 原始速度，本次采样间隔脉冲增量
    int32_t speed_filt;  // 滤波平滑后的速度
}Coder_t;

extern int32_t encoder_cnt;        // 主轴脉冲计数，和coder.pos同步
extern int32_t encoder_rpm;         // 主轴转速，单位 rpm
extern int32_t encoder_angle;      // 主轴角度，0~3600，放大10倍，代表0.1°精度

extern uint8_t encoder_z_flag;     // Z相标记位：1=收到Z脉冲，等待采样点清零
extern uint8_t pinch_z_flag;
extern uint16_t tim6_sample_ms;

void Coder_Init(void);              // 编码器初始化，启动TIM编码器、TIM6定时采样
void Coder_Reset(void);             // 编码器手动清零，重置位置、角度、速度
void Coder_Callback(void);          // TIM6定时采样回调，计算脉冲、转速、角度
void Coder_Z_Phase_Callback(void);   // Z相中断处理函数，仅置标记，不在中断内清零
void TIM6_SetSampleMs(uint16_t ms);

#endif



