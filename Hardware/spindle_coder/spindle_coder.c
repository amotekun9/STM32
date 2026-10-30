/************************************************
 * @file    bsp_encoder.c
 * @brief   编码器BSP底层驱动,AB相正交4倍频采集,TIM6更新测速
 * @author  MO wenzhi
 * @version V1.0
 * @date    2026.10.09
 * @note    适配百纳BN5058系列编码,PD12:A PD13:B
 ***********************************************/


#include "spindle_coder.h"
#include "math.h"

extern TIM_HandleTypeDef htim4;
extern UploadData_t g_upload_data;

static Coder_t coder;               // 编码器结构体：存放脉冲、原始速度、滤波后速度
static int16_t enc_last = 0;        // 上一次采样的TIM4计数器值，用于计算delta
int32_t encoder_cnt = 0;            // 单圈脉冲计数（和coder.pos同步）
int32_t encoder_rpm = 0;            // 主轴转速 rpm
int32_t encoder_angle = 0;          // 单圈角度，单位：0.1°，0~3599 代表0.0~359.9°
uint8_t encoder_z_flag = 0;         // Z相标记：1表示收到Z脉冲，等待采样点清零
uint8_t pinch_z_flag = 0;
uint16_t tim6_sample_ms = 1;

/**
 * @brief 编码器初始化
 * @note 初始化变量、启动TIM4编码器模式、启动TIM6定时采样中断
 */
void Coder_Init(void)
{
    coder.pos = 0;
    coder.speed_raw = 0;
    coder.speed_filt = 0;
    enc_last = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    encoder_cnt = 0;
    encoder_rpm = 0;
    encoder_angle = 0;
    encoder_z_flag = 0;
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);   // 启动TIM4编码器
    HAL_TIM_Base_Start_IT(&htim6);                                 // 启动TIM6定时采样中断
}

/**
 * @brief 编码器软件手动清零
 * @note 屏/上位机下发清零指令时调用，重置单圈角度、速度、计数器
 */
void Coder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);   // 硬件TIM4计数器清零
    coder.pos = 0;
    coder.speed_raw = 0;
    coder.speed_filt = 0;
    enc_last = 0;
    encoder_cnt = 0;
    encoder_rpm = 0;
    encoder_angle = 0;
    encoder_z_flag = 0;
}

/**
 * @brief Z相外部中断回调函数
 * @note Z脉冲到来，只置标志位，不在EXTI中断里执行清零，避免异步时序竞争
 */
void Coder_Z_Phase_Callback(void)
{
    encoder_z_flag = 1;   // 给编码器清零用
    pinch_z_flag = 1;     // 给夹棍任务触发用
    // DebugPrint("Z PULSE flag set");
}


/**
 * @brief TIM6定时中断回调，编码器采样、测速、角度计算
 * @note 清零动作放在这里，和读取硬件计数器同步，消除清零到采样之间多出脉冲的问题
 */
void Coder_Callback(void)
{
    int16_t enc_now = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t delta = enc_now - enc_last;
    if(delta > 16384)
    {
        delta -= 65536;
    }
    else if(delta < -16384)
    {
        delta += 65536;
    }
    // Z标记：编码器清零逻辑，只识别 encoder_z_flag
    if(encoder_z_flag == 1)
    {
        __HAL_TIM_SET_COUNTER(&htim4, 0);
        coder.pos = 0;
        enc_last = 0;
        encoder_angle = 0;
        encoder_z_flag = 0;   // 只清编码器自己的flag
        return;
    }
    coder.pos += delta;
    coder.speed_raw = delta;
    coder.speed_filt = (coder.speed_filt * (CODER_FILTER_K - 1) + delta) / CODER_FILTER_K;
    enc_last = enc_now;
    encoder_cnt = coder.pos;

    encoder_rpm = abs(coder.speed_filt) * 65934 / (4 * ENC_PULSE_PER_ROUND);
    g_upload_data.encoder_rpm_up = encoder_rpm;
    encoder_angle = (coder.pos * 3600L) / (4L * ENC_PULSE_PER_ROUND)/10;
    g_upload_data.encoder_angle_up = abs(encoder_angle);
}

/**
 * @brief EXTI外部中断总回调
 * @param GPIO_Pin 触发中断的引脚号
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_12)
    {
        Coder_Z_Phase_Callback();
        return;
    }
}

/**
 * @brief 设置TIM6编码器采样周期，单位ms
 */
void TIM6_SetSampleMs(uint16_t ms)
{
    if(ms < 1) ms = 1;
    if(ms > 900) ms = 900;
    tim6_sample_ms = ms;
    uint32_t arr = (uint32_t)ms * 72 - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim6, (uint16_t)arr);
    htim6.Instance->EGR = TIM_EGR_UG;
}


