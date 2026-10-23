/************************************************
 * @file    bsp_encoder.c
 * @brief   编码器BSP底层驱动,AB相正交4倍频采集,TIM7定时测速
 * @author  MO wenzhi
 * @version V1.0
 * @date    2026.10.09
 * @note    适配百纳BN5058系列编码,PC10:A PC11:B
 ***********************************************/

/* ====================头文件包含区==================== */
#include "spindle_coder.h"

extern UploadData_t g_upload_data;

/****************宏定义区***********************/
#define ANGLE_FULL_ROUND     360        // 一圈360度
#define RPM_CONST_60         60
#define RPM_CONST_1000       1000
#define FILTER_SCALE         10
#define ANGLE_FIX_SCALE      100        // 角度定点放大系数

/****************全局变量区***********************/
uint8_t encoder_A_last = 0;
uint8_t encoder_B_last = 0;
int32_t encoder_cnt = 0;
uint8_t encoder_z_flag = 0;

int32_t encoder_zero_offset = 0;//Z相零点偏移量

int16_t encoder_angle = 0;
int16_t encoder_rpm = 0;
static int32_t encoder_cnt_last = 0;//上一次采样的脉冲值

uint8_t encoder_sample_flag = 0;

/***************函数定义区***********************/
/*********************************************
 * @brief  获取编码器当前脉冲计数值
 * @param[in]  None
 * @return int32_t 编码器脉冲计数，正数代表正转，负数代表反转
 * @note   采用关全局中断方式做原子读取，避免读取过程中被外部中断修改计数，造成数据错乱
 * @warning 短时关闭全局中断，关中断区间仅执行简单变量拷贝，无耗时操作
 *********************************************/
int32_t Spl_Encoder_GetCount(void)
{
    int32_t val;
    __disable_irq();
    val = encoder_cnt;
    __enable_irq();
    return val;
}


/*************************************
 * @brief  清零编码器脉冲计数值
 * @param[in]  None
 * @return  无
 * @note   修改全局计数变量，操作前后关开中断，保证读写原子性
 * @warning 短时关闭全局中断，不要在关中断区间执行耗时操作
 *************************************/
void Spl_Encoder_ResetCount(void)
{
    __disable_irq();    /**< 关闭全局中断，防止修改encoder_cnt时被中断打断 */
    encoder_cnt = 0;    /**< 编码器脉冲计数清零 */
    __enable_irq();     /**< 恢复全局中断 */
}


/******************************************
 * @brief  计算编码器角度与转速(多圈)
 * @param[in]  sample_ms 测速采样周期，单位ms
 * @return  无
 * @note    M法测速，4倍频正交采集；基于当前脉冲值计算角度，脉冲差值计算转速
 * @warning 本函数可在中断上下文调用，内部不允许阻塞操作
 ******************************************/
void Spl_Encoder_CalcSpeedAndAngle(uint32_t sample_ms)
{
    int32_t cnt_raw     = 0;
    int32_t cnt_now     = 0;
    int32_t delta_p     = 0;
    int32_t angle_temp  = 0;
    int32_t angle_all   = 0;
    int32_t rpm_tmp     = 0;
    int16_t rpm         = 0;
    uint16_t rpm_abs    = 0;

    cnt_raw = Spl_Encoder_GetCount();   // 读取编码器当前脉冲计数值
    cnt_now = cnt_raw;
    delta_p = cnt_raw - encoder_cnt_last;    // 计算采样间隔内脉冲变化量
    encoder_cnt_last = cnt_raw;              // 保存当前脉冲，供下一次差值计算

    // ========== 角度计算 纯整型 0~359 ==========
    angle_temp = cnt_now * 360;                          // 脉冲换算总角度
    angle_all = angle_temp / ENCODER_PUL_PER_ROUND;      // 得到累计总角度
    encoder_angle = (int16_t)(angle_all % 360);          // 取模得到单圈角度
    if (encoder_angle < 0)
    {
        encoder_angle += 360;                            // 负角度补偿，保证0~359
    }
    g_upload_data.encoder_angle_up = (uint16_t)encoder_angle;  // 角度存入上报结构体

    // ========== M 法转速计算 纯整型，不限幅 ==========
    delta_p = abs(delta_p);                                        // 脉冲差值取绝对值，只算转速大小
    rpm_tmp = delta_p * 60 * 1000;                                 // M法转速公式分子
    rpm_tmp = rpm_tmp / (ENCODER_PUL_PER_ROUND * sample_ms);       // 除以分母得到转速
    rpm = (int16_t)rpm_tmp;
    encoder_rpm = rpm;                                             // 全局转速变量，供给电机控制
    rpm_abs = (rpm >= 0) ? (uint16_t)rpm : (uint16_t)(-rpm);       // 转速绝对值，用于界面上报
    g_upload_data.encoder_rpm_up = rpm_abs;                         // 转速存入上报结构体
}

/**************************************
 * @brief  GPIO外部中断回调函数
 * @param[in]  GPIO_Pin触发本次中断的GPIO管脚
 * @return  无
 * @note    PC10为A相、PC11为B相，实现正交4倍频判向计数；PC12为Z相，Z脉冲触发时清零编码器数值
 * @warning 运行在中断上下文,禁止调用耗时函数、阻塞接口
 *************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t A_now, B_now;
    encoder_sample_flag = 1; /**< 标记编码器产生边沿采样事件 */
	
	if(GPIO_Pin == GPIO_PIN_12)
    {
        //Z相到来，清零原始脉冲计数器
        encoder_cnt = 0;
        //【关键适配】同步把上一次记录也清零，消除巨大delta跳变
        encoder_z_flag = 1;
        return;
    }
	
    if(GPIO_Pin == GPIO_PIN_10 || GPIO_Pin == GPIO_PIN_11)
    {
        A_now = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10); /**< 读取当前A相电平 */
        B_now = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11); /**< 读取当前B相电平 */
        /* 4倍频正交判向逻辑，通过前后AB相电平组合判断转向并累加脉冲计数 */
        if(encoder_A_last == 0)
        {
            if(encoder_B_last == 0)
            {
                if(A_now == 1 && B_now == 0) encoder_cnt++;
                if(A_now == 0 && B_now == 1) encoder_cnt--;
            }
            else
            {
                if(A_now == 0 && B_now == 0) encoder_cnt++;
                if(A_now == 1 && B_now == 1) encoder_cnt--;
            }
        }
        else
        {
            if(encoder_B_last == 0)
            {
                if(A_now == 1 && B_now == 1) encoder_cnt++;
                if(A_now == 0 && B_now == 0) encoder_cnt--;
            }
            else
            {
                if(A_now == 0 && B_now == 1) encoder_cnt++;
                if(A_now == 1 && B_now == 0) encoder_cnt--;
            }
        }
        encoder_A_last = A_now; /**< 更新A相历史电平，用于下一次边沿判断 */
        encoder_B_last = B_now; /**< 更新B相历史电平，用于下一次边沿判断 */
    }
}

/**************************************
 * @brief  编码器定时采样处理函数
 * @param[in]  None
 * @return  无
 * @note    由TIM6每1ms调用一次;执行转速计算并做一阶低通滤波
 * @warning 运行在中断上下文,禁止调用耗时函数、阻塞接口
 *************************************/
void Spl_Encoder_Timer_ISR(void)
{
    Spl_Encoder_CalcSpeedAndAngle(1);  // 定时器采集周期为1ms
	scissor_window_check(2000,60);  // 剪刀电机位置检测，2000us脉冲间隔，60个编码器脉冲偏移量
}





