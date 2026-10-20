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
#include "math.h"
#include "tpc_slave.h"
#include "app_main.h"
/****************宏定义区***********************/


/**********************************************/

/****************全局变量区***********************/
uint8_t encoder_A_last = 0;
uint8_t encoder_B_last = 0;
int32_t encoder_cnt = 0;
uint8_t encoder_z_flag = 0;

int32_t encoder_zero_offset = 0;//Z相零点偏移量

float encoder_angle = 0.0f;
float encoder_rpm = 0.0f;
static int32_t encoder_cnt_last = 0;//上一次采样的脉冲值

uint8_t encoder_sample_flag = 0;
extern UploadData_t g_upload_data;
/***********************************************/


/****************静态函数声明***********************/


/**********************************************/


/***************函数定义区***********************/

/**************************************************
 * @brief  编码器硬件初始化
 * @param[in]  None
 * @return  无
 * @note   PC10:A相、PC11:B相配置为双边沿外部中断，开启内部上拉；
 *         PC12:Z相配置为下降沿外部中断，开启内部上拉；
 *         初始化读取AB相初始电平，用于正交判向。
 * @warning 调用前需保证系统时钟已初始化。
 *************************************************/
void Spl_Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE(); /**< 使能GPIOC外设时钟 */

    /* PC10 A相，双边沿外部中断，开启内部上拉 */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PC11 B相，双边沿外部中断，开启内部上拉 */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PC12 Z相，下降沿外部中断，开启内部上拉 */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* 读取AB相初始电平，保存为历史状态，用于正交4倍频判向 */
    encoder_A_last = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);
    encoder_B_last = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);
}

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
    int32_t cnt_raw = Spl_Encoder_GetCount(); //原始总脉冲
    int32_t cnt_now = cnt_raw - encoder_zero_offset; //减去零点偏移，作为相对单圈脉冲

    int32_t delta_p = cnt_raw - encoder_cnt_last; //【重点！delta_p使用原始总脉冲，不受Z相偏移影响！】
    encoder_cnt_last = cnt_raw; //保存原始总脉冲，用于下一次delta计算

    float angle_temp = cnt_now * (360.0f / ENCODER_PUL_PER_ROUND);
    // 单圈角度0~360
    encoder_angle = fmod(angle_temp, 360.0f);
    if(encoder_angle < 0.0f)
    {
        encoder_angle += 360.0f;
    }
    g_upload_data.encoder_angle_up = (uint16_t)encoder_angle; 

    /* M法计算转速，单位rpm */
    float T = sample_ms / 1000.0f;
    encoder_rpm = ( (float)delta_p / (ENCODER_PUL_PER_ROUND * T) ) * 60.0f;
    g_upload_data.encoder_rpm_up = fabsf(encoder_rpm);
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
    else if(GPIO_Pin == GPIO_PIN_12)
    {
        //Z相到来，设置零点偏移，不清原始脉冲计数器！！！
        encoder_zero_offset = encoder_cnt;
    }
}

/**************************************
 * @brief  编码器定时采样处理函数
 * @param[in]  None
 * @return  无
 * @note    由TIM6每20ms调用一次;执行转速计算并做一阶低通滤波
 * @warning 运行在中断上下文,禁止调用耗时函数、阻塞接口
 *************************************/
void Spl_Encoder_Timer_ISR(void)
{
    static float rpm_filter = 0.0f;
    const float filter_k = 0.1f;
    Spl_Encoder_CalcSpeedAndAngle(100);  // 你的采集周期是100ms，同步改掉20

    // 只限制最大绝对值
    if(encoder_rpm > 3000)
    {
        encoder_rpm = rpm_filter;
    }
    if(encoder_rpm < -3000)
    {
        encoder_rpm = rpm_filter;
    }

    rpm_filter = rpm_filter * (1.0f - filter_k) + encoder_rpm * filter_k;
    encoder_rpm = rpm_filter;
}





