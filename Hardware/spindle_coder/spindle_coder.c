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
extern CIA402_Node_t g_servo_node_1,g_servo_node_2;
static Coder_t coder;               // 编码器结构体：存放脉冲、原始速度、滤波后速度
static int16_t enc_last = 0;        // 上一次采样的TIM4计数器值，用于计算delta
int32_t encoder_cnt = 0;            // 单圈脉冲计数（和coder.pos同步）
int32_t encoder_rpm = 0;            // 主轴转速 rpm
int32_t encoder_angle = 0;          // 单圈角度，单位：0.1°，0~3599 代表0.0~359.9°
uint8_t encoder_z_flag = 0;         // Z相标记：1表示收到Z脉冲，等待采样点清零
uint8_t pinch_z_flag = 0;
uint16_t tim6_sample_ms = 1;


volatile uint8_t encoder_z_flag_cnt = 0;        
//static int16_t z_flag_save_tim4 = 0;


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
	// 关闭TIM6更新中断，防止和TIM6回调抢变量，临界区保护
	__HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);

	// 编码器零点清零（这里就是原来TIM6里面那一堆清零）
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	coder.pos = 0;
	enc_last = 0;
	encoder_angle = 0;
	encoder_z_flag_cnt ++;
	if (encoder_z_flag_cnt ==3)
	{
		encoder_z_flag_cnt = 2;
	}	
	encoder_z_flag = 1;   // 现在这个flag只做标记，TIM6里面不再用它清零
	pinch_z_flag = 1;	  // 夹棍触发标志保留不变

	// 打开TIM6中断
	__HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
}

/**
 * @brief TIM6定时中断回调，编码器采样、测速、角度计算
 * @note 清零动作放在这里，和读取硬件计数器同步，消除清零到采样之间多出脉冲的问题
 */
 void Coder_Callback(void)
{
    // 读取TIM4编码器当前硬件计数值（16位）
    int16_t enc_now = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
	int16_t delta = enc_now - enc_last;
	delta = enc_now - enc_last;

    // 16位计数器溢出修正：TIM4编码器是16位，0~65535循环
    // 正向溢出：delta > 16384，说明计数器从0翻到65535，减去65536得到真实增量
    // 反向溢出：delta < -16384，说明计数器从65535翻到0，加上65536得到真实减量
    if(delta > 16384)
    {
        delta -= 65536;
    }
    else if(delta < -16384)
    {
        delta += 65536;
    }

    coder.pos += delta;                       // 累加脉冲，记录单圈累计位置
    coder.speed_raw = delta;                  // 原始速度：本次采样周期内的脉冲变化量
    // 一阶低通滤波：平滑速度，抑制抖动，滤波系数 K=4
//    coder.speed_filt = (coder.speed_filt * (CODER_FILTER_K - 1) + delta) / CODER_FILTER_K;
    enc_last = enc_now;                      // 更新本次计数值，作为下一次采样的基准
    encoder_cnt = coder.pos;
    /**
     * M法测速公式：RPM = (Δ × 60000) / (4 × P × Ts)
     * Δ：滤波后的脉冲增量 coder.speed_filt
     * 4：TIM4编码器四倍频（TI12模式，AB边沿都计数）
     * P：ENC_PULSE_PER_ROUND，编码器单圈原始脉冲数
     * Ts：CODER_SAMPLE_MS，TIM6采样周期，单位ms
     * 60000 = 60s × 1000ms，时间换算系数
     */
//    int32_t rpm_temp = (int32_t)abs(coder.speed_filt) * 60000L / (4L * ENC_PULSE_PER_ROUND * CODER_SAMPLE_MS);
//    encoder_rpm = rpm_temp;
//    g_upload_data.encoder_rpm_up = encoder_rpm;


    /**
     * 角度计算，输出单位：0.1°
     * 公式：角度(0.1°) = (pos × 3600) / (4 × P) / 10
     * 3600：360° × 10，放大10倍，输出0.1°精度
     * 4：四倍频
     * P：编码器单圈脉冲
     * 末尾 /10：还原放大倍数
     */
    encoder_angle = (coder.pos * 3600L) / (4L * ENC_PULSE_PER_ROUND)/10;
    if(encoder_angle < 0)
    {
        encoder_angle = encoder_angle + 360;
    }
    g_upload_data.encoder_angle_up = encoder_angle;
	
	scissor_window_check(2000,80);

//	printf("enc_now=%d,enc_angle=%d,delta=%d,encoder_rpm=%d\n\n",enc_now,encoder_angle,delta,encoder_rpm);
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


