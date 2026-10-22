#include "scissor_driver.h"

#define SCISSOR_PC8_PORT  GPIOC
#define SCISSOR_PC8_PIN   GPIO_PIN_8
#define SCISSOR_PC9_PORT  GPIOC
#define SCISSOR_PC9_PIN   GPIO_PIN_9

typedef enum
{
    SCISSOR_IDLE = 0,
    SCISSOR_OPEN_RUNNING,
    SCISSOR_CLOSE_RUNNING
} Scissor_StateTypeDef;

// 必须加 volatile，防止主循环与中断之间产生编译器缓存优化问题
static volatile Scissor_StateTypeDef scissor_state = SCISSOR_IDLE;
static volatile uint32_t scissor_tick = 0;
static volatile uint32_t scissor_duration_us = 0;

static inline uint32_t DWT_GetUs(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000UL);
}

void scissor_init(void)
{
    // 初始化引脚默认状态（根据WCGD103板卡要求，默认不给信号时置高电平）
    HAL_GPIO_WritePin(SCISSOR_PC8_PORT, SCISSOR_PC8_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SCISSOR_PC9_PORT, SCISSOR_PC9_PIN, GPIO_PIN_SET);
    scissor_state = SCISSOR_IDLE;
}

// 触发打开：对应WCGD103模式四的信号①
void scissor_open_trig(uint16_t us)
{
    if(scissor_state != SCISSOR_IDLE) return;
    scissor_duration_us = us;
    scissor_tick = DWT_GetUs();
    
    // 模式四互斥：先确保信号②(PC9)处于关闭状态(SET)，再拉低信号①(PC8)
    HAL_GPIO_WritePin(SCISSOR_PC9_PORT, SCISSOR_PC9_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SCISSOR_PC8_PORT, SCISSOR_PC8_PIN, GPIO_PIN_RESET);
    scissor_state = SCISSOR_OPEN_RUNNING;
}

// 触发闭合：对应WCGD103模式四的信号②
void scissor_close_trig(uint16_t us)
{
    if(scissor_state != SCISSOR_IDLE) return;
    scissor_duration_us = us;
    scissor_tick = DWT_GetUs();
    
    // 模式四互斥：先确保信号①(PC8)处于关闭状态(SET)，再拉低信号②(PC9)
    HAL_GPIO_WritePin(SCISSOR_PC8_PORT, SCISSOR_PC8_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SCISSOR_PC9_PORT, SCISSOR_PC9_PIN, GPIO_PIN_RESET);
    scissor_state = SCISSOR_CLOSE_RUNNING;
}

/**
 * @brief       剪刀主轴窗口检测与动作触发函数
 * @param       pulse_us 电磁铁有效通电时间（单位：微秒，μs）
 * @param       offset_encoder_cnt 窗口脉冲宽度（即窗口的容差范围，决定触发时的角度区间大小）
 */
void scissor_window_check(const uint16_t pulse_us, const uint8_t offset_encoder_cnt)
{
	// 静态局部变量：用于实现“单圈只触发一次”的锁存机制
    static bool win_close_latch = 0;
    static bool win_open_latch = 0;

    // ==================== 1. 闭合（Close）窗口区间检测 ====================
    // 检查当前主轴编码器位置是否在 [设定值, 设定值 + 窗口宽度] 的开区间内
	if (( ((encoder_cnt>= modbus_holding_reg[4]) && (encoder_cnt <= modbus_holding_reg[4] + offset_encoder_cnt)) )
		|| ( ((encoder_cnt >= -(modbus_holding_reg[4] + offset_encoder_cnt)) && (encoder_cnt <= -modbus_holding_reg[4])) ) )
    {
        if (win_close_latch == 0)	// 若本圈还没有触发过闭合
        {
            scissor_close_trig(pulse_us);	// 触发闭合动作（驱动板信号②）
            win_close_latch = 1;	// 置位锁存，防止本圈内由于多次中断重复触发
        }
    }
    else
    {
        win_close_latch = 0;	// 一旦主轴转出该窗口，立即释放锁存，等待下一圈重新触发
    }

	// ==================== 2. 打开（Open）窗口区间检测 ====================
	if (( ((encoder_cnt>= modbus_holding_reg[5]) && (encoder_cnt <= modbus_holding_reg[5] + offset_encoder_cnt)) )
		|| ( ((encoder_cnt >= -(modbus_holding_reg[5] + offset_encoder_cnt)) && (encoder_cnt <= -modbus_holding_reg[5])) ) )

    {
        if (win_open_latch == 0)	// 若本圈还没有触发过打开
        {
            scissor_open_trig(pulse_us);	// 触发打开动作（驱动板信号①）
            win_open_latch = 1;	// 置位锁存，防止本圈重复触发
        }
    }
    else
    {
        win_open_latch = 0;	// 一旦主轴转出该窗口，立即释放锁存
    }
}

// 放到 while(1) 循环轮询
void scissor_poll(void)
{
    if(scissor_state == SCISSOR_IDLE) return;

    uint32_t now = DWT_GetUs();
    if(now - scissor_tick >= scissor_duration_us)
    {
        // 脉冲时间到，恢复驱动板控制信号为默认高电平（断开输出）
        HAL_GPIO_WritePin(SCISSOR_PC8_PORT, SCISSOR_PC8_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SCISSOR_PC9_PORT, SCISSOR_PC9_PIN, GPIO_PIN_SET);
        scissor_state = SCISSOR_IDLE;
    }
}

uint8_t scissor_is_idle(void)
{
    return scissor_state == SCISSOR_IDLE;
}

