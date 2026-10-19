#include "bsp_delay.h"

static uint32_t g_fac_us = 0;       /* us延时倍乘数，即每 1us 对应的 CPU 周期数 */

/* 如果某些 CMSIS 版本没有这两个宏，这里做兼容 */
#ifndef CoreDebug_DEMCR_TRCENA_Msk
#define CoreDebug_DEMCR_TRCENA_Msk   (1UL << 24)
#endif

#ifndef DWT_CTRL_CYCCNTENA_Msk
#define DWT_CTRL_CYCCNTENA_Msk       (1UL << 0)
#endif

/**
 * @brief     初始化延迟函数
 * @param     sysclk: 系统时钟频率, 即CPU频率(rcc_c_ck), 单位 MHz, 例如 72
 * @retval    无
 */
void delay_init(uint16_t sysclk)
{
    g_fac_us = sysclk;

    /*
     * 使能 DWT 周期计数器，用于 delay_ns。
     * DWT->CYCCNT 是 CPU 周期计数器。
     * 72MHz 时，1 个计数 = 1 / 72MHz ≈ 13.89ns。
     */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

		/* 2. 清零周期计数器（确保从一个已知状态开始） */
		DWT->CYCCNT = 0;	 
		
		/* 3.使能 DWT 周期计数器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief     纳秒级延时函数
 * @param     nns: 要延时的纳秒数
 * @retval    无
 * @note      STM32F103 @72MHz 时，最小理论步进约 13.89ns，因此 delay_ns(1) 不可能真的只延时 1ns。
 *						注意：nns 不能过大，72MHz 下 nns 不能超过 59652309 ns 
 */
void delay_ns(uint32_t nns)
{
    uint32_t ticks;
    uint32_t start;

    /* 没有初始化或延时为 0 时直接返回 */
    if ((nns == 0U) || (g_fac_us == 0U)) {
        return;
    }

    /* 
     * g_fac_us 表示 1us 对应的 CPU 周期数。例如 72MHz 时，g_fac_us = 72。nns 纳秒对应的周期数：ticks = nns * g_fac_us / 1000
     * 这里使用 +999 向上取整，尽量保证理论延时不小于输入值。
     *
     * 使用纯 32 位运算，利用 Cortex-M3/M4 的硬件 32位除法器 
     * 计算速度极快 (2~12个周期内完成)。
     * 
     */
    ticks = (nns * g_fac_us + 999U) / 1000U;

    start = DWT->CYCCNT;

    /* 等待循环 */
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) {
        /* 空指令，防止编译器过度优化，同时略微降低功耗 */
        __NOP(); 
    }
}

/**
 * @brief     延时nus
 * @note      无论是否使用OS, 都是用时钟摘取法来做us延时
 * @param     nus: 要延时的us数
 * @note      nus取值范围: 0 ~ (2^32 / fac_us) (fac_us一般等于系统主频, 自行套入计算)
 * @retval    无
 */
void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;        /* LOAD的值 */

    ticks = nus * g_fac_us;                 /* 需要的节拍数 */

    told = SysTick->VAL;                    /* 刚进入时的计数器值 */
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;        /* 注意SYSTICK是一个递减的计数器 */
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;

            if (tcnt >= ticks)
            {
                break;                      /* 时间超过/等于要延迟的时间,则退出 */
            }
        }
    }
}

/**
 * @brief     延时nms
 * @param     nms: 要延时的ms数 (0< nms <= (2^32 / fac_us / 1000))
 * @retval    无
 */
void delay_ms(uint16_t nms)
{
    delay_us((uint32_t)(nms * 1000));       /* 普通方式延时 */
}

/**
 * @brief       HAL库内部函数用到的延时
 * @note        HAL库的延时默认用Systick，如果我们没有开Systick的中断会导致调用这个延时后无法退出
 * @param       Delay : 要延时的毫秒数
 * @retval      None
 */
void HAL_Delay(uint32_t Delay)
{
    delay_ms(Delay);
}
