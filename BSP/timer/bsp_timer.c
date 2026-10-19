#include "bsp_timer.h"

void bsp_TIM_Base_Start_IT(void)
{
    HAL_TIM_Base_Start_IT(&BASIC_TIMx_HANDLE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BASIC_TIMx)
    {
		
    }
}

/**
  * @brief ：
  * 想进该中断必须在stm32f1xx_it.c中的SysTick_Handler中调用HAL_SYSTICK_IRQHandler();
  */
void HAL_SYSTICK_Callback(void)
{
    
}

