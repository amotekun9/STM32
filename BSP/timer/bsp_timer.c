#include "bsp_timer.h"
#include "spindle_coder.h"

void bsp_TIM_Base_Start_IT(void)
{
    HAL_TIM_Base_Start_IT(&BASIC_TIM6_HANDLE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BASIC_TIM6)
    {
      Coder_Callback();
    }
}

/**
  * @brief ：
  * 想进该中断必须在stm32f1xx_it.c中的SysTick_Handler中调用HAL_SYSTICK_IRQHandler();
  */
void HAL_SYSTICK_Callback(void)
{
    
}

