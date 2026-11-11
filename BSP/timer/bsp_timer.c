#include "bsp_timer.h"
#include "spindle_coder.h"
#include "selvage_driver.h"

extern CIA402_Node_t g_servo_node_1,g_servo_node_2;

static int16_t selvage_delta,sel_delta = 0;
static uint8_t send_can_cnt = 0;
int32_t actual_selvage_delta = 0;
int32_t into_send_selvage_delta = 0;

static uint8_t time_selvage_triggle_cnt = 0;

void bsp_TIM6_Base_Start_IT(void)
{
    HAL_TIM_Base_Start_IT(&BASIC_TIM6_HANDLE);
}

void bsp_TIM7_Base_Start_IT(void)
{
    HAL_TIM_Base_Start_IT(&BASIC_TIM7_HANDLE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == BASIC_TIM6)
	{
		time_selvage_triggle_cnt++;
		sel_delta = Coder_Callback();
		selvage_delta+= sel_delta;
		actual_selvage_delta = 2*selvage_delta;
		send_can_cnt ++;
		scissor_window_check(2000,32);
		
		if (send_can_cnt==4)
		{
			CAN_TX_Process();
			send_can_cnt = 0;
		}


		if (time_selvage_triggle_cnt == 80)
		{
			if (actual_selvage_delta != 0)
			{
				CANopen_SendRxPDO1_ControlwordAndPos(&g_servo_node_1,0x000f,actual_selvage_delta);
				CANopen_SendRxPDO1_ControlwordAndPos(&g_servo_node_2,0x000f,-actual_selvage_delta);
				CANopen_SendRxPDO1_ControlwordAndPos(&g_servo_node_1,0x005f,actual_selvage_delta);
				CANopen_SendRxPDO1_ControlwordAndPos(&g_servo_node_2,0x005f,-actual_selvage_delta);
//				into_send_selvage_delta+=4;
//				printf("send_cnt=%d\n",into_send_selvage_delta);
			}
			selvage_delta = 0;
			actual_selvage_delta = 0;
			time_selvage_triggle_cnt = 0;
		}
		
      /* 处理电子夹纱器请求 */
      Pinch_Motor_task(); //夹持电机任务，处理主机请求; 
	}
	
	if (htim->Instance == BASIC_TIM7)
	{
		Z_pulse_Output();                 //Z脉冲输出，拉低PD9s
	}
}

/**
  * @brief ：
  * 想进该中断必须在stm32f1xx_it.c中的SysTick_Handler中调用HAL_SYSTICK_IRQHandler();
  */
void HAL_SYSTICK_Callback(void)
{
    
}

