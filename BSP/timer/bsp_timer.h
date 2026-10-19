#ifndef __BSP_TIMER_H
#define	__BSP_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "stm32f1xx_hal.h"
#include "bsp_usart.h"
#include "bsp_rs485.h"
#include "biss_coder.h"
#include "canopen.h"
#include "vfd_driver.h"

#define BASIC_TIMx                       TIM6
#define BASIC_TIMx_HANDLE  				 htim6
extern TIM_HandleTypeDef BASIC_TIMx_HANDLE;


void bsp_TIM_Base_Start_IT(void);



#ifdef __cplusplus
}
#endif

#endif /* __BSP_TIMER_H */ 

