#ifndef __DELAY_H
#define __DELAY_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "stm32f1xx_hal.h"

void delay_init(uint16_t sysclk);   /* 初始化延迟函数 */
void delay_ns(uint32_t nns);	/*纳秒级延时函数,入参范围1~59652309,实际最低延时13.89ns,做不到1ns*/
void delay_ms(uint16_t nms);    /* 延时nms */
void delay_us(uint32_t nus);    /* 延时nus */
void HAL_Delay(uint32_t Delay);   /* HAL库的延时函数，HAL库内部用到 */
		
#ifdef __cplusplus
}
#endif

#endif /* __DELAY_H */ 






























