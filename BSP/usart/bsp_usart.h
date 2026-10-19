#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "stdio.h"
#include "stm32f1xx_hal.h"

#define USART_UX         		USART1
#define USART_UX_HANDLE  		huart1
extern UART_HandleTypeDef 	USART_UX_HANDLE;       		/* HAL UART¾ä±ú */


void bsp_Usart_Receive_IT(void);               
void Usart_SendString(uint8_t *str);
void Usart_SendArray(const uint8_t *pData, uint16_t Size);
void Usart_Send_U8(uint8_t data);
void Usart_Send_U16(uint16_t data);
void Usart_Send_U32(uint32_t data);


#endif /* __BSP_USART_H */


