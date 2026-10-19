#include "bsp_usart.h"
#include <stdio.h>
#include <stdarg.h>

uint8_t usart_rx_byte;
static char print_buf[PRINT_BUF_LEN];

/******************************************************************************************/
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */
/* ================= 1. 禁用半主机模式 ================= */
#if (__ARMCC_VERSION >= 6010050)            /* AC6 编译器 */
__asm(".global __use_no_semihosting\n\t");  /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");    /* AC6 下声明 main 无参数 */
#else                                        /* AC5 编译器 */
#pragma import(__use_no_semihosting)
struct __FILE
{
    int handle;
};
#endif

/* ================= 2. 补全标准 C 库所需的底层桩函数 ================= */
/* 避免链接器报 _sys_exit 缺失错误 */
int _sys_exit(int32_t x)
{
    (void)x;
    return 0;
}

/* 定义 __stdout 供 printf 使用 */
FILE __stdout;

/* ================= 3. 重定向 fputc ================= */
int fputc(int32_t ch, FILE *f)
{
    /* 以 USART_UX 为例，等待发送数据寄存器空 (TC / 0x40) */
    while ((USART_UX->SR & 0x40) == 0);
    USART_UX->DR = (uint8_t)ch;
    return ch;
}

//重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
    while ((USART_UX->SR & 0x20) == 0); /* 等待 RXNE */
    return (int)(USART_UX->DR & 0xFF);
}

void bsp_Usart_Receive_IT(void)
{
    /* HAL_UART_IRQHandler 需要一个活跃的接收传输，而不仅仅是 RXNEIE。 */
    (void)HAL_UART_Receive_IT(&USART_UX_HANDLE, &usart_rx_byte, 1U);
}

void Usart_Send_U8(uint8_t data)
{
    /* 1. 等待发送数据寄存器空 (TXE / Bit 7 / 0x80) */
    while ((USART_UX->SR & USART_SR_TXE) == 0);
    
    /* 2. 写入 DR 寄存器，硬件开始发送 */
    USART_UX->DR = data;
}

void Usart_Send_U16(uint16_t data)
{
    /* 1. 发送高 8 位 (MSB) */
    Usart_Send_U8((uint8_t)(data >> 8));
    
    /* 2. 发送低 8 位 (LSB) */
    Usart_Send_U8((uint8_t)(data & 0xFF));
}

void Usart_Send_U32(uint32_t data)
{
    Usart_Send_U8((uint8_t)(data >> 24)); // 高 24-31 位
    Usart_Send_U8((uint8_t)(data >> 16)); // 高 16-23 位
    Usart_Send_U8((uint8_t)(data >> 8));  // 高 8-15 位
    Usart_Send_U8((uint8_t)(data & 0xFF));// 低 0-7 位
}

/***************** 发送字符串 **********************/
void Usart_SendString(uint8_t *str)
{
    if (str == NULL)
    {
        return;
    }

    while (*str != '\0')
    {
        (void)HAL_UART_Transmit(&USART_UX_HANDLE, str, 1U, 1000U);
        str++;
    }
}

void Usart_SendArray(const uint8_t *pData, uint16_t Size)
{
    /* 参数合法性检查 */
    if (pData == NULL || Size == 0) return;

    for (uint16_t i = 0; i < Size; i++)
    {
        /* 1. 等待发送数据寄存器空 (TXE / Bit 7 / 0x80) */
        while ((USART_UX->SR & USART_SR_TXE) == 0);

        /* 2. 写入 DR 寄存器，触发发送 */
        USART_UX->DR = pData[i];
    }
}

void DebugPrint(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(print_buf, PRINT_BUF_LEN, fmt, args);
    va_end(args);
    Usart_SendString((uint8_t *)print_buf);
}
