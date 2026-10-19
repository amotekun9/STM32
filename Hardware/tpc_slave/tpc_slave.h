/************************************************
 * @file    tpc_slave.c
 * @brief   TPC从机处理程序
 * @author  MO wenzhi
 * @version V1.0
 * @date    2026.10.09
 * @note    适配百纳BN5058系列编码,PC10:A PC11:B
 ***********************************************/
 
#ifndef __TPC_SLAVE_H
#define __TPC_SLAVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "bsp_rs485.h"

#define MODBUS_SLAVE_ADDR       1
#define MODBUS_RX_BUF_SIZE      128
#define MODBUS_TX_BUF_SIZE      128
#define MODBUS_HOLD_REG_COUNT   100

extern uint16_t modbus_holding_reg[];
void modbus_slave_task(void);
void modbus_slave_init(void);

#ifdef __cplusplus
}
#endif
#endif
