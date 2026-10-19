#ifndef __TPC_SLAVE_H
#define __TPC_SLAVE_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "modbus_rtu.h"
#include "bsp_rs485.h"

/* 变频器响应处理与状态更新接口 */
void App_Process_TPC_Request(RS485_Dev_t *dev);



	
#ifdef __cplusplus
}
#endif

#endif /*__TPC_SLAVE_H */

