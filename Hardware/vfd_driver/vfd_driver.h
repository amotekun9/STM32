#ifndef __VFD_DRIVER_H
#define __VFD_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "modbus_rtu.h"
#include "bsp_rs485.h"

/* 变频器响应处理与状态更新接口 */
void App_Process_VFD_Response(RS485_Dev_t *dev);

/* 快捷控制变频器 API (配合异步 RS485 发送) */
bool VFD_ControlRun(RS485_Dev_t *dev, uint8_t slave_addr, uint16_t cmd);
bool VFD_SetFrequency(RS485_Dev_t *dev, uint8_t slave_addr, int16_t freq_val);
bool VFD_ReadStatus(RS485_Dev_t *dev, uint8_t slave_addr);
	
#ifdef __cplusplus
}
#endif

#endif /* __VFD_DRIVER_H */
