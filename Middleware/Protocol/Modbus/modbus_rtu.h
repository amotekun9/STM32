#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "bsp_rs485.h"  /* 引入 RS485 底层驱动以实现一键控制 */

/*********************** Modbus 从机地址定义 ***********************/
#define DOWNLINK_VFD_ADDR		0x01	/*变频器地址*/
#define UPLINK_SELF_ADDR		0x02	/*自身地址*/

/*********************** Modbus 功能码定义 ***********************/
#define MODBUS_FC_READ_HOLDING_REG     0x03    /* 读保持寄存器 (功能码 0x03) */
#define MODBUS_FC_WRITE_SINGLE_REG    0x06    /* 写单个寄存器 (功能码 0x06) */

/*********************** 9600 变频器常用地址定义 ***********************/
#define VFD_ADDR_CONTROL_CMD           0x2000  /* 控制命令地址 (只写 0x06) */
#define VFD_ADDR_DIGITAL_DO            0x2001  /* 数字输出端子控制 (只写) */
#define VFD_ADDR_ANALOG_AM             0x2002  /* 模拟输出 AM 控制 (只写) */
#define VFD_ADDR_ANALOG_AO2            0x2003  /* 模拟输出 AO2 控制 (只写) */
#define VFD_ADDR_PULSE_FMP             0x2004  /* 高速脉冲 FMP 控制 (只写) */
#define VFD_ADDR_SET_FREQ              0x1000  /* 通讯设定值/设定频率 (-10000 ~ 10000) */
#define VFD_ADDR_RUN_STATUS            0x3000  /* 变频器运行状态 (只读 0x03) */
#define VFD_ADDR_FAULT_CODE            0x8000  /* 变频器故障代码 (只读 0x03) */

/*********************** 变频器控制命令码定义 (地址 0x2000) ***********************/
#define VFD_CMD_FORWARD_RUN            0x0001  /* 正转运行 */
#define VFD_CMD_REVERSE_RUN            0x0002  /* 反转运行 */
#define VFD_CMD_FORWARD_JOG            0x0003  /* 正转点动 */
#define VFD_CMD_REVERSE_JOG            0x0004  /* 反转点动 */
#define VFD_CMD_COAST_STOP             0x0005  /* 自由停机 */
#define VFD_CMD_DECEL_STOP             0x0006  /* 减速停机 */
#define VFD_CMD_FAULT_RESET            0x0007  /* 故障复位 */

/*********************** 变频器状态码定义 (地址 0x3000) ***********************/
#define VFD_STATUS_FORWARD             0x0001  /* 正转运行中 */
#define VFD_STATUS_REVERSE             0x0002  /* 反转运行中 */
#define VFD_STATUS_STOP                0x0003  /* 停机中 */

/*********************** Modbus 校验与解析结构体 ***********************/
#define MAX_MODBUS_REG_COUNT  16  // 根据你的实际需求设定最大支持寄存器数量

typedef struct {
    uint8_t  slave_addr;    /* 从机地址 */
    uint8_t  func_code;     /* 功能码 */
    uint16_t reg_addr;      /* 寄存器起始地址 */
    uint16_t reg_value[MAX_MODBUS_REG_COUNT];  /* 寄存器数值 / 读取到的数据 / 写入的数值 */
    uint16_t reg_num;       /* 读取的寄存器数量 */
    bool     is_error;      /* 是否为异常应答 */
    uint8_t  error_code;    /* 异常错误码 (01:指令码错 02:地址错 03:数据错 04:无法处理) */
} ModbusFrame_t;

/*********************** 核心 API 声明 ***********************/

uint16_t Modbus_CRC16(const uint8_t *pBuffer, uint16_t length);/* CRC16 校验 */

/* 主机 (Master) 组帧打包 API */
uint16_t Modbus_Master_BuildReadReg(uint8_t *pTxBuf, uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_num);
uint16_t Modbus_Master_BuildWriteReg(uint8_t *pTxBuf, uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_val);

/* 主机 (Master) 响应帧解析 API */
bool Modbus_Master_ParseResponse(const uint8_t *pRxBuf, uint16_t rx_len, ModbusFrame_t *pFrame);



#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_RTU_H */

