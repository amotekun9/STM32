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
#include "stdint.h"
#include "pinch_driver.h"

/************************* 上报结构体（MCU → 触摸屏，xxx_up）*************************/
typedef struct
{
    uint16_t encoder_angle_up;      // modbus_holding_reg[2]：主轴当前角度
    uint16_t gear_ratio_up;         // modbus_holding_reg[3]：电子绞边减速比
    uint16_t left_edge_pos_up;      // modbus_holding_reg[4]：左侧电子绞边位置
    uint16_t right_edge_pos_up;     // modbus_holding_reg[5]：右侧电子绞边位置
    uint16_t cut_star_angle_up;       // modbus_holding_reg[6]：自动剪切时间间隔
    uint16_t cut_return_angle_up;       // modbus_holding_reg[8]：自动剪切时间间隔
} UploadData_t;

/************************* 控制指令bit定义（触摸屏 → MCU）*************************/
#define CTRL_BIT_LEFT_EDGE_JOG_FWD  (1U << 1)   // bit0：左侧绞边慢动正转
#define CTRL_BIT_LEFT_EDGE_JOG_REV  (1U << 2)   // bit1：左侧绞边慢动反转
#define CTRL_BIT_LEFT_EDGE_HOME     (1U << 3)   // bit2：左侧绞边归位

#define CTRL_BIT_RIGHT_EDGE_JOG_FWD (1U << 4)   // bit3：右侧绞边慢动正转
#define CTRL_BIT_RIGHT_EDGE_JOG_REV (1U << 5)   // bit4：右侧绞边慢动反转
#define CTRL_BIT_RIGHT_EDGE_HOME    (1U << 6)   // bit5：右侧绞边归位

#define CTRL_BIT_CUT_ACTION         (1U << 7)   // bit6：电子剪刀剪切
#define CTRL_BIT_CUT_HOME           (1U << 8)   // bit7：电子剪刀归位

#define CTRL_BIT_CLAMP_JOG_FWD      (1U << 9)   // bit8：电子夹棍慢动正转
#define CTRL_BIT_CLAMP_JOG_REV      (1U << 10)  // bit9：电子夹棍慢动反转


#define MODBUS_SLAVE_ADDR       1
#define MODBUS_RX_BUF_SIZE      128
#define MODBUS_TX_BUF_SIZE      128
#define MODBUS_HOLD_REG_COUNT   100

extern UploadData_t g_upload_data;
extern uint16_t g_ctrl_cmd_reg;
extern uint16_t modbus_holding_reg[];

// Modbus 上报刷新函数
void ModbusHolding_RefreshUpload(void);
// Modbus指令解析函数（放在主循环调用）
void Modbus_CmdParse(void);

void modbus_slave_task(void);
void modbus_slave_init(void);


#ifdef __cplusplus
}
#endif
#endif
