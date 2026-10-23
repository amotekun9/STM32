#ifndef _PINCH_DRIVER_H
#define _PINCH_DRIVER_H
#include <stdint.h>
#include <stdbool.h>
#include "bsp_can.h"
#include <string.h>
#include <stdlib.h>


#ifdef __cplusplus
extern "C" {
#endif

/************************ 独立错误码（改名，避免和CIA402重复） ************************/
typedef enum {
    STEP_CO_OK = 0,
    STEP_CO_ERROR_PARAM,
    STEP_CO_ERROR_HW,
    STEP_CO_ERROR_BUSY,
    STEP_CO_ERROR_TIMEOUT,
    STEP_CO_ERROR_ABORT
} STEP_CO_Err_t;

/************************ SDO命令码 ************************/
#define SDO_CMD_WRITE_1B        0x2F
#define SDO_CMD_WRITE_2B        0x2B
#define SDO_CMD_WRITE_4B        0x23
#define SDO_CMD_WRITE_ACK       0x60
#define SDO_CMD_ABORT           0x80

/************************ CiA402 对象字典索引 ************************/
#define OD_CONTROL_WORD         0x6040
#define OD_OP_MODE              0x6060
#define OD_TARGET_VEL           0x60FF

#define STEP_OP_PROFILE_VEL     3   /* 轮廓速度模式PV，改名防止和cia402冲突 */

#define OD_TARGET_POS           0x607A
#define OD_PROFILE_VEL          0x6081
#define STEP_OP_PROFILE_POS     1   /* 轮廓位置模式PP */
#define CTRL_WORD_1F            0x1F

#define OD_MOTOR_RATED_CURR    0x2031    //额定电流，只读
#define OD_TORQUE_MAX_CURR     0x203B    //力矩上限电流，sub 01

#define RPM_STOP_THRESHOLD 5             //判断主轴电机运动状态

/**
 * @brief STEP专用CAN报文结构体，避免和CanFrame_t重定义冲突
 */
typedef struct {
    uint32_t cob_id;
    uint8_t  data_len;
    uint8_t  data[8];
} StepCanFrame_t;

/**
 * @brief 轻量CANopen节点结构体，仅SDO通信
 */
typedef struct
{
    uint8_t node_id;
    bool    sdo_pending;
    uint16_t sdo_timeout_ms;
    uint32_t sdo_abort_code;
} CAN_Step_Node_t;

/************************ API 函数声明 ************************/
//电机初始化
void CANOPEN_Motor_Init(CAN_Step_Node_t *node, uint8_t node_id);
STEP_CO_Err_t CANOPEN_SendFrame(CAN_Step_Node_t *node,uint8_t cmd,uint16_t index,uint8_t subindex,uint32_t data);
STEP_CO_Err_t CANOPEN_Motor_SetTorqueMaxCurrent(CAN_Step_Node_t *node, uint32_t curr_val);

//设置PV速度模式
STEP_CO_Err_t CANOPEN_Motor_SetPVMode(CAN_Step_Node_t *node);
//速度配置：设置目标速度
STEP_CO_Err_t CANOPEN_Motor_SetVelocity(CAN_Step_Node_t *node, int32_t speed_val);////速度配置，PV模式专用
//控制字0x06 停机
STEP_CO_Err_t CANOPEN_Motor_Disable(CAN_Step_Node_t *node);
//控制字0x07 准备上电
STEP_CO_Err_t CANOPEN_Motor_Ready(CAN_Step_Node_t *node);
//控制字0x0F 使能运行
STEP_CO_Err_t CANOPEN_Motor_Enable(CAN_Step_Node_t *node);

STEP_CO_Err_t CANOPEN_Motor_SetPPMode(CAN_Step_Node_t *node);
STEP_CO_Err_t CANOPEN_Motor_SetTargetPos(CAN_Step_Node_t *node, int32_t pos_val);
STEP_CO_Err_t CANOPEN_Motor_SetProfileVel(CAN_Step_Node_t *node, uint32_t vel_val);//速度配置，PP模式专用
STEP_CO_Err_t CANOPEN_Motor_NewSetPoint(CAN_Step_Node_t *node); //PP专属触发点位

void CANOPEN_Motor_CtrlWord_EnableOp(CAN_Step_Node_t *node);
void CANOPEN_Motor_CtrlWord_NewSetpoint(CAN_Step_Node_t *node);


void Pinch_Motor_task(void); //电子夹纱器电机任务，放在主循环while(1)轮询调用



//SDO超时tick，1ms调用一次
void CANOPEN_SdoTick(CAN_Step_Node_t *node);
//CAN接收回调，收到报文调用
void CANOPEN_RxProcess(CAN_Step_Node_t *node, const StepCanFrame_t *frame);

#ifdef __cplusplus
}
#endif

#endif

