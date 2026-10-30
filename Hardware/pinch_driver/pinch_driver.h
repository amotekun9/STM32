#ifndef _PINCH_DRIVER_H
#define _PINCH_DRIVER_H
#include <stdint.h>
#include <stdbool.h>
#include "bsp_can.h"
#include <string.h>
#include <stdlib.h>
#include "canopen.h"


#ifdef __cplusplus
extern "C" {
#endif

#define STEP_OP_PROFILE_VEL     3   /* 轮廓速度模式PV，改名防止和cia402冲突 */

#define OD_PROFILE_VEL          0x6081
#define STEP_OP_PROFILE_POS     1   /* 轮廓位置模式PP */
#define CTRL_WORD_1F            0x1F

#define OD_MOTOR_RATED_CURR    0x2031    //额定电流，只读
#define OD_TORQUE_MAX_CURR     0x203B    //力矩上限电流，sub 01

#define RPM_STOP_THRESHOLD 5             //判断主轴电机运动状态

#define DRV_MODE_PP     0   // 位置模式（织造）
#define DRV_MODE_PV     1   // 速度模式（慢动）
#define WORK_STATE_WEAVE  0 // 织造工况
#define WORK_STATE_JOG    1 // 慢动工况

extern uint8_t g_drv_mode ;    // 上电默认PP，和驱动器一致，
extern uint8_t g_work_state ;


/************************ API 函数声明 ************************/
//电机初始化
void STEP_Motor_Init(CAN_Step_Node_t *node);

CO_Err_t CANOPEN_Motor_SetTorqueMaxCurrent(CAN_Step_Node_t *node, uint32_t curr_val);

//设置PV速度模式
CO_Err_t CANOPEN_Motor_SetPVMode(CAN_Step_Node_t *node);
//速度配置：设置目标速度
CO_Err_t CANOPEN_Motor_SetVelocity(CAN_Step_Node_t *node, int32_t speed_val);////速度配置，PV模式专用
//控制字0x06 停机
CO_Err_t CANOPEN_Motor_Disable(CAN_Step_Node_t *node);
//控制字0x07 准备上电
CO_Err_t CANOPEN_Motor_Ready(CAN_Step_Node_t *node);
//控制字0x0F 使能运行
CO_Err_t CANOPEN_Motor_Enable(CAN_Step_Node_t *node);

CO_Err_t CANOPEN_Motor_SetPPMode(CAN_Step_Node_t *node);
CO_Err_t CANOPEN_Motor_SetTargetPos(CAN_Step_Node_t *node, int32_t pos_val);
CO_Err_t CANOPEN_Motor_SetProfileVel(CAN_Step_Node_t *node, uint32_t vel_val);//速度配置，PP模式专用
CO_Err_t CANOPEN_Motor_NewSetPoint(CAN_Step_Node_t *node); //PP专属触发点位

void CANOPEN_Motor_CtrlWord_EnableOp(CAN_Step_Node_t *node);
void CANOPEN_Motor_CtrlWord_NewSetpoint(CAN_Step_Node_t *node);

void Pinch_Motor_task(void); //电子夹纱器电机任务，放在主循环while(1)轮询调用

#ifdef __cplusplus
}
#endif

#endif

