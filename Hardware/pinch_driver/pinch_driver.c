/************************************************
 * @file    bsp_stepmotor.c
 * @brief   步进电机BSP底层驱动
 * @author  MO wenzhi
 * @version V1.0
 * @date    2026.10.09
 * @note    适配铠硕步进电机
 ***********************************************/

#include "pinch_driver.h"
#include <string.h>
#include "bsp_can.h"
#include "app_main.h"
/************************ 工具宏 ************************/
#define LSB(x)         ((uint8_t)((x) & 0xFF))
#define MSB(x)         ((uint8_t)(((x) >> 8) & 0xFF))
#define SDO_TIMEOUT_MS  500

extern CAN_Step_Node_t g_step_node;

int32_t target_speed_N = 0;
int32_t target_speed_O = 0;   

bool CAN_StepSendFrame(const StepCanFrame_t *pStepFrm)
{
    if(pStepFrm == NULL)
    {  
        return false;
    }
    CanFrame_t frm;
    frm.cob_id = pStepFrm->cob_id;
    frm.data_len = pStepFrm->data_len;
    memcpy(frm.data, pStepFrm->data, 8);
    return CAN_SendFrame(&frm); //调用你原来底层CAN发送函数
}
/**
 * @brief SDO底层发送单条SDO报文
 */
static STEP_CO_Err_t SDO_SendFrame(CAN_Step_Node_t *node, uint8_t cmd, uint16_t index, uint8_t subidx, uint32_t data)
{
    if (node == NULL)
        return STEP_CO_ERROR_PARAM;
    if (node->sdo_pending)
        return STEP_CO_ERROR_BUSY;
    
    StepCanFrame_t frame;
    memset(&frame, 0, sizeof(StepCanFrame_t));
    frame.cob_id = 0x600 + node->node_id;
    frame.data_len = 8;
    frame.data[0] = cmd;
    frame.data[1] = LSB(index);
    frame.data[2] = MSB(index);
    frame.data[3] = subidx;
    frame.data[4] = LSB(data);
    frame.data[5] = MSB(data);
    frame.data[6] = (data >> 16) & 0xFF;
    frame.data[7] = (data >> 24) & 0xFF;
    
    if (!CAN_StepSendFrame(&frame))
    {  
        return STEP_CO_ERROR_HW;
    }
    // node->sdo_pending = true;
    // node->sdo_timeout_ms = SDO_TIMEOUT_MS;
    // node->sdo_abort_code = 0;
    return STEP_CO_OK;
}
/**
 * @brief 电机节点初始化
 */
void CANOPEN_Motor_Init(CAN_Step_Node_t *node, uint8_t node_id)
{
    if(node == NULL)
        return;
    memset(node, 0, sizeof(CAN_Step_Node_t));
    node->node_id = node_id;
    node->sdo_pending = false;
    node->sdo_timeout_ms = SDO_TIMEOUT_MS;
    node->sdo_abort_code = 0;

    // ===== PV速度模式初始化代码块 =====
    CANOPEN_Motor_SetTorqueMaxCurrent(node, 3000);//配置电流1200mA
	delay_ms(5);
    // CANOPEN_SendFrame(node, SDO_CMD_WRITE_1B, OD_OP_MODE, 0x00, STEP_OP_PROFILE_VEL);
    CANOPEN_Motor_SetVelocity(node, 0);
	delay_ms(5);
    CANOPEN_Motor_SetPVMode(node);
	delay_ms(5);
    CANOPEN_Motor_Disable(node);
	delay_ms(5);
    CANOPEN_Motor_Ready(node);
	delay_ms(5);
    CANOPEN_Motor_Enable(node);
	delay_ms(5);
}
/**
 * @brief 设置PV轮廓速度模式，0x6060写3
 */
STEP_CO_Err_t CANOPEN_Motor_SetPVMode(CAN_Step_Node_t *node)
{   
    if(node == NULL)
        return STEP_CO_ERROR_PARAM;
    return SDO_SendFrame(node, SDO_CMD_WRITE_1B, OD_OP_MODE, 0x00, STEP_OP_PROFILE_VEL);
}
/**
 * @brief 设置目标速度 0x60FF
 */
STEP_CO_Err_t CANOPEN_Motor_SetVelocity(CAN_Step_Node_t *node, int32_t speed_val)
{
    if(node == NULL)
        return STEP_CO_ERROR_PARAM;
    return SDO_SendFrame(node, SDO_CMD_WRITE_4B, OD_TARGET_VEL, 0x00, (uint32_t)speed_val);
}
/**
 * @brief 控制字0x06，停机指令
 */
STEP_CO_Err_t CANOPEN_Motor_Disable(CAN_Step_Node_t *node)
{
    if(node == NULL)
        return STEP_CO_ERROR_PARAM;
    return SDO_SendFrame(node, SDO_CMD_WRITE_2B, OD_CONTROL_WORD, 0x00, 0x0006);
}
/**
 * @brief 控制字0x07，准备上电
 */
STEP_CO_Err_t CANOPEN_Motor_Ready(CAN_Step_Node_t *node)
{
    if(node == NULL)
        return STEP_CO_ERROR_PARAM;
    return SDO_SendFrame(node, SDO_CMD_WRITE_2B, OD_CONTROL_WORD, 0x00, 0x0007);
}
/**
 * @brief 控制字0x0F，伺服使能运行
 */
STEP_CO_Err_t CANOPEN_Motor_Enable(CAN_Step_Node_t *node)
{
    if(node == NULL)
        return STEP_CO_ERROR_PARAM;
    return SDO_SendFrame(node, SDO_CMD_WRITE_2B, OD_CONTROL_WORD, 0x00, 0x000F);
}
/**
 * @brief 设置伺服为PP轮廓位置模式
 */
STEP_CO_Err_t CANOPEN_Motor_SetPPMode(CAN_Step_Node_t *node)
{
    if(node == NULL)
        return STEP_CO_ERROR_PARAM;
    return SDO_SendFrame(node, SDO_CMD_WRITE_1B, OD_OP_MODE, 0x00, STEP_OP_PROFILE_POS);
}
/**
 * @brief 设置PP目标位置脉冲 OD_TARGET_POS(0x607A)
 * @param pos_val 目标脉冲位置 int32
 */
STEP_CO_Err_t CANOPEN_Motor_SetTargetPos(CAN_Step_Node_t *node, int32_t pos_val)
{        
    if(node == NULL)
        return STEP_CO_ERROR_PARAM;
    return SDO_SendFrame(node, SDO_CMD_WRITE_4B, OD_TARGET_POS, 0x00, pos_val);
}
/**
 * @brief PP模式轮廓运行速度 OD_PROFILE_VEL(0x6081)
 */
STEP_CO_Err_t CANOPEN_Motor_SetProfileVel(CAN_Step_Node_t *node, uint32_t vel_val)
{   
    if(node == NULL)
        return STEP_CO_ERROR_PARAM;
    return SDO_SendFrame(node, SDO_CMD_WRITE_4B, OD_PROFILE_VEL, 0x00, vel_val);
}
/**
 * @brief PP模式：New Setpoint 控制字0x1F，触发点位运动，PP专用
 */
STEP_CO_Err_t CANOPEN_Motor_NewSetPoint(CAN_Step_Node_t *node)
{   
    if(node == NULL)
        return STEP_CO_ERROR_PARAM;
    return SDO_SendFrame(node, SDO_CMD_WRITE_2B, OD_CONTROL_WORD, 0x00, CTRL_WORD_1F);
}
/**
 * @brief 通用SDO写报文封装，放到bsp_stepmotor.c
 * @param node 节点指针
 * @param cmd SDO写指令（SDO_CMD_WRITE_1B / WRITE_2B / WRITE_4B）
 * @param index 对象字典索引
 * @param subindex 子索引
 * @param data 待写入数据
 * @retval STEP_CO_Err_t
 */
STEP_CO_Err_t CANOPEN_SendFrame(CAN_Step_Node_t *node,uint8_t cmd,uint16_t index,uint8_t subindex,uint32_t data)
{        
    if(node == NULL)
    {
        return STEP_CO_ERROR_PARAM;
    }
    return SDO_SendFrame(node, cmd, index, subindex, data);
}
/**
 * @brief 设置力矩模式最大电流，单位mA
 */
STEP_CO_Err_t CANOPEN_Motor_SetTorqueMaxCurrent(CAN_Step_Node_t *node, uint32_t curr_val)
{
    if(node == NULL)
        return STEP_CO_ERROR_PARAM;
    return CANOPEN_SendFrame(node, SDO_CMD_WRITE_4B, OD_TORQUE_MAX_CURR, 0x01, curr_val);
}
/**
 * @brief 1ms Tick，SDO超时判断
 */
void CANOPEN_SdoTick(CAN_Step_Node_t *node)
{
    if(node == NULL)
        return;
    if(node->sdo_pending)
    {
        if(node->sdo_timeout_ms > 0)
        {
            node->sdo_timeout_ms --;
        }
        else
        {
            node->sdo_pending = false;
            node->sdo_abort_code = 0x05040000; //SDO 超时abort码
        }
    }
}
/**
 * @brief CAN接收回调，解析SDO应答
 */
void CANOPEN_RxProcess(CAN_Step_Node_t *node, const StepCanFrame_t *frame)
{
    //双指针判空
    if(node == NULL || frame == NULL)
        return;
    uint32_t sdo_rx_cobid = 0x580 + node->node_id;
    if(frame->cob_id != sdo_rx_cobid)
        return;
    if(!node->sdo_pending)
        return;
    uint8_t cmd = frame->data[0];
    if(cmd == SDO_CMD_WRITE_ACK)
    {
        node->sdo_pending = false;
        node->sdo_abort_code = 0;
    }
    else if(cmd == SDO_CMD_ABORT)
    {
        node->sdo_pending = false;
        node->sdo_abort_code = (uint32_t)frame->data[4] | ((uint32_t)frame->data[5] << 8)
                | ((uint32_t)frame->data[6] << 16) | ((uint32_t)frame->data[7] << 24);
    }
}

/**
 * @brief  电子夹纱器电机任务，放在主循环while(1)轮询调用
 * @note   依赖全局变量：encoder_rpm、target_speed_N、target_speed_O、g_step_node
 */
void Pinch_Motor_task(void)
{
    // DebugPrint("encoder_rpm: %d\r\n",modbus_holding_reg[3]);
    modbus_holding_reg[2]=encoder_angle;
    // modbus_holding_reg[2]= encoder_cnt;
    target_speed_N = modbus_holding_reg[3];
    target_speed_N = 10*encoder_rpm ;

    if(target_speed_N != target_speed_O)
    {
        target_speed_O = target_speed_N;
        if(target_speed_N > 200)
        {
            target_speed_N = 200;
        }
//         DebugPrint("target_speed_N: %d, target_speed_O: %d\r\n", target_speed_N, target_speed_O);
        CANOPEN_Motor_SetVelocity(&g_step_node, target_speed_N);
		delay_ms(5);
        CANOPEN_Motor_Enable(&g_step_node);
    }
}
