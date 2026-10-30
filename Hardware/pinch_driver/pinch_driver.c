#include "pinch_driver.h"
#include "app_main.h"
#include "canopen.h"

/************************ 工具宏 ************************/
#define LSB(x)         ((uint8_t)((x) & 0xFF))
#define MSB(x)         ((uint8_t)(((x) >> 8) & 0xFF))

extern CAN_Step_Node_t g_step_node;
uint8_t g_drv_mode = DRV_MODE_PP;    // 上电默认PP，和驱动器一致，
uint8_t g_work_state = WORK_STATE_WEAVE;

int32_t target_cnt_N = 0;
int32_t target_cnt_O = 0;   

bool CAN_StepSendFrame(const CanFrame_t *pStepFrm)
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
 * @brief 电机节点初始化
 */
void STEP_Motor_Init(CAN_Step_Node_t *node)
{
    if(node == NULL)
    {
        return ;
    }
    // ===== PV速度模式初始化代码块 =====
    CANOPEN_Motor_SetTorqueMaxCurrent(node, 1000);//配置电流mA
	delay_ms(5);
    // CANOPEN_Motor_SetVelocity(node, 200);
	// delay_ms(5);
    // CANOPEN_Motor_SetPVMode(node);
	// delay_ms(5);
    // CANOPEN_Motor_Disable(node);
	// delay_ms(5);
    // CANOPEN_Motor_Ready(node);
	// delay_ms(5);
    // CANOPEN_Motor_Enable(node);
	// delay_ms(5);

    // ===== PP位置模式初始化代码块 =====
    CANOPEN_Motor_SetTargetPos(node, 0);   //607A = 0 位置配置脉冲数，一个脉冲0.01°精度
    delay_ms(5);
    CANOPEN_Motor_SetProfileVel(node, 400); //6081 = 200  速度配置，减速比1/2，输入800，实际400
    delay_ms(5);
    CANOPEN_Motor_SetPPMode(node);          //6060 = 1    
    delay_ms(5);
    CANOPEN_Motor_Disable(node);            //控制字0x06，停机
    delay_ms(5);
    CANOPEN_Motor_Ready(node);              //控制字0x07，准备上电
    delay_ms(5);
    CANopen_SDO_Write16(node, OD_CONTROL_WORD, 0x00, 79);
    delay_ms(5);
    CANopen_SDO_Write16(node, OD_CONTROL_WORD, 0x00, 95);
    delay_ms(5);
    CANOPEN_Motor_Disable(node);            //控制字0x06，停机

}

/**
 * @brief 设置PV轮廓速度模式，0x6060写3
 */
CO_Err_t CANOPEN_Motor_SetPVMode(CAN_Step_Node_t *node)
{   
    if(node == NULL)
    {
        return CO_ERROR_PARAM;
    }
    return CANopen_SDO_Write8(node, OD_OP_MODE,0x00, STEP_OP_PROFILE_VEL);
}

/**
 * @brief 设置目标速度 0x60FF
 */
CO_Err_t CANOPEN_Motor_SetVelocity(CAN_Step_Node_t *node, int32_t speed_val)
{
    if(node == NULL)
    {
        return CO_ERROR_PARAM;
    }
    return CANopen_SDO_Write32(node,OD_TARGET_VEL, 0x00, (uint32_t)speed_val);
}

/**
 * @brief 控制字0x06，停机指令
 */
CO_Err_t CANOPEN_Motor_Disable(CAN_Step_Node_t *node)
{
    if(node == NULL)
    {
        return CO_ERROR_PARAM;
    }
    return CANopen_SDO_Write16(node,OD_CONTROL_WORD, 0x00, 0x0006);
}

/**
 * @brief 控制字0x07，准备上电
 */
CO_Err_t CANOPEN_Motor_Ready(CAN_Step_Node_t *node)
{
    if(node == NULL)
    {
        return CO_ERROR_PARAM;
    }
    return CANopen_SDO_Write16(node,OD_CONTROL_WORD, 0x00, 0x0007);
}

/**
 * @brief 控制字0x0F，伺服使能运行
 */
CO_Err_t CANOPEN_Motor_Enable(CAN_Step_Node_t *node)
{
    if(node == NULL)
    {
        return CO_ERROR_PARAM;
    }
    return CANopen_SDO_Write16(node, OD_CONTROL_WORD, 0x00, 0x000F);
}

/**
 * @brief 设置伺服为PP轮廓位置模式
 */
CO_Err_t CANOPEN_Motor_SetPPMode(CAN_Step_Node_t *node)
{
    if(node == NULL)
    {
        return CO_ERROR_PARAM;
    }
    return CANopen_SDO_Write8(node,OD_OP_MODE, 0x00, STEP_OP_PROFILE_POS);
}

/**
 * @brief 设置PP目标位置脉冲 OD_TARGET_POS(0x607A)
 * @param pos_val 目标脉冲位置 int32
 */
CO_Err_t CANOPEN_Motor_SetTargetPos(CAN_Step_Node_t *node, int32_t pos_val)
{        
    if(node == NULL)
    {
        return CO_ERROR_PARAM;
    }
    return CANopen_SDO_Write32(node, OD_TARGET_POS, 0x00, pos_val);
}

/**
 * @brief PP模式轮廓运行速度 OD_PROFILE_VEL(0x6081)
 */
CO_Err_t CANOPEN_Motor_SetProfileVel(CAN_Step_Node_t *node, uint32_t vel_val)
{   
    if(node == NULL)
    {
        return CO_ERROR_PARAM;
    }
    return CANopen_SDO_Write32(node, OD_PROFILE_VEL, 0x00, vel_val);
}

/**
 * @brief PP模式：New Setpoint 控制字0x1F，触发点位运动，PP专用
 */
CO_Err_t CANOPEN_Motor_NewSetPoint(CAN_Step_Node_t *node)
{   
    if(node == NULL)
    {
        return CO_ERROR_PARAM;
    }
    return CANopen_SDO_Write16(node, OD_CONTROL_WORD, 0x00, CTRL_WORD_1F);
}

/**
 * @brief  写控制字 0x4F (79) : Enable Operation
 * @param  p_node 电机节点指针
 */
void CANOPEN_Motor_CtrlWord_EnableOp(CAN_Step_Node_t *node)
{
    // 0x6040 控制字是2字节
    CANopen_SDO_Write16(node, OD_CONTROL_WORD, 0x00, 79);
}

/**
 * @brief  写控制字 0x5F (95) : Enable Operation + New Setpoint 触发新目标位置
 * @param  p_node 电机节点指针
 */
void CANOPEN_Motor_CtrlWord_NewSetpoint(CAN_Step_Node_t *node)
{
	if(node == NULL)
    {
        return ;
    }
    CANopen_SDO_Write16(node, OD_CONTROL_WORD, 0x00, 95);
}

/**
 * @brief 设置力矩模式最大电流，单位mA
 */
CO_Err_t CANOPEN_Motor_SetTorqueMaxCurrent(CAN_Step_Node_t *node, uint32_t curr_val)
{
    if(node == NULL)
    {
        return CO_ERROR_PARAM;
    }
    return CANopen_SDO_Write32(node, OD_TORQUE_MAX_CURR, 0x01, curr_val);
}

/**
 * @brief  电子夹纱器电机任务，放在主循环while(1)轮询调用
 * @note   依赖全局变量：encoder_rpm、target_speed_N、target_speed_O、g_step_node
 */
void Pinch_Motor_task(void)
{
    int32_t current_encoder_cnt = 0;
    int32_t target_cnt_N = 0;
    static int32_t last_dir_cnt = 0;
    static uint8_t last_run_sta = 0;
    uint8_t z_trigger_flag = 0;
    uint8_t now_run_sta;

    current_encoder_cnt = encoder_cnt;
    target_cnt_N = current_encoder_cnt / 4;
    z_trigger_flag = 0;

    now_run_sta = (encoder_rpm > RPM_STOP_THRESHOLD) ? 1 : 0;

    if(g_work_state == WORK_STATE_JOG)
    {
        // // 慢动工况：目标PV速度模式
        if(g_drv_mode != DRV_MODE_PV)
        {
            CANOPEN_Motor_SetPVMode(&g_step_node);
            delay_ms(5);
            CANOPEN_Motor_Ready(&g_step_node);
            g_drv_mode = DRV_MODE_PV;
        }
        // 慢动：关闭Z相点位，不跟随
        pinch_z_flag = 0;
        last_run_sta = now_run_sta;
    }
    else if(g_work_state == WORK_STATE_WEAVE)
    {
        // 织造工况：目标PP位置模式
        if(g_drv_mode != DRV_MODE_PP)
        {
            CANOPEN_Motor_SetPPMode(&g_step_node);
            delay_ms(5);
            CANOPEN_Motor_Ready(&g_step_node);
            g_drv_mode = DRV_MODE_PP;
        }

        // 织造模式下，再根据主轴转速判断使能/失能
        if(now_run_sta != last_run_sta)
        {
            if(now_run_sta == 0)
            {
                // 主轴停：织造停机，失能
                CANOPEN_Motor_Disable(&g_step_node);
                last_run_sta = now_run_sta;
            }
            else
            {
                // 主轴运转：保持PP，已经使能
                CANOPEN_Motor_Ready(&g_step_node);
                last_run_sta = now_run_sta;
            }
        }

        // Z相点位逻辑，仅织造生效
        if(pinch_z_flag == 1)
        {
            pinch_z_flag = 0;
            z_trigger_flag = 1;
            if(last_dir_cnt > 0)
            {
                target_cnt_N = 500;
            }
            else
            {
                target_cnt_N = -500;
            }
        }

        if(current_encoder_cnt != 0)
        {
            last_dir_cnt = current_encoder_cnt;
        }

        if(z_trigger_flag == 1)
        {
            target_cnt_O = target_cnt_N;
            delay_ms(5);
            CANOPEN_Motor_SetTargetPos(&g_step_node, target_cnt_N);
            delay_ms(5);
            CANOPEN_Motor_CtrlWord_EnableOp(&g_step_node);
            delay_ms(5);
            CANOPEN_Motor_CtrlWord_NewSetpoint(&g_step_node);
        }
    }
}








