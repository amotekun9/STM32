#include "selvage_driver.h"
#include "bsp_delay.h"
#include "bsp_usart.h"
#include "spindle_coder.h"

extern CIA402_Node_t g_servo_node_1,g_servo_node_2;

/*将异步RPDO通讯参数常用配置以及运动配置保存到flash初始化*/
void canopen_selvage_nosave_async_pdo_pp_mode_init(CIA402_Node_t *node)
{
    if(node == NULL)
        return;
	CIA402_SetOpMode(node,OP_PROFILE_POS);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6081, 0x00,72000);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6083, 0x00, 20);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6084, 0x00, 20);
	delay_ms(6);
	CIA402_MapRxPDO1_ControlwordAndPos(node);
	delay_ms(6);
//	CIA402_MapRxPDO2_OnlyProfileVel(node);
	
//	CIA402_MapRxPDO3_ProfileAccelAndDecel(node);
	
//	CIA402_MapRxPDO2_OnlyMode(node);
//	
//	CIA402_MapRxPDO4_OnlyVel(node);
//	
//	CIA402_MapTxPDO1_StatuswordAndActualPos(node);
	
//	CIA402_SetTxPDO_TransType(node,1,255);
	
//	CIA402_EnableTxPDO1(node,1);
	
	CIA402_SetRxPDO_TransType(node,1,255);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x1010, 0x01, 0x65766173);
	delay_ms(6);
	CANopen_SDO_Write16(node,0x6040,0,0x06);
	delay_ms(6);
	CANopen_SDO_Write16(node,0x6040,0,0x07);
	delay_ms(6);
	CANopen_SDO_Write16(node,0x6040,0,0x0f);
	delay_ms(6);
	CANopen_NMT_SendCmd(node,0x01);
	delay_ms(6);
}

/*将同步RPDO通讯参数常用配置以及运动配置保存到flash初始化*/
void canopen_selvage_nosave_sync_pdo_pp_mode_init(CIA402_Node_t *node)
{
    if(node == NULL)
        return;
//	CANopen_NMT_SendCmd(node,0x80);
//	
	CIA402_SetOpMode(node,OP_PROFILE_POS);
	
	CANopen_SDO_Write32(node, 0x6081, 0x00, 72000);
	
	CANopen_SDO_Write32(node, 0x6083, 0x00, 20);
	
	CANopen_SDO_Write32(node, 0x6084, 0x00, 20);
	
	CIA402_MapRxPDO1_ControlwordAndPos(node);
	
	CIA402_MapRxPDO2_OnlyMode(node);
	
	CIA402_MapRxPDO4_OnlyVel(node);
	
	CIA402_MapTxPDO1_StatuswordAndActualPos(node);
	
	CIA402_SetTxPDO_TransType(node,1,1);
	
	CIA402_EnableTxPDO1(node,1);
	
	CIA402_EnableTxPDO2(node,0);
	
	CANopen_SetCyclePeriod(node,2000);
	
	CIA402_SetRxPDO_TransType(node,1,1);
	
	CANopen_SDO_Write32(node, 0x1010, 0x01, 0x65766173);
	
	CANopen_SDO_Write16(node,0x6040,0,0x06);
	
	CANopen_SDO_Write16(node,0x6040,0,0x07);
	
	CANopen_SDO_Write16(node,0x6040,0,0x0f);
	
	CANopen_NMT_SendCmd(node,0x01);
	
}

/*将异步RPDO通讯参数常用配置以及运动配置保存到flash初始化*/
void canopen_selvage_nosave_async_pdo_pv_mode_init(CIA402_Node_t *node)
{
    if(node == NULL)
        return;
	CIA402_SetOpMode(node,OP_PROFILE_VEL);
	
	CANopen_SDO_Write32(node, 0x6083, 0x00, 20);
	
	CANopen_SDO_Write32(node, 0x6084, 0x00, 20);
	
	CIA402_MapRxPDO1_ControlwordModeAndVel(node);
	
	CIA402_MapRxPDO2_OnlyMode(node);
	
	CIA402_MapRxPDO4_OnlyVel(node);
	
	CIA402_MapTxPDO1_StatuswordAndActualPos(node);
	
	CIA402_SetTxPDO_TransType(node,1,255);
	
	CIA402_EnableTxPDO1(node,1);
	
	CIA402_EnableTxPDO2(node,0);
	
	CIA402_SetRxPDO_TransType(node,1,255);
	
	CANopen_SDO_Write32(node, 0x1010, 0x01, 0x65766173);
	
	CANopen_SDO_Write16(node,0x6040,0,0x06);
	
	CANopen_SDO_Write16(node,0x6040,0,0x07);
	
	CANopen_SDO_Write16(node,0x6040,0,0x0f);
	
	CANopen_NMT_SendCmd(node,0x01);
	
}

/*将同步RPDO通讯参数常用配置以及运动配置保存到flash初始化*/
void canopen_selvage_nosave_sync_pdo_pv_mode_init(CIA402_Node_t *node)
{
    if(node == NULL)
        return;
	CIA402_SetOpMode(node,OP_PROFILE_VEL);
	
	CANopen_SDO_Write32(node, 0x6083, 0x00, 20);
	
	CANopen_SDO_Write32(node, 0x6084, 0x00, 20);
	
	CIA402_MapRxPDO1_ControlwordModeAndVel(node);
	
	CIA402_MapRxPDO2_OnlyMode(node);
	
	CIA402_MapRxPDO4_OnlyVel(node);
	
	CIA402_MapTxPDO1_StatuswordAndActualPos(node);
	
	CIA402_SetTxPDO_TransType(node,1,1);
	
	CIA402_EnableTxPDO1(node,1);
	
	CIA402_EnableTxPDO2(node,0);
	
	CANopen_SetCyclePeriod(node,3000);
	
	CIA402_SetRxPDO_TransType(node,1,1);
	
	CANopen_SDO_Write32(node, 0x1010, 0x01, 0x65766173);
	
	CANopen_SDO_Write16(node,0x6040,0,0x06);
	
	CANopen_SDO_Write16(node,0x6040,0,0x07);
	
	CANopen_SDO_Write16(node,0x6040,0,0x0f);
	
	CANopen_NMT_SendCmd(node,0x01);
	
}

/**
 * @brief 绞边节点位置模式初始化
 */
void canopen_selvage_pp_position_init(CIA402_Node_t *node)
{
    if(node == NULL)
        return;
	CIA402_SetTxPDO_TransType(node,1,255);
	
//	CIA402_SetTxPDO_TransType(node,2,255);
//	
	CIA402_EnableTxPDO1(node,1);
	
//	CIA402_EnableTxPDO2(node,1);
//	
	CANopen_NMT_SendCmd(node,0x01);
	
	CANopen_SendRxPDO2_OnlyMode(node,OP_PROFILE_POS);
	
	CANopen_SendRxPDO1(node,0x06);
	
	CANopen_SendRxPDO1(node,0x07);
	
	CANopen_SendRxPDO1(node,0x0f);
	
}

/**
 * @brief 绞边节点速度模式初始化
 */
void canopen_selvage_speed_init(CIA402_Node_t *node)
{
    if(node == NULL)
        return;
	CIA402_SetTxPDO_TransType(node,1,255);
	
	CIA402_SetTxPDO_TransType(node,2,255);
	
	CIA402_EnableTxPDO1(node,1);
	
	CANopen_NMT_SendCmd(node,0x01);
	
	CANopen_SendRxPDO2_OnlyMode(node,OP_PROFILE_VEL);
	
	CANopen_SendRxPDO1(node,0x06);
	
	CANopen_SendRxPDO1(node,0x07);
	
	CANopen_SendRxPDO1(node,0x0f);
	
}

/**
 * @brief 绞边节点CSP位置模式初始化
 */
void canopen_selvage_nosave_csp_position_init(CIA402_Node_t *node)
{
    if(node == NULL)
        return;
	CIA402_SetOpMode(node,OP_CSP_POS);
	
	CANopen_SDO_Write32(node, 0x6081, 0x00, 72000);
	
	CANopen_SDO_Write32(node, 0x6083, 0x00, 20);
	
	CANopen_SDO_Write32(node, 0x6084, 0x00, 20);
	
	CIA402_MapRxPDO1_ControlwordAndPos(node);
	
	CIA402_MapRxPDO2_OnlyMode(node);
	
	CIA402_MapRxPDO4_OnlyVel(node);
	
	CIA402_MapTxPDO1_StatuswordAndActualPos(node);
	
	CIA402_SetTxPDO_TransType(node,1,1);
	
	CIA402_EnableTxPDO1(node,1);
	
	CANopen_SetCyclePeriod(node,3000);
	
	CIA402_SetRxPDO_TransType(node,1,1);
	
	CANopen_SDO_Write32(node, 0x1010, 0x01, 0x65766173);
	
	CANopen_SDO_Write16(node,0x6040,0,0x06);
	
	CANopen_SDO_Write16(node,0x6040,0,0x07);
	
	CANopen_SDO_Write16(node,0x6040,0,0x0f);
	
	CANopen_NMT_SendCmd(node,0x01);
	

}

// 电子绞边点动
void selvage_sdo_jog(CIA402_Node_t *node,int32_t target_pos)
{
	if(node == NULL)
		return;
	CANopen_SDO_Write32(node,OD_TARGET_POS,0,(uint32_t)target_pos);
//	
	CANopen_SDO_Write16(node,OD_CONTROL_WORD,0,0x0f);
//	
	CANopen_SDO_Write16(node,OD_CONTROL_WORD,0,0x5f);
//	
}

// 电子绞边点动
void selvage_pdo_jog(CIA402_Node_t *node,int32_t target_pos)
{
	if(node == NULL)
		return;
	//电子绞边切换为位置模式
	CANopen_SendRxPDO2_OnlyMode(node,OP_PROFILE_POS);
//	
	CANopen_SendRxPDO3_OnlyPos(node,(uint32_t)target_pos);
//	
	CANopen_SendRxPDO1(node,0x0f);
//	
	CANopen_SendRxPDO1(node,0x5f);
//	
}

/*将异步RPDO通讯参数常用配置以及运动配置保存到flash初始化*/
void test_canopen_selvage_nosave_async_pdo_pp_mode_init(CIA402_Node_t *node)
{
    if(node == NULL)
        return;
	CIA402_SetOpMode(node,OP_PROFILE_POS);
	
//	CANopen_SDO_Write32(node, 0x607A, 0x00,2880);
//	
	CANopen_SDO_Write32(node, 0x6081, 0x00,72000);
	
	CANopen_SDO_Write32(node, 0x6083, 0x00, 20);
	
	CANopen_SDO_Write32(node, 0x6084, 0x00, 20);
	
	CIA402_MapRxPDO1_ControlwordAndPos(node);
	
	CIA402_MapRxPDO2_OnlyProfileVel(node);
	
	CIA402_MapRxPDO3_ProfileAccelAndDecel(node);
	
	CIA402_SetRxPDO_TransType(node,1,255);
	
	CIA402_SetRxPDO_TransType(node,2,255);
	
	CIA402_SetRxPDO_TransType(node,3,255);
	
	CIA402_MapTxPDO1_StatuswordAndActualPos(node);
	
	CIA402_SetTxPDO_TransType(node,1,255);
	
	CIA402_EnableRxPDO1(node,1);
	
	CIA402_EnableRxPDO2(node,1);
	
	CIA402_EnableRxPDO3(node,1);
	
	CIA402_EnableTxPDO1(node,1);
	
	CANopen_SDO_Write32(node, 0x1010, 0x01, 0x65766173);
	
	CANopen_SDO_Write16(node,0x6040,0,0x06);
	
	CANopen_SDO_Write16(node,0x6040,0,0x07);
	
	CANopen_SDO_Write16(node,0x6040,0,0x0f);
	
	CANopen_NMT_SendCmd(node,0x01);
	
}

/*============================================================
 * 电子绞边角度显示参数定义
 *============================================================
 * 实际机械：主轴 1 圈 = 绞边 2 圈（传动比 1:2）
 * 显示比例：主轴 2° = 绞边显示 1°（显示比 2:1）
 * 伺服电机：一圈 1440 脉冲
 *
 * 推导：
 *   主轴转 720°(2圈) → 绞边实际转 1440°(4圈) → 伺服走 5760 脉冲
 *   此时显示刚好回绕一圈(360°)
 *   每显示 1° = 5760 / 360 = 16 个脉冲
 *============================================================*/
#define SERVO_PPR             1440   // 伺服电机一圈脉冲数
#define MECH_RATIO            2      // 传动比：主轴1圈 = 绞边4圈
#define DISPLAY_RATIO         2      // 显示比：主轴2° = 绞边显示1°

// 显示回绕一圈对应的伺服脉冲数 = 1440 × 2 × 2 = 5760
#define DISPLAY_WRAP_PULSES   (SERVO_PPR * MECH_RATIO * DISPLAY_RATIO)

// 每显示 1° 对应的伺服脉冲数 = 5760 / 360 = 16
#define PULSES_PER_DEG        (DISPLAY_WRAP_PULSES / 360)
void upload_selvage_angle(void)
{
    int32_t rel_pos;
    uint32_t left_pos, right_pos;

    // ===== 左绞边 =====
    rel_pos = (int32_t)g_servo_node_1.actual_pos - (int32_t)g_servo_node_1.zero_offset;

    // 取模回绕（限制在 0 ~ 5760 范围内）
    rel_pos = rel_pos % (int32_t)DISPLAY_WRAP_PULSES;
    if (rel_pos < 0)
    {
        rel_pos += DISPLAY_WRAP_PULSES;   // 负数补偿，保证 0 ~ 5759
    }

    // 转换为显示角度：0 ~ 359
    left_pos = (uint32_t)(rel_pos / PULSES_PER_DEG);


    // ===== 右绞边 =====
    rel_pos = (int32_t)g_servo_node_2.actual_pos - (int32_t)g_servo_node_2.zero_offset;

    rel_pos = rel_pos % (int32_t)DISPLAY_WRAP_PULSES;
    if (rel_pos < 0)
    {
        rel_pos += DISPLAY_WRAP_PULSES;
    }

    right_pos = (uint32_t)(rel_pos / PULSES_PER_DEG);


    // ===== 上传 =====
    g_upload_data.left_edge_pos_up  = left_pos;    // 0 ~ 359
    g_upload_data.right_edge_pos_up = right_pos;   // 0 ~ 359
}
