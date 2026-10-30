#include "selvage_driver.h"
#include "bsp_delay.h"

/**
 * @brief 没将RPDO通讯参数常用配置以及运动配置保存到flash则绞边节点调用该初始化
 */
void canopen_selvage_nosave_async_pdo_init(CIA402_Node_t *node)
{
    if(node == NULL)
        return;
	CIA402_SetOpMode(node,OP_PROFILE_POS);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6081, 0x00, 1800000);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6083, 0x00, 10);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6084, 0x00, 10);
	delay_ms(6);
	CANopen_SDO_Write16(node,0x6040,0,0x06);
	delay_ms(6);
	CANopen_SDO_Write16(node,0x6040,0,0x07);
	delay_ms(6);
	CANopen_SDO_Write16(node,0x6040,0,0x0f);
	delay_ms(6);
	CIA402_MapRxPDO1_ControlwordAndPos(node);
	delay_ms(6);
	CIA402_MapRxPDO2_OnlyMode(node);
	delay_ms(6);
	CIA402_MapRxPDO4_OnlyVel(node);
	delay_ms(6);
	CIA402_MapTxPDO1_StatuswordAndActualPos(node);
	delay_ms(6);
	CIA402_SetTxPDO_TransType(node,1,255);
	delay_ms(6);
	CIA402_EnableTxPDO1(node,1);
	delay_ms(6);
	CIA402_SetRxPDO_TransType(node,1,255);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x1010, 0x01, 0x65766173);
	delay_ms(6);
	CANopen_NMT_SendCmd(node,0x01);
	delay_ms(6);
}

/**
 * @brief 没将RPDO通讯参数常用配置以及运动配置保存到flash则绞边节点调用该初始化
 */
void canopen_selvage_nosave_sync_pdo_init(CIA402_Node_t *node)
{
    if(node == NULL)
        return;
	CIA402_SetOpMode(node,OP_PROFILE_POS);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6081, 0x00, 1800000);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6083, 0x00, 10);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6084, 0x00, 10);
	delay_ms(6);
	CANopen_SDO_Write16(node,0x6040,0,0x06);
	delay_ms(6);
	CANopen_SDO_Write16(node,0x6040,0,0x07);
	delay_ms(6);
	CANopen_SDO_Write16(node,0x6040,0,0x0f);
	delay_ms(6);
	CIA402_MapRxPDO1_ControlwordAndPos(node);
	delay_ms(6);
	CIA402_MapRxPDO2_OnlyMode(node);
	delay_ms(6);
	CIA402_MapRxPDO4_OnlyVel(node);
	delay_ms(6);
	CIA402_MapTxPDO1_StatuswordAndActualPos(node);
	delay_ms(6);
	CIA402_SetTxPDO_TransType(node,1,255);
	delay_ms(6);
	CIA402_EnableTxPDO1(node,1);
	delay_ms(6);
	CIA402_SetRxPDO_TransType(node,1,1);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x1010, 0x01, 0x65766173);
	delay_ms(6);
	CANopen_NMT_SendCmd(node,0x01);
	delay_ms(6);
}

/**
 * @brief 绞边节点位置模式初始化
 */
void canopen_selvage_position_init(CIA402_Node_t *node)
{
    if(node == NULL)
        return;
	CIA402_SetTxPDO_TransType(node,1,255);
	delay_ms(6);
//	CIA402_SetTxPDO_TransType(node,2,255);
//	delay_ms(6);
	CIA402_EnableTxPDO1(node,1);
	delay_ms(6);
//	CIA402_EnableTxPDO2(node,1);
//	delay_ms(6);
	CANopen_NMT_SendCmd(node,0x01);
	delay_ms(6);
	CANopen_SendRxPDO2_OnlyMode(node,OP_PROFILE_POS);
	delay_ms(6);
	CANopen_SendRxPDO1(node,0x06);
	delay_ms(6);
	CANopen_SendRxPDO1(node,0x07);
	delay_ms(6);
	CANopen_SendRxPDO1(node,0x0f);
	delay_ms(6);
}

/**
 * @brief 绞边节点速度模式初始化
 */
void canopen_selvage_speed_init(CIA402_Node_t *node)
{
    if(node == NULL)
        return;
	CIA402_SetTxPDO_TransType(node,1,255);
	delay_ms(6);
	CIA402_SetTxPDO_TransType(node,2,255);
	delay_ms(6);
	CIA402_EnableTxPDO1(node,1);
	delay_ms(6);
	CANopen_NMT_SendCmd(node,0x01);
	delay_ms(6);
	CANopen_SendRxPDO2_OnlyMode(node,OP_PROFILE_VEL);
	delay_ms(6);
	CANopen_SendRxPDO1(node,0x06);
	delay_ms(6);
	CANopen_SendRxPDO1(node,0x07);
	delay_ms(6);
	CANopen_SendRxPDO1(node,0x0f);
	delay_ms(6);
}

// 电子绞边点动
void selvage_sdo_jog_relative(CIA402_Node_t *node,int32_t target_pos)
{
	if(node == NULL)
		return;
	CANopen_SDO_Write32(node,OD_TARGET_POS,0,(uint32_t)target_pos);
	delay_ms(6);
	CANopen_SDO_Write16(node,OD_CONTROL_WORD,0,0x5f);
	delay_ms(6);
	CANopen_SDO_Write16(node,OD_CONTROL_WORD,0,0x0f);
	delay_ms(6);
}

// 电子绞边点动
void selvage_pdo_jog_relative(CIA402_Node_t *node,int32_t target_pos)
{
	if(node == NULL)
		return;
	//电子绞边切换为位置模式
	CANopen_SendRxPDO2_OnlyMode(node,OP_PROFILE_POS);
	delay_ms(6);
	CANopen_SendRxPDO3_OnlyPos(node,(uint32_t)target_pos);
	delay_ms(6);
	CANopen_SendRxPDO1(node,0x5f);
	delay_ms(6);
	CANopen_SendRxPDO1(node,0x0f);
	delay_ms(6);
}


