#include "selvage_driver.h"
#include "bsp_delay.h"
#include "bsp_usart.h"
#include "spindle_coder.h"

extern TIM_HandleTypeDef htim4;
extern CIA402_Node_t g_servo_node_1,g_servo_node_2;
extern uint8_t encoder_z_flag;     // Z相标记位：1=收到Z脉冲，等待采样点清零
extern volatile uint8_t encoder_z_flag_cnt;     
static int16_t selvage_last = 0;        // 上一次采样的TIM4计数器值，用于计算delta
int32_t selvage_rpm = 0;

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
//	delay_ms(6);
	CIA402_SetOpMode(node,OP_PROFILE_POS);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6081, 0x00, 72000);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6083, 0x00, 20);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6084, 0x00, 20);
	delay_ms(6);
	CIA402_MapRxPDO1_ControlwordAndPos(node);
	delay_ms(6);
	CIA402_MapRxPDO2_OnlyMode(node);
	delay_ms(6);
	CIA402_MapRxPDO4_OnlyVel(node);
	delay_ms(6);
	CIA402_MapTxPDO1_StatuswordAndActualPos(node);
	delay_ms(6);
	CIA402_SetTxPDO_TransType(node,1,1);
	delay_ms(6);
	CIA402_EnableTxPDO1(node,1);
	delay_ms(6);
	CIA402_EnableTxPDO2(node,0);
	delay_ms(6);
	CANopen_SetCyclePeriod(node,2000);
	delay_ms(6);
	CIA402_SetRxPDO_TransType(node,1,1);
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

/*将异步RPDO通讯参数常用配置以及运动配置保存到flash初始化*/
void canopen_selvage_nosave_async_pdo_pv_mode_init(CIA402_Node_t *node)
{
    if(node == NULL)
        return;
	CIA402_SetOpMode(node,OP_PROFILE_VEL);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6083, 0x00, 20);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6084, 0x00, 20);
	delay_ms(6);
	CIA402_MapRxPDO1_ControlwordModeAndVel(node);
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
	CIA402_EnableTxPDO2(node,0);
	delay_ms(6);
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
void canopen_selvage_nosave_sync_pdo_pv_mode_init(CIA402_Node_t *node)
{
    if(node == NULL)
        return;
	CIA402_SetOpMode(node,OP_PROFILE_VEL);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6083, 0x00, 20);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6084, 0x00, 20);
	delay_ms(6);
	CIA402_MapRxPDO1_ControlwordModeAndVel(node);
	delay_ms(6);
	CIA402_MapRxPDO2_OnlyMode(node);
	delay_ms(6);
	CIA402_MapRxPDO4_OnlyVel(node);
	delay_ms(6);
	CIA402_MapTxPDO1_StatuswordAndActualPos(node);
	delay_ms(6);
	CIA402_SetTxPDO_TransType(node,1,1);
	delay_ms(6);
	CIA402_EnableTxPDO1(node,1);
	delay_ms(6);
	CIA402_EnableTxPDO2(node,0);
	delay_ms(6);
	CANopen_SetCyclePeriod(node,3000);
	delay_ms(6);
	CIA402_SetRxPDO_TransType(node,1,1);
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

/**
 * @brief 绞边节点位置模式初始化
 */
void canopen_selvage_pp_position_init(CIA402_Node_t *node)
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

/**
 * @brief 绞边节点CSP位置模式初始化
 */
void canopen_selvage_nosave_csp_position_init(CIA402_Node_t *node)
{
    if(node == NULL)
        return;
	CIA402_SetOpMode(node,OP_CSP_POS);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6081, 0x00, 72000);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6083, 0x00, 20);
	delay_ms(6);
	CANopen_SDO_Write32(node, 0x6084, 0x00, 20);
	delay_ms(6);
	CIA402_MapRxPDO1_ControlwordAndPos(node);
	delay_ms(6);
	CIA402_MapRxPDO2_OnlyMode(node);
	delay_ms(6);
	CIA402_MapRxPDO4_OnlyVel(node);
	delay_ms(6);
	CIA402_MapTxPDO1_StatuswordAndActualPos(node);
	delay_ms(6);
	CIA402_SetTxPDO_TransType(node,1,1);
	delay_ms(6);
	CIA402_EnableTxPDO1(node,1);
	delay_ms(6);
	CANopen_SetCyclePeriod(node,3000);
	delay_ms(6);
	CIA402_SetRxPDO_TransType(node,1,1);
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

// 电子绞边点动
void selvage_sdo_jog(CIA402_Node_t *node,int32_t target_pos)
{
	if(node == NULL)
		return;
	CANopen_SDO_Write32(node,OD_TARGET_POS,0,(uint32_t)target_pos);
	delay_ms(6);
	CANopen_SDO_Write16(node,OD_CONTROL_WORD,0,0x0f);
	delay_ms(6);
	CANopen_SDO_Write16(node,OD_CONTROL_WORD,0,0x5f);
	delay_ms(6);
}

// 电子绞边点动
void selvage_pdo_jog(CIA402_Node_t *node,int32_t target_pos)
{
	if(node == NULL)
		return;
	//电子绞边切换为位置模式
	CANopen_SendRxPDO2_OnlyMode(node,OP_PROFILE_POS);
	delay_ms(6);
	CANopen_SendRxPDO3_OnlyPos(node,(uint32_t)target_pos);
	delay_ms(6);
	CANopen_SendRxPDO1(node,0x0f);
	delay_ms(6);
	CANopen_SendRxPDO1(node,0x5f);
	delay_ms(6);
}

void Selvage_Callback(void)
{
//	static bool last_run_sta = 0;
//	bool now_run_sta = (encoder_rpm != 0) ? 1 : 0;

    // 读取TIM4编码器当前硬件计数值（16位）
    int16_t enc_now = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
	int16_t enc_delta = enc_now - selvage_last;
	int32_t selvage_delta = enc_delta;
	static int16_t speed_filt = 0;
	
	if (encoder_z_flag)
	{
		encoder_z_flag = 0;
//		printf("selvage_last=%d\n",selvage_last);
		if (enc_delta <0 && selvage_last >0)
		{
			if (selvage_last > 720)
			{
				selvage_delta = enc_now + 1440 - selvage_last;
			}
		}
		else if (enc_delta >0 && selvage_last<0)
		{
			if (selvage_last < -720)
			{
				selvage_delta = enc_now - 1440 - selvage_last;
			}
		}
		if (encoder_z_flag_cnt == 1)
		{
			selvage_delta = enc_now;
		}
	}

//	selvage_delta *= 25;
    selvage_last = enc_now;               // 更新本次计数值，作为下一次采样的基准

	// 一阶低通滤波：平滑速度，抑制抖动，滤波系数 K=4
    speed_filt = (speed_filt * (CODER_FILTER_K - 1) + selvage_delta) / CODER_FILTER_K;
	encoder_rpm = (int32_t)abs(speed_filt) * 60000L / (4L * ENC_PULSE_PER_ROUND * SELVAGE_SAMPLE_MS);
    g_upload_data.encoder_rpm_up = encoder_rpm;
	

	//如果中断采集主轴有脉冲变化，将该变化同步给绞边进行位置跟随指令，给100毫秒的跟随完成时间
	if (selvage_delta != 0)
	{
		/*位置跟随模式*/
		CANopen_SendRxPDO1_ControlwordAndPos(&g_servo_node_1,0x000f,(int32_t)selvage_delta);
		CANopen_SendRxPDO1_ControlwordAndPos(&g_servo_node_2,0x000f,(int32_t)selvage_delta);
//		delay_ms(2);
		CANopen_SendRxPDO1_ControlwordAndPos(&g_servo_node_1,0x005f,(int32_t)selvage_delta);
		CANopen_SendRxPDO1_ControlwordAndPos(&g_servo_node_2,0x005f,(int32_t)selvage_delta);

		/*周期同步位置跟随模式*/
//		CANopen_SendRxPDO1_ControlwordAndPos(&g_servo_node_1,0x000f,(int32_t)selvage_delta);
//		CANopen_SendRxPDO1_ControlwordAndPos(&g_servo_node_2,0x000f,(int32_t)selvage_delta);
//		delay_ms(6);
//		CANopen_SendSYNC();
//		delay_ms(1);
//		CANopen_SendRxPDO1_ControlwordAndPos(&g_servo_node_1,0x005f,(int32_t)selvage_delta);
//		CANopen_SendRxPDO1_ControlwordAndPos(&g_servo_node_2,0x005f,(int32_t)selvage_delta);
//		delay_ms(6);
//		CANopen_SendSYNC();
//		delay_ms(20);
	}

	/*速度跟随模式*/
//	if (now_run_sta ==1)
//	{
//		CANopen_SendRxPDO4_OnlyVel(&g_servo_node_1,encoder_rpm*10);
//		CANopen_SendRxPDO4_OnlyVel(&g_servo_node_2,encoder_rpm*10);
//		delay_ms(6);
//	}
//    if(now_run_sta != last_run_sta)
//    {
//        if(now_run_sta == 0)	//主轴从运行到停止
//        {
//		    CANopen_SendRxPDO4_OnlyVel(&g_servo_node_1,0);
//			CANopen_SendRxPDO4_OnlyVel(&g_servo_node_2,0);
//			delay_ms(6);
//        }
//        else	//主轴从停止变为运行态
//        {
//        	
//        }
//        last_run_sta = now_run_sta;                    // 更新状态记录
//    }
	
	/* 绞边与主轴角度1:1*/
	uint32_t left_pos,right_pos;

	if ((g_servo_node_1.actual_pos -g_servo_node_1.zero_offset) < 0 )
	{

		left_pos = 360 +((g_servo_node_1.actual_pos -g_servo_node_1.zero_offset)%1440)/4;
	}
	else
	{
		left_pos = ((g_servo_node_1.actual_pos -g_servo_node_1.zero_offset)%1440)/4;
	}
	
	if ((g_servo_node_2.actual_pos -g_servo_node_2.zero_offset) < 0 )
	{

		right_pos = 360 +((g_servo_node_2.actual_pos -g_servo_node_2.zero_offset)%1440)/4;
	}
	else
	{
		right_pos = ((g_servo_node_2.actual_pos -g_servo_node_2.zero_offset)%1440)/4;
	}
	g_upload_data.left_edge_pos_up = left_pos;
	g_upload_data.right_edge_pos_up = right_pos;
	int32_t dev_angle = encoder_angle - g_upload_data.left_edge_pos_up;
	if ( dev_angle < 0)
	{
		dev_angle += 360;
	}
//	printf("enc_now=%d,enc_angle=%d,selvage_angle=%d,dev_angle=%d,enc_delta=%d,selvage_delta=%d,encoder_rpm=%d\n\n",enc_now,encoder_angle,g_upload_data.left_edge_pos_up,dev_angle,enc_delta,selvage_delta,encoder_rpm);
}


