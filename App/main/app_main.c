#include "app_main.h"

/* 全局 CiA402 节点实例（方便在 Keil Watch 窗口中观察实时状态） */
CIA402_Node_t g_servo_node_1,g_servo_node_2;
CAN_Step_Node_t g_step_node;
extern int32_t into_send_selvage_delta;
extern int16_t selvage_delta;
extern int32_t actual_selvage_delta;

void app_main(void)
{
    delay_init(72);                           //加载保存参数
    delay_ms(6000);
    RS485_All_Init();
    RS485_Dev_t *vfd = RS485_GetDevice(&RS485_HANDLE); 
    RS485_Dev_t *tpc = RS485_GetDevice(&MAX485_HANDLE);
	
    // CAN 初始化（例如设置 3 秒超时）
	if (!BSP_CAN_Init_WithTimeout(3000))
	{
		// 3 秒内初始化失败的处理
		Error_Handler(); 
	}
	
    Coder_Init();                          //主轴编码器初始化
    modbus_slave_init();                               //与触摸屏通讯任务
    upload_param_load();    
	CIA402_InitAndRegisterNode(&g_servo_node_1, 1);
    CIA402_InitAndRegisterNode(&g_servo_node_2, 2);
    CIA402_InitAndRegisterNode(&g_step_node, 3);
//	bsp_TIM7_Base_Start_IT();
    bsp_TIM6_Base_Start_IT();
	canopen_selvage_nosave_async_pdo_pp_mode_init(&g_servo_node_1);
	canopen_selvage_nosave_async_pdo_pp_mode_init(&g_servo_node_2);
    STEP_Motor_Init(&g_step_node);                       //电子夹辊步进电机初始化
    
    while (1)
    {
//    	printf("cnt =%d,angle=%d,rpm = %d,low=%d,high=%d,low_send=%d,high_send=%d\n",into_send_selvage_delta,encoder_angle,encoder_rpm,low_speed_cnt,high_speed_cnt,low_send_cnt,high_send_cnt);
        /* 双路高频轮询看门狗与数据同步 */
        RS485_Poll(vfd);
        RS485_Poll(tpc);
		
        /* 处理变频器响应 */
        App_Process_VFD_Response(vfd);

        /* 处理 HMI 侧请求 */
        modbus_slave_task();  //单片机modbus从机任务，必须不断轮询，处理主机请求;
        ModbusHolding_RefreshUpload();// 业务数据写入保持寄存器
        Modbus_CmdParse();            // 解析触摸屏下发的控制bit指令
        
        
		scissor_poll(); //剪刀电机任务，必须不断轮询，处理主机请求; 
		
		CANopen_ProcessBus();   //处理 CANopen 总线
		
    }
}



