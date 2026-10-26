#include "app_main.h"

/* 全局 CiA402 节点实例（方便在 Keil Watch 窗口中观察实时状态） */
CIA402_Node_t g_servo_node_1;
CIA402_Node_t g_servo_node_2;
CAN_Step_Node_t g_step_node;


void app_main(void)
{
    delay_init(72);
    bsp_TIM_Base_Start_IT();
    RS485_All_Init();
    RS485_Dev_t *vfd = RS485_GetDevice(&RS485_HANDLE); 
    RS485_Dev_t *tpc = RS485_GetDevice(&MAX485_HANDLE);
    modbus_slave_init();                               //与触摸屏通讯任务
    upload_param_load();  //加载保存参数
    BSP_CAN_Start();
	CIA402_InitAndRegisterNode(&g_servo_node_1, 1);
    CIA402_InitAndRegisterNode(&g_servo_node_2, 2);
    CIA402_InitAndRegisterNode(&g_step_node, 3);
    STEP_Motor_Init(&g_step_node);

    while (1)
    {
        /* 双路高频轮询看门狗与数据同步 */
        RS485_Poll(vfd);
        RS485_Poll(tpc);

        /* 处理变频器响应 */
        App_Process_VFD_Response(vfd);

        /* 处理 HMI 侧请求 */
        modbus_slave_task();  //单片机modbus从机任务，必须不断轮询，处理主机请求;
        ModbusHolding_RefreshUpload();// 业务数据写入保持寄存器
        Modbus_CmdParse();            // 解析触摸屏下发的控制bit指令

        /* 处理电子夹纱器请求 */
        Pinch_Motor_task(); //夹持电机任务，必须不断轮询，处理主机请求; 
        
		scissor_poll(); //剪刀电机任务，必须不断轮询，处理主机请求; 

		CANopen_ProcessBus();   //处理 CANopen 总线
		
    }
}



