#include "app_main.h"

/* 全局 CiA402 节点实例（方便在 Keil Watch 窗口中观察实时状态） */
CIA402_Node_t g_servo_node;

void app_main(void)
{
    delay_init(72);
//    bsp_TIM_Base_Start_IT();
	RS485_All_Init();
	RS485_Dev_t *vfd = RS485_GetDevice(&RS485_HANDLE);
    RS485_Dev_t *tpc = RS485_GetDevice(&MAX485_HANDLE);
	
	/* 2. 初始化 CiA402 节点 (CAN ID = 4) */
	CIA402_InitNode(&g_servo_node, 4);
	
	/* 3. 启动 CAN 接收及中断 */
	BSP_CAN_Start();

	
    while (1)
    {
    	/* 双路高频轮询看门狗与数据同步 */
        RS485_Poll(vfd);
        RS485_Poll(tpc);
    
        /* 处理变频器响应 */
        App_Process_VFD_Response(vfd);

        /* 处理 HMI 侧请求 */
        App_Process_TPC_Request(tpc);
		
        BSP_CAN_ProcessRx(&g_servo_node);
    }
}



