#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "canopen.h"
#include "ring_buffer.h"
#include "bsp_usart.h"

/* 引用 CiA402 协议栈头文件，使用 CanFrame_t 和 CIA402_Node_t */

/* ==================== 缓冲区配置 ==================== */
#define CAN_RX_BUF_SIZE     512U    /* 接收环形队列长度 */
#define CAN_TX_BUF_SIZE     512U     /* 发送环形队列长度，需为 2 的幂 */


extern CAN_HandleTypeDef hcan; /* CAN 硬件句柄（CubeMX 生成） */

bool CAN_HW_Send(const CanFrame_t *frame);


/**
 * @brief  初始化并使能 CAN 外设及接收中断（默认全通滤波器，接收全部标准/扩展帧）
 * @return true=成功, false=失败
 * @note   如需硬件过滤收紧，调用后在主循环调用 BSP_CAN_ConfigNodeFilters() 覆盖
 */
bool BSP_CAN_Start(void);

bool BSP_CAN_Init_WithTimeout(uint32_t timeout_ms);

/**
 * @brief  发送一帧 CAN 报文（CANopen 协议栈底层发送接口，带 Tx 邮箱保护）
 * @param  frame: 待发送帧指针
 * @return true=成功, false=失败（参数错误 / 3 个 Tx 邮箱全满）
 * @note   邮箱满时返回失败，由上层决定重试策略（PDO 类报文下次发送即覆盖，无需重试）
 */
bool CAN_SendFrame(const CanFrame_t *frame);

/**
 * @brief  从环形队列取出一帧 CAN 报文
 * @param  out_frame: 输出帧指针
 * @return true=成功取出, false=队列为空
 */
bool BSP_CAN_ReadFrame(CanFrame_t *out_frame);

/**
 * @brief  发送队列调度：5ms 定时器中断中调用
 * @note   每次中断遍历发送队列，对每个从机最多发送一帧
 */
void CAN_TX_Process(void);

/**
 * @brief  为单个从站节点配置硬件滤波器（可选，替代默认全通）
 * @param  node_id:    从站节点 ID（1~127）
 * @param  start_bank: 起始滤波器组号（STM32F1 共 14 组，每节点占用 5 组）
 * @return true=配置成功, false=参数越界
 * @note   覆盖的 COB-ID：广播(0x000~0x0FF) + TxPDO1(0x180) + TxPDO2(0x280)
 *         + SDO响应(0x580) + 心跳/引导(0x700)，每段精确匹配 node_id；
 *         仅接收标准帧（IDE=0），屏蔽扩展帧，可显著降低无关中断负载
 */
bool BSP_CAN_ConfigNodeFilters(uint8_t node_id, uint8_t start_bank);

/**
 * @brief  获取发送队列中待发送帧数（调试用）
 */
uint16_t CAN_TX_GetPendingCount(void);

#ifdef __cplusplus
}
#endif

#endif /* _BSP_CAN_H_ */
