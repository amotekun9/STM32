#include "bsp_can.h"

/* ==================== 接收队列 ==================== */
static CanFrame_t    g_can_rx_pool[CAN_RX_BUF_SIZE];
static RingBuffer_t  g_can_rx_queue;

/* ==================== 发送队列 ==================== */
static CanFrame_t    g_can_tx_pool[CAN_TX_BUF_SIZE];
static RingBuffer_t  g_can_tx_queue;

/* 周期发送标记：记录本 5ms 周期内哪些节点已发送过 */
static bool g_node_sent_flag[128];

/**
 * @brief  从 COB-ID 解析目标节点 ID
 * @return 0 表示广播/无节点（NMT、SYNC 等），1~127 表示从机
 */
static uint8_t CAN_GetNodeIDFromCOBID(uint32_t cob_id)
{
    /* SDO 主站 -> 从站：0x600 + node_id */
    if (cob_id >= 0x600U && cob_id <= 0x67FU) return (uint8_t)(cob_id - 0x600U);

    /* RxPDO1 主站 -> 从站：0x200 + node_id */
    if (cob_id >= 0x200U && cob_id <= 0x27FU) return (uint8_t)(cob_id - 0x200U);

    /* RxPDO2 主站 -> 从站：0x300 + node_id */
    if (cob_id >= 0x300U && cob_id <= 0x37FU) return (uint8_t)(cob_id - 0x300U);

    /* RxPDO3 主站 -> 从站：0x400 + node_id */
    if (cob_id >= 0x400U && cob_id <= 0x47FU) return (uint8_t)(cob_id - 0x400U);

    /* RxPDO4 主站 -> 从站：0x500 + node_id */
    if (cob_id >= 0x500U && cob_id <= 0x57FU) return (uint8_t)(cob_id - 0x500U);

    /* 心跳/Bootup 等（主站一般不发给从站，保留解析） */
    if (cob_id >= 0x700U && cob_id <= 0x77FU) return (uint8_t)(cob_id - 0x700U);

    /* NMT(0x000)、SYNC(0x080)、EMCY 等：广播/无节点 */
    return 0U;
}



/**
 * @brief 初始化并使能 CAN：配置滤波器、启动、初始化 RingBuffer、使能 FIFO0 接收中断
 */
bool BSP_CAN_Start(void)
{
     /* 初始化接收队列 */
    RingBuffer_Init(&g_can_rx_queue, g_can_rx_pool, CAN_RX_BUF_SIZE, sizeof(CanFrame_t));

    /* 初始化发送队列 */
    RingBuffer_Init(&g_can_tx_queue, g_can_tx_pool, CAN_TX_BUF_SIZE, sizeof(CanFrame_t));

    CAN_FilterTypeDef sFilterConfig;

    /* 全通模式：Mask 全 0 表示所有标识符位均不检查 */
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
    sFilterConfig.SlaveStartFilterBank = 0;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        return false;
    }

    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        return false;
    }

    /* 使能 FIFO0 消息挂起中断 */
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return false;
    }

    return true;
}

/**
 * @brief  底层硬件发送：组装 TxHeader 并调用 HAL
 * @return true=成功，false=邮箱满
 */
bool CAN_HW_Send(const CanFrame_t *frame)

{
    if (frame == NULL)
    {
        return false;
    }

    CAN_TxHeaderTypeDef tx_header;
	uint32_t tick_start = HAL_GetTick();
    uint32_t tx_mailbox;

    /* CANopen 默认使用 Standard ID（11 位 COB-ID） */
    tx_header.StdId = frame->cob_id & 0x7FF;
    tx_header.ExtId = 0;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = (frame->data_len > 8) ? 8 : frame->data_len;
    tx_header.TransmitGlobalTime = DISABLE;

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
	{
	    if (HAL_GetTick() - tick_start > 10) // 超时10ms
	    {
	        return false;
	    }
	}

    return (HAL_CAN_AddTxMessage(&hcan, &tx_header, (uint8_t *)frame->data, &tx_mailbox) == HAL_OK);
}

/**
 * @brief  发送一帧 CAN 报文（非阻塞：仅入队）
 * @note   队列满时返回 false，上层可选择丢弃或重试
 */
bool CAN_SendFrame(const CanFrame_t *frame)
{
    if (frame == NULL) return false;
    return RingBuffer_Push(&g_can_tx_queue, frame);
}

///**
// * @brief  CiA402 协议栈要求的底层硬件发送接口
// * @note  HAL_CAN_AddTxMessage 内部阻塞轮询等待，高负载下不丢静默指令
// */
//bool CAN_SendFrame(const CanFrame_t *frame)
//{
//    if (frame == NULL)
//    {
//        return false;
//    }
//
//    CAN_TxHeaderTypeDef tx_header;
//	uint32_t tick_start = HAL_GetTick();
//    uint32_t tx_mailbox;
//
//    /* CANopen 默认使用 Standard ID（11 位 COB-ID） */
//    tx_header.StdId = frame->cob_id & 0x7FF;
//    tx_header.ExtId = 0;
//    tx_header.IDE   = CAN_ID_STD;
//    tx_header.RTR   = CAN_RTR_DATA;
//    tx_header.DLC   = (frame->data_len > 8) ? 8 : frame->data_len;
//    tx_header.TransmitGlobalTime = DISABLE;
//
//    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
//	{
//	    if (HAL_GetTick() - tick_start > 10) // 超时10ms
//	    {
//	        return false;
//	    }
//	}
//
//    return (HAL_CAN_AddTxMessage(&hcan, &tx_header, (uint8_t *)frame->data, &tx_mailbox) == HAL_OK);
//}

/**
 * @brief 从接收队列取出一帧原始报文（调用通用 RingBuffer_Pop）
 */
bool BSP_CAN_ReadFrame(CanFrame_t *out_frame)
{
    if (out_frame == NULL)
    {
        return false;
    }
    
    /* 使用通用环形缓冲区弹出接口 */
    return RingBuffer_Pop(&g_can_rx_queue, out_frame);
}

/**
 * @brief  发送队列调度函数 —— 必须在 5ms 定时器中断中调用
 * @note   每次中断：
 *         1. 清空本周期节点发送标记；
 *         2. 遍历发送队列，对每个节点最多发送一帧；
 *         3. 同一节点已发送过，则将该帧重新入队，等下一个 5ms；
 *         4. 广播帧（NMT/SYNC）不受节点限制，直接发送。
 */
void CAN_TX_Process(void)
{
    CanFrame_t frame;
    uint16_t   checked = 0U;

    /* 防止极端情况下死循环：最多检查整个队列长度 */
    const uint16_t max_check = CAN_TX_BUF_SIZE;

    /* 清零本周期发送标记 */
    memset(g_node_sent_flag, 0, sizeof(g_node_sent_flag));

    while ((checked < max_check) && !RingBuffer_IsEmpty(&g_can_tx_queue)) {
        if (!RingBuffer_Pop(&g_can_tx_queue, &frame)) {
            break;
        }
        checked++;

        uint8_t node_id = CAN_GetNodeIDFromCOBID(frame.cob_id);

        /* ---------- 广播帧 / 无节点帧：直接尝试发送 ---------- */
        if (node_id == 0U) {
            if (!CAN_HW_Send(&frame)) {
                /* 邮箱满：重新入队，等下一个周期 */
                RingBuffer_Push(&g_can_tx_queue, &frame);
            }
            continue;
        }

        /* ---------- 节点帧：检查本周期是否已发送过 ---------- */
        if (!g_node_sent_flag[node_id]) {
            if (CAN_HW_Send(&frame)) {
                g_node_sent_flag[node_id] = true;   /* 标记该节点本周期已发送 */
            } else {
                /* 邮箱满：重新入队 */
                RingBuffer_Push(&g_can_tx_queue, &frame);
            }
        } else {
            /* 该节点本周期已发过一帧，重新入队等下一个 5ms */
            RingBuffer_Push(&g_can_tx_queue, &frame);
        }
    }
}

/**
 * @brief  获取发送队列中待发送帧数（调试/监控）
 */
uint16_t CAN_TX_GetPendingCount(void)
{
    return RingBuffer_GetCount(&g_can_tx_queue);
}

/**
 * @brief 为单个从站节点配置硬件滤波器（可选，替代默认全通）
 * @note  bxCAN 掩码语义：Mask 位 = 1 必须匹配，= 0 不检查。
 *        32 位过滤器寄存器中 STID 位于高 16 位寄存器的 bit15:5，
 *        IDE 位位于低 16 位寄存器 bit2（标准帧 = 0）。
 */
bool BSP_CAN_ConfigNodeFilters(uint8_t node_id, uint8_t start_bank)
{
    /* 每节点占用 5 个 filter bank，F1 共 14 组 */
    if (node_id == 0U || node_id > 127U || start_bank > 9U)
    {
        return false;
    }

    CAN_FilterTypeDef f;

    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation = CAN_FILTER_ENABLE;
    f.SlaveStartFilterBank = 0;

    /* 1) 广播组：NMT(0x000) + SYNC(0x080) + EMCY(0x080+id)
     *    只检查 STID 高 3 位 bit10:8 = 000，即接收 0x000~0x0FF 标准帧 */
    f.FilterBank = start_bank;
    f.FilterIdHigh = 0x0000;
    f.FilterIdLow = 0x0000;
    f.FilterMaskIdHigh = 0xE000; /* 检查 STID bit10:8（寄存器高 16 位 bit15:13） */
    f.FilterMaskIdLow = 0x0004;  /* 检查 IDE=0（只收标准帧），RTR 不检查 */
    HAL_CAN_ConfigFilter(&hcan, &f);

    /* 2) 节点专用段：TxPDO1(0x180) / TxPDO2(0x280) / SDO响应(0x580) / 心跳引导(0x700) */
    static const uint16_t cob_bases[4] = {0x180U, 0x280U, 0x580U, 0x700U};

    for (uint8_t i = 0; i < 4U; i++)
    {
        uint16_t cob_id = (uint16_t)(cob_bases[i] + node_id);

        f.FilterBank = (uint8_t)(start_bank + 1U + i);
        f.FilterIdHigh = (uint16_t)(cob_id << 5);          /* STID -> 高 16 位 bit15:5 */
        f.FilterIdLow = 0x0000;                            /* IDE=0, RTR=0 */
        f.FilterMaskIdHigh = 0xFFE0;                       /* 检查 STID 全部 11 位 */
        f.FilterMaskIdLow = 0x0004;                        /* 检查 IDE=0，RTR 不检查 */
        if (HAL_CAN_ConfigFilter(&hcan, &f) != HAL_OK)
        {
            return false;
        }
    }

    return true;
}

/* ==================== 中断回调（ISR 内只做最轻量的事） ==================== */

/**
 * @brief CAN 接收 FIFO0 中断回调：仅入队，不处理协议
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
    if (hcan_ptr->Instance != CAN1)
    {
        return;
    }

    CAN_RxHeaderTypeDef rx_header;
    CanFrame_t frame;

    if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &rx_header, frame.data) != HAL_OK)
    {
        return;
    }

    if (rx_header.IDE == CAN_ID_STD)
    {
        frame.cob_id = rx_header.StdId;
    }
    else
    {
        frame.cob_id = rx_header.ExtId;
    }

    frame.data_len = rx_header.DLC;

    /* 使用通用环形缓冲区压入接口写入数据 */
    RingBuffer_Push(&g_can_rx_queue, &frame);
}
