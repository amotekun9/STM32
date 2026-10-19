/**
 * @file    canopen.c
 * @brief   CANopen CiA 301 / CiA 402 协议栈核心实现文件
 * @details 实现了 SDO 快速下载、NMT 状态机控制、PDO 高速收发以及 DS402 运动控制指令
 */

#include "canopen.h"

/* 字节提取宏定义 (小端模式拆分) */
#define LSB(x)      ((uint8_t)((x) & 0xFF))         /* 获取 16 位数据的低字节 */
#define MSB(x)      ((uint8_t)(((x) >> 8) & 0xFF))    /* 获取 16 位数据的高字节 */

/**
 * @brief  底层 SDO 帧组装与发送内部静态函数
 * @param  node:     目标节点指针
 * @param  cmd:      SDO 命令字 (如 0x23 写 4 字节，0x2F 写 1 字节等)
 * @param  index:    对象字典索引 (16 位)
 * @param  subindex: 对象字典子索引 (8 位)
 * @param  data:     待写入的 32 位有效数据
 * @return CO_Err_t: CO_OK=成功, CO_ERROR_PARAM=参数错, CO_ERROR_HW=硬件发送错
 */
static CO_Err_t SDO_SendFrame(CIA402_Node_t *node, uint8_t cmd, uint16_t index, uint8_t subindex, uint32_t data)
{
    if (!node)
    {
        return CO_ERROR_PARAM;
    }

    CanFrame_t frame;
    /* SDO 请求 COB-ID = 0x600 + Node_ID */
    frame.cob_id = COB_ID_SDO_TX_BASE + node->node_id;
    frame.data_len = 8; /* SDO 标准固定为 8 字节 */
    
    /* 组装 SDO 报文格式：[Command] [Index Low] [Index High] [SubIndex] [Data 4Bytes] */
    frame.data[0] = cmd;
    frame.data[1] = LSB(index);
    frame.data[2] = MSB(index);
    frame.data[3] = subindex;
    
    /* 小端模式：低字节在前，高字节在后 */
    frame.data[4] = (uint8_t)(data & 0xFF);
    frame.data[5] = (uint8_t)((data >> 8) & 0xFF);
    frame.data[6] = (uint8_t)((data >> 16) & 0xFF);
    frame.data[7] = (uint8_t)((data >> 24) & 0xFF);

    return CAN_SendFrame(&frame) ? CO_OK : CO_ERROR_HW;
}

/**
 * @brief  初始化 CANopen 节点控制块
 * @param  node:    目标节点结构体指针
 * @param  node_id: 节点物理 ID (1 ~ 127)
 */
void CIA402_InitNode(CIA402_Node_t *node, uint8_t node_id)
{
    if (!node || node_id == 0 || node_id > 127)
    {
        return;
    }
    memset(node, 0, sizeof(CIA402_Node_t));
    node->node_id = node_id;
}

/**
 * @brief  配置从站心跳监测时间 (Object 0x1017)
 * @param  node:         目标节点指针
 * @param  heartbeat_ms: 心跳周期 (单位: 毫秒)
 */
CO_Err_t CIA402_SetHeartbeatTime(CIA402_Node_t *node, uint16_t heartbeat_ms)
{
    return CANopen_SDO_Write16(node, 0x1017, 0x00, heartbeat_ms);
}

/**
 * @brief  向指定节点发送 NMT 网络管理命令
 * @param  node: 目标节点指针
 * @param  cmd:  NMT 命令字 (0x01=进入Operational, 0x02=停止, 0x80=进入Pre-Op等)
 */
CO_Err_t CANopen_NMT_SendCmd(CIA402_Node_t *node, uint8_t cmd)
{
    if (!node) return CO_ERROR_PARAM;
    CanFrame_t frame = { COB_ID_NMT, 2, {cmd, node->node_id} };
    return CAN_SendFrame(&frame) ? CO_OK : CO_ERROR_HW;
}

/**
 * @brief  广播发送 NMT 网络管理命令 (针对所有节点)
 * @param  cmd: NMT 命令字
 */
CO_Err_t CANopen_NMT_Broadcast(uint8_t cmd)
{
    CanFrame_t frame = { COB_ID_NMT, 2, {cmd, 0x00} };
    return CAN_SendFrame(&frame) ? CO_OK : CO_ERROR_HW;
}

/**
 * @brief  发送 SYNC 同步帧 (COB-ID = 0x080)
 */
CO_Err_t CANopen_SendSYNC(void)
{
    CanFrame_t frame = { COB_ID_SYNC, 0, {0} };
    return CAN_SendFrame(&frame) ? CO_OK : CO_ERROR_HW;
}

/**
 * @brief  SDO 写入 1 字节数据
 */
CO_Err_t CANopen_SDO_Write8(CIA402_Node_t *node, uint16_t index, uint8_t subindex, uint8_t value)
{
    return SDO_SendFrame(node, SDO_CMD_WRITE_1B, index, subindex, value);
}

/**
 * @brief  SDO 写入 2 字节数据
 */
CO_Err_t CANopen_SDO_Write16(CIA402_Node_t *node, uint16_t index, uint8_t subindex, uint16_t value)
{
    return SDO_SendFrame(node, SDO_CMD_WRITE_2B, index, subindex, value);
}

/**
 * @brief  SDO 写入 4 字节数据
 */
CO_Err_t CANopen_SDO_Write32(CIA402_Node_t *node, uint16_t index, uint8_t subindex, uint32_t value)
{
    return SDO_SendFrame(node, SDO_CMD_WRITE_4B, index, subindex, value);
}

/**
 * @brief  设置指定 TxPDO 的传输类型 (Transmission Type)
 * @param  node:       目标节点指针
 * @param  pdo_index:  TxPDO 序号 (1 ~ 4)
 * @param  trans_type: 传输类型 (1=每个SYNC同步触发, 255=异步事件触发)
 */
CO_Err_t CIA402_SetTxPDO_TransType(CIA402_Node_t *node, uint8_t pdo_index, uint8_t trans_type)
{
    if (!node || pdo_index < 1 || pdo_index > 4) return CO_ERROR_PARAM;
    uint16_t index = 0x1800 + (pdo_index - 1); /* TxPDO 通信参数基址 0x1800起步 */
    return CANopen_SDO_Write8(node, index, 0x02, trans_type);
}

/************************ RxPDO 发送接口实现 ************************/

/**
 * @brief  通过 RxPDO1 发送控制字 (主->从，2字节纯数据，无索引)
 */
CO_Err_t CANopen_SendRxPDO1(CIA402_Node_t *node, uint16_t ctrl_word)
{
    if (!node) return CO_ERROR_PARAM;
    CanFrame_t frame;
    frame.cob_id = COB_ID_RXPDO1_BASE + node->node_id; /* 例如 0x200 + 4 = 0x204 */
    frame.data_len = 2;
    frame.data[0] = LSB(ctrl_word);
    frame.data[1] = MSB(ctrl_word);
    node->control_word = ctrl_word;
    return CAN_SendFrame(&frame) ? CO_OK : CO_ERROR_HW;
}

/**
 * @brief  通过 RxPDO2 同时发送控制字与目标位置 (主->从，6字节纯数据)
 */
CO_Err_t CANopen_SendRxPDO2_Pos(CIA402_Node_t *node, uint16_t ctrl_word, int32_t target_pos)
{
    if (!node) return CO_ERROR_PARAM;
    CanFrame_t frame;
    frame.cob_id = COB_ID_RXPDO2_BASE + node->node_id; /* 例如 0x300 + 4 = 0x304 */
    frame.data_len = 6;
    
    /* 前 2 字节：控制字 (0x6040) */
    frame.data[0] = LSB(ctrl_word);
    frame.data[1] = MSB(ctrl_word);
    
    /* 后 4 字节：目标位置 (0x607A) */
    frame.data[2] = (uint8_t)(target_pos & 0xFF);
    frame.data[3] = (uint8_t)((target_pos >> 8) & 0xFF);
    frame.data[4] = (uint8_t)((target_pos >> 16) & 0xFF);
    frame.data[5] = (uint8_t)((target_pos >> 24) & 0xFF);
    
    node->control_word = ctrl_word;
    return CAN_SendFrame(&frame) ? CO_OK : CO_ERROR_HW;
}

/************************ 接收分发处理 (解析从站上报的 TxPDO) ************************/

/**
 * @brief  CANopen 接收报文分发处理函数 (在主循环或接收中断安全上下文中调用)
 * @param  node:  目标节点指针
 * @param  frame: 收到的 CAN 帧指针
 */
void CANopen_RecvHandler(CIA402_Node_t *node, const CanFrame_t *frame)
{
    if (!node || !frame) return;
	
    /* 1. 匹配 TxPDO1（从->主：状态字 + 实际位置） */
    if (frame->cob_id == (COB_ID_TXPDO1_BASE + node->node_id))
    {
        if (frame->data_len >= 2)
        {
            node->status_word = (uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8);
        }
        if (frame->data_len >= 6)
        {
            uint32_t pos_raw = (uint32_t)frame->data[2] | ((uint32_t)frame->data[3] << 8) |
                               ((uint32_t)frame->data[4] << 16) | ((uint32_t)frame->data[5] << 24);
            node->actual_pos = (int32_t)pos_raw;
        }
        return;
    }

    /* 2. 匹配 TxPDO2（从->主：实际速度 + 实际转矩） */
    if (frame->cob_id == (COB_ID_TXPDO2_BASE + node->node_id))
    {
        if (frame->data_len >= 4)
        {
            uint32_t vel_raw = (uint32_t)frame->data[0] | ((uint32_t)frame->data[1] << 8) |
                               ((uint32_t)frame->data[2] << 16) | ((uint32_t)frame->data[3] << 24);
            node->actual_vel = (int32_t)vel_raw;
        }
        if (frame->data_len >= 6)
        {
            uint16_t tor_raw = (uint16_t)frame->data[4] | ((uint16_t)frame->data[5] << 8);
            node->actual_tor = (int16_t)tor_raw;
        }
        return;
    }
}

/************************ DS402 运动控制高级 API 实现 ************************/

/**
 * @brief  设置驱动器运行模式 (Object 0x6060)
 */
CO_Err_t CIA402_SetOpMode(CIA402_Node_t *node, OpMode_t mode)
{
    if (!node) return CO_ERROR_PARAM;
    node->target_op_mode = mode;
    return CANopen_SDO_Write8(node, OD_OP_MODE, 0, (uint8_t)mode);
}

/**
 * @brief  伺服安全失能 (通过下发控制字 0x0000 切换至 Switch On Disabled 状态)
 */
CO_Err_t CIA402_ServoDisable(CIA402_Node_t *node)
{
    if (!node) return CO_ERROR_PARAM;
    return CANopen_SendRxPDO1(node, 0x0000);
}

/**
 * @brief  PP (Profile Position) 模式下发目标位置运动指令
 * @param  node:        目标节点指针
 * @param  target_pos:  目标位置 (脉冲数)
 * @param  is_absolute: true=绝对定位模式, false=相对定位模式
 */
CO_Err_t CIA402_PPMoveTo(CIA402_Node_t *node, int32_t target_pos, bool is_absolute)
{
    if (!node) return CO_ERROR_PARAM;
    
    uint16_t ctrl = node->control_word;
    if (is_absolute) ctrl &= ~CTRL_REL_POS; /* 绝对定位：Bit 6 清 0 */
    else             ctrl |= CTRL_REL_POS;  /* 相对定位：Bit 6 置 1 */

    ctrl |= CTRL_NEW_SETPOINT;              /* 触发 Bit 4 上升沿开始运动规划 */
    return CANopen_SendRxPDO2_Pos(node, ctrl, target_pos);
}

/**
 * @brief  通用 PDO 使能/禁用配置函数 (自动处理 Bit 31 开关)
 * @param  node:      目标节点指针
 * @param  is_tx:     true=操作 TxPDO (基址0x1800), false=操作 RxPDO (基址0x1400)
 * @param  pdo_index: PDO 序号 (1 ~ 4)
 * @param  enable:    true=使能 (Bit31清0), false=禁用 (Bit31置1)
 * @param  cob_id:    通信 COB-ID
 */
CO_Err_t CIA402_SetPDO_Enable(CIA402_Node_t *node, bool is_tx, uint8_t pdo_index, bool enable, uint16_t cob_id)
{
    if (!node || pdo_index < 1 || pdo_index > 4)
    {
        return CO_ERROR_PARAM;
    }

    /* 根据类型选择通信参数字典基址：RxPDO=0x1400, TxPDO=0x1800 */
    uint16_t base_index = is_tx ? 0x1800 : 0x1400;
    uint16_t index = base_index + (pdo_index - 1);

    /* 拼装 32 位 COB-ID 数据 (Bit 31 决定使能状态) */
    uint32_t val = (uint32_t)(cob_id & 0x7FF);
    if (!enable)
    {
        val |= 0x80000000U; /* Bit 31 置 1 代表禁用该 PDO */
    }

    /* 写入子索引 0x01 (COB-ID 配置项) */
    return CANopen_SDO_Write32(node, index, 0x01, val);
}

/**
 * @brief  PP 模式一键综合调试函数 (组合下发运行模式、速度、加减速、目标位置并触发)
 */
CO_Err_t CIA402_PP_DebugMove(CIA402_Node_t *node, int32_t target_pos, uint32_t profile_vel, uint32_t accel, uint32_t decel, bool is_absolute)
{
    if (!node)
    {
        return CO_ERROR_PARAM;
    }

    /* 1. 确保当前切换为 PP 模式 (0x6060 = 1) */
    CANopen_SDO_Write8(node, OD_OP_MODE, 0, (uint8_t)OP_PROFILE_POS);
    
    /* 2. 下发 PP 模式三大运动学参数 */
    CANopen_SDO_Write32(node, 0x6081, 0, profile_vel);  /* 轮廓速度 (0x6081) */
    CANopen_SDO_Write32(node, 0x6083, 0, accel);        /* 加速度斜率 (0x6083) */
    CANopen_SDO_Write32(node, 0x6084, 0, decel);        /* 减速时间斜率 (0x6084) */
    
    /* 3. 写入目标位置 (0x607A) */
    CANopen_SDO_Write32(node, OD_TARGET_POS, 0, (uint32_t)target_pos);

    /* 4. 组装控制字：使能输出 (0x000F) 并拉高 Bit 4 (New Setpoint) */
    uint16_t ctrl = 0x000F; 
    
    if (is_absolute) {
        ctrl &= ~CTRL_REL_POS; 
    } else {
        ctrl |= CTRL_REL_POS;  
    }
    
    ctrl |= CTRL_NEW_SETPOINT; 

    /* 通过 RxPDO2 将控制字与位置一体化下发 */
    return CANopen_SendRxPDO2_Pos(node, ctrl, target_pos);
}

/**
 * @brief  设置目标速度 (Object 0x60FF，常用于 PV 模式)
 */
CO_Err_t CIA402_SetTargetSpeed(CIA402_Node_t *node, int32_t target_vel)
{
    if (!node)
    {
        return CO_ERROR_PARAM;
    }
    return CANopen_SDO_Write32(node, OD_TARGET_VEL, 0x00, (uint32_t)target_vel);
}

/**
 * @brief  设置轮廓加速度 (Object 0x6083)
 */
CO_Err_t CIA402_SetProfileAcceleration(CIA402_Node_t *node, uint32_t accel)
{
    if (!node)
    {
        return CO_ERROR_PARAM;
    }
    return CANopen_SDO_Write32(node, 0x6083, 0x00, accel);
}

/**
 * @brief  设置轮廓减速度 (Object 0x6084)
 */
CO_Err_t CIA402_SetProfileDeceleration(CIA402_Node_t *node, uint32_t decel)
{
    if (!node)
    {
        return CO_ERROR_PARAM;
    }
    return CANopen_SDO_Write32(node, 0x6084, 0x00, decel);
}

/************************ 各通道 PDO 独立使能/禁用封装函数 ************************/

CO_Err_t CIA402_EnableRxPDO1(CIA402_Node_t *node, bool enable)
{
    if (!node) return CO_ERROR_PARAM;
    uint32_t cob_id = 0x200 + node->node_id;
    if (!enable) cob_id |= 0x80000000U;
    return CANopen_SDO_Write32(node, 0x1400, 0x01, cob_id);
}

CO_Err_t CIA402_EnableRxPDO2(CIA402_Node_t *node, bool enable)
{
    if (!node) return CO_ERROR_PARAM;
    uint32_t cob_id = 0x300 + node->node_id;
    if (!enable) cob_id |= 0x80000000U;
    return CANopen_SDO_Write32(node, 0x1401, 0x01, cob_id);
}

CO_Err_t CIA402_EnableRxPDO3(CIA402_Node_t *node, bool enable)
{
    if (!node) return CO_ERROR_PARAM;
    uint32_t cob_id = 0x400 + node->node_id;
    if (!enable) cob_id |= 0x80000000U;
    return CANopen_SDO_Write32(node, 0x1400, 0x01, cob_id);
}

CO_Err_t CIA402_EnableRxPDO4(CIA402_Node_t *node, bool enable)
{
    if (!node) return CO_ERROR_PARAM;
    uint32_t cob_id = 0x500 + node->node_id;
    if (!enable) cob_id |= 0x80000000U;
    return CANopen_SDO_Write32(node, 0x1401, 0x01, cob_id);
}

CO_Err_t CIA402_EnableTxPDO1(CIA402_Node_t *node, bool enable)
{
    if (!node) return CO_ERROR_PARAM;
    uint32_t cob_id = 0x180 + node->node_id;
    if (!enable) cob_id |= 0x80000000U;
    return CANopen_SDO_Write32(node, 0x1800, 0x01, cob_id);
}

CO_Err_t CIA402_EnableTxPDO2(CIA402_Node_t *node, bool enable)
{
    if (!node) return CO_ERROR_PARAM;
    uint32_t cob_id = 0x280 + node->node_id;
    if (!enable) cob_id |= 0x80000000U;
    return CANopen_SDO_Write32(node, 0x1801, 0x01, cob_id);
}

CO_Err_t CIA402_EnableTxPDO3(CIA402_Node_t *node, bool enable)
{
    if (!node) return CO_ERROR_PARAM;
    uint32_t cob_id = 0x380 + node->node_id;
    if (!enable) cob_id |= 0x80000000U;
    return CANopen_SDO_Write32(node, 0x1801, 0x01, cob_id);
}

CO_Err_t CIA402_EnableTxPDO4(CIA402_Node_t *node, bool enable)
{
    if (!node) return CO_ERROR_PARAM;
    uint32_t cob_id = 0x480 + node->node_id;
    if (!enable) cob_id |= 0x80000000U;
    return CANopen_SDO_Write32(node, 0x1801, 0x01, cob_id);
}
