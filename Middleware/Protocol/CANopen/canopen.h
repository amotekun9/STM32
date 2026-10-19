/**
 * @file    canopen.h
 * @brief   CANopen CiA 301 / CiA 402 轻量级运动控制协议栈头文件
 * @details 提供标准 CANopen 报文宏定义、对象字典索引、节点状态结构体及核心 API 声明
 */

#ifndef _CANOPEN_H
#define _CANOPEN_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif
	
/************************ 返回值定义 ************************/
typedef enum {
    CO_OK = 0,           /* 操作成功 / 无错误 */
    CO_ERROR_PARAM,      /* 参数错误（如指针为空、索引越界等） */
    CO_ERROR_HW          /* 硬件错误（如 CAN 发送邮箱满、底层发送失败等） */
} CO_Err_t;

/************************ CANopen 标准 COB-ID 定义 ************************/
#define COB_ID_NMT              0x000   /* NMT 网络管理广播 ID */
#define COB_ID_SYNC             0x080   /* SYNC 同步帧 ID */
#define COB_ID_EMCY_BASE        0x080   /* EMCY 紧急报文基址 */
#define COB_ID_TXPDO1_BASE      0x180   /* TxPDO1 默认基址 (从->主: 状态字+位置) */
#define COB_ID_RXPDO1_BASE      0x200   /* RxPDO1 默认基址 (主->从: 控制字) */
#define COB_ID_TXPDO2_BASE      0x280   /* TxPDO2 默认基址 (从->主: 速度+转矩) */
#define COB_ID_RXPDO2_BASE      0x300   /* RxPDO2 默认基址 (主->从: 控制字+目标位置) */
#define COB_ID_SDO_RX_BASE      0x580   /* SDO 从->主 响应基址 */
#define COB_ID_SDO_TX_BASE      0x600   /* SDO 主->从 请求基址 */
#define COB_ID_BOOTUP_BASE      0x700   /* 节点上线/心跳报文基址 */

/************************ SDO 命令字 (Command Specifier) ************************/
#define SDO_CMD_WRITE_1B        0x2F    /* SDO 写 1 字节命令 */
#define SDO_CMD_WRITE_2B        0x2B    /* SDO 写 2 字节命令 */
#define SDO_CMD_WRITE_4B        0x23    /* SDO 写 4 字节命令 */
#define SDO_CMD_READ_REQ        0x40    /* SDO 读请求命令 */

/************************ CiA402 对象字典索引 (Object Dictionary) ************************/
#define OD_CONTROL_WORD         0x6040  /* 控制字 (Controlword) */
#define OD_STATUS_WORD          0x6041  /* 状态字 (Statusword) */
#define OD_OP_MODE              0x6060  /* 运行模式 (Modes of Operation) */
#define OD_ACTUAL_POS           0x6064  /* 实际位置 (Position Actual Value) */
#define OD_ACTUAL_VEL           0x606C  /* 实际速度 (Velocity Actual Value) */
#define OD_ACTUAL_TOR           0x6077  /* 实际转矩 (Torque Actual Value) */
#define OD_TARGET_POS           0x607A  /* 目标位置 (Target Position) */
#define OD_TARGET_VEL           0x60FF  /* 目标速度 (Target Velocity) */

/************************ 控制字位定义 (0x6040 Controlword) ************************/
#define CTRL_SWITCH_ON          (1 << 0)    /* Bit 0: 准备接通 */
#define CTRL_ENABLE_VOLT        (1 << 1)    /* Bit 1: 电压使能 */
#define CTRL_QUICK_STOP         (1 << 2)    /* Bit 2: 快速停止 (0有效) */
#define CTRL_ENABLE_OP          (1 << 3)    /* Bit 3: 运行使能 */
#define CTRL_NEW_SETPOINT       (1 << 4)    /* Bit 4: 新设定点触发 (PP模式专用) */
#define CTRL_REL_POS            (1 << 6)    /* Bit 6: 相对/绝对定位选择 (0=绝对, 1=相对) */
#define CTRL_FAULT_RESET        (1 << 7)    /* Bit 7: 故障复位 (上升沿有效) */
#define CTRL_HALT               (1 << 8)    /* Bit 8: 暂停 */

/************************ 状态字位定义 (0x6041 Statusword) ************************/
#define STAT_TARGET_REACH       (1 << 10)   /* Bit 10: 目标到达标志 */

/************************ 枚举与数据结构定义 ************************/

/**
 * @brief CiA 402 驱动器运行模式枚举
 */
typedef enum {
    OP_NO_MODE      = 0,      /* 无模式 / 保留 */
    OP_PROFILE_POS  = 1,      /* 轮廓位置模式 (PP 模式) */
    OP_PROFILE_VEL  = 3,      /* 轮廓速度模式 (PV 模式) */
    OP_PROFILE_TOR  = 4,      /* 轮廓转矩模式 (TQ 模式) */
    OP_HOMING       = 6,      /* 回零模式 (HM 模式) */
    OP_INTERP_POS   = 7,      /* 插补位置模式 (IP 模式) */
} OpMode_t;

/**
 * @brief 标准 CAN 帧结构体
 */
typedef struct {
    uint32_t cob_id;          /* 帧 ID (标准 11 位 COB-ID) */
    uint8_t  data_len;        /* 数据长度 (0 ~ 8 字节) */
    uint8_t  data[8];         /* 数据载荷 */
} CanFrame_t;

/**
 * @brief CANopen 从站节点对象结构体 (CiA 402 架构)
 */
typedef struct {
    uint8_t         node_id;            /* 节点 ID (1 ~ 127) */
    OpMode_t        target_op_mode;     /* 当前设定的目标运行模式 */

    uint16_t        control_word;       /* 当前下发的控制字 (0x6040) */
    uint16_t        status_word;        /* 最新收到的状态字 (0x6041，保留原始反馈) */
    int32_t         actual_pos;         /* 实际位置反馈 (0x6064) */
    int32_t         actual_vel;         /* 实际速度反馈 (0x606C) */
    int16_t         actual_tor;         /* 实际转矩反馈 (0x6077) */
} CIA402_Node_t;

/************************ 硬件底层发送接口声明 ************************/
/**
 * @brief 底层 CAN 发送函数 (由 BSP 层实现)
 * @param frame: 待发送的 CAN 帧指针
 * @return true=发送成功, false=发送失败/邮箱满
 */
extern bool CAN_SendFrame(const CanFrame_t *frame);
extern bool BSP_CAN_ReadFrame(CanFrame_t *out_frame);
/************************ 核心 API 函数声明 ************************/

/* 节点初始化与心跳配置 */
void CIA402_InitNode(CIA402_Node_t *node, uint8_t node_id);
bool CIA402_InitAndRegisterNode(CIA402_Node_t *node, uint8_t node_id);
CO_Err_t CIA402_SetHeartbeatTime(CIA402_Node_t *node, uint16_t heartbeat_ms); 

/* NMT 网络管理与同步指令 */
CO_Err_t CANopen_NMT_SendCmd(CIA402_Node_t *node, uint8_t cmd);             
CO_Err_t CANopen_NMT_Broadcast(uint8_t cmd);                                
CO_Err_t CANopen_SendSYNC(void);                                            

/* SDO 读写基础接口 */
CO_Err_t CANopen_SDO_Write8(CIA402_Node_t *node, uint16_t index, uint8_t subindex, uint8_t value);
CO_Err_t CANopen_SDO_Write16(CIA402_Node_t *node, uint16_t index, uint8_t subindex, uint16_t value);
CO_Err_t CANopen_SDO_Write32(CIA402_Node_t *node, uint16_t index, uint8_t subindex, uint32_t value);

/* PDO 基础配置与高速收发 */
CO_Err_t CIA402_SetTxPDO_TransType(CIA402_Node_t *node, uint8_t pdo_index, uint8_t trans_type);
CO_Err_t CANopen_SendRxPDO1(CIA402_Node_t *node, uint16_t ctrl_word);       
CO_Err_t CANopen_SendRxPDO2_Pos(CIA402_Node_t *node, uint16_t ctrl_word, int32_t target_pos);
void CANopen_RecvHandler(CIA402_Node_t *node, const CanFrame_t *frame);     

/* 运动控制核心 API */
CO_Err_t CIA402_SetOpMode(CIA402_Node_t *node, OpMode_t mode);              
CO_Err_t CIA402_ServoDisable(CIA402_Node_t *node);                          
CO_Err_t CIA402_PPMoveTo(CIA402_Node_t *node, int32_t target_pos, bool is_absolute); 
CO_Err_t CIA402_SetPDO_Enable(CIA402_Node_t *node, bool is_tx, uint8_t pdo_index, bool enable, uint16_t cob_id);
CO_Err_t CIA402_PP_DebugMove(CIA402_Node_t *node, int32_t target_pos, uint32_t profile_vel, uint32_t accel, uint32_t decel, bool is_absolute);
CO_Err_t CIA402_SetTargetSpeed(CIA402_Node_t *node, int32_t target_vel);
CO_Err_t CIA402_SetProfileAcceleration(CIA402_Node_t *node, uint32_t accel);
CO_Err_t CIA402_SetProfileDeceleration(CIA402_Node_t *node, uint32_t decel);

/*CANopen 协议层总线分发函数*/
void CANopen_ProcessBus(void);

/* 各通道 PDO 独立使能/禁用 API (Bit 31 控制) */
CO_Err_t CIA402_EnableRxPDO1(CIA402_Node_t *node, bool enable);
CO_Err_t CIA402_EnableRxPDO2(CIA402_Node_t *node, bool enable);
CO_Err_t CIA402_EnableRxPDO3(CIA402_Node_t *node, bool enable);
CO_Err_t CIA402_EnableRxPDO4(CIA402_Node_t *node, bool enable);
CO_Err_t CIA402_EnableTxPDO1(CIA402_Node_t *node, bool enable);
CO_Err_t CIA402_EnableTxPDO2(CIA402_Node_t *node, bool enable);
CO_Err_t CIA402_EnableTxPDO3(CIA402_Node_t *node, bool enable);
CO_Err_t CIA402_EnableTxPDO4(CIA402_Node_t *node, bool enable);

#ifdef __cplusplus
}
#endif

#endif /* _CANOPEN_H */
