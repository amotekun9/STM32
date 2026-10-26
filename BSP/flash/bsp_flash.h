#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#include "stm32f1xx_hal.h"

extern uint16_t modbus_holding_reg[];

/**
 * @brief  需要保存到Flash的参数结构体
 * @note   和 UploadData_t 里面需要持久化保存的成员一一对应
 */
typedef struct
{
    uint16_t weft_density_up;        // 纬密
    uint16_t cut_star_angle_up;      // 剪刀启动角度
    uint16_t cut_return_angle_up;    // 剪刀回位角度
} SaveParam_t;

#define PARAM_STORE_ADDR     0x0807F800U  // Flash参数存储地址

void upload_param_load(void);           // 上电加载参数，从Flash读取到内存
void upload_param_save(void);           // 保存参数，将当前参数写入Flash

#endif




