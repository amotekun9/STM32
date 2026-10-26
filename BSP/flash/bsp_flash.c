#include "bsp_flash.h"
#include "app_main.h"

// 外部上传结构体，存放需要持久化的工艺参数
extern UploadData_t g_upload_data;
// Modbus寄存器数组，参数加载后同步到寄存器
extern uint16_t modbus_holding_reg[];

/**
 * @brief  读取Flash数据（Flash读不需要解锁，直接地址访问）
 * @param  addr  Flash起始地址
 * @param  p_data 读出数据存放缓冲区
 * @param  len    读取字节长度
 */
static void bsp_flash_read(uint32_t addr, uint8_t *p_data, uint16_t len)
{
    for(uint16_t i = 0U; i < len; i++)
    {
        p_data[i] = *(uint8_t *)(addr + i);
    }
}

/**
 * @brief  Flash写入函数：先擦页，再按半字写入
 * @param  addr  Flash存储起始地址，必须页对齐
 * @param  p_data 待写入的数据缓存
 * @param  len    数据长度（字节），必须是偶数
 * @retval 0：成功；-1：地址未页对齐；-2：擦除失败；-3：写入失败
 */
static int8_t bsp_flash_write(uint32_t addr, uint8_t *p_data, uint16_t len)
{
    FLASH_EraseInitTypeDef erase_init = {0};
    uint32_t page_err = 0U;
    uint16_t i = 0U;

    // 判断地址是否为页对齐（F1一页2048字节 0x800）
    if((addr & 0x000007FFU) != 0U)
    {
        return -1;
    }

    HAL_FLASH_Unlock();  // Flash解锁，才能擦除/写入

    erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = addr;
    erase_init.NbPages     = 1U;

    // 擦除一页Flash
    if(HAL_FLASHEx_Erase(&erase_init, &page_err) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return -2;
    }

    // 按半字(16bit)循环写入
    while(i < len)
    {
        uint16_t halfword = p_data[i];
        if((i + 1U) < len)
        {
            halfword |= ((uint16_t)p_data[i+1]) << 8U;
        }
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, halfword) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return -3;
        }
        i += 2U;
    }

    HAL_FLASH_Lock(); // 操作完成锁定Flash
    return 0;
}

/**
 * @brief  上电加载参数：从Flash读取参数到g_upload_data，同步到Modbus寄存器
 * @note   Flash空白（全部0xFFFF）时，加载内置默认参数
 */
void upload_param_load(void)
{
    SaveParam_t temp;
    // 读取Flash里保存的参数
    bsp_flash_read(PARAM_STORE_ADDR, (uint8_t *)&temp, sizeof(SaveParam_t));

    // 判断：三个参数全是0xFFFF代表Flash空白，使用默认值
    if(temp.weft_density_up == 0xFFFFU && temp.cut_star_angle_up == 0xFFFFU && temp.cut_return_angle_up == 0xFFFFU)
    {
        g_upload_data.weft_density_up      = 800U;
        g_upload_data.cut_star_angle_up    = 100U;
        g_upload_data.cut_return_angle_up  = 200U;
        // upload_param_save(); // 打开注释：首次上电自动把默认值写入Flash，只写一次
    }
    else
    {
        // Flash内有有效参数，直接加载
        g_upload_data.weft_density_up      = temp.weft_density_up;
        g_upload_data.cut_star_angle_up    = temp.cut_star_angle_up;
        g_upload_data.cut_return_angle_up  = temp.cut_return_angle_up;
    }

    // 同步到Modbus holding寄存器，触摸屏刷新显示
    modbus_holding_reg[1] = g_upload_data.weft_density_up;
    modbus_holding_reg[4] = g_upload_data.cut_star_angle_up;
    modbus_holding_reg[5] = g_upload_data.cut_return_angle_up;
}

/**
 * @brief  参数保存：将当前UploadData_t里的工艺参数写入Flash
 * @note   配合触摸屏保存按钮指令调用，不自动保存
 */
void upload_param_save(void)
{
    SaveParam_t buf;
    // 从上传结构体取出当前参数
    buf.weft_density_up      = g_upload_data.weft_density_up;
    buf.cut_star_angle_up    = g_upload_data.cut_star_angle_up;
    buf.cut_return_angle_up  = g_upload_data.cut_return_angle_up;

    // 写入Flash
    bsp_flash_write(PARAM_STORE_ADDR, (uint8_t *)&buf, sizeof(SaveParam_t));
}

