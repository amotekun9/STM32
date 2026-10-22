#ifndef __SCISSOR_DRIVER_H
#define __SCISSOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "bsp_delay.h"
#include "tpc_slave.h"

extern int32_t encoder_cnt;
extern UploadData_t g_upload_data;

void scissor_init(void);
void scissor_open_trig(uint16_t ms);
void scissor_close_trig(uint16_t ms);
void scissor_poll(void);
uint8_t scissor_is_idle(void);
void scissor_window_check(const uint16_t pulse_us, const uint8_t offset_encoder_cnt);


#ifdef __cplusplus
}
#endif

#endif /* __SCISSOR_DRIVER_H */ 

