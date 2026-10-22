#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "stm32f1xx_hal.h"
#include "bsp_can.h"
#include "bsp_delay.h"
#include "biss_coder.h"
#include "bsp_delay.h"
#include "bsp_usart.h"
#include "bsp_rs485.h"
#include "bsp_timer.h"
#include "canopen.h"
#include "modbus_rtu.h"
#include "vfd_driver.h"
#include "tpc_slave.h"
#include "spindle_coder.h"
#include "pinch_driver.h"
#include "scissor_driver.h"
	
void app_main(void);


#ifdef __cplusplus
}
#endif

#endif /* __APP_MAIN_H */ 
