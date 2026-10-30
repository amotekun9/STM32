#ifndef _SELVAGE_DERIER_H
#define _SELVAGE_DERIER_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "canopen.h"

void canopen_selvage_nosave_async_pdo_init(CIA402_Node_t *node);
void canopen_selvage_nosave_sync_pdo_init(CIA402_Node_t *node);
void canopen_selvage_position_init(CIA402_Node_t *node);
void canopen_selvage_speed_init(CIA402_Node_t *node);
void selvage_sdo_jog_relative(CIA402_Node_t *node,int32_t target_pos); 
void selvage_sdo_jog_relative(CIA402_Node_t *node,int32_t target_pos); 
void selvage_pdo_jog_relative(CIA402_Node_t *node,int32_t target_pos); 

#ifdef __cplusplus
}
#endif

#endif /* _SELVAGE_DERIER_H */
