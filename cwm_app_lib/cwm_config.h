#ifndef __CWM_CONFIG_H__
#define __CWM_CONFIG_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"
#include "cwm_port.h"





enum{
    E_STATE_STANDBY,
    E_STATE_STANDBY_SPV,
    E_STATE_HS_ORIT,
    E_STATE_SPV_WHOLE,
    E_STATE_SPV_PCBA,
    E_STATE_SPV_SIX_FACE,
    E_STATE_ORIG_EUL_CALI,
    
    E_ALGO_FUNC_LOG_CTL,
    E_ALGO_FUNC_AG_AVG_VALUE,
    E_ALGO_FUNC_SAVE_BEFORE_POWEROFF,
};
enum{
    E_ALGO_EVENT_OPEN,
    E_ALGO_EVENT_CLOSE,
};

void algo_init(void);
uint16_t algo_get_odr(void);
void algo_data_handle(void);
void algo_spv_cali_disable_mode(void);
void algo_state_handle(uint16_t id, uint16_t event, void* param);


#ifdef __cplusplus
}
#endif

#endif




