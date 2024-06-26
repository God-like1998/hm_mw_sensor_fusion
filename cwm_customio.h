#ifndef __CWM_CUSTOMIO_H__
#define __CWM_CUSTOMIO_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "cwm_lib.h"
/*某些平台会使用 printf 函数，需要 include "stdio.h" */
#include "stdio.h"


#define cwm_taskENTER_CRITICAL()  {if(pdFALSE == xPortIsInsideInterrupt()) {taskENTER_CRITICAL();}}
#define cwm_taskEXIT_CRITICAL()   {if(pdFALSE == xPortIsInsideInterrupt()) {taskEXIT_CRITICAL();}}
#define CWM_OS_dbgPrintf(format,...)  printf(format,##__VA_ARGS__)

extern os_api customio_os_api;
extern const uint16_t defautl_odr;
extern const int dml_vendor_config[16];
extern const int dml_hw_config[16];
extern const int dml_ag_config[16];
extern const int dml_ag_pref_config_default[16];
extern const int dml_ag_pref_config_standby[16];
extern const int dml_mag_config[16];
extern const int dml_hs_orien_config[16] ;
extern const int dml_hs_intf_config[16];
extern const int dml_log_config[16];
extern const int dml_log_debug_config[16];
extern const float algo_quiet_lev;
extern const uint32_t algo_quiet_timeout_min;

void customio_read_flash_cali(uint8_t* data,uint32_t len);
void customio_save_flash_cali(uint8_t* data,uint32_t len);
void customio_read_flash_eul(uint8_t* data,uint32_t len);
void customio_save_flash_eul(uint8_t* data,uint32_t len);

void customio_init(void);
void customio_read_ag(uint8_t type, float *f, uint8_t idx, uint8_t nums);
void customio_notify_spv_cali_result(uint16_t whl,uint16_t pcba,uint16_t sixf);
void customio_notify_ori_eul_cali_result(uint16_t steps,uint16_t status);
void customio_read_eul_qua(float *f);
void customio_read_ag_avg_value(float *f);


#ifdef __cplusplus
}
#endif

#endif




