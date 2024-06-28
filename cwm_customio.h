#ifndef __CWM_CUSTOMIO_H__
#define __CWM_CUSTOMIO_H__

#ifdef __cplusplus
extern "C"{
#endif


#include "cwm_lib.h"


extern int CWM_OS_dbgPrintf(const char * format,...);
void cwm_taskENTER_CRITICAL(void);
void cwm_taskEXIT_CRITICAL(void);

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
extern const int dml_hs_run_init_angle_config[16];
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




