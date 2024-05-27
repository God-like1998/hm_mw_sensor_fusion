#ifndef __CWM_PORT_H__
#define __CWM_PORT_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"
#include "cwm_lib.h"
#include "cwm_lib_dml.h"


void cwm_algo_task(void *param);


#define SPV_MODE_WHOLE_MACHINE  1
#define SPV_MODE_PCBA           2
#define SPV_MODE_SIX_FACE       5  

#define ANGLE_INIT_FIR         1
#define ANGLE_INIT_SEC         2
#define ANGLE_INIT_THI         3

void cwm_outside_algo_sensor_enable(uint32_t idx);
void cwm_outside_algo_sensor_disable(uint32_t idx);
void cwm_outside_algo_setting_control(int32_t cid , SettingControl_t* scl);

void cwm_outside_log_debug_ctr(uint32_t ctr);
void cwm_outside_headset_ori_ctr(uint32_t ctr);
void cwm_outside_driver_ctr(uint32_t ctr);
void cwm_outside_spv_calib_en(uint32_t mode);
void cwm_outside_spv_calib_dis(void);
void cwm_outside_angle_init_en(uint32_t times);
void cwm_outside_angle_init_dis(void);
void cwm_outside_get_aver_ag_rawdata(void);
void cwm_outside_save_auto_calib_value(void);

#ifdef __cplusplus
}
#endif

#endif






































