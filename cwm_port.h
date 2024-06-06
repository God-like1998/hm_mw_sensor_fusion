#ifndef __CWM_PORT_H__
#define __CWM_PORT_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"

enum{
    E_CALI_FAIL,
    E_CALI_RUNNING,
    E_CALI_SUCCESS,
};
enum{
    E_CALI_SPV_WHOLE = 1,
    E_CALI_SPV_PCBA = 2,
    E_CALI_SIX_FACE = 5,
};
enum{
    E_ORI_EUL_CALI_FAIL,
    E_ORI_EUL_CALI_RUNNING,
    E_ORI_EUL_CALI_SUCCESS,
};
enum{
    E_ANGLE_INIT_STEP1 = 1,
    E_ANGLE_INIT_STEP2 = 2,
    E_ANGLE_INIT_STEP3 = 3,
};


struct acc_t{
    float ax;
    float ay;
    float az;
};
struct gyro_t{
    float gx;
    float gy;
    float gz;
};
struct ag_t{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
};
struct eul_qua_t{
    float yaw;
    float pitch;
    float roll;
    float x;
    float y;
    float z;
    float w;
};


void cwm_algo_task(void *pvParameters);
void cwm_log_debug_ctl(uint32_t ctr);
void cwm_hs_algo_ctl(uint32_t ctr);
void cwm_spv_cali_en(uint32_t mode);
void cwm_spv_cali_dis(void);
void cwm_ori_eul_cali_en(uint32_t steps);
void cwm_ori_eul_cali_dis(void);
void cwm_avg_ag_value_en(void);
void cwm_save_before_poweroff(void);


#ifdef __cplusplus
}
#endif

#endif




