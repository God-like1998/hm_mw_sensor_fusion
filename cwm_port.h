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
struct ag_cali_back_t{
    /*工厂校正数据*/
    uint16_t crc_16;//校验和 
    uint16_t spv_whole_status: 2;//spv 整机校正状态
    uint16_t spv_pcba_status: 2;//spv pcba 校正状态
    uint16_t sixface_status: 2;//六面校正状态
    uint16_t valid:1;//数据可用
    int32_t ax;
    int32_t ay;
    int32_t az;
    int32_t gx;
    int32_t gy;
    int32_t gz;

    /*算法持续校正数据*/
    uint16_t auto_crc_16;//校验和 
    uint16_t auto_valid:1;//数据可用
    int32_t auto_gx;
    int32_t auto_gy;
    int32_t auto_gz;
};
struct original_eul_t{
    uint16_t crc_16;
    uint16_t step1_status:2;        //第一步校正状态
    uint16_t step2_status:2;        //第二步校正状态
    uint16_t step3_status:2;        //第三步校正状态
    uint16_t running_steps:4;       //当前执行的校正步骤
    uint16_t valid:1;//数据可用
    float yaw;
    float pitch;
    float roll;
};


void cwm_algo_task( void *pvParameters );
void cwm_log_debug_ctl(uint32_t ctr);
void cwm_hs_algo_ctl(uint32_t ctr);
// void cwm_sensor_ctr(uint32_t ctr);
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