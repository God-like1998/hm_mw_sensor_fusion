#ifndef __CWM_PORT_H__
#define __CWM_PORT_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"

enum{
    E_CALI_FAIL,//ʧ��
    E_CALI_RUNNING,//ִ����
    E_CALI_SUCCESS,//�ɹ�
};
enum{
    E_CALI_SPV_WHOLE = 1,
    E_CALI_SPV_PCBA = 2,
    E_CALI_SIX_FACE = 5,
};
enum{
    E_ORI_EUL_CALI_FAIL,//ʧ��
    E_ORI_EUL_CALI_RUNNING,//ִ����
    E_ORI_EUL_CALI_SUCCESS,//�ɹ�
};
enum{
    E_ANGLE_INIT_STEP1 = 1,//��һ��
    E_ANGLE_INIT_STEP2 = 2,//�ڶ���
    E_ANGLE_INIT_STEP3 = 3,//������
};


struct acc_t{
    float ax;//��λ��m/s^2
    float ay;//��λ��m/s^2
    float az;//��λ��m/s^2
};
struct gyro_t{
    float gx;//��λ��rad/s
    float gy;//��λ��rad/s
    float gz;//��λ��rad/s
};
struct ag_t{
    float ax;//��λ��m/s^2
    float ay;//��λ��m/s^2
    float az;//��λ��m/s^2
    float gx;//��λ��rad/s
    float gy;//��λ��rad/s
    float gz;//��λ��rad/s
};
struct eul_qua_t{
    float yaw;  //(degrees)
    float pitch;//(degrees)
    float roll; //(degrees)
    float x;    //quaternion_x
    float y;    //quaternion_y
    float z;    //quaternion_z
    float w;    //quaternion_w
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




