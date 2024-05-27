#ifndef __CWM_CONFIG_H__
#define __CWM_CONFIG_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"
#include "stdio.h"
#include "cwm_port.h"



#define CWM_OS_dbgPrintf(format,...)  printf(format,##__VA_ARGS__)

#define ALGO_CONFIG_VERSION "HM_M3_HSET_0.0.1.3"
#define ALGO_HS_MAX 25
#define ALGO_TEST_EN        1

enum{
    SPV_CALIB_PARAM,
    ANGLE_INIT_PARAM,
};

enum{
    AVER_AG_RAWDATA_EN,
    AVER_AG_RAWDATA_DIS,
};

enum{
    SPV_CALIB_FAIL,
    SPV_CALIB_RUNNING,
    SPV_CALIB_SUCCESS,
};

enum{
    ANGLE_INIT_FAIL,
    ANGLE_INIT_RUNNING,
    ANGLE_INIT_SUCCESS,
};

typedef struct{
	float yaw;
    float pitch;
    float roll;
    float x;
    float y;
    float z;
    float w;
}hs_t;

typedef struct{
	uint8_t size;
	uint8_t write;
	uint8_t read;
	hs_t hs[ALGO_HS_MAX];
}hs_buffer_t;

typedef struct{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
}ag_aver_t;

typedef struct{

    /*工厂校正数据*/
    uint16_t crc_16;                  //校验和 
    uint16_t whole_machine_status: 2; //spv 整机校正状态
    uint16_t pcba_status: 2;          //spv pcba 校正状态
    uint16_t sixface_status: 2;       //六面校正状态
    uint16_t valid:1;                 //数据可用
    int32_t ax;
    int32_t ay;
    int32_t az;
    int32_t gx;
    int32_t gy;
    int32_t gz;

    /*算法持续校正数据*/
    uint16_t auto_crc16;                   //校验和
    uint16_t auto_valid:1;                 //数据可用
    int32_t auto_gx;
    int32_t auto_gy;
    int32_t auto_gz;
}spv_calib_value_t;


typedef struct{
    uint16_t crc_16;                //校验和
    uint16_t step1_status:2;        //第一次校正状态
    uint16_t step2_status:2;        //第二次校正状态
    uint16_t step3_status:2;        //第三次校正状态
    uint16_t valid:1;               //数据可用
    float yaw;
    float pitch;
    float roll;
}angle_init_value_t;

typedef struct{

    uint16_t odr;
    uint16_t event_spv_calib_finish:1;    //事件，用于通知处理 SPV 校正数据
    uint16_t event_angle_init_finish:1;    //事件，用于通知处理 初始角度数据
    uint16_t event_acc_gyro_aver_finish:1;   //事件，用于通知处理 ACC&GYRO 的平均值数据
    uint16_t en_acc_gyro_aver:1;                   //开关，用于控制 ACC&GYRO 的平均值数据的开关

	hs_buffer_t hs_buffer;                //欧拉角和四元数缓存
    ag_aver_t ag_aver_value;                    //ACC&GYRO 的平均值数据

    /*保存进 FALSH*/
    spv_calib_value_t spv_calib_value;    // SPV 校正数据
    angle_init_value_t angle_init_value;  // 初始角度数据


}algo_info_t;




void cwm_algo_init(void);
void cwm_algo_data_handle(void);

void algo_log_debug_ctr(uint32_t ctr);
void algo_headset_ori_ctr(uint32_t ctr);
void algo_driver_ctr(uint32_t ctr);
void algo_spv_calib_en(uint32_t mode);
void algo_angle_init_en(uint32_t times);
void algo_get_aver_ag_rawdata(void);
void algo_save_auto_calib_value(void);

#ifdef __cplusplus
}
#endif

#endif






