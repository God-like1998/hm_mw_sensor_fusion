#ifndef __CWM_CONFIG_H__
#define __CWM_CONFIG_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"
#include "cwm_port.h"

/*版本号说明：SDK_EAR0.1.2.3
SDK：表示该项目是 sdk 工程。其它项目可根据实际填入项目代号
HSET：头戴耳机项目;EAR：TWS 耳机项目；WAT: 手表项目
0：freertos；（zephyr ：1）
1：sdk 大版本号
2：sdk 小版本号
3：fae 针对客户更新的版本号
*/
#define ALGO_CONFIG_VERSION "SDK_HSET0.0.3.0"
#define ALGO_RES_MAX_COUNT  25


#define ALGO_QUIET_TIME_SEC     (60*10)
#define ALGO_QUIET_LV1   (0.05f)
#define ALGO_QUIET_LV2   (0.10f)
#define ALGO_QUIET_LV3   (0.15f)
#define ALGO_QUIET_LV4   (0.20f)
#define ALGO_QUIET_LV5   (0.25f)
#define ALGO_QUIET_LV6   (0.30f)
#define ALGO_QUIET_LV7   (0.35f)
#define ALGO_QUIET_LV8   (0.40f)
#define ALGO_QUIET_LV9   (0.45f)
#define ALGO_QUIET_LV10  (0.50f)
#define ALGO_QUIET_LV11  (0.55f)
#define ALGO_QUIET_LV12  (0.60f)
#define ALGO_QUIET_LV13  (0.65f)
#define ALGO_QUIET_LV14  (0.70f)
#define ALGO_QUIET_LV15  (0.75f)
#define ALGO_STANDBY_QUIET_THRESHOLD ALGO_QUIET_LV7


struct ag_avg_t{
    uint16_t a_cnts;
    uint16_t g_cnts;
    struct ag_t ag_sum;
    struct ag_t ag;
};
struct algo_ag_t{
    uint8_t size;
    uint8_t write;
    uint8_t read;
    uint8_t data_type;
    struct ag_t data[ALGO_RES_MAX_COUNT];
};
struct algo_res_t{
    uint8_t size;
    uint8_t write;
    uint8_t read;
    struct eul_qua_t data[ALGO_RES_MAX_COUNT];
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
struct algo_info_t{
    uint16_t odr;//算法 odr
    uint16_t event_cali_finish:1;           //事件，用于通知保存校正数据
    uint16_t event_orig_eul_finish:1;       //事件，用于通知保存原始角度数据
    uint16_t event_ag_avg_value_1s_finish:1;//事件，用于通知 1s ag 平均值结束
    uint16_t en_ag_avg_value_1s:1;          //开关，用于通知 1s ag 功能开关
    struct ag_avg_t ag_avg_value_1s;
    struct algo_ag_t algo_ag;   //sensor acc、gyro缓存
    struct algo_res_t algo_res;//欧拉角和四元素缓存

    //需要保存到 flash 的数据
    struct ag_cali_back_t ag_cali_value;//acc、gyro 校正参数
    struct original_eul_t original_eul;//初始化角度
};




enum{
    E_STATE_LEV0,
    E_STATE_LEV1,
    E_STATE_LEV2,
    E_STATE_LEV3,
    E_ALGO_STATE_MAX,
};
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
void algo_log_debug_ctl(uint32_t ctr);
void algo_hs_algo_ctl(uint32_t ctr);
void algo_sensor_ctr(uint32_t ctr);
void algo_spv_cali_en(uint32_t mode);
void algo_spv_cali_dis(void);
void algo_original_eul_cali_en(uint32_t steps);
void algo_original_eul_cali_dis(void);
void algo_avg_ag_value_en(void);
void algo_save_before_poweroff(void);
void algo_state_handle(uint16_t id, uint16_t event, void* param);


#ifdef __cplusplus
}
#endif

#endif




