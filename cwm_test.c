#include "stdint.h"
#include "string.h"
#include "stdbool.h"
#include "cwm_lib.h"
#include "cwm_lib_dml.h"
#include "cwm_config.h"
#include "cwm_port.h"
#include "cwm_diskio.h"
#include "cwm_test.h"

#if defined(ALGO_TEST_EN) && (1 == ALGO_TEST_EN)
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
extern struct ag_cali_back_t* get_algo_dev_info_ag_cali_value(void);
/**************************************************algo queue**************************************************/
void test_print_calibration_value(void)
{
    /*获取算法持续校正数据  （DYNAMIC_CALI_CHECK_TIME）s 执行一次  */
    #define DYNAMIC_CALI_CHECK_TIME (1)    
    static uint32_t auto_cali_run_ticks = 0;
    static struct ag_cali_back_t ag_cali_value = {0};
    struct ag_cali_back_t* cali_value_p = get_algo_dev_info_ag_cali_value();

    auto_cali_run_ticks++;
    if(auto_cali_run_ticks > (algo_get_odr()*DYNAMIC_CALI_CHECK_TIME)){
        auto_cali_run_ticks = 0;

        SettingControl_t scl;
        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[1] = 1;
        CWM_SettingControl(SCL_SENS_CALI_CTRL_A,&scl);

        if((ag_cali_value.auto_gx != scl.iData[5]) ||
        (ag_cali_value.auto_gy != scl.iData[6]) ||
        (ag_cali_value.auto_gz != scl.iData[7])){
            ag_cali_value.auto_gx = scl.iData[5];
            ag_cali_value.auto_gy = scl.iData[6];
            ag_cali_value.auto_gz = scl.iData[7];

            static int32_t auto_cali_value_change_cnts = 0;
            auto_cali_value_change_cnts++;
            CWM_OS_dbgPrintf("[algo]test auto_cali changes: %d, value: %d,%d,%d, auto_cli-fac_cali: %d,%d,%d\n",
            auto_cali_value_change_cnts,
            ag_cali_value.auto_gx,
            ag_cali_value.auto_gy,
            ag_cali_value.auto_gz,
            ag_cali_value.auto_gx - cali_value_p->gx,
            ag_cali_value.auto_gy - cali_value_p->gy,
            ag_cali_value.auto_gz - cali_value_p->gz);
        }
    }
}

void algo_test(void)
{
    test_print_calibration_value();
}


#endif
