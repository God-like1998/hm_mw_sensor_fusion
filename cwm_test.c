#include "stdint.h"
#include "string.h"
#include "stdbool.h"
#include "cwm_lib.h"
#include "cwm_lib_dml.h"
#include "cwm_config.h"
#include "cwm_port.h"

#if defined(ALGO_TEST_EN) && (1 == ALGO_TEST_EN)
extern algo_info_t algo_info;
/**************************************************algo queue**************************************************/
void test_print_calibration_value(void)
{
    /*获取算法持续校正数据  （DYNAMIC_CALI_CHECK_TIME）s 执行一次  */
    #define DYNAMIC_CALI_CHECK_TIME (1)    
    static uint32_t ctn_cali_run_ticks = 0;
    static spv_calib_value_t spv_calib_value_test;
    spv_calib_value_t* spv_calib_value = &algo_info.spv_calib_value;

    ctn_cali_run_ticks++;
    if(ctn_cali_run_ticks > (algo_info.odr*DYNAMIC_CALI_CHECK_TIME)){
        ctn_cali_run_ticks = 0;

        SettingControl_t scl;
        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[1] = 1;
        CWM_SettingControl(SCL_SENS_CALI_CTRL_A,&scl);

        if((spv_calib_value_test.auto_gx != scl.iData[5]) ||
        (spv_calib_value_test.auto_gy != scl.iData[6]) ||
        (spv_calib_value_test.auto_gz != scl.iData[7])){
            spv_calib_value_test.auto_gx = scl.iData[5];
            spv_calib_value_test.auto_gy = scl.iData[6];
            spv_calib_value_test.auto_gz = scl.iData[7];

            static int32_t ctn_cali_value_change_cnts = 0;
            ctn_cali_value_change_cnts++;
            CWM_OS_dbgPrintf("[cwm_test]: changes_count: %d, auto_cali_test: %d,%d,%d, auto-whole: %d,%d,%d\n",
            ctn_cali_value_change_cnts,
            spv_calib_value_test.auto_gx,
            spv_calib_value_test.auto_gy,
            spv_calib_value_test.auto_gz,
            spv_calib_value_test.auto_gx - spv_calib_value->gx,
            spv_calib_value_test.auto_gy - spv_calib_value->gy,
            spv_calib_value_test.auto_gz - spv_calib_value->gz);
        }
    }
}

void algo_test(void)
{
    test_print_calibration_value();
}


#endif
