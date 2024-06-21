#include "stdint.h"
#include "string.h"
#include "stdbool.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "cwm_lib.h"
#include "cwm_lib_dml.h"
#include "cwm_config.h"
#include "cwm_customer.h"
#include "cwm_common.h"
#include "cwm_port.h"
#include "cwm_test.h"


#define MESSAGE_SAMPLE_CODE 0





/****************************************************************************************************/

enum{
    E_ALGO_MSG_LOG_DEBUG_CTL,
    E_ALGO_MSG_HS_ALGO_CTL,
    E_ALGO_MSG_SPV_CALI_EN,
    E_ALGO_MSG_SPV_CALI_DIS,
    E_ALGO_MSG_ORI_EUL_CALI_EN,
    E_ALGO_MSG_ORI_EUL_CALI_DIS,
    E_ALGO_MSG_AVG_AG_VALUE_EN,
    E_ALGO_MSG_SAVE_BEFORE_POWEROFF,


#if defined(MESSAGE_SAMPLE_CODE) && (1 == MESSAGE_SAMPLE_CODE)
    //DML INTERFACE
    E_DML_ENABLE = 0xF0,
    E_DML_DISABLE,
    E_DML_SETING_CONTROL,
#endif
};

/****************************************************************************************************/
static bool algo_running = true;
static uint32_t period_ms;/*算法运行周期*/
/****************************************************************************************************/



/****************************************************task************************************************/


void algo_message_handle(uint32_t id,uint8_t* data)
{
    CWM_OS_dbgPrintf("[algo]algo_message_handle %u %u\n",id,*((uint32_t*)data));
    switch (id){
        case E_ALGO_MSG_LOG_DEBUG_CTL:{
            uint32_t* ctr = (uint32_t*)data;
            algo_state_handle(E_ALGO_FUNC_LOG_CTL,NULL,ctr);
        }            
        break;
        case E_ALGO_MSG_HS_ALGO_CTL:{
            uint32_t* ctr = (uint32_t*)data;
            if(*ctr){
                algo_state_handle(E_STATE_HS_ORIT,E_ALGO_EVENT_OPEN,ctr);
            }else{
                algo_state_handle(E_STATE_HS_ORIT,E_ALGO_EVENT_CLOSE,ctr);
            }
        }  
        break;
        case E_ALGO_MSG_SPV_CALI_EN:{
            uint32_t* mode = (uint32_t*)data;
            switch(*mode){
                case E_CALI_SPV_WHOLE:
                    algo_state_handle(E_STATE_SPV_WHOLE,E_ALGO_EVENT_OPEN,mode);
                break;
                case E_CALI_SPV_PCBA:
                    algo_state_handle(E_STATE_SPV_PCBA,E_ALGO_EVENT_OPEN,mode);
                break;
                case E_CALI_SIX_FACE:
                    algo_state_handle(E_STATE_SPV_SIX_FACE,E_ALGO_EVENT_OPEN,mode);
                break;
            }
        }
        break;
        case E_ALGO_MSG_SPV_CALI_DIS:{
            algo_spv_cali_disable_mode();
        }
        break;
        case E_ALGO_MSG_ORI_EUL_CALI_EN:{
            uint32_t* steps = (uint32_t*)data;
            algo_state_handle(E_STATE_ORIG_EUL_CALI,E_ALGO_EVENT_OPEN,steps);
        }
        break;
        case E_ALGO_MSG_ORI_EUL_CALI_DIS:{
            algo_state_handle(E_STATE_ORIG_EUL_CALI,E_ALGO_EVENT_CLOSE,NULL);
        }
        break;
        case E_ALGO_MSG_AVG_AG_VALUE_EN:{
            algo_state_handle(E_ALGO_FUNC_AG_AVG_VALUE,NULL,NULL);
        }
        break;
        case E_ALGO_MSG_SAVE_BEFORE_POWEROFF:{
            algo_state_handle(E_ALGO_FUNC_SAVE_BEFORE_POWEROFF,NULL,NULL);
        }
        break;


#if defined(MESSAGE_SAMPLE_CODE) && (1 == MESSAGE_SAMPLE_CODE)
        case E_DML_ENABLE:{
            uint32_t* id = (uint32_t*)data;
            CWM_Sensor_Enable(*id);
        }
        break;
        case E_DML_DISABLE:{
            uint32_t* id = (uint32_t*)data;
            CWM_Sensor_Disable(*id);
        }
        break;
        case E_DML_SETING_CONTROL:{
            uint32_t* id = (uint32_t*)&data[0];
            SettingControl_t* scl = (SettingControl_t*)&data[4];
            CWM_SettingControl(*id,scl);
        }
        break;
#endif
        default:
        break;
    }
}

void cwm_algo_task(void *pvParameters){
    /*消息队列初始化*/
    algo_message_init();

    /*不同平台之间有差异，在这里提供差异化的初始化接口*/
    diskio_init();

    algo_init();

    period_ms = 1000/algo_get_odr();

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for(;;){
        struct algo_msg_t msg;
        while(!get_msg((uint8_t*)&msg)){
            algo_message_handle(msg.id,&msg.data[0]);
        }

        if(true == algo_running){
            CWM_Dml_process();            
        }
        algo_data_handle();

        #if defined(ALGO_TEST_EN) && (1 == ALGO_TEST_EN)
        algo_test();
        #endif

        period_ms = 1000/algo_get_odr();
        // CWM_OS_dbgPrintf("[cwm_algo_task]: \n");
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(period_ms));
    }
}

/****************************************************外部调用接口************************************************/
/*ctr=1:enable; 0:disable*/
void cwm_log_debug_ctl(uint32_t ctr)
{
    message_to_algo(E_ALGO_MSG_LOG_DEBUG_CTL,ctr);
}
/*ctr=1:enable; 0:disable*/
void cwm_hs_algo_ctl(uint32_t ctr)
{
    message_to_algo(E_ALGO_MSG_HS_ALGO_CTL,ctr);
}
/*
enum{
    E_CALI_SPV_WHOLE = 1,
    E_CALI_SPV_PCBA = 2,
    E_CALI_SIX_FACE = 5,
};
mode=1:E_CALI_SPV_WHOLE; 2:E_CALI_SPV_PCBA; 5:E_CALI_SIX_FACE
*/
void cwm_spv_cali_en(uint32_t mode)
{
    message_to_algo(E_ALGO_MSG_SPV_CALI_EN,mode);
}

void cwm_spv_cali_dis(void)
{
    message_to_algo(E_ALGO_MSG_SPV_CALI_DIS,0);
}
/*
enum{
    E_ANGLE_INIT_STEP1,
    E_ANGLE_INIT_STEP2,
    E_ANGLE_INIT_STEP3,
};
steps=0:E_ANGLE_INIT_STEP1; 1:E_ANGLE_INIT_STEP2; 2:E_ANGLE_INIT_STEP3
*/
void cwm_ori_eul_cali_en(uint32_t steps)
{
    message_to_algo(E_ALGO_MSG_ORI_EUL_CALI_EN,steps);
}

void cwm_ori_eul_cali_dis(void)
{
    message_to_algo(E_ALGO_MSG_ORI_EUL_CALI_DIS,0);
}

void cwm_avg_ag_value_en(void)
{
    message_to_algo(E_ALGO_MSG_AVG_AG_VALUE_EN,0);
}

void cwm_save_before_poweroff(void)
{
    message_to_algo(E_ALGO_MSG_SAVE_BEFORE_POWEROFF,0);
}


#if defined(MESSAGE_SAMPLE_CODE) && (1 == MESSAGE_SAMPLE_CODE)
//DML INTERFACE
void cwm_dml_enable(uint32_t id)
{
    message_to_algo(E_DML_ENABLE,id);
}
void cwm_dml_disable(uint32_t id)
{
    message_to_algo(E_DML_DISABLE,id);
}
void cwm_dml_seting_control(uint32_t id, SettingControl_t* scl)
{
    uint8_t tmp_data[4+4*16];
    memcpy(&tmp_data[0],&id,4);
    memcpy(&tmp_data[4],scl,4*16);
    data_to_algo(E_DML_SETING_CONTROL,tmp_data,4+4*16);
}
#endif







