
#include "stdint.h"
#include "string.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "queue.h"




#include "cwm_config.h"
#include "cwm_common.h"
#include "cwm_port.h"
#include "cwm_test.h"


#include "atci.h"




static bool algo_running = true;     /*算法调用条件*/
uint32_t algo_period_ms;      /*算法调用周期*/


/*** AT COMMAND ****/

static bool algo_atci_parse_cmd(char *string)
{

    bool result = true;
    int num = atoi(&string[8]);
    printf("string num = %d",num);
    switch(num)
    {
        case 1: 
            printf("AT cmd AT+ALGO=1 to enable IDX(100)");
            cwm_outside_headset_ori_ctr(1);
        break;

        case 2: 
            printf("AT cmd AT+ALGO=2 to disable IDX(100)");
            cwm_outside_headset_ori_ctr(0);
        break;

        case 3: 
            printf("AT cmd AT+ALGO=3 to enable sensor driver");
            cwm_outside_driver_ctr(1);
        break;

        case 4: 
            printf("AT cmd AT+ALGO=4 to disable sensor driver");
            cwm_outside_driver_ctr(0);
        break; 

        case 5: 
            printf("AT cmd AT+ALGO=5 to enable SPV_MODE_WHOLE_MACHINE");
            cwm_outside_spv_calib_en(SPV_MODE_WHOLE_MACHINE);
        break;

        case 6: 
            printf("AT cmd AT+ALGO=6 to enable SPV_MODE_PCBA");
            cwm_outside_spv_calib_en(SPV_MODE_PCBA);
        break;

        case 7: 
            printf("AT cmd AT+ALGO=7 to enable disable SPV_MODE_SIX_FACE");
            cwm_outside_spv_calib_en(SPV_MODE_SIX_FACE);
        break;

        case 8: 
            printf("AT cmd AT+ALGO=8 to disable SPV_CALIB");
            cwm_outside_spv_calib_dis();
        break;

        case 9: 
            printf("AT cmd AT+ALGO=9 to enable ANGLE_INIT_FIR");
            cwm_outside_angle_init_en(ANGLE_INIT_FIR);
        break;

        case 10: 
            printf("AT cmd AT+ALGO=10 to enable ANGLE_INIT_SEC");
            cwm_outside_angle_init_en(ANGLE_INIT_SEC);
        break;

        case 11: 
            printf("AT cmd AT+ALGO=11 to enable ANGLE_INIT_THI");
            cwm_outside_angle_init_en(ANGLE_INIT_THI);
        break;

        case 12: 
            printf("AT cmd AT+ALGO=12 to disable ANGLE_INIT_CALIB");
            cwm_outside_angle_init_dis();
        break;

        case 13: 
            printf("AT cmd AT+ALGO=13 to get CWM_ALGO_GET_AVER_AG_RAWDATA");
            cwm_outside_get_aver_ag_rawdata();
        break; 

        case 14:
            printf("AT cmd AT+ALGO=14 to get CWM_ALGO_GET_AUTO_CALIB_VALUE");
            cwm_outside_save_auto_calib_value();  
        
    }

    return result;

}

static atci_status_t algo_atci_reg_callback(atci_parse_cmd_param_t *parse_cmd)
{
    atci_response_t output = {{0}};
    LOG_I(common, "[ALGO_ATCI] mode:%d, CMD:%s", parse_cmd->mode, parse_cmd->string_ptr);
    if (parse_cmd->mode == ATCI_CMD_MODE_EXECUTION) {
        /* Execute mode command, such as "AT+CMD=<op>. */
        bool result;
        result = algo_atci_parse_cmd(parse_cmd->string_ptr);
        if (result) {
            /* Parse atci cmd successed. */
            strncpy((char *)output.response_buf, "Parse OK\n", strlen("Parse OK\n") + 1);
        } else {
            /* Parse atci cmd failed. */
            strncpy((char *)output.response_buf, "Parse failed\n", strlen("Parse failed\n") + 1);
        }
        output.response_len = strlen((char *)output.response_buf);
        output.response_flag = 0 | ATCI_RESPONSE_FLAG_APPEND_OK;
        atci_send_response(&output);
    } else {
        /* Others unsupported atci cmd, such as ATCI_CMD_MODE_TESTING. */
        strncpy((char *)output.response_buf, "Not Support\n", strlen("Not Support\n") + 1);
        output.response_len = strlen((char *)output.response_buf);
        output.response_flag = 0 | ATCI_RESPONSE_FLAG_APPEND_ERROR;
        atci_send_response(&output);
    }
    return ATCI_STATUS_OK;
}

static atci_cmd_hdlr_item_t algo_at_cmd[] = {
    {"AT+ALGO", algo_atci_reg_callback, 0, 0},
   
};

#define ALGO_ATCI_COMMAND_COUNT (sizeof(algo_at_cmd)/sizeof(atci_cmd_hdlr_item_t))  /* The count of atci cmd handlers. */

void algo_atci_init(void)
{
    atci_status_t ret = atci_register_handler(&algo_at_cmd[0], ALGO_ATCI_COMMAND_COUNT);
    if (ret == ATCI_STATUS_OK) {
        /* Register atci handler successed. */
        printf("algo_atci register success");
    } else {
        /* Register atci handler failed. */
        printf("algo_atci register fail");
    }
}



 

/*外部调用接口发送算法消息*/
/*一个 ID 对应着一个功能，同时也对应了一个数据结构，解析时则按照这个数据结构进行解析*/
enum{
   CWM_ALGO_SENSOR_ENABLE,
   CWM_ALGO_SENSOR_DISABLE,
   CWM_ALGO_SETTING_CONTROL,

   CWM_ALGO_LOG_DEBUG_CONTROL,
   CWM_ALGO_HEADSET_ORI_CONTROL,
   CWM_ALGO_DRIVER_CONTROL,
   CWM_ALGO_SPV_CALIB_EN,
   CWM_ALGO_SPV_CALIB_DIS,
   CWM_ALGO_ANGLE_INIT_EN,
   CWM_ALGO_ANGLE_INIT_DIS,
   CWM_ALGO_GET_AVER_AG_RAWDATA,
   CWM_ALGO_SAVE_AUTO_CALIB_VALUE,
   
};

void cwm_outside_algo_sensor_enable(uint32_t idx)
{
    
    send_single_to_algo_message(CWM_ALGO_SENSOR_ENABLE,idx);
    
}

void cwm_outside_algo_sensor_disable(uint32_t idx)
{
    
    send_single_to_algo_message(CWM_ALGO_SENSOR_DISABLE,idx);
    
}

void cwm_outside_algo_setting_control(int32_t cid , SettingControl_t* scl)
{
    
    uint8_t tmp_data[4+4*16];
    memcpy(&tmp_data[0],&cid,4);
    memcpy(&tmp_data[4],scl,4*16);
    send_multi_to_algo_message(CWM_ALGO_SETTING_CONTROL,tmp_data,4+4*16);

}   

void cwm_outside_log_debug_ctr(uint32_t ctr)
{
	
    send_single_to_algo_message(CWM_ALGO_LOG_DEBUG_CONTROL,ctr);
    
}

void cwm_outside_headset_ori_ctr(uint32_t ctr)
{
    
	send_single_to_algo_message(CWM_ALGO_HEADSET_ORI_CONTROL,ctr);
    
}

void cwm_outside_driver_ctr(uint32_t ctr)
{
    
	send_single_to_algo_message(CWM_ALGO_DRIVER_CONTROL,ctr);
    
}

void cwm_outside_spv_calib_en(uint32_t mode)
{
    
    send_single_to_algo_message(CWM_ALGO_SPV_CALIB_EN,mode);
	
}

void cwm_outside_spv_calib_dis(void)
{
    
    send_single_to_algo_message(CWM_ALGO_SPV_CALIB_DIS,0);
	
}

void cwm_outside_angle_init_en(uint32_t times)
{
    
    send_single_to_algo_message(CWM_ALGO_ANGLE_INIT_EN,times);
    
}

void cwm_outside_angle_init_dis(void)
{
    
    send_single_to_algo_message(CWM_ALGO_ANGLE_INIT_DIS,0);
    
}

void cwm_outside_get_aver_ag_rawdata(void)
{

    send_single_to_algo_message(CWM_ALGO_GET_AVER_AG_RAWDATA,0);

}

void cwm_outside_save_auto_calib_value(void)
{

    send_single_to_algo_message(CWM_ALGO_SAVE_AUTO_CALIB_VALUE,0);
}

/*解析算法消息*/
void cwm_algo_message_handle(void)
{
    algo_msg_t msg;
    while(receive_from_algo_message((uint8_t*)&msg) == 0){
        switch (msg.id){    

            case CWM_ALGO_SENSOR_ENABLE:  //开启任意算法
                {
                    uint32_t* id = (uint32_t*)&msg.data[0];
                    CWM_Sensor_Enable(*id);
                }
            break;

            case CWM_ALGO_SENSOR_DISABLE: //关闭任意算法
                {
                    uint32_t* id = (uint32_t*)&msg.data[0];
                    CWM_Sensor_Disable(*id);
                }
            break;

            case CWM_ALGO_SETTING_CONTROL: //设置任意CID命令
                {
                    int* id = (int*)&msg.data[0];
                    SettingControl_t* scl = (SettingControl_t*)&msg.data[4];
                    CWM_SettingControl(*id,scl);
                }
            break;

            case CWM_ALGO_LOG_DEBUG_CONTROL:  // LOG DEBUG 开关
                {
                    uint32_t* ctr = (uint32_t*)&msg.data[0];
                    algo_log_debug_ctr(*ctr);
                }
            break;

            case CWM_ALGO_HEADSET_ORI_CONTROL:  //IDX(100) 开关
                {
                    uint32_t* ctr = (uint32_t*)&msg.data[0];
                    algo_headset_ori_ctr(*ctr);
                }
            break;

            case CWM_ALGO_DRIVER_CONTROL:   //sensor 驱动开关
                {
                    uint32_t* ctr = (uint32_t*)&msg.data[0];
                    algo_driver_ctr(*ctr);
                    if(*ctr){
                        algo_running = true;
                    }
                    else{
                        algo_running = false;
                    }
                }
            break;

            case CWM_ALGO_SPV_CALIB_EN:        //使能 SPV 校正算法：整机校正，PCBA校正，六面校正
                {
                    // 关闭自动校正
                    SettingControl_t scl;
                    memset(&scl, 0, sizeof(scl));
                    scl.iData[0] = 1;
                    scl.iData[3] = -1;
                    CWM_SettingControl(SCL_SENS_CALI_CONFIG, &scl);

                    uint32_t* mode = (uint32_t*)&msg.data[0];
                    algo_spv_calib_en(*mode); 
                }
            break;
            
            case CWM_ALGO_SPV_CALIB_DIS:      //关闭 SPV 校正算法
                {
                    //打开自动校正
                    SettingControl_t scl;
                    memset(&scl, 0, sizeof(scl));
                    scl.iData[0] = 1;
                    CWM_SettingControl(SCL_SENS_CALI_CONFIG, &scl);

                    CWM_Sensor_Disable(20);
                }
            break;
            
            case CWM_ALGO_ANGLE_INIT_EN:       //使能初始角度校正算法：第一次校正，第二次校正，第三次校正
                {
                    uint32_t* times = (uint32_t*)&msg.data[0];
                    algo_angle_init_en(*times);
                }
            break;
            
            case CWM_ALGO_ANGLE_INIT_DIS:     //关闭初始角度校正算法
                {
                    CWM_Sensor_Disable(112);
                }
            break;

            case CWM_ALGO_GET_AVER_AG_RAWDATA:  //获取ACC&GYRO 原始数据平均值
                {
                    algo_get_aver_ag_rawdata();
                }
            break;
            
            case CWM_ALGO_SAVE_AUTO_CALIB_VALUE: //保存自动校正的 gyro 校正值
                {
                    algo_save_auto_calib_value();
                }
            break;
            
            default:
            break;
        }
    }
    
}




/*算法任务运行*/
void cwm_algo_task(void *param)
{
    
    //AT COMMAND 
    algo_atci_init();
    
    //算法消息队列初始化
    algo_message_init();
    
    //算法库&DML库初始化
    cwm_algo_init();
    
    static TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        
        //解析算法消息
        cwm_algo_message_handle();
        
        if(algo_running){
            CWM_Dml_process();
        }
        
        //处理监听数据
        cwm_algo_data_handle();

        #if defined(ALGO_TEST_EN) && (1 == ALGO_TEST_EN)
        algo_test();
        #endif

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(algo_period_ms));   //绝对延时
    }
    
    
}




































