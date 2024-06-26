#include "stdint.h"
#include "string.h"
#include "stdbool.h"
#include "stdio.h"

#include "cwm_lib.h"
#include "cwm_customio.h"
#include "cwm_config.h"
#include "cwm_port.h"


#include "FreeRTOS.h"

/*平台相关头文件*/
#include "syslog.h"
#include "timers.h"
#include "hal_log.h"
#include "hal_gpio.h"
#include "hal_i2c_master.h"
#include "hal_platform.h"
#include "hal_gpt.h"

#include "nvkey.h"
#include "nvkey_id_list.h"

#define CWM_ALGO_TEST     "cwm_algo_test"

/*** AT COMMAND ****/
#include "atci.h"
static bool algo_atci_parse_cmd(char *string)
{

    bool result = true;
    int num = atoi(&string[8]);
    printf("string num = %d",num);
    switch(num)
    {
        case 1:
            printf("AT cmd AT+ALGO=1 to enable IDX(100)");
            cwm_hs_algo_ctl(1);
        break;

        case 2:
            printf("AT cmd AT+ALGO=2 to disable IDX(100)");
            cwm_hs_algo_ctl(0); 
        break;

        case 3: 
            printf("AT cmd AT+ALGO=3 to enable log debug");
            cwm_log_debug_ctl(1);
        break;

        case 4: 
            printf("AT cmd AT+ALGO=4 to disable log debug");
            cwm_log_debug_ctl(0);
        break; 

        case 5: 
            printf("AT cmd AT+ALGO=5 to enable E_CALI_SPV_WHOLE");
            cwm_spv_cali_en(E_CALI_SPV_WHOLE);
        break;

        case 6: 
            printf("AT cmd AT+ALGO=6 to enable E_CALI_SPV_PCBA");
            cwm_spv_cali_en(E_CALI_SPV_PCBA);
        break;

        case 7: 
            printf("AT cmd AT+ALGO=7 to enable disable E_CALI_SIX_FACE");
            cwm_spv_cali_en(E_CALI_SIX_FACE);
        break;

        case 8: 
            printf("AT cmd AT+ALGO=8 to disable SPV_CALIB");
            cwm_spv_cali_dis();
        break;

        case 9: 
            printf("AT cmd AT+ALGO=9 to enable E_ANGLE_INIT_STEP1");
            cwm_ori_eul_cali_en(E_ANGLE_INIT_STEP1);
        break;

        case 10: 
            printf("AT cmd AT+ALGO=10 to enable E_ANGLE_INIT_STEP2");
            cwm_ori_eul_cali_en(E_ANGLE_INIT_STEP2);
        break;

        case 11: 
            printf("AT cmd AT+ALGO=11 to enable E_ANGLE_INIT_STEP3");
            cwm_ori_eul_cali_en(E_ANGLE_INIT_STEP3);
        break;

        case 12: 
            printf("AT cmd AT+ALGO=12 to disable ANGLE_INIT_CALIB");
            cwm_ori_eul_cali_dis();
        break;

        case 13: 
            printf("AT cmd AT+ALGO=13 to get CWM_ALGO_GET_AVER_AG_RAWDATA");
            cwm_avg_ag_value_en();
        break; 

        case 14:
            printf("AT cmd AT+ALGO=14 to get CWM_ALGO_GET_AUTO_CALIB_VALUE");
            cwm_save_before_poweroff();  
        break;
        
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



/****************************************************打印接口************************************************/
// int CWM_OS_dbgPrintf(const char * format,...) 
// {
//     va_list argList;
//     int size = 0;
//     char tBuffer[200];
//     memset(tBuffer,0,200);

//     va_start(argList, format);
//     size = vsnprintf(tBuffer, sizeof(tBuffer) - 1, format, argList);
//     va_end(argList);

//     app_log_str(tBuffer);

//     return 0;
// }

/****************************************************dml、algo 需要实现的接口************************************************/
ATTR_RWDATA_IN_NONCACHED_SYSRAM_4BYTE_ALIGN static uint8_t i2c_dma_write_buf[256];
ATTR_RWDATA_IN_NONCACHED_SYSRAM_4BYTE_ALIGN static uint8_t i2c_dma_read_buf[512];

void OS_algo_i2c_init(void)
{
	hal_i2c_config_t i2c;
    hal_i2c_status_t ret;

    hal_gpio_init(HAL_GPIO_33);
	hal_gpio_init(HAL_GPIO_34);
	hal_pinmux_set_function(HAL_GPIO_33, 6);
	hal_pinmux_set_function(HAL_GPIO_34, 6);
    i2c.frequency = HAL_I2C_FREQUENCY_400K;
    ret = hal_i2c_master_init(HAL_I2C_MASTER_2, &i2c); // initialize i2c master.
    if (ret != HAL_I2C_STATUS_OK) {
        log_hal_msgid_info("[" CWM_ALGO_TEST "] I2C%d initla fail,ret =%d\r\n", 2, HAL_I2C_MASTER_2, ret);
    } else {
        log_hal_msgid_info("[" CWM_ALGO_TEST "] I2C%d initla Done!\r\n", 1, HAL_I2C_MASTER_2);
    }
}

int OS_algo_i2c_read(uint16_t slaveAddr, uint16_t reg, int regLength, uint8_t *readData, int readDataSize, int busIndex)
{
    hal_i2c_status_t status = HAL_I2C_STATUS_OK;
    hal_i2c_send_to_receive_config_ex_t i2c_dma_config;
    hal_i2c_running_status_t running_status = {
        .running_status = HAL_I2C_STATUS_BUS_BUSY,
    };

    i2c_dma_write_buf[0] = reg & 0xff;

    i2c_dma_config.slave_address = slaveAddr;
    i2c_dma_config.send_data = i2c_dma_write_buf;
    i2c_dma_config.send_packet_length = 1;
    i2c_dma_config.send_bytes_in_one_packet = 1;
    i2c_dma_config.receive_buffer = i2c_dma_read_buf;
    i2c_dma_config.receive_packet_length = 1;
    i2c_dma_config.receive_bytes_in_one_packet = readDataSize;

    status = hal_i2c_master_send_to_receive_dma_ex(HAL_I2C_MASTER_2, &i2c_dma_config);
    if(status != HAL_I2C_STATUS_OK) {
        log_hal_msgid_info("[" CWM_ALGO_TEST "] I2C read error code = %d", 1, status);
        return -1;
    }

    while(1) {
        hal_i2c_master_get_running_status(HAL_I2C_MASTER_2, &running_status);
        if(running_status.running_status != HAL_I2C_STATUS_BUS_BUSY) {
            break;
        }
        hal_gpt_delay_us(1);
        // vTaskDelay(1);
    }

    if (readData == NULL) {
        log_hal_msgid_info("[" CWM_ALGO_TEST "] readData is NULL", 0);
    } else {
        memcpy(readData, i2c_dma_read_buf, readDataSize);
    }

    return 0;
}

int OS_algo_i2c_write(uint16_t slaveAddr, uint16_t reg, int regLength, uint8_t *writeData, int writeDataSize, int busIndex)
{
    hal_i2c_status_t status = HAL_I2C_STATUS_OK;
    hal_i2c_send_config_t i2c_buf_config;
    hal_i2c_running_status_t running_status = {
        .running_status = HAL_I2C_STATUS_BUS_BUSY,
    };

    i2c_dma_write_buf[0] = reg & 0xff;
    if (writeData != NULL && writeDataSize != 0) {
        memcpy(&i2c_dma_write_buf[1], writeData, writeDataSize);
    }

    i2c_buf_config.slave_address = slaveAddr;
    i2c_buf_config.send_data = i2c_dma_write_buf;
    i2c_buf_config.send_packet_length = 1;
    i2c_buf_config.send_bytes_in_one_packet = 1 + writeDataSize;

    status = hal_i2c_master_send_dma_ex(HAL_I2C_MASTER_2, &i2c_buf_config);
    if (status != HAL_I2C_STATUS_OK) {
        log_hal_msgid_info("[" CWM_ALGO_TEST "] cwm i2c write error code = %d", 1, status);
        return -1;
    }

    while (1) {
        hal_i2c_master_get_running_status(HAL_I2C_MASTER_2, &running_status);
        if(running_status.running_status != HAL_I2C_STATUS_BUS_BUSY) {
            break;
        }
        hal_gpt_delay_us(1);
        // vTaskDelay(1);
    }

    return 0;
}

uint64_t OS_algo_get_time_ns(void)
{
    static uint32_t last_count = 0;
    static uint64_t systime = 0;
    uint32_t count;
    uint32_t duration;

    taskENTER_CRITICAL();
	hal_gpt_get_free_run_count(HAL_GPT_CLOCK_SOURCE_1M, &count);
    hal_gpt_get_duration_count(last_count, count, &duration);

    last_count = count;
    systime += duration;
    taskEXIT_CRITICAL();

	return (systime * 1000); //(uint64_t) HAL_GetTick() * 1000000;
}

void OS_algo_usleep(unsigned int udelay)
{
   hal_gpt_delay_us(udelay);
}

int OS_algo_printstr(const char * format)
{
    if (format == NULL) {
        log_hal_msgid_info("[" CWM_ALGO_TEST "] format is NULL", 0);
        return 0;
    }
    CWM_OS_dbgPrintf("%s", format);
	return 0;
}


os_api customio_os_api = {
    .dbgOutput = OS_algo_printstr,
    .GetTimeNs = OS_algo_get_time_ns,
    .uSleep = OS_algo_usleep,
    .i2cRead = OS_algo_i2c_read,
    .i2cWrite = OS_algo_i2c_write,
};

/****************************************************配置 sensor 相关参数************************************************/
#define CWM_DEFAUL_ODR     50
const uint16_t defautl_odr = CWM_DEFAUL_ODR;
const int dml_vendor_config[16] = {1,2};
const int dml_hw_config[16] = {1,2,0,0,0,0,3002301,1+8};
const int dml_ag_config[16] = {1,1,1,2,1,0,25,22,CWM_DEFAUL_ODR,16,1000,0,0};
const int dml_mag_config[16] = {1,1,1,1,1,0,0,4,CWM_DEFAUL_ODR};
const int dml_hs_orien_config[16] = {1,15000,0,15,8,2,1,0,0,0,0,1,0,2};
const int dml_hs_intf_config[16] = {1,0,3,11,7};                                            /*M3 {1,0,3,11,7}*/
const int dml_ag_pref_config_default[16] = {1,0,0,150,120,100};
const int dml_ag_pref_config_standby[16] = {1,0,0,100};
const int dml_log_config[16] = {1,0,0,1+2+4+8,64+5,3,-1-1-2-4-8-16-32,-1};
const int dml_log_debug_config[16] = {1,1+2+4+8+16};
const float algo_quiet_lev = 0.15f;
const uint32_t algo_quiet_timeout_min = 60*10;

/****************************************************flash 读写需要实现的接口************************************************/
void customio_read_flash_cali(uint8_t* data,uint32_t len)
{
    uint8_t* addr = data;
    nvkey_status_t state = nvkey_read_data(NVID_CWM_SPV_CALIB_PARAMETERS,addr,&len);
    CWM_OS_dbgPrintf("[algo] read spv calib value from flash state %d ",state);
}

void customio_save_flash_cali(uint8_t* data,uint32_t len)
{
    const uint8_t* addr = (const uint8_t*)data;
    nvkey_status_t status = nvkey_write_data(NVID_CWM_SPV_CALIB_PARAMETERS,addr,len);
    CWM_OS_dbgPrintf("[algo] save spv calib value to flash  state: %d",status);
}

void customio_read_flash_eul(uint8_t* data,uint32_t len)
{
   uint8_t* addr = data;
   nvkey_status_t state = nvkey_read_data(NVID_CWM_ANGLE_INIT_PARAMETERS,addr,&len);
   CWM_OS_dbgPrintf("[algo] read angle init value from flash state %d ",state);
}

void customio_save_flash_eul(uint8_t* data,uint32_t len)
{
    const uint8_t* addr = (const uint8_t*)data;
    nvkey_status_t status = nvkey_write_data(NVID_CWM_ANGLE_INIT_PARAMETERS,addr,len);
    CWM_OS_dbgPrintf("[algo] save angle init value to flash  state: %d",status);
}


/****************************************************客户接口：实现数据传输************************************************/
void customio_init(void)
{
    //IIC 硬件初始化
    OS_algo_i2c_init();

    //AT COMMAND 
    algo_atci_init();

}
/*算法输出 ag 数据，在算法设置中调用，
type：
    bit0:    1: ACC
    bit1:    2: GYRO
float *f:
    struct ag_t{
        float ax;//单位：m/s^2
        float ay;//单位：m/s^2
        float az;//单位：m/s^2
        float gx;//单位：rad/s
        float gy;//单位：rad/s
        float gz;//单位：rad/s
    };
uint8_t idx:
    当前 ag 数据包序号
uint8_t nums：
     ag 数据总包数
*/
void customio_read_ag(uint8_t type, float *f, uint8_t idx, uint8_t nums)
{
    // CWM_OS_dbgPrintf("[customio]read acc-gyro: %f,%f,%f   ,%f,%f,%f\n",
    //         f[0],f[1],f[2],
    //         f[3],f[4],f[5]);
}
/*算法输出欧拉角和四元素数据：在算法设置中调用，
float *f:
    struct eul_qua_t{
        float yaw;  //(degrees)
        float pitch;//(degrees)
        float roll; //(degrees)
        float x;    //quaternion_x
        float y;    //quaternion_y
        float z;    //quaternion_z
        float w;    //quaternion_w
    };
*/
void customio_read_eul_qua(float *f)
{
    CWM_OS_dbgPrintf("[customio]read eul-qua: yaw=%f pitch=%f roll=%f x=%f y=%f z=%f w=%f\n",
            f[0],f[1],f[2],
            f[3],f[4],f[5],f[6]);
}
/*算法输出 ag 数据 1s 平均值，在算法设置中调用，
float *f:
    struct ag_t{
        float ax;//单位：m/s^2
        float ay;//单位：m/s^2
        float az;//单位：m/s^2
        float gx;//单位：rad/s
        float gy;//单位：rad/s
        float gz;//单位：rad/s
    };
*/
void customio_read_ag_avg_value(float *f)
{
    CWM_OS_dbgPrintf("[customio]ag avg:ax=%f,ay=%f,az=%f,gx=%f,gy=%f,gz=%f\n",
            f[0],f[1],f[2],
            f[3],f[4],f[5]);
}
/*算法输出 spv 校正结果，在算法设置中调用，
校正结果枚举：
    enum{
        E_CALI_FAIL,//失败
        E_CALI_RUNNING,//执行中
        E_CALI_SUCCESS,//成功
    };
whl：整机校正结果。
pcba：pcba 校正结果。
sixf：六面校正结果。
*/
void customio_notify_spv_cali_result(uint16_t whl,uint16_t pcba,uint16_t sixf)
{
    CWM_OS_dbgPrintf("[customio]customio_notify_spv_cali_result : whl=%d pcba=%d sixf=%d  [2==success / 0==fail]\n",
        whl,
        pcba,
        sixf); 
}
/*算法输出初始化角度校正结果，在算法设置中调用，
steps：当前校正步骤：
    enum{
        E_ANGLE_INIT_STEP1 = 1,//第一步
        E_ANGLE_INIT_STEP2 = 2,//第二步
        E_ANGLE_INIT_STEP3 = 3,//第三步
    };
status：当前校正步骤结果：
    enum{
        E_ORI_EUL_CALI_FAIL,//失败
        E_ORI_EUL_CALI_RUNNING,//执行中
        E_ORI_EUL_CALI_SUCCESS,//成功
    };
*/
void customio_notify_ori_eul_cali_result(uint16_t steps,uint16_t status)
{
    CWM_OS_dbgPrintf("[customio]customio_notify_ori_eul_cali_result: runing steps=%d status=%d  [2==success / 0==fail]\n",
        steps,
        status);
}





