
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cwm_lib.h"
#include "cwm_lib_dml.h"
#include "cwm_config.h"

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
algo_info_t algo_info;
extern uint32_t algo_period_ms;




/*** BASIC API ****/
ATTR_RWDATA_IN_NONCACHED_SYSRAM_4BYTE_ALIGN static uint8_t i2c_dma_write_buf[256];
ATTR_RWDATA_IN_NONCACHED_SYSRAM_4BYTE_ALIGN static uint8_t i2c_dma_read_buf[512];

void bsp_gsensor_platform_i2c_init_m3(void)
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

int CWM_OS_i2cRead_m3(uint16_t slaveAddr, uint16_t reg, int regLength, uint8_t *readData, int readDataSize, int busIndex)
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

int CWM_OS_i2cWrite_m3(uint16_t slaveAddr, uint16_t reg, int regLength, uint8_t *writeData, int writeDataSize, int busIndex)
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

int CWM_OS_dbgOutput_m3(const char * format)
{
    if (format == NULL) {
        log_hal_msgid_info("[" CWM_ALGO_TEST "] format is NULL", 0);
        return 0;
    }
    CWM_OS_dbgPrintf("%s", format);
    return 0;
}

void CWM_OS_uSleep_m3(uint32_t udelay)
{
    hal_gpt_delay_us(udelay);
}

uint64_t CWM_OS_GetTimeNs_m3(void)
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

/*** BASIC API ****/





////////////////////////////////////////////////////////////////本地调用接口////////////////////////////////////////////////////////

void test_read_hs_data(float *f)
{
    CWM_OS_dbgPrintf("[cwm_test]: ypr = %f,%f,%f   xyzw = %f,%f,%f,%f",f[0],f[1],f[2],f[3],f[4],f[5],f[6]);
}

void test_read_spv_calib_value(void)
{
    spv_calib_value_t* p = &algo_info.spv_calib_value;
    CWM_OS_dbgPrintf("[cwm_test] spv crc_16 = %d WM_status = %d PB_status = %d SF_status = %d valid = %d ax = %d ay = %d az = %d gx = %d gy = %d gz = %d",
            p->crc_16,p->whole_machine_status,p->pcba_status,p->sixface_status,p->valid,p->ax,p->ay,p->az,p->gx,p->gy,p->gz);

}

void test_read_angle_init_value(void)
{
    angle_init_value_t* p = &algo_info.angle_init_value;
    CWM_OS_dbgPrintf("[cwm_test] angle crc_16 = %d step1_status = %d  step2_status = %d step3_status = %d valid = %d yaw = %f pitch = %f roll = %f ",
            p->crc_16,p->step1_status,p->step2_status,p->step3_status,p->valid,p->yaw,p->pitch,p->roll);

}

void test_read_aver_ag_rawdata(void)
{
    ag_aver_t*p = &algo_info.ag_aver_value;
    CWM_OS_dbgPrintf("[cwm_test] acc_gyro_aver_value ax = %f ay = %f az = %f gx = %f gy = %f gz = %f ",p->ax,p->ay,p->az,p->gx,p->gy,p->gz);

}

static uint16_t check_sum(uint8_t* data, uint32_t len)
{
    if(NULL == data){
        return 0;
    }

    uint16_t sum = 0;
    for(uint32_t i = 0; i < len; i++){
        sum += data[i];
    }
    return sum;
}

static void update_spv_calib_value_checksum(void)
{
    //计算校验时，注意 4 字节对齐问题 （crc_16 之后工厂校正数据的每个字节之和）
    uint8_t* addr =  (uint8_t*)&algo_info.spv_calib_value.crc_16 + 2 ;
    uint32_t len = 2 + 24;
    algo_info.spv_calib_value.crc_16 = check_sum(addr,len);
}

static void update_auto_calib_value_checksum(void)
{
    //计算校验时，注意 4 字节对齐问题 （auto_crc_16 之后算法持续校正数据的每个字节之和）
    uint8_t* addr =  (uint8_t*)&algo_info.spv_calib_value.auto_crc16 + 2 ;
    uint32_t len = 2 + 12;
    algo_info.spv_calib_value.auto_crc16 = check_sum(addr,len);
}

static void update_angle_init_value_checksum(void)
{
    //计算校验时，注意 4 字节对齐问题 （crc_16 之后每个字节之和）
    uint8_t* addr =  (uint8_t*)&algo_info.angle_init_value.crc_16 + 2;
    uint32_t len = 2 + 12;
    algo_info.angle_init_value.crc_16 = check_sum(addr,len);
}

static void algo_read_param_from_flash(void) 
{

    //读取 spv calib 数据
    spv_calib_value_t spv_calib_test;
    uint8_t* spv_calib_test_addr = (uint8_t*)&spv_calib_test;
    uint32_t spv_calib_test_len = sizeof(spv_calib_value_t);
    nvkey_status_t state_1 = nvkey_read_data(NVID_CWM_SPV_CALIB_PARAMETERS,spv_calib_test_addr,&spv_calib_test_len);
    CWM_OS_dbgPrintf("[cwm_debug] read spv calib value from flash state %d ",state_1);

    if(spv_calib_test_addr != NULL){

        //校验工厂校正数据（whole_machine_status --> gz）
        uint8_t* addr = spv_calib_test_addr + 2;
        uint32_t len = 2 + 24;
        uint16_t crc_16 = check_sum(addr,len);

        if(crc_16 != spv_calib_test.crc_16){

            algo_info.spv_calib_value.valid = 0;
            algo_info.spv_calib_value.whole_machine_status = SPV_CALIB_FAIL;
            algo_info.spv_calib_value.pcba_status = SPV_CALIB_FAIL;
            algo_info.spv_calib_value.sixface_status = SPV_CALIB_FAIL;
            update_spv_calib_value_checksum();
            CWM_OS_dbgPrintf("[cwm_debug] spv calib crc verification failed");
        }
        else{
            memcpy(&algo_info.spv_calib_value.crc_16,&spv_calib_test.crc_16,2+2+24);
            CWM_OS_dbgPrintf("[cwm_debug] spv calib crc verification passed");
        }

        CWM_OS_dbgPrintf("[cwm_debug] spv calib flash crc_16 = %d whole_machine_status = %d pcba_status = %d sixface_status = %d valid = %d",
                algo_info.spv_calib_value.crc_16,algo_info.spv_calib_value.whole_machine_status, algo_info.spv_calib_value.pcba_status,
                algo_info.spv_calib_value.sixface_status,algo_info.spv_calib_value.valid);
        CWM_OS_dbgPrintf("[cwm_debug] spv calib flash ax = %d ay = %d az %d gx = %d gy = %d gz = %d",
                algo_info.spv_calib_value.ax,algo_info.spv_calib_value.ay,algo_info.spv_calib_value.az,
                algo_info.spv_calib_value.gx,algo_info.spv_calib_value.gy,algo_info.spv_calib_value.gz);


        //校验自动校正数据（auto_valid --> auto_gz）
        addr = spv_calib_test_addr + 2 + 2 + 24 + 2;
        len = 2 + 12;
        crc_16 = check_sum(addr,len);

        if(crc_16 != spv_calib_test.auto_crc16){

            algo_info.spv_calib_value.auto_valid = 0;
            update_auto_calib_value_checksum();
            CWM_OS_dbgPrintf("[cwm_debug] auto calib crc verification failed");
        }
        else{
            memcpy(&algo_info.spv_calib_value.auto_crc16,&spv_calib_test.auto_crc16,2+2+12);
            CWM_OS_dbgPrintf("[cwm_debug] auto calib crc verification passed");
        }
        CWM_OS_dbgPrintf("[cwm_debug] spv auto calib flash auto_crc16 = %d auto_valid = %d  auto_gx = %d auto_gy = %d auto_gz = %d",
                algo_info.spv_calib_value.auto_crc16,algo_info.spv_calib_value.auto_valid,
                algo_info.spv_calib_value.auto_gx,algo_info.spv_calib_value.auto_gy,algo_info.spv_calib_value.auto_gz);
    }

    //读取 angle init 数据
    angle_init_value_t angle_init_test;
    uint8_t* angle_init_test_addr = (uint8_t*)&angle_init_test;
    uint32_t angle_init_test_len = sizeof(angle_init_value_t);
    nvkey_status_t state_2 = nvkey_read_data(NVID_CWM_ANGLE_INIT_PARAMETERS,angle_init_test_addr,&angle_init_test_len);
    CWM_OS_dbgPrintf("[cwm_debug] read angle init value from flash state %d ",state_2);

    if(angle_init_test_addr != NULL){

        //校验初始角度数据（step1_status --> roll)
        uint8_t* addr = angle_init_test_addr + 2;
        uint32_t len = 2 + 12;
        uint16_t crc_16 = check_sum(addr,len);

        if(crc_16 != angle_init_test.crc_16){

            algo_info.angle_init_value.valid = 0;
            algo_info.angle_init_value.step1_status = ANGLE_INIT_FAIL;
            algo_info.angle_init_value.step2_status = ANGLE_INIT_FAIL;
            algo_info.angle_init_value.step3_status = ANGLE_INIT_FAIL;
            update_angle_init_value_checksum();
            CWM_OS_dbgPrintf("[cwm_debug] angle init crc verification failed");
        }
        else{
            memcpy(&algo_info.angle_init_value.crc_16,&angle_init_test.crc_16,2+2+12);
            CWM_OS_dbgPrintf("[cwm_debug] angle init crc verification passed");
        }

        CWM_OS_dbgPrintf("[cwm_debug] angle init flash crc_16 = %d step1_status %d step2_status = %d step3_status = %d valid = %d",
                algo_info.angle_init_value.crc_16,algo_info.angle_init_value.step1_status,algo_info.angle_init_value.step2_status,
                algo_info.angle_init_value.step3_status,algo_info.angle_init_value.valid);
        CWM_OS_dbgPrintf("[cwm_debug] angle init flash yaw = %f pitch = %f roll = %f",
                algo_info.angle_init_value.yaw,algo_info.angle_init_value.pitch,algo_info.angle_init_value.roll);

    }


}

static void algo_write_param_to_flash(uint32_t type)
{

    if(SPV_CALIB_PARAM == type){
        //将 SPV 校正数据写入 FLASH 
        const uint8_t* addr = (const uint8_t *)&algo_info.spv_calib_value;
        uint32_t len = sizeof(spv_calib_value_t);
        nvkey_status_t status = nvkey_write_data(NVID_CWM_SPV_CALIB_PARAMETERS,addr,len);
        CWM_OS_dbgPrintf("[cwm_debug] save spv calib value to flash  state: %d",status);

        //检查写入 FLASH 的数据是否正确
        spv_calib_value_t test;
        uint8_t* test_addr = (uint8_t*)&test;
        uint32_t test_len = sizeof(spv_calib_value_t);
        nvkey_status_t state = nvkey_read_data(NVID_CWM_SPV_CALIB_PARAMETERS,test_addr,&test_len);
        CWM_OS_dbgPrintf("[cwm_debug] read spv calib value from flash state %d ",state);
        if(memcmp(&algo_info.spv_calib_value,&test,sizeof(spv_calib_value_t)) == 0){
            CWM_OS_dbgPrintf("[cwm_debug] save spv calib value to falsh success");
        }
        else{
            CWM_OS_dbgPrintf("[cwm_debug] save spv calib value to falsh fail");  
        }
        CWM_OS_dbgPrintf("[cwm_debug] spv calib flash crc_16 = %d whole_machine_status = %d pcba_status = %d sixface_status = %d valid = %d",
                test.crc_16,test.whole_machine_status, test.pcba_status,
                test.sixface_status,test.valid);
        CWM_OS_dbgPrintf("[cwm_debug] spv calib flash ax = %d ay = %d az %d gx = %d gy = %d gz = %d",
                test.ax,test.ay,test.az,
                test.gx,test.gy,test.gz);
        CWM_OS_dbgPrintf("[cwm_debug] spv auto calib flash auto_crc16 = %d auto_valid = %d  auto_gx = %d auto_gy = %d auto_gz = %d",
                test.auto_crc16,test.auto_valid,
                test.auto_gx,test.auto_gy,test.auto_gz);

    }
    else if(ANGLE_INIT_PARAM == type){
        //将 初始角度校正数据 写入 FLASH
        const uint8_t* addr = (const uint8_t *)&algo_info.angle_init_value;
        uint32_t len = sizeof(angle_init_value_t);
        nvkey_status_t status = nvkey_write_data(NVID_CWM_ANGLE_INIT_PARAMETERS,addr,len);
        CWM_OS_dbgPrintf("[cwm_debug] save angle init value to flash  state: %d",status);

        //检查写入 FLASH 的数据是否正确
        angle_init_value_t test;
        uint8_t* test_addr = (uint8_t*)&test;
        uint32_t test_len = sizeof(angle_init_value_t);
        nvkey_status_t state = nvkey_read_data(NVID_CWM_ANGLE_INIT_PARAMETERS,test_addr,&test_len);
        CWM_OS_dbgPrintf("[cwm_debug] read angle init value from flash state %d ",state);
        if(memcmp(&algo_info.angle_init_value,&test,sizeof(angle_init_value_t)) == 0){
            CWM_OS_dbgPrintf("[cwm_debug] save angle init value to falsh success");
        }
        else{
            CWM_OS_dbgPrintf("[cwm_debug] save angle init value to falsh fail");
        }
        CWM_OS_dbgPrintf("[cwm_debug] angle init flash crc_16 = %d step1_status %d step2_status = %d step3_status = %d valid = %d",
                test.crc_16,test.step1_status,test.step2_status,
                test.step3_status,test.valid);
        CWM_OS_dbgPrintf("[cwm_debug] angle init flash yaw = %f pitch = %f roll = %f",
                test.yaw,test.pitch,test.roll);

    }
}

static void algo_set_spv_calib_angle_init(void)
{
    SettingControl_t scl;
    if(algo_info.spv_calib_value.valid){
        //设置 SPV 校正信息
        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[1] = 2;
        scl.iData[2] = algo_info.spv_calib_value.ax;
        scl.iData[3] = algo_info.spv_calib_value.ay;
        scl.iData[4] = algo_info.spv_calib_value.az;
        if(algo_info.spv_calib_value.auto_valid){
            scl.iData[5] = algo_info.spv_calib_value.auto_gx;
            scl.iData[6] = algo_info.spv_calib_value.auto_gy;
            scl.iData[7] = algo_info.spv_calib_value.auto_gz;
        }
        else{
            scl.iData[5] = algo_info.spv_calib_value.gx;
            scl.iData[6] = algo_info.spv_calib_value.gy;
            scl.iData[7] = algo_info.spv_calib_value.gz;
        }
        CWM_SettingControl(SCL_SENS_CALI_CTRL_A, &scl);
    }
    
    if(algo_info.angle_init_value.valid){
        //设置 初始角度信息
        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[2] = algo_info.angle_init_value.yaw;
        scl.iData[3] = algo_info.angle_init_value.pitch;
        scl.iData[4] = algo_info.angle_init_value.roll;
        CWM_SettingControl(SCL_HS_INTF_CONFIG, &scl);
    }
    else{
        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[2] = 3;
        scl.iData[3] = 11;
        scl.iData[4] = 7;
        CWM_SettingControl(SCL_HS_INTF_CONFIG, &scl);
    }
    
}

static void algo_write_to_hs_buffer(float* f)
{

    memcpy(&algo_info.hs_buffer.hs[algo_info.hs_buffer.write++],f,sizeof(hs_t));
    algo_info.hs_buffer.size++;
    if(algo_info.hs_buffer.write >= ALGO_HS_MAX){
        algo_info.hs_buffer.write = 0;
    }
    if(algo_info.hs_buffer.size >= ALGO_HS_MAX){
        algo_info.hs_buffer.size = ALGO_HS_MAX;
        algo_info.hs_buffer.read = algo_info.hs_buffer.write;
    }

}

static void algo_read_from_hs_buffer(void)
{
    
    for(uint8_t i = 0; i < algo_info.hs_buffer.size; i++){
        float* f = (float*)&algo_info.hs_buffer.hs[algo_info.hs_buffer.read++];

        //此处添加客户接口
        test_read_hs_data(f);

        if(algo_info.hs_buffer.read >= ALGO_HS_MAX){
            algo_info.hs_buffer.read = 0;
        }
    }
    algo_info.hs_buffer.size = 0;
    
}

static void cwm_ap_sensor_listen(pSensorEVT_t sensorEVT)
{
	float *f = sensorEVT->fData;
    switch (sensorEVT->sensorType)
	{
        case IDX_ACCEL:          
            if(algo_info.en_acc_gyro_aver){
                static count = 0;
                static float buf[3] = {0};
                count++;
                if(count > (algo_info.odr)){
                    CWM_OS_dbgPrintf("[cwm_listen] acc_uncalib ax ay az = %f %f %f",f[3],f[4],f[5]);
                    buf[0] += f[3];
                    buf[1] += f[4];
                    buf[2] += f[5];
                    if(count == (algo_info.odr + 10)){
                        algo_info.ag_aver_value.ax = buf[0]/10;
                        algo_info.ag_aver_value.ay = buf[1]/10;
                        algo_info.ag_aver_value.az = buf[2]/10;

                        count = 0;
                        memset(buf,0,sizeof(buf));
                        algo_info.event_acc_gyro_aver_finish = 1;
                    }
                }
            }
        break;

        case IDX_GYRO:     
            if(algo_info.en_acc_gyro_aver){
                static count = 0;
                static float buf[3] = {0};
                count++;
                if(count > (algo_info.odr)){
                    CWM_OS_dbgPrintf("[cwm_listen] gyro_uncalib gx gy gz = %f %f %f",f[3],f[4],f[5]);
                    buf[0] += f[3];
                    buf[1] += f[4];
                    buf[2] += f[5];
                    if(count == (algo_info.odr + 10)){
                        algo_info.ag_aver_value.gx = buf[0]/10;
                        algo_info.ag_aver_value.gy = buf[1]/10;
                        algo_info.ag_aver_value.gz = buf[2]/10;

                        count = 0;
                        memset(buf,0,sizeof(buf));
                        algo_info.event_acc_gyro_aver_finish = 1;
                    }
                }
            }
        break;

        case 100:
            CWM_OS_dbgPrintf("[cwm_listen] ypr = %f %f %f xyzw = %f %f %f %f",f[0],f[1],f[2],f[3],f[4],f[5],f[6]);
            algo_write_to_hs_buffer(sensorEVT->fData);
        break;

        case IDX_ALGO_FACTORY_CALI:
            CWM_OS_dbgPrintf("[cwm_listen] spv calib result : %f, %f (1, 1 is success) acc_bias=%.6f, %.6f, %.6f gyr_bias=%.6f, %.6f, %.6f",
                     (double)f[0], (double)f[1], (double)f[4], (double)f[5], (double)f[6], (double)f[7], (double)f[8], (double)f[9]); 
            if((1 == sensorEVT->fData[0]) && (1 == sensorEVT->fData[1])){
                if(SPV_CALIB_RUNNING == algo_info.spv_calib_value.whole_machine_status){

                    algo_info.spv_calib_value.gx = (int32_t)(sensorEVT->fData[7] * 1000000);
                    algo_info.spv_calib_value.gy = (int32_t)(sensorEVT->fData[8] * 1000000);
                    algo_info.spv_calib_value.gz = (int32_t)(sensorEVT->fData[9] * 1000000);
                    algo_info.spv_calib_value.whole_machine_status = SPV_CALIB_SUCCESS;
                    CWM_OS_dbgPrintf("[cwm_listen] update spv calib whole machine value success");
                }
                else if(SPV_CALIB_RUNNING == algo_info.spv_calib_value.pcba_status){

                    algo_info.spv_calib_value.ax = (int32_t)(sensorEVT->fData[4] * 1000000);
                    algo_info.spv_calib_value.ay = (int32_t)(sensorEVT->fData[5] * 1000000);
                    algo_info.spv_calib_value.az = (int32_t)(sensorEVT->fData[6] * 1000000);
                    algo_info.spv_calib_value.gx = (int32_t)(sensorEVT->fData[7] * 1000000);
                    algo_info.spv_calib_value.gy = (int32_t)(sensorEVT->fData[8] * 1000000);
                    algo_info.spv_calib_value.gz = (int32_t)(sensorEVT->fData[9] * 1000000);
                    algo_info.spv_calib_value.pcba_status = SPV_CALIB_SUCCESS;
                    CWM_OS_dbgPrintf("[cwm_listen] update spv calib pcba value success");

                }
                else if(SPV_CALIB_RUNNING == algo_info.spv_calib_value.sixface_status){

                    algo_info.spv_calib_value.ax = (int32_t)(sensorEVT->fData[4] * 1000000);
                    algo_info.spv_calib_value.ay = (int32_t)(sensorEVT->fData[5] * 1000000);
                    algo_info.spv_calib_value.az = (int32_t)(sensorEVT->fData[6] * 1000000);
                    algo_info.spv_calib_value.sixface_status = SPV_CALIB_SUCCESS;
                    CWM_OS_dbgPrintf("[cwm_listen] update spv calib sixface value success");
                }

                algo_info.spv_calib_value.valid = 1;
                update_spv_calib_value_checksum();
                algo_info.event_spv_calib_finish = 1;
                cwm_outside_spv_calib_dis();
            } 
            else if((1 == sensorEVT->fData[0]) && (0 == sensorEVT->fData[1])){
                if(SPV_CALIB_RUNNING == algo_info.spv_calib_value.whole_machine_status){

                    CWM_OS_dbgPrintf("[cwm_listen] spv calib whole machine fail: reason=%.1f", (double)f[2]);
                    algo_info.spv_calib_value.whole_machine_status = SPV_CALIB_FAIL;
                }
                else if(SPV_CALIB_RUNNING == algo_info.spv_calib_value.pcba_status){

                    CWM_OS_dbgPrintf("[cwm_listen] spv calib pcba fail: reason=%.1f", (double)f[2]);
                    algo_info.spv_calib_value.pcba_status = SPV_CALIB_FAIL;
                }
                else if(SPV_CALIB_RUNNING == algo_info.spv_calib_value.sixface_status){

                    CWM_OS_dbgPrintf("[cwm_listen] spv calib sixface fail: reason=%.1f", (double)f[2]);
                    algo_info.spv_calib_value.sixface_status = SPV_CALIB_FAIL;
                }

                update_spv_calib_value_checksum();
                algo_info.event_spv_calib_finish = 1;
                cwm_outside_spv_calib_dis();

            }  
        break;

        case 112:
            CWM_OS_dbgPrintf("[cwm_listen] angle %.4f success_2/ready_1/fail_0: %.4f stage: %.4f init angle: yaw=%.4f, pitch=%.4f, roll=%.4f",
                    (double)f[1], (double)f[2], (double)f[6], (double)f[3], (double)f[4], (double)f[5]);
            if((2 == sensorEVT->fData[2]) && (1 == sensorEVT->fData[6])){
                algo_info.angle_init_value.step1_status = ANGLE_INIT_SUCCESS;
                CWM_OS_dbgPrintf("[cwm_listen] angle init step 1 success ");

                update_angle_init_value_checksum();
                algo_info.event_angle_init_finish = 1;
            }           
            else if((1 == sensorEVT->fData[2]) && (1 == sensorEVT->fData[6])){
                algo_info.angle_init_value.step2_status = ANGLE_INIT_SUCCESS;
                CWM_OS_dbgPrintf("[cwm_listen] angle init step 2 success ");

                update_angle_init_value_checksum();
                algo_info.event_angle_init_finish = 1;
            }
            else if((2 == sensorEVT->fData[2]) && (2 == sensorEVT->fData[6])){
                algo_info.angle_init_value.step3_status = ANGLE_INIT_SUCCESS;
                CWM_OS_dbgPrintf("[cwm_listen] angle init step 3 success ");
                algo_info.angle_init_value.yaw = sensorEVT->fData[3];
                algo_info.angle_init_value.pitch = sensorEVT->fData[4];
                algo_info.angle_init_value.roll = sensorEVT->fData[5];
                CWM_OS_dbgPrintf("[cwm_listen] update angle_init_value success ");


                algo_info.angle_init_value.valid = 1;
                update_angle_init_value_checksum();
                algo_info.event_angle_init_finish = 1;
                cwm_outside_angle_init_dis();

            }
            else if((0 == sensorEVT->fData[2]) && (0 == sensorEVT->fData[6])){
                CWM_OS_dbgPrintf("[cwm_listen] angle init step 1 fail ");
                algo_info.angle_init_value.step1_status = ANGLE_INIT_FAIL;

                update_angle_init_value_checksum();
                algo_info.event_angle_init_finish = 1;
                cwm_outside_angle_init_dis();

            }
            else if((0 == sensorEVT->fData[2]) && (1 == sensorEVT->fData[6])) {
                if(ANGLE_INIT_SUCCESS == algo_info.angle_init_value.step1_status){
                    CWM_OS_dbgPrintf("[cwm_listen] angle init step 2 fail ");
                    algo_info.angle_init_value.step2_status = ANGLE_INIT_FAIL;
                }

                else if(ANGLE_INIT_SUCCESS == algo_info.angle_init_value.step2_status){
                    CWM_OS_dbgPrintf("[cwm_listen] angle init step 3 fail ");
                    algo_info.angle_init_value.step3_status = ANGLE_INIT_FAIL;
                }

                update_angle_init_value_checksum();
                algo_info.event_angle_init_finish = 1;
                cwm_outside_angle_init_dis();
            }
        break;

        default:
        break;
  	}
}

static void cwm_algo_init_m3(void)
{
    int i = 0;
    SettingControl_t scl;
    
    /* -----------------algo_dml_init------------------------ */
    os_api api = {
        .GetTimeNs    = CWM_OS_GetTimeNs_m3,
        .dbgOutput    = CWM_OS_dbgOutput_m3,
        .i2cRead      = CWM_OS_i2cRead_m3,
        .i2cWrite     = CWM_OS_i2cWrite_m3,
        .uSleep       = CWM_OS_uSleep_m3,
    };

    CWM_LibPreInit(&api);

    /* 设置MCU芯片信息, 必须在 CWM_LibPreInit() 之后， CWM_LibPostInit() 之前设置 */
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 2;; // 0: mcu_auto_detect 2: skip_mcu_auto_detect
    CWM_SettingControl(SCL_CHIP_VENDOR_CONFIG, &scl);

    CWM_LibPostInit(cwm_ap_sensor_listen);
    CWM_Dml_LibInit();

    char chipInfo[64];
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 1;
    scl.iData[2] = (int)chipInfo;
    scl.iData[3] = sizeof(chipInfo);
    scl.iData[4] = 0;
    scl.iData[5] = 0;
    scl.iData[6] = 0;
    CWM_SettingControl(SCL_GET_CHIP_INFO, &scl);
    
    CWM_OS_dbgPrintf("[cwm_init] chapor have_security = %d.%d ret_buff_size = %d  chipInfo = %s", scl.iData[5], scl.iData[6], scl.iData[4], chipInfo);
    CWM_OS_dbgPrintf("[cwm_init] chapor chip_settings = %d, %d, %d", scl.iData[9], scl.iData[10], scl.iData[11]);
    
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    CWM_SettingControl(SCL_GET_LIB_INFO, &scl);
    CWM_OS_dbgPrintf("[cwm_init] version:%d.%d.%d.%d product:%d model:%d", scl.iData[1], scl.iData[2], scl.iData[3], scl.iData[4], scl.iData[5], scl.iData[6]);

    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    CWM_SettingControl(SCL_DML_GET_LIB_INFO, &scl);
    CWM_OS_dbgPrintf("[cwm_init] chapor dml version:%d.%d.%d.%d product:%d model:%d", scl.iData[1], scl.iData[2], scl.iData[3], scl.iData[4], scl.iData[5], scl.iData[6]);
   
    /* -----------algo_setting----------------- */
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 50;
    scl.iData[2] = 2;
    CWM_SettingControl(SCL_ALGO_PROC_CONFIG, &scl);

	memset(&scl, 0, sizeof(scl));
	scl.iData[0] = 1;
	scl.iData[1] = 1;        // enable DT calib
	CWM_SettingControl(SCL_INPUT_DT_CONFIG, &scl);
    
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[3] = 1 + 4 + 8;
    scl.iData[4] = 64 + 5;
    scl.iData[5] = 3;
    // scl.iData[5] = 3+16;
    scl.iData[6] = -1 - 1 - 2 - 4 - 8 - 16 - 32; //- 8192
    scl.iData[7] = -1; 
    CWM_SettingControl(SCL_LOG, &scl);

    /* ---------dml_config---------------------- */
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 1 + 2 + 4 + 8 + 16;
    //CWM_SettingControl(SCL_DML_DEBUG, &scl);

    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 2;
    scl.iData[6] = 3002301;
    scl.iData[7] = 1+8;      // i2c add setting：can find device，try 1+8 instead 1+4.
    CWM_SettingControl(SCL_DML_DRV_HW_CONFIG, &scl);

    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 1;     //[in/out] lib_process_method (default: auto)   (0: default, -1: 取得所有指令參數值, 1: auto, 2: CWM_process, 3: CWM_process2)
    CWM_SettingControl(SCL_DML_DRV_INIT, &scl);

    memset(&scl,0,sizeof(scl));
    scl.iData[0] = 1;
    CWM_SettingControl(SCL_DML_GET_INITED_LIST, &scl);
    CWM_OS_dbgPrintf("[cwm_init] DML find device = %d", scl.iData[2]);  //找到硬體的數量    (-1: 未初始化, 0: 失敗, 1 - 8)
    for (i = 0; i < scl.iData[2]; i ++) {
        CWM_OS_dbgPrintf("[cwm_init] DML device = [%d]: hw_id=%d hw_attr=%d", i, scl.iData[6 + i * 2], scl.iData[7 + i * 2]);
    }

    memset(&scl, 0, sizeof(scl));
	scl.iData[0] = 1;
    scl.iData[1] = 1;
    scl.iData[2] = 1;
	scl.iData[3] = 2;     // 1:poll 2:fifo
	scl.iData[4] = 1;     // 1:any 2:dt 3:custom
	// scl.iData[5] = 20000; // custom_data_period (default: 0)  
	scl.iData[6] = 25;    //fifo count
	scl.iData[7] = 22;     //axis
	scl.iData[8] = 50;    //odr (default: 50)
	scl.iData[9] = 16;    //acc range (default: 16)
	scl.iData[10] = 1000; // gyro range (default: 2000)
	scl.iData[11] = 0;    // interrupt_mode_pin1 : (bit control, 0: nothing, 1: data_ready, 2: any_motion, 4: fifo_watermark)
	scl.iData[12] = 0;    // interrupt_mode_pin2 : (bit control, 0: nothing, 1: data_ready, 2: any_motion, 4: fifo_watermark)
	CWM_SettingControl(SCL_DML_DRV_AG_CONFIG, &scl);
    algo_info.odr = scl.iData[8];
    algo_period_ms = 1000/algo_info.odr;
    CWM_OS_dbgPrintf("[cwm_init] ODR = %d , algo_interval_ms = %d",algo_info.odr,algo_period_ms);

    //初始化时默认关闭驱动
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 1;   //[in/out] enable_method (default: manual_enable)   (0: default, 1: manual_enable, 2: auto_enable, -1: 取得參數值)
    scl.iData[2] = 0;   // [in/out] manual_enable_sensorData (default: all_off) ( bit control 0: manual_all_off, 1: acc, 2: gyro, 4: mag, 8: acc_temp)
    CWM_SettingControl(SCL_DML_DRV_ENABLE, &scl);
    
    /* --------- headset function config---------------------- */

    // Set orientation config
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 6000;
    scl.iData[3] = 15;
    scl.iData[4] = 8;
    scl.iData[5] = 2;
    scl.iData[6] = 1;
    scl.iData[11] = 1;
    scl.iData[13] = 2;
    CWM_SettingControl(SCL_HS_ORIEN_CONFIG, &scl);
    
    
}



////////////////////////////////////////////////////////////////外部调用接口////////////////////////////////////////////////////////

void cwm_algo_init(void)
{
    
    CWM_OS_dbgPrintf("[algo_init] config version V%s\n",ALGO_CONFIG_VERSION);
    
    //IIC 初始化
    bsp_gsensor_platform_i2c_init_m3();
    
    //从 FLASH 中读出存储的数据
    memset((uint8_t*)&algo_info,0,sizeof(algo_info_t));
    algo_read_param_from_flash();
    
    //算法初始化
    cwm_algo_init_m3();
    
}

void cwm_algo_data_handle(void)
{

	//欧拉角和四元数的处理
	algo_read_from_hs_buffer();

    //SPV 校正数据的处理
    if(algo_info.event_spv_calib_finish){

        algo_info.event_spv_calib_finish = 0;
        algo_write_param_to_flash(SPV_CALIB_PARAM);

        //此处添加客户使用的外部接口
        test_read_spv_calib_value();

    }

    //初始角度数据的处理
    if(algo_info.event_angle_init_finish){

        algo_info.event_angle_init_finish = 0;
        algo_write_param_to_flash(ANGLE_INIT_PARAM);

        //此处添加客户使用的外部接口
        test_read_angle_init_value();

        
    }

    //ACC&GYRO 平均数据的处理
    if(algo_info.event_acc_gyro_aver_finish){

        algo_info.event_acc_gyro_aver_finish = 0;
        algo_info.en_acc_gyro_aver = 0;
        CWM_Sensor_Disable(IDX_ACCEL);
        CWM_Sensor_Disable(IDX_GYRO);

        //此处添加客户使用的外部接口
        test_read_aver_ag_rawdata();
        
    }
}




void algo_log_debug_ctr(uint32_t ctr)
{
    CWM_OS_dbgPrintf("[cwm_debug] algo_log_debug_ctr ctr = %d",ctr);
    SettingControl_t scl;
    if (ctr)
    {
        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[3] = 1 + 4 + 8;
        scl.iData[4] = 64 + 5;
        scl.iData[5] = 3;
        // scl.iData[5] = 3+16;
        scl.iData[6] = -1 - 1 - 2 - 4 - 8 - 16 - 32; //- 8192
        scl.iData[7] = -1; 
        CWM_SettingControl(SCL_LOG, &scl);

        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[1] = 1 + 2 + 4 + 8 + 16;
        CWM_SettingControl(SCL_DML_DEBUG, &scl);
    }
    else
    {
        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        CWM_SettingControl(SCL_LOG, &scl);

        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        CWM_SettingControl(SCL_DML_DEBUG, &scl);
    }
    
}

void algo_headset_ori_ctr(uint32_t ctr)
{
    CWM_OS_dbgPrintf("[cwm_debug] algo_headset_ori_ctr ctr = %d",ctr);
    if(ctr){

        //打开SENSOR驱动
        algo_driver_ctr(1);

        //设置SPV校正数据和初始角度数据
        algo_set_spv_calib_angle_init();

        CWM_Sensor_Enable(100);
    }
    else{

        CWM_Sensor_Disable(100);
    }

}

void algo_driver_ctr(uint32_t ctr)
{
    CWM_OS_dbgPrintf("[cwm_debug] algo_driver_ctr ctr = %d",ctr);
    SettingControl_t scl;
    if (ctr) {
       
        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[1] = 1; // enable DT calib
        CWM_SettingControl(SCL_INPUT_DT_CONFIG, &scl);

        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[1] = 1;
        scl.iData[2] = 3;
        CWM_SettingControl(SCL_DML_DRV_ENABLE, &scl);

        CWM_Sensor_Enable(IDX_ACCEL);
        CWM_Sensor_Enable(IDX_GYRO);

    } else {

        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[1] = 1;
        scl.iData[2] = 0;
        CWM_SettingControl(SCL_DML_DRV_ENABLE, &scl);

        //关闭所有用到的算法
        CWM_Sensor_Disable(100);
        CWM_Sensor_Disable(IDX_ACCEL);
        CWM_Sensor_Disable(IDX_GYRO);

    }

}

void algo_spv_calib_en(uint32_t mode)
{

    CWM_OS_dbgPrintf("[cwm_debug] algo_spv_machine_enable calib_mode:%d",mode);
    SettingControl_t scl;
    switch(mode){
        case SPV_MODE_WHOLE_MACHINE:
            algo_info.spv_calib_value.whole_machine_status = SPV_CALIB_RUNNING;
        break;

        case SPV_MODE_PCBA:
            algo_info.spv_calib_value.pcba_status = SPV_CALIB_RUNNING;
        break;

        case SPV_MODE_SIX_FACE:
            algo_info.spv_calib_value.sixface_status = SPV_CALIB_RUNNING;
        break;

        default :
            CWM_OS_dbgPrintf("[cwm_debug] spv calib_mode error");
        return;
    }

    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 5; // z pluse point to sky
    scl.iData[2] = 5; // time-out period
    scl.iData[3] = 50;       // odr
    scl.iData[4] = 28400000; // gyro_Lsb_Dps
    scl.iData[5] = 3000;     // acc_noise
    scl.iData[6] = 120000;  // gyro_noise
    scl.iData[7] = 50000;   // acc_bias
    scl.iData[8] = 3300000; // gyro_bias
    CWM_SettingControl(SCL_FACTORY_CALI_CONFIG, &scl);

    // [1] = 1 整机校正， [1] = 2 PCBA校正，[1] = 5 六面校正
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = mode;
    CWM_SettingControl(SCL_FACTORY_CALI_MODE, &scl);

	CWM_Sensor_Disable(IDX_ALGO_FACTORY_CALI);
    CWM_Sensor_Enable(IDX_ALGO_FACTORY_CALI);
}

void algo_angle_init_en(uint32_t times)
{

    CWM_OS_dbgPrintf("[cwm_debug] algo_angle_init times = %d",times);
    SettingControl_t scl;
    switch(times){    
        case ANGLE_INIT_FIR:
            algo_info.angle_init_value.step1_status = ANGLE_INIT_FAIL;
            algo_info.angle_init_value.step2_status = ANGLE_INIT_FAIL;
            algo_info.angle_init_value.step3_status = ANGLE_INIT_FAIL;
            CWM_Sensor_Enable(112);

            memset(&scl, 0, sizeof(scl));
            scl.iData[0] = 1;
            scl.iData[1] = 2;
            scl.iData[2] = 1;
            CWM_SettingControl(SCL_HS_RUN_INIT_ANGLE, &scl);

            algo_info.angle_init_value.step1_status = SPV_CALIB_RUNNING;
        break;

        case ANGLE_INIT_SEC:
            memset(&scl, 0, sizeof(scl));
            scl.iData[0] = 1;
            scl.iData[1] = 2;
            scl.iData[2] = 2;
            CWM_SettingControl(SCL_HS_RUN_INIT_ANGLE, &scl);

            algo_info.angle_init_value.step2_status = SPV_CALIB_RUNNING;
        break;

        case ANGLE_INIT_THI:
            memset(&scl, 0, sizeof(scl));
            scl.iData[0] = 1;
            scl.iData[1] = 2;
            scl.iData[2] = 3;
            CWM_SettingControl(SCL_HS_RUN_INIT_ANGLE, &scl);

            algo_info.angle_init_value.step3_status = SPV_CALIB_RUNNING;
        break;

        default:
        break;
    }
    
}

void algo_get_aver_ag_rawdata(void)
{
    CWM_OS_dbgPrintf("[cwm_debug] algo_get_aver_ag_rawdata");

    algo_info.en_acc_gyro_aver = 1;
    memset(&algo_info.ag_aver_value,0,sizeof(ag_aver_t));
    CWM_Sensor_Enable(IDX_ACCEL);
    CWM_Sensor_Enable(IDX_GYRO);
}

void algo_save_auto_calib_value(void)
{

    CWM_OS_dbgPrintf("[cwm_debug] algo_save_auto_calib_value");

    SettingControl_t scl;

    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 1;
    CWM_SettingControl(SCL_SENS_CALI_CTRL_A, &scl);

    algo_info.spv_calib_value.auto_valid = 1;
    algo_info.spv_calib_value.auto_gx = scl.iData[5];
    algo_info.spv_calib_value.auto_gy = scl.iData[6];
    algo_info.spv_calib_value.auto_gz = scl.iData[7];

    CWM_OS_dbgPrintf("[cwm_debug] auto_calib_value : auto_gx = %d auto_gy = %d auto_gz = %d",algo_info.spv_calib_value.auto_gx,
                      algo_info.spv_calib_value.auto_gy,algo_info.spv_calib_value.auto_gz);
    
    update_auto_calib_value_checksum();

    algo_write_param_to_flash(SPV_CALIB_PARAM);
    
}





















