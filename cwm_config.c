#include "stdint.h"
#include "string.h"
#include "stdbool.h"
#include "math.h"
#include "cwm_lib.h"
#include "cwm_lib_dml.h"
#include "cwm_config.h"
#include "cwm_diskio.h"
#include "cwm_port.h"


#define SENSOR_ACC 1
#define SENSOR_GYR 2
#define SENSOR_DEFAULT  0
#define SENSOR_LP1      1
#define SENSOR_HP       2

struct algo_info_t algo_dev_info;

#define OPEN(a,param) {if(NULL != a->open) a->open(param);}
#define CLOSE(a,param) {if(NULL != a->close) a->close(param);}
struct algo_t{
    uint16_t id;
    uint8_t levle;
    uint8_t is_recoverable;
    void (*open)(void*);
    void (*handle)(void*);
    void (*close)(void*);
};
struct func_t{
    uint16_t id;
    void (*handle)(void*);
};
static const struct algo_t* algo_current_lev[E_ALGO_STATE_MAX] = {NULL};
static const struct algo_t* algo_current = NULL;

#define A_BUF_MAX 50
static float quiet_a[A_BUF_MAX];
static uint8_t quiet_a_num;
static uint32_t time_cnts;
static uint8_t fg_quiet;

////////////////////////////////////////////////////////////////本地接口////////////////////////////////////////////////////////
void algo_set_odr(uint16_t odr);

static uint16_t check_sum(uint8_t* data, uint32_t len)
{
    if(NULL == data) return 0;

    uint16_t sum = 0;
    for(uint32_t i = 0; i < len; i++){
        sum += data[i];
    }
    return sum;
}

static void update_cali_value_checksum(void)
{
    /*产测的校正数据: 2 + 26  计算校验时，注意 4 字节对齐问题 */
    uint32_t len = 26;
    uint8_t* addr =  (uint8_t*)&algo_dev_info.ag_cali_value.crc_16 + 2;
    algo_dev_info.ag_cali_value.crc_16 = check_sum(addr,len);
}

static void update_auto_cali_value_checksum(void)
{
    /*自动校正数据: 2 + 14  计算校验时，注意 4 字节对齐问题 */
    uint32_t len = 14;
    uint8_t* addr =  (uint8_t*)&algo_dev_info.ag_cali_value.auto_crc_16 + 2;
    algo_dev_info.ag_cali_value.auto_crc_16 = check_sum(addr,len);
}

static void update_original_eul_checksum(void)
{
     /*初始角度数据: 2 + 14  计算校验时，注意 4 字节对齐问题 */
    uint32_t len = sizeof(struct original_eul_t) - 2;
    uint8_t* addr =  (uint8_t*)&algo_dev_info.original_eul.crc_16 + 2;
    algo_dev_info.original_eul.crc_16 = check_sum(addr,len);
}

static void algo_read_param_from_flash(void)
{
    //此处添加客户接口
    struct ag_cali_back_t cali_value = {0};
    diskio_read_flash_cali((uint8_t*)&cali_value,sizeof(cali_value));

    uint8_t* cali_addr = (uint8_t*)&cali_value;
    /*注意：当前校正数据中包含两个部分，一个是产测的校正数据，另一个是自动校正数据，
    由于客户对 flash 接口的限制导致无法区分为 2 个数据存储块，所以使用一个结构体存储数据*/

    /*产测的校正数据: 2 + 26*/
    struct ag_cali_back_t* ag = (struct ag_cali_back_t*)cali_addr;        
    uint32_t len = 26;/*计算校验时，注意 4 字节对齐问题*/ 
    uint8_t* addr =  cali_addr + 2;
    uint16_t crc_16 = check_sum(addr,len);
    if(crc_16 != ag->crc_16){
        CWM_OS_dbgPrintf("[algo]read cali fail %d,%d\n",crc_16,ag->crc_16);            

        algo_dev_info.ag_cali_value.valid = 0;
        algo_dev_info.ag_cali_value.spv_whole_status = E_CALI_FAIL;
        algo_dev_info.ag_cali_value.spv_pcba_status = E_CALI_FAIL;
        algo_dev_info.ag_cali_value.sixface_status = E_CALI_FAIL;        
    }
    else{            
        memcpy(&algo_dev_info.ag_cali_value.crc_16,cali_addr,2+26);
        CWM_OS_dbgPrintf("[algo]read cali success\n");
    }


    /*自动校正数据: 2 + 14*/
    addr =  cali_addr + 2 + 26 + 2;
    len = 14;/*计算校验时，注意 4 字节对齐问题*/ 
    crc_16 = check_sum(addr,len);
    if(crc_16 != ag->auto_crc_16){
        CWM_OS_dbgPrintf("[algo]read auto cali fail %d,%d\n",crc_16,ag->auto_crc_16);            

        algo_dev_info.ag_cali_value.auto_valid = 0;        
    }
    else{            
        memcpy(&algo_dev_info.ag_cali_value.auto_crc_16,&cali_addr[2 + 26],2+14);
        CWM_OS_dbgPrintf("[algo]read auto cali success\n");
    }    


    //此处添加客户接口
    struct original_eul_t ori_eul_value ={0};
    diskio_read_flash_eul((uint8_t*)&ori_eul_value,sizeof(ori_eul_value));
    
    uint8_t* ori_eul_addr = (uint8_t*)&ori_eul_value; 

    /*初始化角度: 2 + 14*/
    len = sizeof(struct original_eul_t) - 2;
    addr =  ori_eul_addr + 2;
    crc_16 = check_sum(addr,len);
    if(crc_16 != ori_eul_value.crc_16){
        CWM_OS_dbgPrintf("[algo]read orig eul fail %d,%d\n",crc_16,ori_eul_value.crc_16);

        algo_dev_info.original_eul.valid = 0;
        algo_dev_info.original_eul.step1_status = E_ORI_EUL_CALI_FAIL;
        algo_dev_info.original_eul.step2_status = E_ORI_EUL_CALI_FAIL;
        algo_dev_info.original_eul.step3_status = E_ORI_EUL_CALI_FAIL;
    }
    else{
        memcpy(&algo_dev_info.original_eul,ori_eul_addr,sizeof(struct original_eul_t));
        CWM_OS_dbgPrintf("[algo]read orig eul success\n");
    }
}



static void algo_quiet_enable(bool reset)
{
    CWM_OS_dbgPrintf("[algo]algo_quiet_enable %d\n",reset);
    if(reset){
        fg_quiet = 1;
    }
    memset(quiet_a,0,sizeof(quiet_a));
    quiet_a_num = 0;
}
static void algo_quiet_disable(void)
{
    CWM_OS_dbgPrintf("[algo]algo_quiet_disable\n");
    fg_quiet = 0;
}
static void algo_quiet_check_time(void)
{
    time_cnts++;
    if(time_cnts > algo_get_odr()*ALGO_QUIET_TIME_SEC){
        time_cnts = 0;
        algo_quiet_enable(true);
    }
}
static bool algo_quiet_process(uint8_t type, float *f)
{
    if((fg_quiet) && (NULL != algo_current)){
        // CWM_OS_dbgPrintf("[algo]algo_quiet_process id=%u,ty=%u\n",algo_current->id,type);
        if((E_STATE_STANDBY == algo_current->id) && (type & SENSOR_ACC)){
            float ax = f[0];
            float ay = f[1];
            float az = f[2];


            // CWM_OS_dbgPrintf("[algo]algo_quiet_process num=%d\n",quiet_a_num);

            quiet_a[quiet_a_num++] = sqrt(ax*ax + ay*ay + az*az);

            // static float pre,current;
            // current = sqrt(ax*ax + ay*ay + az*az);
            // CWM_OS_dbgPrintf("[algo]algo_quiet_process a=%4f,%4f\n",pre,current);
            // pre = current;

            if(quiet_a_num >= A_BUF_MAX) quiet_a_num = 0;

            for (int i = 1; i < A_BUF_MAX; i++) {
                if(fabs(quiet_a[i] - quiet_a[i - 1]) > ALGO_STANDBY_QUIET_THRESHOLD) {
                    // CWM_OS_dbgPrintf("[algo]algo_quiet_process a:%d,%4f,%4f,%4f,%4f\n",i,fabs(quiet_a[i] - quiet_a[i - 1]),quiet_a[i - 1],quiet_a[i],ALGO_STANDBY_QUIET_THRESHOLD);
                    return false;
                }
            }
            CWM_OS_dbgPrintf("[algo]algo_quiet_process ===========quiet===========\n");
            return true;
        }
    }
    return false;
}

/*当 sensor 模式切换时（a->ag or ag->a or...），sensor 的 acc、gyro 缓存需要清除后重新开始缓存*/
void algo_ag_buf_set(uint8_t data_type)
{
    struct algo_ag_t* buf = &algo_dev_info.algo_ag;
    memset(buf,0,sizeof(struct algo_ag_t));
    buf->data_type = data_type;
    CWM_OS_dbgPrintf("[algo]algo_ag_buf_set: %d\n",data_type);
}
/*将 sensor 数据写入缓存区，注意：必须先 acc 后 gyro，否则 size 容易出现异常*/
static void algo_ag_write(uint32_t id, float* f)
{
    struct algo_ag_t* buf = &algo_dev_info.algo_ag;
    // CWM_OS_dbgPrintf("[algo]algo_ag_write: %d,%d\n",buf->data_type,id);
    if((buf->data_type & SENSOR_GYR) && (SENSOR_GYR == id)){
        memcpy(&buf->data[buf->write++].gx,f,4*3);
        buf->size++;
        if(buf->write >= ALGO_RES_MAX_COUNT){
            buf->write = 0;
        }
        if(buf->size >= ALGO_RES_MAX_COUNT){
            buf->size = ALGO_RES_MAX_COUNT;
            buf->read = buf->write;
        }
    }else if((SENSOR_ACC == buf->data_type) && (SENSOR_ACC == id)){
        memcpy(&buf->data[buf->write++].ax,f,4*3);
        buf->size++;
        if(buf->write >= ALGO_RES_MAX_COUNT){
            buf->write = 0;
        }
        if(buf->size >= ALGO_RES_MAX_COUNT){
            buf->size = ALGO_RES_MAX_COUNT;
            buf->read = buf->write;
        }
    }else if((buf->data_type & SENSOR_ACC) && (SENSOR_ACC == id)){
        memcpy(&buf->data[buf->write].ax,f,4*3);
    }
}
static void algo_ag_read(void)
{
    struct algo_ag_t* buf = &algo_dev_info.algo_ag;
    for(uint8_t i = 0; i < buf->size; i++){
        float* f = (float*)&buf->data[buf->read++];

        if(algo_quiet_process(buf->data_type,f)){
            uint32_t mode = E_CALI_SPV_WHOLE;
            algo_state_handle(E_STATE_STANDBY_SPV,E_ALGO_EVENT_OPEN,&mode);
        }

        /*此处添加客户接口：将 acc、gyro 传给客户*/
        diskio_read_ag(buf->data_type,f,i,buf->size);

        if(buf->read >= ALGO_RES_MAX_COUNT){
            buf->read = 0;
        }
    }
    buf->size = 0;
}


static void algo_res_write(float* f)
{
    struct algo_res_t* buf = &algo_dev_info.algo_res;
    memcpy(&buf->data[buf->write++],f,sizeof(struct eul_qua_t));
    buf->size++;
    if(buf->write >= ALGO_RES_MAX_COUNT){
        buf->write = 0;
    }
    if(buf->size >= ALGO_RES_MAX_COUNT){
        buf->size = ALGO_RES_MAX_COUNT;
        buf->read = buf->write;
    }
}

static void algo_res_read(void)
{
    struct algo_res_t* buf = &algo_dev_info.algo_res;
    for(uint8_t i = 0; i < buf->size; i++){
        float* f = (float*)&buf->data[buf->read++];

        /*此处添加客户接口：将算法结果传给客户*/
        diskio_read_eul_qua(f);

        if(buf->read >= ALGO_RES_MAX_COUNT){
            buf->read = 0;
        }
    }
    buf->size = 0;
}


static void OS_algo_listen(pSensorEVT_t sensorEVT) {
    float* f = sensorEVT->fData;
	switch (sensorEVT->sensorType) {
		case IDX_ACCEL:
            algo_ag_write(SENSOR_ACC,f);
            if(algo_dev_info.en_ag_avg_value_1s){
                uint16_t* run_cnts = &algo_dev_info.ag_avg_value_1s.a_cnts;
                float* ax_sum = &algo_dev_info.ag_avg_value_1s.ag_sum.ax;
                float* ay_sum = &algo_dev_info.ag_avg_value_1s.ag_sum.ay;
                float* az_sum = &algo_dev_info.ag_avg_value_1s.ag_sum.az;

                (*run_cnts)++;
                if(*run_cnts > algo_get_odr()){
                    *ax_sum += f[3];
                    *ay_sum += f[4];
                    *az_sum += f[5];
                    if(*run_cnts == (algo_get_odr() + 10)){
                        algo_dev_info.ag_avg_value_1s.ag.ax = *ax_sum/10;
                        algo_dev_info.ag_avg_value_1s.ag.ay = *ay_sum/10;
                        algo_dev_info.ag_avg_value_1s.ag.az = *az_sum/10;

                        algo_dev_info.event_ag_avg_value_1s_finish = 1;
                    }
                }
            }
        break;
        case IDX_GYRO:
            algo_ag_write(SENSOR_GYR,f);
            if(algo_dev_info.en_ag_avg_value_1s){
                uint16_t* run_cnts = &algo_dev_info.ag_avg_value_1s.g_cnts;
                float* gx_sum = &algo_dev_info.ag_avg_value_1s.ag_sum.gx;
                float* gy_sum = &algo_dev_info.ag_avg_value_1s.ag_sum.gy;
                float* gz_sum = &algo_dev_info.ag_avg_value_1s.ag_sum.gz;

                (*run_cnts)++;
                if(*run_cnts > algo_get_odr()){
                    *gx_sum += f[3];
                    *gy_sum += f[4];
                    *gz_sum += f[5];
                    if(*run_cnts == (algo_get_odr() + 10)){
                        algo_dev_info.ag_avg_value_1s.ag.gx = *gx_sum/10;
                        algo_dev_info.ag_avg_value_1s.ag.gy = *gy_sum/10;
                        algo_dev_info.ag_avg_value_1s.ag.gz = *gz_sum/10;

                        algo_dev_info.event_ag_avg_value_1s_finish = 1;
                    }
                }
            }
        break;
        case IDX_ALGO_SPV:
            if((1 == f[0]) && (1 == f[1])){
                if(E_CALI_RUNNING == algo_dev_info.ag_cali_value.spv_whole_status){
                    algo_dev_info.ag_cali_value.gx = (int32_t)(f[7] * 1000000);
                    algo_dev_info.ag_cali_value.gy = (int32_t)(f[8] * 1000000);
                    algo_dev_info.ag_cali_value.gz = (int32_t)(f[9] * 1000000);
                    algo_dev_info.ag_cali_value.spv_whole_status = E_CALI_SUCCESS;
                    algo_quiet_disable();
                    CWM_OS_dbgPrintf("[algo]IDX_ALGO_SPV whole E_CALI_SUCCESS:gx=%d,gy=%d,gz=%d\n",
                        algo_dev_info.ag_cali_value.gx,
                        algo_dev_info.ag_cali_value.gy,
                        algo_dev_info.ag_cali_value.gz);
                }
                else if(E_CALI_RUNNING == algo_dev_info.ag_cali_value.spv_pcba_status){
                    algo_dev_info.ag_cali_value.ax = (int32_t)(f[4] * 1000000);
                    algo_dev_info.ag_cali_value.ay = (int32_t)(f[5] * 1000000);
                    algo_dev_info.ag_cali_value.az = (int32_t)(f[6] * 1000000);
                    algo_dev_info.ag_cali_value.gx = (int32_t)(f[7] * 1000000);
                    algo_dev_info.ag_cali_value.gy = (int32_t)(f[8] * 1000000);
                    algo_dev_info.ag_cali_value.gz = (int32_t)(f[9] * 1000000);
                    algo_dev_info.ag_cali_value.spv_pcba_status = E_CALI_SUCCESS;
                    CWM_OS_dbgPrintf("[algo]IDX_ALGO_SPV pcba E_CALI_SUCCESS:ax=%d,ay=%d,az=%d,gx=%d,gy=%d,gz=%d\n",
                        algo_dev_info.ag_cali_value.ax,
                        algo_dev_info.ag_cali_value.ay,
                        algo_dev_info.ag_cali_value.az,
                        algo_dev_info.ag_cali_value.gx,
                        algo_dev_info.ag_cali_value.gy,
                        algo_dev_info.ag_cali_value.gz);
                }
                else if(E_CALI_RUNNING == algo_dev_info.ag_cali_value.sixface_status){
                    algo_dev_info.ag_cali_value.ax = (int32_t)(f[4] * 1000000);
                    algo_dev_info.ag_cali_value.ay = (int32_t)(f[5] * 1000000);
                    algo_dev_info.ag_cali_value.az = (int32_t)(f[6] * 1000000);
                    algo_dev_info.ag_cali_value.sixface_status = E_CALI_SUCCESS;
                    CWM_OS_dbgPrintf("[algo]IDX_ALGO_SPV sixface E_CALI_SUCCESS:ax=%d,ay=%d,az=%d\n",
                        algo_dev_info.ag_cali_value.ax,
                        algo_dev_info.ag_cali_value.ay,
                        algo_dev_info.ag_cali_value.az);
                }
                algo_dev_info.ag_cali_value.valid = 1;
                algo_dev_info.event_cali_finish = 1;
                cwm_spv_cali_dis();
            }
            else if((1 == f[0]) && (0 == f[1])){
                if(E_CALI_RUNNING == algo_dev_info.ag_cali_value.spv_whole_status){
                    algo_dev_info.ag_cali_value.spv_whole_status = E_CALI_FAIL;
                    CWM_OS_dbgPrintf("[algo]IDX_ALGO_SPV whole E_CALI_FAIL\n");
                }
                else if(E_CALI_RUNNING == algo_dev_info.ag_cali_value.spv_pcba_status){
                    algo_dev_info.ag_cali_value.spv_pcba_status = E_CALI_FAIL;
                    CWM_OS_dbgPrintf("[algo]IDX_ALGO_SPV pcba E_CALI_FAIL\n");
                }
                else if(E_CALI_RUNNING == algo_dev_info.ag_cali_value.sixface_status){
                    algo_dev_info.ag_cali_value.sixface_status = E_CALI_FAIL;
                    CWM_OS_dbgPrintf("[algo]IDX_ALGO_SPV sixface E_CALI_FAIL\n");
                }

                algo_dev_info.event_cali_finish = 1;
                cwm_spv_cali_dis();
            }
        break;
        case 100:
            algo_res_write(f);
            // CWM_OS_dbgPrintf("[algo][100]: %f,%f,%f\n",
            //     f[0],
            //     f[1],
            //     f[2]);
        break;
        case 112:
            CWM_OS_dbgPrintf("[algo][112] :%f,%f\n",f[2],f[6]);
            /*
            step1 success:[2],[6] = 2,1
            step2 success:[2],[6] = 1,1
            step3 success:[2],[6] = 2,2
            */
            switch(algo_dev_info.original_eul.running_steps){
                case E_ANGLE_INIT_STEP1:
                    if((2 == f[2]) && (1 == f[6]) && (E_ORI_EUL_CALI_RUNNING == algo_dev_info.original_eul.step1_status)){
                        algo_dev_info.original_eul.step1_status = E_ORI_EUL_CALI_SUCCESS;
                        /*第一步时，更新 pitch、roll 数据*/
                        algo_dev_info.original_eul.valid = 1;
                        algo_dev_info.original_eul.pitch = f[4];
                        algo_dev_info.original_eul.roll = f[5];
                    }else{
                        algo_dev_info.original_eul.step1_status = E_ORI_EUL_CALI_FAIL;
                        cwm_ori_eul_cali_dis();
                    }
                    algo_dev_info.event_orig_eul_finish = 1;
                break;
                case E_ANGLE_INIT_STEP2:
                    if((1 == f[2]) && (1 == f[6]) && (E_ORI_EUL_CALI_RUNNING == algo_dev_info.original_eul.step2_status)){
                        algo_dev_info.original_eul.step2_status = E_ORI_EUL_CALI_SUCCESS;
                    }else{
                        algo_dev_info.original_eul.step2_status = E_ORI_EUL_CALI_FAIL;
                        cwm_ori_eul_cali_dis();
                    }
                    algo_dev_info.event_orig_eul_finish = 1;
                break;
                case E_ANGLE_INIT_STEP3:
                    if((2 == f[2]) && (2 == f[6]) && (E_ORI_EUL_CALI_RUNNING == algo_dev_info.original_eul.step3_status)){
                        algo_dev_info.original_eul.step3_status = E_ORI_EUL_CALI_SUCCESS;
                        /*第一步时，更新 yaw， pitch、roll 与第一步相同*/
                        algo_dev_info.original_eul.valid = 1;
                        algo_dev_info.original_eul.yaw = f[3];
                        algo_dev_info.original_eul.pitch = f[4];
                        algo_dev_info.original_eul.roll = f[5];
                    }else{
                        algo_dev_info.original_eul.step3_status = E_ORI_EUL_CALI_FAIL;
                    }
                    cwm_ori_eul_cali_dis();
                    algo_dev_info.event_orig_eul_finish = 1;
                break;
            }
        break;
        case 21:
            CWM_OS_dbgPrintf("[algo][21]: gyro auro cali: %f\n",
                f[2]);
        break;
        default:
            break;
    }
}

static void set_sensor(uint8_t en, uint8_t sensor, uint16_t odr, uint16_t power_mode)
{
    if (en) {
        SettingControl_t scl;
        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[1] = 2;
        CWM_SettingControl(SCL_DML_DRV_AG_CONFIG, &scl);
        scl.iData[1] = 1;
        scl.iData[8] = odr;
	    CWM_SettingControl(SCL_DML_DRV_AG_CONFIG, &scl);

        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[1] = odr;
        scl.iData[2] = 2;
        CWM_SettingControl(SCL_ALGO_PROC_CONFIG, &scl);

        // memset(&scl, 0, sizeof(scl));
        // scl.iData[0] = 1;
        // scl.iData[1] = 2;
        // CWM_SettingControl(SCL_DML_DRV_AG_PERF_CONFIG, &scl);
        // scl.iData[1] = 1;
        // // scl.iData[8] = odr;
	    // CWM_SettingControl(SCL_DML_DRV_AG_PERF_CONFIG, &scl);

        // memset(&scl, 0, sizeof(scl));
        // scl.iData[0] = 1;
        // scl.iData[1] = 2;
        // CWM_SettingControl(SCL_DML_DRV_M_CONFIG, &scl);
        // scl.iData[1] = 1;
        // scl.iData[8] = odr;
	    // CWM_SettingControl(SCL_DML_DRV_M_CONFIG, &scl);

        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[1] = 1; // enable DT calib
        CWM_SettingControl(SCL_INPUT_DT_CONFIG, &scl);

        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[1] = 1;
        scl.iData[2] = sensor;
        CWM_SettingControl(SCL_DML_DRV_ENABLE, &scl);

        if(sensor & 1) CWM_Sensor_Enable(IDX_ACCEL);
        if(sensor & 2) CWM_Sensor_Enable(IDX_GYRO);

        algo_ag_buf_set(sensor);
        algo_set_odr(odr);
    }else{
        SettingControl_t scl;
        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[1] = 1;
        scl.iData[2] = 0;
        CWM_SettingControl(SCL_DML_DRV_ENABLE, &scl);

        //关闭所有用到的算法
        CWM_Sensor_Disable(IDX_ACCEL);
        CWM_Sensor_Disable(IDX_GYRO);
    }
}

static void hs_algo_init(void)
{
    SettingControl_t scl;
    if(algo_dev_info.ag_cali_value.valid){
        //设置 SPV 校正信息
        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[1] = 2;
        scl.iData[2] = algo_dev_info.ag_cali_value.ax;
        scl.iData[3] = algo_dev_info.ag_cali_value.ay;
        scl.iData[4] = algo_dev_info.ag_cali_value.az;
        if(algo_dev_info.ag_cali_value.auto_valid){
            scl.iData[5] = algo_dev_info.ag_cali_value.auto_gx;
            scl.iData[6] = algo_dev_info.ag_cali_value.auto_gy;
            scl.iData[7] = algo_dev_info.ag_cali_value.auto_gz;
        }
        else{
            scl.iData[5] = algo_dev_info.ag_cali_value.gx;
            scl.iData[6] = algo_dev_info.ag_cali_value.gy;
            scl.iData[7] = algo_dev_info.ag_cali_value.gz;
        }
        CWM_SettingControl(SCL_SENS_CALI_CTRL_A, &scl);
        CWM_OS_dbgPrintf("[algo]set cali ag value:auto=%d,ax=%d,ay=%d,az=%d,gx=%d,gy=%d,gz=%d\n",
            algo_dev_info.ag_cali_value.auto_valid,scl.iData[2],scl.iData[3], scl.iData[4],scl.iData[5],scl.iData[6], scl.iData[7]);
    }
    if(algo_dev_info.original_eul.valid){
        /*设置 初始角度信息*/
        memset(&scl, 0, sizeof(scl));
        scl.iData[0] = 1;
        scl.iData[2] = algo_dev_info.original_eul.yaw;
        scl.iData[3] = algo_dev_info.original_eul.pitch;
        scl.iData[4] = algo_dev_info.original_eul.roll;
        CWM_SettingControl(SCL_HS_INTF_CONFIG, &scl);
        CWM_OS_dbgPrintf("[algo]set ori eul:y=%d,p=%d,r=%d\n",scl.iData[2],scl.iData[3], scl.iData[4]);
    }else{
        memcpy(&scl,dml_hs_intf_config,sizeof(scl));
        CWM_SettingControl(SCL_HS_INTF_CONFIG, &scl);
        CWM_OS_dbgPrintf("[algo]set ori eul default:y=%d,p=%d,r=%d\n",scl.iData[2],scl.iData[3], scl.iData[4]);
    }
}

static void dml_algo_init(void)
{
    int i = 0;
    SettingControl_t scl;
    
    /* -----------------algo_dml_init------------------------ */
    CWM_LibPreInit(&diskio_os_api);

    /* 设置MCU芯片信息, 必须在 CWM_LibPreInit() 之后， CWM_LibPostInit() 之前设置 */
    memcpy(&scl,dml_vendor_config,sizeof(scl));
    CWM_SettingControl(SCL_CHIP_VENDOR_CONFIG, &scl);

    CWM_LibPostInit(OS_algo_listen);
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
    
    CWM_OS_dbgPrintf("[algo] have_security = %d.%d ret_buff_size = %d  chipInfo = %s\n", scl.iData[5], scl.iData[6], scl.iData[4], chipInfo);
    CWM_OS_dbgPrintf("[algo] chip_settings = %d, %d, %d\n", scl.iData[9], scl.iData[10], scl.iData[11]);
    
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    CWM_SettingControl(SCL_GET_LIB_INFO, &scl);
    CWM_OS_dbgPrintf("[algo] algo version:%d.%d.%d.%d product:%d model:%d\n", scl.iData[1], scl.iData[2], scl.iData[3], scl.iData[4], scl.iData[5], scl.iData[6]);

    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    CWM_SettingControl(SCL_DML_GET_LIB_INFO, &scl);
    CWM_OS_dbgPrintf("[algo] dml version:%d.%d.%d.%d product:%d model:%d\n", scl.iData[1], scl.iData[2], scl.iData[3], scl.iData[4], scl.iData[5], scl.iData[6]);
   
    /* -----------algo_setting----------------- */
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = CWM_DEFAUL_ODR;
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
    // memset(&scl, 0, sizeof(scl));
    // scl.iData[0] = 1;
    // scl.iData[1] = 1 + 2 + 4 + 8 + 16;
    //CWM_SettingControl(SCL_DML_DEBUG, &scl);

    memcpy(&scl,dml_hw_config,sizeof(scl));
    CWM_SettingControl(SCL_DML_DRV_HW_CONFIG, &scl);

    memcpy(&scl,dml_ag_config,sizeof(scl));
	CWM_SettingControl(SCL_DML_DRV_AG_CONFIG, &scl);

    // memcpy(&scl,dml_mag_config,sizeof(scl));
	// CWM_SettingControl(SCL_DML_DRV_M_CONFIG, &scl);

    // memcpy(&scl,dml_ag_perf_config,sizeof(scl));
    // CWM_SettingControl(SCL_DML_DRV_AG_PERF_CONFIG, &scl);

    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 1;     //[in/out] lib_process_method (default: auto)   (0: default, -1: 取得所有指令參數值, 1: auto, 2: CWM_process, 3: CWM_process2)
    CWM_SettingControl(SCL_DML_DRV_INIT, &scl);

    memset(&scl,0,sizeof(scl));
    scl.iData[0] = 1;
    CWM_SettingControl(SCL_DML_GET_INITED_LIST, &scl);
    CWM_OS_dbgPrintf("[algo] DML find device = %d\n", scl.iData[2]);  //找到硬體的數量    (-1: 未初始化, 0: 失敗, 1 - 8)
    for (i = 0; i < scl.iData[2]; i ++) {
        CWM_OS_dbgPrintf("[algo] DML device = [%d]: hw_id=%d hw_attr=%d\n", i, scl.iData[6 + i * 2], scl.iData[7 + i * 2]);
    }

    /*开机 sensor 默认关闭驱动*/
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 1;   //[in/out] enable_method (default: manual_enable)   (0: default, 1: manual_enable, 2: auto_enable, -1: 取得參數值)
    scl.iData[2] = 0;   // [in/out] manual_enable_sensorData (default: all_off) ( bit control 0: manual_all_off, 1: acc, 2: gyro, 4: mag, 8: acc_temp)
    CWM_SettingControl(SCL_DML_DRV_ENABLE, &scl);

    /* ---------headset function---------------------- */
    /*开机 algo 默认关闭*/
    //设置SPV校正数据和初始角度数据
    // hs_algo_init();

    // Set orientation config
    memcpy(&scl,dml_hs_orien_config,sizeof(scl));
    CWM_SettingControl(SCL_HS_ORIEN_CONFIG, &scl);
    
    /*--------- ACC & GYRO --------------------------*/
    /*开机 sensor 默认关闭*/
    // CWM_Sensor_Enable(IDX_ACCEL);
    // CWM_Sensor_Enable(IDX_GYRO);
    
}

static void spv_dis(void)
{
    CWM_Sensor_Disable(IDX_ALGO_SPV);
    set_sensor(0,0,0,0);
}

void algo_standby_open(void* param)
{
    CWM_OS_dbgPrintf("[algo]algo_standby_open\n");
    set_sensor(1,SENSOR_ACC,CWM_DEFAUL_ODR/2,SENSOR_DEFAULT);
}
void algo_standby_close(void* param)
{
    CWM_OS_dbgPrintf("[algo]algo_standby_close\n");
    set_sensor(0,0,0,0);
}
void algo_standby_spv_open(void* param)
{
    CWM_OS_dbgPrintf("[algo]algo_standby_spv_open\n");
    set_sensor(1,SENSOR_ACC+SENSOR_GYR,CWM_DEFAUL_ODR,SENSOR_DEFAULT);
    
    uint32_t mode = *((uint32_t*)param);
    algo_spv_cali_en(mode);
}
void algo_standby_spv_close(void* param)
{
    CWM_OS_dbgPrintf("[algo]algo_standby_spv_close\n");
    spv_dis();
    algo_quiet_enable(false);
}
void algo_hs_orit_open(void* param)
{
    CWM_OS_dbgPrintf("[algo]algo_hs_orit_open\n");
    set_sensor(1,SENSOR_ACC+SENSOR_GYR,CWM_DEFAUL_ODR,SENSOR_DEFAULT);
    hs_algo_init();

    //打开自动校正
    SettingControl_t scl;
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    CWM_SettingControl(SCL_SENS_CALI_CONFIG, &scl);

    CWM_Sensor_Enable(100);
}
void algo_hs_orit_close(void* param)
{
    CWM_OS_dbgPrintf("[algo]algo_hs_orit_close\n");
    CWM_Sensor_Disable(100);
    set_sensor(0,0,0,0);
}
void algo_spv_whl_cali_open(void* param)
{
    if(NULL == param)   return;
    CWM_OS_dbgPrintf("[algo]algo_spv_whl_cali_open=%d\n", *((uint32_t*)param));
    set_sensor(1,SENSOR_ACC+SENSOR_GYR,CWM_DEFAUL_ODR,SENSOR_DEFAULT);
    
    uint32_t mode = *((uint32_t*)param);
    algo_spv_cali_en(mode);
}
void algo_spv_whl_cali_close(void* param)
{
    CWM_OS_dbgPrintf("[algo]algo_spv_whl_cali_close\n");
    spv_dis();
}
void algo_spv_pcb_cali_open(void* param)
{
    if(NULL == param)   return;
    CWM_OS_dbgPrintf("[algo]algo_spv_pcb_cali_open=%d\n", *((uint32_t*)param));
    set_sensor(1,SENSOR_ACC+SENSOR_GYR,CWM_DEFAUL_ODR,SENSOR_DEFAULT);
    
    uint32_t mode = *((uint32_t*)param);
    algo_spv_cali_en(mode);
}
void algo_spv_pcb_cali_close(void* param)
{
    CWM_OS_dbgPrintf("[algo]algo_spv_pcb_cali_close\n");
    spv_dis();
}
void algo_spv_sfa_cali_open(void* param)
{
    if(NULL == param)   return;
    CWM_OS_dbgPrintf("[algo]algo_spv_sfa_cali_open=%d\n", *((uint32_t*)param));
    set_sensor(1,SENSOR_ACC+SENSOR_GYR,CWM_DEFAUL_ODR,SENSOR_DEFAULT);
    
    uint32_t mode = *((uint32_t*)param);
    algo_spv_cali_en(mode);
}
void algo_spv_sfa_cali_close(void* param)
{
    CWM_OS_dbgPrintf("[algo]algo_spv_sfa_cali_close\n");
    spv_dis();
}
void algo_orig_eul_cali_open(void* param)
{
    if(NULL == param)   return;
    CWM_OS_dbgPrintf("[algo]algo_orig_eul_cali_open\n", *((uint32_t*)param));
    set_sensor(1,SENSOR_ACC+SENSOR_GYR,CWM_DEFAUL_ODR,SENSOR_DEFAULT);
    hs_algo_init();

    //打开自动校正
    SettingControl_t scl;
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    CWM_SettingControl(SCL_SENS_CALI_CONFIG, &scl);

    CWM_Sensor_Enable(100);
    algo_original_eul_cali_en(*((uint32_t*)param));
}
void algo_orig_eul_cali_close(void* param)
{
    CWM_OS_dbgPrintf("[algo]algo_orig_eul_cali_close\n");
    CWM_Sensor_Disable(100);
    CWM_Sensor_Disable(112);
    set_sensor(0,0,0,0);
}
void algo_func_log_ctl(void* param)
{
    if(NULL ==  param)  return;
    uint32_t* ctr = (uint32_t*)param;

    CWM_OS_dbgPrintf("[algo]algo_func_log_ctl\n");
    algo_log_debug_ctl(*ctr);
}
void algo_func_ag_avg_value(void* param)
{
    CWM_OS_dbgPrintf("[algo]algo_func_ag_avg_value\n");
    algo_avg_ag_value_en();
}
void algo_func_save_before_poweroff(void* param)
{
    CWM_OS_dbgPrintf("[algo]algo_func_save_before_poweroff\n");
    algo_save_before_poweroff();
}


////////////////////////////////////////////////////////////////算法状态管理////////////////////////////////////////////////////////
/*算法状态分为 4 层：
    E_STATE_LEV3（最高）：
    E_STATE_LEV2：
    E_STATE_LEV1：
    E_STATE_LEV0（最低）：
高优先级可以打断同级、低优先级状态。打断时，先执行之前的状态的 close，后执行新状态的 open。
状态结束后，会返回上一级的状态。
状态无法打断自己，如当前 E_STATE_ORIG_EUL_CALI，如果有新状态 E_STATE_ORIG_EUL_CALI E_ALGO_EVENT_OPEN 则不会被打断，
    即只会执行新状态的 open。
*/
static const struct algo_t algo_state_table[] ={
    {E_STATE_STANDBY,        E_STATE_LEV0,1,algo_standby_open,      NULL,algo_standby_close},/*level1 表示待机状态，只有开启，没有关闭*/
    {E_STATE_STANDBY_SPV,    E_STATE_LEV1,0,algo_standby_spv_open,  NULL,algo_standby_spv_close},
    {E_STATE_HS_ORIT,        E_STATE_LEV2,1,algo_hs_orit_open,      NULL,algo_hs_orit_close},
    {E_STATE_SPV_WHOLE,      E_STATE_LEV3,1,algo_spv_whl_cali_open, NULL,algo_spv_whl_cali_close},
    {E_STATE_SPV_PCBA,       E_STATE_LEV3,1,algo_spv_pcb_cali_open, NULL,algo_spv_pcb_cali_close},
    {E_STATE_SPV_SIX_FACE,   E_STATE_LEV3,1,algo_spv_sfa_cali_open, NULL,algo_spv_sfa_cali_close},
    {E_STATE_ORIG_EUL_CALI,  E_STATE_LEV3,1,algo_orig_eul_cali_open,NULL,algo_orig_eul_cali_close},
};
static const struct func_t algo_func_table[] ={
    {E_ALGO_FUNC_LOG_CTL,                algo_func_log_ctl},/*level1 表示待机状态，只有开启，没有关闭*/
    {E_ALGO_FUNC_AG_AVG_VALUE,           algo_func_ag_avg_value},
    {E_ALGO_FUNC_SAVE_BEFORE_POWEROFF,   algo_func_save_before_poweroff},
};

void algo_state_handle(uint16_t id, uint16_t event, void* param){
    const struct func_t* func;
    for(uint16_t i = 0; i < sizeof(algo_func_table)/sizeof(algo_func_table[0]); i++){
        func = &algo_func_table[i];
        if(NULL == func) {CWM_OS_dbgPrintf("[algo]algo_state_handle func NULL\n");break;}

        if((id == func->id) && (NULL != func->handle)){
            func->handle(param);
            return;
        }
    }

    const struct algo_t* algo;
    for(uint16_t i = 0; i < sizeof(algo_state_table)/sizeof(algo_state_table[0]); i++){
        algo = &algo_state_table[i];
        if(NULL == algo) {CWM_OS_dbgPrintf("[algo]algo_state_handle algo NULL\n");break;}

        if(id == algo->id){
            if((E_ALGO_EVENT_OPEN == event) && (algo->levle < E_ALGO_STATE_MAX)){
                if(NULL != algo_current){
                    {CWM_OS_dbgPrintf("[algo]algo_state_handle lv %u >? %u \n",algo->levle,algo_current->levle);}
                    /*
                    先关闭低优先级或同级状态,
                    再执行高优先级状态,
                    如果是同一个状态，则不 close，直接 open       
                    */
                    if((algo->levle >= algo_current->levle)){
                        if(algo->id !=  algo_current->id)
                            CLOSE(algo_current,param);
                        OPEN(algo,param);
                    }
                }else{
                    {CWM_OS_dbgPrintf("[algo]algo_state_handle algo_current NULL\n");}
                    OPEN(algo,param);
                }

                algo_current = algo;
                if(algo->is_recoverable) algo_current_lev[algo->levle] = algo;
            }else if((E_ALGO_EVENT_CLOSE == event) && (algo->levle < E_ALGO_STATE_MAX)){
                algo_current_lev[algo->levle] = NULL;
                CLOSE(algo,param);
                
                /*返回低一级状态*/
                uint16_t level = algo->levle;
                while(level){
                    if(NULL != algo_current_lev[level - 1]){
                        OPEN(algo_current_lev[level - 1],param);
                        algo_current = algo_current_lev[level - 1];
                        return;
                    }
                    else
                        {CWM_OS_dbgPrintf("[algo]algo_state_handle algo_current_lev[%u] \n",level - 1);}
                    level--;
                }
            }
        }
    }
}


////////////////////////////////////////////////////////////////外部接口////////////////////////////////////////////////////////
void algo_init(void)
{
    CWM_OS_dbgPrintf("[algo]config version V%s\n",ALGO_CONFIG_VERSION);

    //从 FLASH 中读出存储的算法数据
    memset((uint8_t*)&algo_dev_info,0,sizeof(algo_dev_info));
    algo_read_param_from_flash();
    algo_set_odr(CWM_DEFAUL_ODR);

    //ALGO&DML 初始化设置
    dml_algo_init();

    algo_state_handle(E_STATE_STANDBY,E_ALGO_EVENT_OPEN,NULL);
    algo_quiet_enable(true);
}

uint16_t algo_get_odr(void)
{
    return algo_dev_info.odr;
}

void algo_set_odr(uint16_t odr)
{
    if(odr)
        algo_dev_info.odr = odr;
    else
        algo_dev_info.odr = 25;
}

static void save_cali_value(void)
{
    update_cali_value_checksum();
    update_auto_cali_value_checksum();

    /*此处添加客户接口：保存校正数据到 flash*/
    diskio_save_flash_cali((uint8_t*)&algo_dev_info.ag_cali_value,sizeof(struct ag_cali_back_t));

    /*写完 flash 后，再读取保存的数据，确保写成功*/
    struct ag_cali_back_t cali_value = {0};
    diskio_read_flash_cali((uint8_t*)&cali_value,sizeof(cali_value));

    if(!memcmp(&algo_dev_info.ag_cali_value,&cali_value,sizeof(struct ag_cali_back_t))){
        CWM_OS_dbgPrintf("[algo]flash write ag_cali_value succuss\n");
    }else{
        CWM_OS_dbgPrintf("[algo]flash write ag_cali_value fail\n");
    }

    CWM_OS_dbgPrintf("[algo]save :ax=%d,ay=%d,az=%d,gx=%d,gy=%d,gz=%d\n",
        cali_value.ax,
        cali_value.ay,
        cali_value.az,
        cali_value.gx,
        cali_value.gy,
        cali_value.gz);
}
void algo_data_handle(void)
{
    algo_quiet_check_time();
    algo_ag_read();

    /*算法结果传给客户*/
    algo_res_read();

    /*保存校正数据*/
    if((algo_dev_info.event_cali_finish) && (E_STATE_STANDBY_SPV != algo_current->id)){
        algo_dev_info.event_cali_finish = 0;

        save_cali_value();

        /*此处添加客户接口：通知客户校正成功 or 失败*/
        uint16_t whl = algo_dev_info.ag_cali_value.spv_whole_status;
        uint16_t pcba = algo_dev_info.ag_cali_value.spv_pcba_status;
        uint16_t sixf = algo_dev_info.ag_cali_value.sixface_status;
        diskio_notify_spv_cali_result(whl,pcba,sixf);
    }else if((algo_dev_info.event_cali_finish) && (E_STATE_STANDBY_SPV == algo_current->id)){
        algo_dev_info.event_cali_finish = 0;
    }

    /*保存原始角度数据*/
    if(algo_dev_info.event_orig_eul_finish){
        algo_dev_info.event_orig_eul_finish = 0;

        update_original_eul_checksum();

        /*此处添加客户接口：通知客户原始角度校正成功 or 失败*/
        uint16_t status = 0;
        if(1 == algo_dev_info.original_eul.running_steps) status = algo_dev_info.original_eul.step1_status;
        else if(2 == algo_dev_info.original_eul.running_steps) status = algo_dev_info.original_eul.step2_status;
        else if(3 == algo_dev_info.original_eul.running_steps) status = algo_dev_info.original_eul.step3_status;
        diskio_notify_ori_eul_cali_result(algo_dev_info.original_eul.running_steps,status);
        CWM_OS_dbgPrintf("[algo]orig eul cali:%d,%d,%d,%d,\n",
            algo_dev_info.original_eul.running_steps,
            algo_dev_info.original_eul.step1_status,
            algo_dev_info.original_eul.step2_status,
            algo_dev_info.original_eul.step3_status);

        if(((E_ANGLE_INIT_STEP3 == algo_dev_info.original_eul.running_steps) &&
        (E_ORI_EUL_CALI_SUCCESS == algo_dev_info.original_eul.step1_status) &&
        (E_ORI_EUL_CALI_SUCCESS == algo_dev_info.original_eul.step2_status) &&
        (E_ORI_EUL_CALI_SUCCESS == algo_dev_info.original_eul.step3_status)) ||
        ((E_ANGLE_INIT_STEP1 == algo_dev_info.original_eul.running_steps) &&
        (E_ORI_EUL_CALI_SUCCESS == algo_dev_info.original_eul.step1_status))){
            
            /*此处添加客户接口：保存原始角度数据到 flash*/
            diskio_save_flash_eul((uint8_t*)&algo_dev_info.original_eul,sizeof(struct original_eul_t));

            /*写完 flash 后，再读取保存的数据，确保写成功*/
            struct original_eul_t ori_eul_value = {0};
            diskio_read_flash_eul((uint8_t*)&ori_eul_value,sizeof(ori_eul_value));
            if(!memcmp(&algo_dev_info.original_eul,&ori_eul_value,sizeof(struct original_eul_t))){
                CWM_OS_dbgPrintf("[algo]flash write original_eul succuss\n");
            }else{
                CWM_OS_dbgPrintf("[algo]flash write original_eul fail\n");
            }
        }
    }

    /*计算 ag 平均值功能完成，关闭功能，将数据传给客户*/
    if(algo_dev_info.event_ag_avg_value_1s_finish){
        algo_dev_info.en_ag_avg_value_1s = 0;
        algo_dev_info.event_ag_avg_value_1s_finish = 0;

        CWM_Sensor_Disable(IDX_ACCEL);
        CWM_Sensor_Disable(IDX_GYRO);

        /*此处添加客户接口：将 ag 平均值传给客户*/
        diskio_read_ag_avg_value((float *)&algo_dev_info.ag_avg_value_1s.ag);
        CWM_OS_dbgPrintf("[algo]ag avg:%d,%d,ax=%f,ay=%f,az=%f,gx=%f,gy=%f,gz=%f\n",
            algo_dev_info.ag_avg_value_1s.a_cnts,
            algo_dev_info.ag_avg_value_1s.g_cnts,
            algo_dev_info.ag_avg_value_1s.ag.ax,
            algo_dev_info.ag_avg_value_1s.ag.ay,
            algo_dev_info.ag_avg_value_1s.ag.az,
            algo_dev_info.ag_avg_value_1s.ag.gx,
            algo_dev_info.ag_avg_value_1s.ag.gy,
            algo_dev_info.ag_avg_value_1s.ag.gz);
    }    
}

void algo_log_debug_ctl(uint32_t ctr)
{
    CWM_OS_dbgPrintf("[algo] algo_log_debug_ctl ctr = %d\n",ctr);
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

void algo_hs_algo_ctl(uint32_t ctr)
{
    CWM_OS_dbgPrintf("[algo] algo_hs_algo_ctl ctr = %d\n",ctr);
    if(ctr){
        //打开SENSOR驱动
        algo_sensor_ctr(1);

        //设置SPV校正数据和初始角度数据
        hs_algo_init();

        CWM_Sensor_Enable(100);
    }
    else{
        CWM_Sensor_Disable(100);
    }
}

void algo_sensor_ctr(uint32_t ctr)
{
    CWM_OS_dbgPrintf("[algo] algo_sensor_ctr ctr = %d\n",ctr);
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

void algo_spv_cali_en(uint32_t mode)
{
    CWM_OS_dbgPrintf("[algo] algo_spv_cali_en mode:%d\n",mode);
    switch(mode){
        case E_CALI_SPV_WHOLE:
            algo_dev_info.ag_cali_value.spv_whole_status = E_CALI_RUNNING;
        break;
        case E_CALI_SPV_PCBA:
            algo_dev_info.ag_cali_value.spv_pcba_status = E_CALI_RUNNING;
        break;
        case E_CALI_SIX_FACE:
            algo_dev_info.ag_cali_value.sixface_status = E_CALI_RUNNING;
        break;
        default :
        return;
    }
	CWM_Sensor_Disable(IDX_ALGO_SPV);

    // 强制关闭校正
    SettingControl_t scl;
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[3] = -1;
    CWM_SettingControl(SCL_SENS_CALI_CONFIG, &scl);

    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 5; // z pluse point to sky
    scl.iData[2] = 5; // time-out period
    scl.iData[3] = CWM_DEFAUL_ODR;       // odr
    scl.iData[4] = 28400000; // gyro_Lsb_Dps
    scl.iData[5] = 3000;     // acc_noise
    scl.iData[6] = 120000;  // gyro_noise
    scl.iData[7] = 50000;   // acc_bias
    scl.iData[8] = 3300000; // gyro_bias
    CWM_SettingControl(SCL_SPV_CONFIG, &scl);

    // [1] = 1 整机校正， [1] = 2 PCBA校正，[1] = 5 六面校正
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = mode;
    CWM_SettingControl(SCL_SPV_MODE, &scl);

    CWM_Sensor_Enable(IDX_ALGO_SPV);
}

void algo_spv_cali_dis(void)
{
    CWM_OS_dbgPrintf("[algo]algo_spv_cali_dis\n");
    //打开自动校正
    SettingControl_t scl;
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    CWM_SettingControl(SCL_SENS_CALI_CONFIG, &scl);

    CWM_Sensor_Disable(IDX_ALGO_SPV);
}

void algo_spv_cali_disable_mode(void)
{
    if(E_STATE_STANDBY_SPV == algo_current->id){
        algo_state_handle(E_STATE_STANDBY_SPV,E_ALGO_EVENT_CLOSE,NULL);
    }else if(E_STATE_SPV_WHOLE == algo_current->id){
        algo_state_handle(E_STATE_SPV_WHOLE,E_ALGO_EVENT_CLOSE,NULL);
    }else if(E_STATE_SPV_PCBA == algo_current->id){
        algo_state_handle(E_STATE_SPV_PCBA,E_ALGO_EVENT_CLOSE,NULL);
    }else if(E_STATE_SPV_SIX_FACE == algo_current->id){
        algo_state_handle(E_STATE_SPV_SIX_FACE,E_ALGO_EVENT_CLOSE,NULL);
    }
}

void algo_original_eul_cali_en(uint32_t steps)
{

    CWM_OS_dbgPrintf("[algo] algo_original_eul_cali_en steps = %d\n",steps);
    SettingControl_t scl;
    switch(steps){    
        case E_ANGLE_INIT_STEP1:
            algo_dev_info.original_eul.step1_status = E_ORI_EUL_CALI_FAIL;
            algo_dev_info.original_eul.step2_status = E_ORI_EUL_CALI_FAIL;
            algo_dev_info.original_eul.step3_status = E_ORI_EUL_CALI_FAIL;
            CWM_Sensor_Enable(112);

            memset(&scl, 0, sizeof(scl));
            scl.iData[0] = 1;
            scl.iData[1] = 2;
            scl.iData[2] = E_ANGLE_INIT_STEP1;
            CWM_SettingControl(SCL_HS_RUN_INIT_ANGLE, &scl);

            algo_dev_info.original_eul.running_steps = E_ANGLE_INIT_STEP1;
            algo_dev_info.original_eul.step1_status = E_ORI_EUL_CALI_RUNNING;
        break;

        case E_ANGLE_INIT_STEP2:
            memset(&scl, 0, sizeof(scl));
            scl.iData[0] = 1;
            scl.iData[1] = 2;
            scl.iData[2] = E_ANGLE_INIT_STEP2;
            CWM_SettingControl(SCL_HS_RUN_INIT_ANGLE, &scl);

            algo_dev_info.original_eul.running_steps = E_ANGLE_INIT_STEP2;
            algo_dev_info.original_eul.step2_status = E_ORI_EUL_CALI_RUNNING;
        break;

        case E_ANGLE_INIT_STEP3:
            memset(&scl, 0, sizeof(scl));
            scl.iData[0] = 1;
            scl.iData[1] = 2;
            scl.iData[2] = E_ANGLE_INIT_STEP3;
            CWM_SettingControl(SCL_HS_RUN_INIT_ANGLE, &scl);

            algo_dev_info.original_eul.running_steps = E_ANGLE_INIT_STEP3;
            algo_dev_info.original_eul.step3_status = E_ORI_EUL_CALI_RUNNING;
        break;

        default:
        break;
    }
    
}

void algo_original_eul_cali_dis(void)
{
    CWM_OS_dbgPrintf("[algo]algo_original_eul_cali_dis\n");
    CWM_Sensor_Disable(112);
}

void algo_avg_ag_value_en(void)
{
    CWM_OS_dbgPrintf("[algo] algo_avg_ag_value_en\n");

    algo_dev_info.en_ag_avg_value_1s = 1;
    memset(&algo_dev_info.ag_avg_value_1s,0,sizeof(struct ag_avg_t));
    CWM_Sensor_Enable(IDX_ACCEL);
    CWM_Sensor_Enable(IDX_GYRO);
}

void algo_save_before_poweroff(void)
{
    SettingControl_t scl;
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 1;
    CWM_SettingControl(SCL_SENS_CALI_CTRL_A, &scl);

    algo_dev_info.ag_cali_value.auto_valid = 1;
    algo_dev_info.ag_cali_value.auto_gx = scl.iData[5];
    algo_dev_info.ag_cali_value.auto_gy = scl.iData[6];
    algo_dev_info.ag_cali_value.auto_gz = scl.iData[7];

    CWM_OS_dbgPrintf("[algo]auto_calib_value befor poweroff:au_gx = %d,au_gy = %d,au_gz = %d\n",algo_dev_info.ag_cali_value.auto_gx,
                      algo_dev_info.ag_cali_value.auto_gy,algo_dev_info.ag_cali_value.auto_gz);
    
    save_cali_value();
}
