#ifndef ___CWM_LIB__H_
#define ___CWM_LIB__H_

#include <stdint.h>
#include "os_api.h"
#include "CWM_EventStructure.h"
#include "CWM_CustomDataStruct.h"

#define SCL_LOG                 0
#define SCL_USER_INFO           1
#define SCL_DATE_TIME           2
#define SCL_SEDENTARY           3
#define SCL_PEDO_RESET          4
#define SCL_REQ_SLEEPING_DATA   5
#define SCL_LOW_POWER_MODE      6
#define SCL_SET_ACTIVITY_MODE   7
#define SCL_GET_CHIP_INFO       8
#define SCL_ACTIVITY_CONFIG     9
#define SCL_LIB_DEBUG           10
#define SCL_PEDO_CONFIG         11
#define SCL_GET_LIB_INFO        12
#define SCL_SWIM_CONFIG         13
#define SCL_BIKING_CONFIG       14
#define SCL_ACTIVITY_PAUSE      15
#define SCL_HAND_UPDOWN_CONFIG  16
#define SCL_CHIP_VENDOR_CONFIG  17
#define SCL_SLEEP_CONFIG        18
#define SCL_ABS_STATIC_CONFIG   19
#define SCL_HEART_RATE_CONFIG   20
#define SCL_REQ_SWIM_EXIT       21
#define SCL_WATCH_FALL_CONFIG   22
#define SCL_AR_ALERT_CONFIG     23
#define SCL_ACT_PAUSE_DETECT    24
#define SCL_WM_CONFIG           25
#define SCL_INACTIVITY_CONFIG   26
#define SCL_SET_INACTIVITY_MODE 27
#define SCL_STAND_CONFIG        28
#define SCL_SS_CONFIG           29
#define SCL_ACT_INFO_CONFIG     30
#define SCL_REQ_ACTIVITY_EXIT   31
#define SCL_ACT_PAI_INFO        33
#define SCL_HAND_WRIST_CONFIG   34
#define SCL_ACT_INTSY_INFO      35
#define SCL_DUMP_WI_INFO        36
#define SCL_ACT_ZONE_TIME       37
#define SCL_ACT_TRAINING_INFO   38
#define SCL_HAND_SHAKE_CONFIG   39
#define SCL_WATCH_INTF_RESET    40
#define SCL_HAND_RUB_CONFIG     41
#define SCL_PEDO_CONFIG2        42
#define SCL_HG_INTF_CONFIG      43
#define SCL_ACT_AUTO_PAUSE_CFG  44
#define SCL_AR_ALERT2_CONFIG    45
#define SCL_WI_SENS_REQ_CFG     46
#define SCL_ACT_SA_METR_INFO    47
#define SCL_SLEEP_ALARM_CFG     48
#define SCL_FITNESS_INFO        49
#define SCL_SZ_BREACH_CONFIG    50
#define SCL_ACT_PAI_INFO2       51
#define SCL_AIC_CONFIG          52
#define SCL_AIC_INFO            53

#define SCL_ALGO_PROC_CONFIG    100
#define SCL_INPUT_SENSOR_CONFIG 101
#define SCL_INPUT_DT_CONFIG     102
#define SCL_SENS_RANGE_DETECT_CONFIG    103
#define SCL_SENS_CALI_CONFIG    110
#define SCL_SENS_CALI_CTRL_MAG  111
#define SCL_SENS_CALI_CTRL_A    112
#define SCL_SPV_CONFIG          113
#define SCL_FACTORY_CALI_CONFIG 113
#define SCL_SPV_MODE            114
#define SCL_FACTORY_CALI_MODE   114
#define SCL_SENS_CALI_SET_MAG   115
#define SCL_SENS_CALI_SET_A     116
#define SCL_SENS_CALI_CTRL_A2   117

#define SCL_GET_HEAP_INFO       120
#define SCL_SEC_VERIFY_CLIENT   150
#define SCL_SEC_VERIFY_SERVER   151

#define SCL_SIN_HR_EXT          200

#define SCL_HS_ORIEN_CONFIG     300
#define SCL_HS_ORIEN_CTRL_EXC   301
#define SCL_HS_ORIEN_RESET      302
#define SCL_HS_PHONE_CONFIG     303
#define SCL_EAR_FALL_CONFIG     304
#define SCL_HS_PHONE_CONFIG2    305
#define SCL_HS_RUN_INIT_ANGLE   306
#define SCL_HS_INTF_CONFIG      307
#define SCL_HS_INTF_RESTART     308
#define SCL_HS_GESTURE_CONFIG   309
#define SCL_HS_NECK_HT_CONFIG   310
#define SCL_HS_ORIEN_CTRL_SYNC  311
#define SCL_HS_TAP_CONFIG       312

#define SCL_FUSION_MOUSE_CONFIG 501
#define SCL_PEDO_POD_CONFIG     502
#define SCL_PEDO_POD_TR_CALI    503
#define SCL_PEDO_POD_RUN_CMD    504

#define SCL_GLASSES_MOT_CONFIG  510
#define SCL_GLASSES_MOT_LEVEL   511
#define SCL_GLASSES_MOT_CALI_CTRL   512
#define SCL_GLASSES_MOT_CMD     513
#define SCL_NB_AR_CONFIG        520

/**
  * @brief Custom Sensor Input sensorId
  * @api CWM_CustomSensorInput
  * @param CustomSensorData.sensorType extSensorId
  * @
  */
#define CUSTOM_ACC              0
#define CUSTOM_GYRO             1
#define CUSTOM_MAG              2
#define CUSTOM_BARO             3
#define CUSTOM_TEMP             4
#define CUSTOM_HEARTRATE        5
#define CUSTOM_GNSS             6
#define CUSTOM_OFFBODY_DETECT   7
#define CUSTOM_ON_CHARGING      8
#define CUSTOM_ACC_ANY_MOTION   9
#define CUSTOM_SENS1            10


/**
  * @brief sensor enable/disable
  * @api CWM_Sensor_Enable CWM_Sensor_Disable
  * @param sensorId
  */
#define IDX_ACCEL                    0
#define IDX_GYRO                     1
#define IDX_MAG                      2
#define IDX_BARO                     3
#define IDX_TEMP                     4
#define IDX_HEARTRATE                5
#define IDX_GNSS                     6
#define IDX_OFFBODY_DETECT           7
#define IDX_REQUEST_SENSOR           11
#define IDX_ALGO_ANY_MOTION          15
#define IDX_ALGO_NO_MOTION           16
#define IDX_ONCHARGING_DETECT        18
#define IDX_ALGO_ABSOLUTE_STATIC     19
#define IDX_ALGO_SPV                 20
#define IDX_ALGO_FACTORY_CALI        20
#define IDX_ALGO_SENS_CALIBRATION    21
#define IDX_ALGO_SENS_RANGE_DETECT   24

typedef struct {
    int iData[16];
} SettingControl_t;


typedef void (*FN_SENSOR_LISTEN)(pSensorEVT_t);


#define LOS_DATA_TYPE_INT               0
#define LOS_DATA_TYPE_STR               1

/**
  * @brief 
  * @param dataType: cmd data type indicater
  *                  0 (LOS_DATA_TYPE_INT): int type data, dataInt accessible
  *                  1 (LOS_DATA_TYPE_STR): string type data, *dataStr and dataStr_length accessible
  */
typedef struct {
    const char  *optionString;
    const char  *sectionName;
    const char  *apiName;
    int         cmdId;
    int         dataType;
    int         dataInt[16];
    const char  *dataStr;
    int         dataStr_length;
} los_cmd_data_t;

typedef void (*FN_LOS_CMD_EXECUTE)(los_cmd_data_t *);


/**
  * @brief  CyweeMotion internal process Initial
  */
void CWM_LibPreInit(os_api *api);

/**
  * @brief  Sensor Event register callback, when sensor event is triggered,
  *         this function will be called to notify
  */
void CWM_LibPostInit(FN_SENSOR_LISTEN pfn);

/**
  * @brief  Sensor enable based on sensor index provided
  * @param  sensorId: sensor register handle, if not setup this handle don't care
  * @retval 0 is success, otherwise is failed
  */
int CWM_Sensor_Enable(uint32_t sensorId);

/**
  * @brief  Sensor disable based on sensor index provided
  * @param  sensorId: sensor register handle, if not setup this handle don't care
  * @retval 0 is success, otherwise is failed
  */
int CWM_Sensor_Disable(uint32_t sensorId);

/**
  * @brief  Read sensor data and output through sensor call-back function
  */
int CWM_process2(int dt_us);

/**
  * @brief  Put custom sensor data.
  * @param  SensorDataInput: sensor data input handle, input format as below:
  *                          sensorType= CustomSensorDataIndex
  *                          fData     = sensor raw data
  * @retval 0 is success, otherwise is failed
  */
int CWM_CustomSensorInput(CustomSensorData *SensorDataInput);

/**
  * @brief  Switch log output by input parameters
  * @param  Control ID
  * @param  Setting control array
  * @retval 0 is success, otherwise is failed
  */
int CWM_SettingControl(int CtrId, SettingControl_t *pData);

/**
  * @brief  Execute option string
  * @param  optionString
  * @param  sectionString section execute order 
  * @param  custCallbackFunc custom option string execute function
  * @param  skipExeDefault don't execute default command
  */
void CWM_LoadOptionString(const char *optionString, \
                          const char *sectionString, \
                          FN_LOS_CMD_EXECUTE custCallbackFunc, \
                          int skipExeDefault);

#endif //___CWM_LIB__H_
