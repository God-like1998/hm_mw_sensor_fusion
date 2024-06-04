

HM_MW_SENSOR_FUSION_SRC = project/ab15xx/apps/headset_ref_design/harman/middleware/hm_mw_sensor_fusion
#C_FILES += $(HM_MW_SENSOR_FUSION_SRC)/cwm_interface.c
C_FILES += $(HM_MW_SENSOR_FUSION_SRC)/cwm_algo_task.c
C_FILES += $(HM_MW_SENSOR_FUSION_SRC)/cwm_common.c
C_FILES += $(HM_MW_SENSOR_FUSION_SRC)/cwm_config.c
C_FILES += $(HM_MW_SENSOR_FUSION_SRC)/cwm_diskio.c
C_FILES += $(HM_MW_SENSOR_FUSION_SRC)/cwm_test.c





#CFLAGS += -I$(SOURCE_DIR)/project/ab15xx/apps/headset_ref_design/harman/driver/g_sensor
CFLAGS += -I$(SOURCE_DIR)/project/ab15xx/apps/headset_ref_design/harman/middleware/hm_mw_sensor_fusion/inc
CFLAGS += -I$(SOURCE_DIR)/project/ab15xx/apps/headset_ref_design/harman/middleware/hm_mw_sensor_fusion

LIBS += $(SOURCE_DIR)/project/ab15xx/apps/headset_ref_design/harman/middleware/hm_mw_sensor_fusion/lib/CWM_LIB_DML_gcc_m4.a
LIBS += $(SOURCE_DIR)/project/ab15xx/apps/headset_ref_design/harman/middleware/hm_mw_sensor_fusion/lib/CWM_LIB_gcc_m4.a



