

HM_MW_SENSOR_FUSION_SRC = project/ab15xx/apps/headset_ref_design/harman/middleware/hm_mw_sensor_fusion

C_FILES += $(HM_MW_SENSOR_FUSION_SRC)/samples/AB1595/cwm_algo_task.c
C_FILES += $(HM_MW_SENSOR_FUSION_SRC)/samples/AB1595/cwm_customio.c


C_FILES += $(HM_MW_SENSOR_FUSION_SRC)/cwm_app_lib/cwm_common.c
C_FILES += $(HM_MW_SENSOR_FUSION_SRC)/cwm_app_lib/cwm_config.c
C_FILES += $(HM_MW_SENSOR_FUSION_SRC)/cwm_app_lib/cwm_test.c



CFLAGS += -I$(SOURCE_DIR)/project/ab15xx/apps/headset_ref_design/harman/middleware/hm_mw_sensor_fusion/inc
CFLAGS += -I$(SOURCE_DIR)/project/ab15xx/apps/headset_ref_design/harman/middleware/hm_mw_sensor_fusion/samples/AB1595
CFLAGS += -I$(SOURCE_DIR)/project/ab15xx/apps/headset_ref_design/harman/middleware/hm_mw_sensor_fusion/cwm_app_lib



LIBS += $(SOURCE_DIR)/project/ab15xx/apps/headset_ref_design/harman/middleware/hm_mw_sensor_fusion/lib/CWM_LIB_DML_gcc_m33.a
LIBS += $(SOURCE_DIR)/project/ab15xx/apps/headset_ref_design/harman/middleware/hm_mw_sensor_fusion/lib/CWM_LIB_gcc_m33.a
# LIBS += $(SOURCE_DIR)/project/ab15xx/apps/headset_ref_design/harman/middleware/hm_mw_sensor_fusion//cwm_app_lib/cwm_app_gcc_m33.a


