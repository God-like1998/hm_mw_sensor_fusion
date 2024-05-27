#ifndef __CWM_COMMON_H__
#define __CWM_COMMON_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"
#include "string.h"
#include "stdbool.h"

#define MSG_DATA_MAX_SIZE (4+4*16)/*max size data: setting id + setting data*/




typedef struct{
    uint32_t id;
    uint8_t data[MSG_DATA_MAX_SIZE];
}algo_msg_t;



void algo_message_init(void);
int32_t send_single_to_algo_message(uint32_t id,uint32_t value);
int32_t send_multi_to_algo_message(uint32_t id,uint8_t* data,uint16_t len);
int32_t receive_from_algo_message(uint8_t* data);





#ifdef __cplusplus
}
#endif




#endif























