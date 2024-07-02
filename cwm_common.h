#ifndef __CWM_COMMON_H__
#define __CWM_COMMON_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"
#include "string.h"
#include "stdbool.h"

#define MSG_DATA_MAX_SIZE (4+4*16)/*max size data: setting id + setting data*/

struct algo_msg_t{
    uint32_t id;
    uint8_t data[MSG_DATA_MAX_SIZE];
};

void algo_message_init(void);
int32_t message_to_algo(uint32_t id,uint32_t value);
int32_t data_to_algo(uint32_t id,uint8_t* data,uint16_t len);
int get_msg(uint8_t* data);


#ifdef __cplusplus
}
#endif

#endif



