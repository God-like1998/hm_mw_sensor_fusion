


#include "stdint.h"
#include "string.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"


#include  "cwm_common.h"
#include  "cwm_config.h"


/**************************************************algo queue**************************************************/
#define QUEUE_MAX_BUF_SIZE (256)/*必须是 2 的整数倍*/
#define cwm_taskENTER_CRITICAL()  {if(pdFALSE == xPortIsInsideInterrupt()) {taskENTER_CRITICAL();}}
#define cwm_taskEXIT_CRITICAL()   {if(pdFALSE == xPortIsInsideInterrupt()) {taskEXIT_CRITICAL();}}

struct queue_t{
    int32_t head;
    int32_t size;
    uint8_t buf[QUEUE_MAX_BUF_SIZE];
};

static struct queue_t queue;


////////////////////////////////////////////////////////////////本地调用接口////////////////////////////////////////////////////////

static void queue_init(void)
{
    memset(&queue,0,sizeof(queue));
}

//queue data: data_len0,data0;  data_len1,data1;  data_len2,data2; ...... data_lenn,datan;
//data0,data1,data2,datan: msg.id,msg.data
static int32_t queue_add(uint8_t* data, uint16_t data_len)
{
    if(NULL == data)
        return -1;

    cwm_taskENTER_CRITICAL();
    if(((queue.size + 2 + data_len) > QUEUE_MAX_BUF_SIZE) || (data_len > sizeof(algo_msg_t))){
        cwm_taskEXIT_CRITICAL();
        CWM_OS_dbgPrintf("[algo_que]add error: queu full\n");
        return -1;
    }

    uint8_t* p = (uint8_t*)&data_len;
    uint16_t des_len = 2;
    while(des_len){
        queue.buf[queue.head++] = *(p++);
        queue.head &= QUEUE_MAX_BUF_SIZE - 1;
        des_len--;
    }
    
    p = data;
    des_len = data_len;
    while(des_len){
        queue.buf[queue.head++] = *(p++);
        queue.head &= QUEUE_MAX_BUF_SIZE - 1;
        des_len--;
    }

    queue.size += data_len + 2;
    cwm_taskEXIT_CRITICAL();
    return 0;
}

static int32_t queue_get(uint8_t* data)
{
    if(NULL == data)
        return -1;

    cwm_taskENTER_CRITICAL();
    if(queue.size <= 2){
        cwm_taskEXIT_CRITICAL();
        return -1;
    }
    
    uint16_t data_len;
    uint16_t tail = (queue.head >= queue.size)?(queue.head - queue.size):(QUEUE_MAX_BUF_SIZE + queue.head - queue.size);
    uint8_t* p = (uint8_t*)&data_len;
    uint16_t des_len = 2;
    while(des_len){
        *(p++) = queue.buf[tail++];
        tail &= QUEUE_MAX_BUF_SIZE - 1;
        des_len--;
    }

    if((data_len > (queue.size - 2)) || (data_len > sizeof(algo_msg_t))){
        cwm_taskEXIT_CRITICAL();
        CWM_OS_dbgPrintf("[algo_que]get error: len too long\n");
        return -1;
    }

    p = data;
    des_len = data_len;
    while(des_len){
        *(p++) = queue.buf[tail++];
        tail &= QUEUE_MAX_BUF_SIZE - 1;
        des_len--;
    }

    queue.size -= data_len + 2;
    cwm_taskEXIT_CRITICAL();
    return 0;
}



////////////////////////////////////////////////////////////////外部调用接口////////////////////////////////////////////////////////

void algo_message_init(void)
{
    queue_init();
}

int32_t send_single_to_algo_message(uint32_t id,uint32_t value)
{
    algo_msg_t msg;
    msg.id = id;
    *((uint32_t*)&msg.data[0]) = value;
    return queue_add((uint8_t*)&msg,4+4);
}

int32_t send_multi_to_algo_message(uint32_t id,uint8_t* data,uint16_t len)
{
    if(len > MSG_DATA_MAX_SIZE)
        return -1;

    algo_msg_t msg;
    msg.id = id;
    memcpy(msg.data,data,len);
    return queue_add((uint8_t*)&msg,4+len);
}

int32_t receive_from_algo_message(uint8_t* data)
{
    
    return queue_get(data);

}




















