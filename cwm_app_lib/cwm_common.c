#include "stdint.h"
#include "string.h"
#include "stdbool.h"
#include  "cwm_common.h"
#include  "cwm_config.h"
#include "cwm_customio.h"
/**************************************************algo queue**************************************************/
#define QUEUE_MAX_BUF_SIZE (256)/*必须是 2 的整数倍*/
struct queue_t{
    int32_t head;
    int32_t size;
    uint8_t buf[QUEUE_MAX_BUF_SIZE];
};
static struct queue_t queue;

/**************************************************本地接口**************************************************/
static void queue_init(void)
{
    memset(&queue,0,sizeof(queue));
}

//入队
//queue data: data_len0,data0;  data_len1,data1;  data_len2,data2; ...... data_lenn,datan;
//data0,data1,data2,datan: msg.id,msg.data
static int32_t queue_add(uint8_t* data, uint16_t data_len)
{
    if(NULL == data)
        return -1;

    cwm_taskENTER_CRITICAL();
    if(((queue.size + 2 + data_len) > QUEUE_MAX_BUF_SIZE) || (data_len > sizeof(struct algo_msg_t))){
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

//出队
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
    //确保从最早入队的数据开始出队（此处的tail其实不是队尾）
    uint16_t tail = (queue.head >= queue.size)?(queue.head - queue.size):(QUEUE_MAX_BUF_SIZE + queue.head - queue.size);
    // uint16_t tail = (queue.head+queue.size)/QUEUE_MAX_BUF_SIZE;
    uint8_t* p = (uint8_t*)&data_len;
    uint16_t des_len = 2;
    //初始化定义2个字节的des_len,来读取2个byte的数据长度是多少，然后通过指针p指向数据长度data_len来获取长度值
    while(des_len){
        *(p++) = queue.buf[tail++];
        tail &= QUEUE_MAX_BUF_SIZE - 1;
        des_len--;
    }

    if((data_len > (queue.size - 2)) || (data_len > sizeof(struct algo_msg_t))){
        cwm_taskEXIT_CRITICAL();
        CWM_OS_dbgPrintf("[algo_que]get error: len too long\n");
        return -1;
    }

    //将指针p指向真正的数据data地址,然后将要读取的数据长度data_len赋值给des_len,循环执行des_len次,将queue里数据处理到指向data的位置
    p = data;
    des_len = data_len;
    while(des_len){
        *(p++) = queue.buf[tail++];
        tail &= QUEUE_MAX_BUF_SIZE - 1;
        des_len--;
    }

    //将队列里的数据大小更新，减去读走的真正数据长度个数data_len以及2byte的数据长度定义
    queue.size -= data_len + 2;
    cwm_taskEXIT_CRITICAL();
    return 0;
}

/**************************************************外部调用接口**************************************************/
void algo_message_init(void)
{
    queue_init();
}

int32_t message_to_algo(uint32_t id,uint32_t value)
{
    struct algo_msg_t msg;
    uint32_t* v = (uint32_t*)&msg.data[0];

    msg.id = id;
    *v = value;

    CWM_OS_dbgPrintf("[algo]message_to_algo %u %u\n",id,value);
    return queue_add((uint8_t*)&msg,4+4);
}

int32_t data_to_algo(uint32_t id,uint8_t* data,uint16_t len)
{
    if(len > MSG_DATA_MAX_SIZE)
        return -1;

    struct algo_msg_t msg;
    msg.id = id;
    memcpy(msg.data,data,len);
    return queue_add((uint8_t*)&msg,4+len);
}

int get_msg(uint8_t* data)
{
    return queue_get(data);
}




