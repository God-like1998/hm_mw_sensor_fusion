#ifndef __OS_API_H__
#define __OS_API_H__

typedef struct{
    int (*dbgOutput)(const char*);
    uint64_t (*GetTimeNs)(void);
    void (*uSleep)(uint32_t);
    int (*i2cRead)(uint16_t slaveAddr, uint16_t reg, int regLength, uint8_t *readData, int readDataSize, int busIndex);
    int (*i2cWrite)(uint16_t slaveAddr, uint16_t reg, int regLength, uint8_t *writeData, int writeDataSize, int busIndex);
    void *(*malloc)(int size);
    void (*free)(void* ptr);
    int (*spiRead)(uint16_t reg, int regLength, uint8_t *readData, int readDataSize, int busIndex);
    int (*spiWrite)(uint16_t reg, int regLength, uint8_t *writeData, int writeDataSize, int busIndex);
} os_api;

typedef os_api OsAPI;

#endif /* __OS_API_H__ */
