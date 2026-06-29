#ifndef __RECV_SEND_H
#define __RECV_SEND_H
#include "main.h"
#include "string.h"

typedef union 
    {
        uint8_t bytes[4];
        float float32;
        int32_t int_32;
        uint32_t uint_32;
    } converter_32;

typedef union {
    uint8_t  bytes[256];    // 原始字节
    int16_t  int16[128];      // int16
    float    float32[64];       // float32
    int32_t  int32[64];       // int32
    uint32_t uint32[64];       // uint32
} Data_buffer_t;


extern volatile float x_recv,y_recv,w_recv;
extern volatile uint8_t g_data_ready;   // 一帧接收完成标志
extern volatile uint8_t g_cmd;           // 解析出的命令
//extern volatile uint8_t g_data[256];         // 解析出的数据
extern volatile uint8_t g_data_len;      // 解析出的数据长度
extern volatile uint8_t g_use_data;

extern volatile Data_buffer_t g_data_buf_rx;

extern uint8_t usart1_rx_buf[1];

void send_id(uint8_t id);
void recv_send_init(UART_HandleTypeDef *huart);
void Send_Cmd_Data(uint8_t cmd,const uint8_t *datas,uint8_t len);
void recv_and_process(uint8_t bytedata);
void Data_Analysis(void);

#endif
