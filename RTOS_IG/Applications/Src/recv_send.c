#include "recv_send.h"
#include "usart.h"

static uint8_t rx_step = 0;           // 解析步骤
static uint16_t rx_cnt = 0;            // 缓冲区计数
static uint8_t rx_buf[300];           // 接收缓冲区
static uint8_t rx_len = 0;            // 数据长度
static uint8_t rx_cmd = 0;            // 命令字节
static uint8_t* data_ptr = NULL;      // 数据指针
static uint16_t rx_crc16 = 0;         // CRC缓存
static UART_HandleTypeDef * self_huart= NULL;
// 解析完成标志和数据
volatile uint8_t g_data_ready = 0;   // 一帧接收完成标志
volatile uint8_t g_cmd = 0;           // 解析出的命令
volatile uint8_t g_data_len = 0;      // 解析出的数据长度
volatile uint8_t g_use_data = 0;

// 自定解析内容
volatile Data_buffer_t g_data_buf_rx; // 解析出的数据
volatile float x_recv,y_recv,w_recv;
 uint8_t usart1_rx_buf[1]={0};

void recv_send_init(UART_HandleTypeDef *huart)
{
  self_huart = huart;
    
  HAL_UART_Receive_IT(&huart6, usart1_rx_buf, 1); 

}
void SendByte(uint8_t data)
{
  if (self_huart == NULL) return;
  HAL_UART_Transmit(self_huart, &data, 1, 100);
}

void Send(const uint8_t *data,uint8_t len)
{
        uint8_t i;
        for (i = 0; i < len; i++)
        {
                SendByte(data[i]);//发送一个字节
        }
}

uint16_t CRC16_Check(const uint8_t *data, uint8_t len)
{
    uint16_t CRC16 = 0xFFFF;
    uint8_t state, i, j;
    for(i = 0; i < len; i++)
    {
        CRC16 ^= data[i];
        for(j = 0; j < 8; j++)
        {
            state = CRC16 & 0x01;
            CRC16 >>= 1;
            if(state)
            {
                CRC16 ^= 0xA001;
            }
        }
    }
    return CRC16;
}

void Send_Cmd_Data(uint8_t cmd,const uint8_t *datas,uint8_t len)
{
    uint8_t buf[300],i,cnt=0;
    uint16_t crc16;
    buf[cnt++] = 0xA5;
    buf[cnt++] = 0x5A;
    buf[cnt++] = len;
    buf[cnt++] = cmd;
    for(i=0;i<len;i++)
    {
        buf[cnt++] = datas[i];
    }
    crc16 = CRC16_Check(buf,len+4);
    buf[cnt++] = crc16>>8;
    buf[cnt++] = crc16&0xFF;
    buf[cnt++] = 0xFF;
    Send(buf,cnt);//调用数据帧发送函数将打包好的数据帧发送出去
}


void send_id(uint8_t id)
{
  Send_Cmd_Data(0x00, (uint8_t*)&id, 1);
}
void recv_and_process(uint8_t bytedata)
{
    switch (rx_step)
    {
        case 0:  // 等待帧头1 (0xA5)
            if (bytedata == 0xA5)
            {
                rx_step = 1;
                rx_cnt = 0;
                rx_buf[rx_cnt++] = bytedata;
            }
            break;
            
        case 1:  // 等待帧头2 (0x5A)
            if (bytedata == 0x5A)
            {
                rx_step = 2;
                rx_buf[rx_cnt++] = bytedata;
            }
            else if (bytedata == 0xA5)  // 重复帧头1，保持状态
            {
                rx_step = 1;
            }
            else  // 错误，重新开始找帧头
            {
                rx_step = 0;
            }
            break;
            
        case 2:  // 接收数据长度
            rx_step = 3;
            rx_buf[rx_cnt++] = bytedata;
            rx_len = bytedata;  // 记录数据长度
            break;
            
        case 3:  // 接收命令字节

            rx_buf[rx_cnt++] = bytedata;
            rx_cmd = bytedata;          // 记录命令
            data_ptr = &rx_buf[rx_cnt]; // 记录数据起始指针，用于计算是否收到够rx_len
            rx_step = (rx_len == 0) ? 5 : 4;  // 如果没数据，直接跳到CRC
            break;
            
        case 4:  // 接收数据内容 (len字节)
            rx_buf[rx_cnt++] = bytedata;
            // 判断收完没：当前指针 - 数据起始指针 == 长度
            if (&rx_buf[rx_cnt] - data_ptr >= rx_len)
            {
                rx_step = 5;
            }
            break;
            
        case 5:  // 接收CRC高8位
            rx_step = 6;
            rx_crc16 = bytedata;  // 先存高8位
            break;
            
        case 6:  // 接收CRC低8位，并校验
            rx_crc16 <<= 8;
            rx_crc16 += bytedata;  // 合成16位CRC
            
            // 计算前面所有字节的CRC（帧头+长度+命令+数据）
            if (rx_crc16 == CRC16_Check(rx_buf, rx_cnt))
            {
                rx_step = 7;  // CRC正确，等帧尾
            }
            else if (bytedata == 0xA5)  // 如果是新帧头，回退
            {
                rx_buf[0] = bytedata;
                rx_step = 1;
                rx_cnt = 1;
            }
            else  // CRC错误，丢弃
            {
                rx_step = 0;
            }
            break;
            
        case 7:  // 接收帧尾 (0xFF)
            if (bytedata == 0xFF)
            {
                // 一帧完整接收成功！
                g_cmd = rx_cmd;
                g_data_len = rx_len;
              if (! g_use_data)  // 如果没有在读取数据，则写入，否则跳过
                {
              memcpy((uint8_t*)g_data_buf_rx.bytes, data_ptr, rx_len);  // 拷贝数据
                g_data_ready = 1;  // 通知主循环
                }
                
                
                
                
                rx_step = 0;  // 重置状态机，准备下一帧
            }
            else if (bytedata == 0xA5)  // 新帧头
            {
                rx_step = 1;
                rx_cnt = 1;
                rx_buf[0] = bytedata;
            }
            else
            {
                rx_step = 0;
            }
            break;
            
        default:
            rx_step = 0;
            break;
    }
}

void Data_Analysis(void)
{
        switch (rx_cmd)
        {
        case 0x00:  // 原数据
                break;
        case 0x01:
            {
                HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
            }   
                
        default:
                // 未知命令，进行错误处理或者忽略
                break;
        }
}
