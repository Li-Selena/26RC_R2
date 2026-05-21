#ifndef __CRC_H__
#define __CRC_H__

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define ARM_REMOTE_X_MIN_MM   (-500.0f )
#define ARM_REMOTE_X_MAX_MM   ( 500.0f )

#define ARM_REMOTE_Y_MIN_MM   (-500.0f )
#define ARM_REMOTE_Y_MAX_MM   ( 500.0f )

#define ARM_REMOTE_Z_MIN_MM   ( 50.0f  )
#define ARM_REMOTE_Z_MAX_MM   ( 500.0f )

#define ARM_REMOTE_RAW_XY_MIN  (-127.0f)   // 接收XY轴原始值最小值
#define ARM_REMOTE_RAW_XY_MAX  ( 127.0f)   // 接收XY轴原始值最大值
#define ARM_REMOTE_RAW_Z_MIN   (50.0f  )   // 接收Z轴原始值最小值
#define ARM_REMOTE_RAW_Z_MAX   (120.0f )   // 接收Z轴原始值最大值

/* 遥控器接收状态 */
typedef enum
{
    STATE_WAIT_HEADER = 0,   /* 等待包头 0xA5 */
    STATE_RECV_DATA,         /* 接收数据区 */
    STATE_RECV_CHECKSUM,     /* 接收校验和 */
    STATE_RECV_TAIL          /* 接收包尾 0x5A */
} ParseState;

/* 遥控器一帧数据区长度：
 * byte0  ~ byte2  : 底盘 Vx / Vy / Vw
 * byte3  ~ byte5  : 腿部 legx / legy / leghtheta
 * byte6           : move_flag
 * byte7           : arm_flag
 * byte8  ~ byte10 : arm_X / arm_Y / arm_Z
 */
#define BT_FRAME_DATA_LEN   14U

extern ParseState BT_Uart10;
extern uint8_t bt_data[BT_FRAME_DATA_LEN];
extern uint8_t data_index;
extern uint8_t checksum;
extern volatile uint8_t bt_parse_ok;
extern uint8_t btReceiveData;

extern UART_HandleTypeDef huart10;

extern uint8_t USB_Task_flag;
extern uint8_t USART_Task_flag ;

/* 腿部控制量 */
extern int8_t leg_flag;
extern float legx;
extern float legy;
extern float leghtheta;

/* 机械臂控制量 */
extern int8_t arm_flag;
extern float arm_X;
extern float arm_Y;
extern float arm_Z;

extern int8_t pre_step_flag;
extern float pre_step_wheel_speed ;

void UART10_Receive(uint8_t receiveData);
void BT_Data_MAC_Process(float *V_x, float *V_y, float *V_w, int8_t *cmd);

#endif