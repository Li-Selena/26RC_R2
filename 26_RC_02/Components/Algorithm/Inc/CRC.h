#ifndef __CRC_H__
#define __CRC_H__

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include "R2_move.h"
#include "R2_climb.h"

#define ARM_REMOTE_X_MIN_MM   (-500.0f)
#define ARM_REMOTE_X_MAX_MM   ( 500.0f)

#define ARM_REMOTE_Y_MIN_MM   (-500.0f)
#define ARM_REMOTE_Y_MAX_MM   ( 500.0f)

#define ARM_REMOTE_Z_MIN_MM   ( 50.0f )
#define ARM_REMOTE_Z_MAX_MM   ( 500.0f)

/* 遥控器接收状态 */
typedef enum
{
    STATE_WAIT_HEADER = 0,
    STATE_RECV_DATA,
    STATE_RECV_CHECKSUM,
    STATE_RECV_TAIL
} ParseState;

/*
 * 数据区 = 36 字节。
 *
 * ── byte 0~7: 8 个模式标志位，每个对应一种 R2 模式 ──
 * 同时只有一个为 1，全 0 = 底盘静止。
 *   byte 0 : RNYV  (ROBOT_NO_YAW_VEL)
 *   byte 1 : RV    (ROBOT_VEL)
 *   byte 2 : WNYV  (WORLD_NO_YAW_VEL)
 *   byte 3 : WV    (WORLD_VEL)
 *   byte 4 : RNYVP (ROBOT_NO_YAW_POS)
 *   byte 5 : RVP   (ROBOT_POS)
 *   byte 6 : WNYVP (WORLD_NO_YAW_POS)
 *   byte 7 : WVP   (WORLD_POS)
 *
 * ── byte 8~11: 4 控制字节 ──
 *   byte 8  : arm_flag       (int8_t,  0=归零 1=使能)
 *   byte 9  : UU_flag        (uint8_t, 0=USART源 1=USB源)
 *   byte 10 : tool_flag      (uint8_t, 0=吸盘 1=夹爪)
 *   byte 11 : tooluse_flag   (uint8_t, 0=闭合 1=张开)
 *
 * ── byte 12~35: 6 float (little-endian) ──
 *   byte 12~15: chassis param1  (VEL:vx(m/s)  / POS:dx(m))
 *   byte 16~19: chassis param2  (VEL:vy(m/s)  / POS:dy(m))
 *   byte 20~23: chassis param3  (VEL:vw(rad/s)/ POS:dyaw(rad))
 *   byte 24~27: arm_x  (mm)
 *   byte 28~31: arm_y  (mm)
 *   byte 32~35: arm_z  (mm)
 */
/* Current USART payload:
 *   byte 0..11  : existing control bytes
 *   byte 12..14 : climb_enable, climb_step, climb_auto
 *   byte 15..38 : 6 little-endian floats
 */
#define BT_FRAME_DATA_LEN   39U

/* float 数据区在帧内的起始偏移 */
#define BT_FRAME_FLOAT_OFFSET  15U

#define USART_CONTROL_TIMEOUT_MS 300U

extern ParseState BT_Uart10;
extern uint8_t bt_data[BT_FRAME_DATA_LEN];
extern uint8_t data_index;
extern uint8_t checksum;
extern volatile uint8_t bt_parse_ok;
extern uint8_t btReceiveData;

extern UART_HandleTypeDef huart10;

extern volatile uint8_t USB_Task_flag;
extern volatile uint8_t USART_Task_flag;

/* 机械臂控制量 */
extern int8_t arm_flag;
extern float arm_X;
extern float arm_Y;
extern float arm_Z;

extern uint8_t tool_flag;
extern uint8_t tooluse_flag;
extern uint8_t climb_enable_flag;
extern uint8_t climb_step_flag;
extern uint8_t climb_auto_flag;

/* R2 底盘控制器句柄（Control_Task.c 定义） */
extern R2_Move_Ctrl_t g_r2_ctrl_usart;
extern R2_Move_Ctrl_t g_r2_ctrl_usb;
extern R2_Climb_Ctrl_t g_r2_climb_usart;
extern R2_Climb_Ctrl_t g_r2_climb_usb;

/* ── 接收调试观测变量 ────────────────────────────────── */

typedef struct
{
    /* 最新一帧解析值 */
    uint8_t  mode;          /* 底盘模式 0-7 */
    float    chs_p1;        /* chassis param1 */
    float    chs_p2;        /* chassis param2 */
    float    chs_p3;        /* chassis param3 */
    float    arm_x;         /* 机械臂 X (mm) */
    float    arm_y;         /* 机械臂 Y (mm) */
    float    arm_z;         /* 机械臂 Z (mm) */
    uint8_t  arm_flag;      /* 机械臂使能 */
    uint8_t  uu_flag;       /* 控制源 */
    uint8_t  tool_flag;     /* 工具选择 */
    uint8_t  tooluse_flag;  /* 工具状态 */
    uint8_t  climb_enable;
    uint8_t  climb_step;
    uint8_t  climb_auto;

    /* 统计 */
    uint32_t frame_count;   /* 成功接收帧计数 */
    uint8_t  checksum_ok;   /* 最新一帧校验通过 */
    uint8_t  checksum_calc; /* 本地计算的校验和 */
    uint8_t  checksum_recv; /* 接收到的校验和 */
    uint32_t last_frame_tick;
    uint32_t checksum_fail_count;
    uint32_t last_checksum_fail_tick;
    uint8_t  control_timeout;
} CRC_Debug_t;

extern CRC_Debug_t crc_dbg;

void UART10_Receive(uint8_t receiveData);
void Control_SetSource(uint8_t source);
void BT_Data_MAC_Process(float *V_x, float *V_y, float *V_w, int8_t *cmd);
void USART_ControlWatchdog_Check(void);
uint8_t USART_ControlWatchdog_IsTimeout(void);
void R2_Chassis_Process(R2_Move_Ctrl_t *ctrl, uint8_t mode,
                        float p1, float p2, float p3);

#endif
