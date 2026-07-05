#ifndef __CRC_H__
#define __CRC_H__

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include "R2_move.h"
#include "R2_climb.h"

#define CONTROL_SOURCE_USART 0U
#define CONTROL_SOURCE_USB   1U
#define CONTROL_SOURCE_NONE  2U

typedef enum
{
    STATE_WAIT_HEADER = 0,
    STATE_RECV_DATA,
    STATE_RECV_CHECKSUM,
    STATE_RECV_TAIL
} ParseState;

/*
 * Legacy UART10 frame length is kept for compatibility with the remote.
 *
 * byte 0..7   : chassis mode one-hot, all zero = stop
 * byte 8      : reserved
 * byte 9      : source flag, 0=USART, 1=USB
 * byte 10..12 : reserved
 * byte 13     : climb_enable
 * byte 14     : climb_step
 * byte 15     : climb_auto
 * byte 16..27 : 3 little-endian floats for chassis
 * byte 28..39 : reserved for the new arm control chain
 */
#define BT_FRAME_DATA_LEN      40U
#define BT_FRAME_FLOAT_OFFSET  16U

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

extern uint8_t climb_enable_flag;
extern uint8_t climb_step_flag;
extern uint8_t climb_auto_flag;

extern R2_Move_Ctrl_t g_r2_ctrl_usart;
extern R2_Move_Ctrl_t g_r2_ctrl_usb;
extern R2_Climb_Ctrl_t g_r2_climb_usart;
extern R2_Climb_Ctrl_t g_r2_climb_usb;

typedef struct
{
    uint8_t mode;
    float chs_p1;
    float chs_p2;
    float chs_p3;
    uint8_t source_flag;
    uint8_t climb_enable;
    uint8_t climb_step;
    uint8_t climb_auto;
    uint8_t reserved8;
    uint8_t reserved10;
    uint8_t reserved11;
    uint8_t reserved12;

    uint32_t frame_count;
    uint8_t checksum_ok;
    uint8_t checksum_calc;
    uint8_t checksum_recv;
    uint32_t last_frame_tick;
    uint32_t checksum_fail_count;
    uint32_t last_checksum_fail_tick;
    uint8_t control_timeout;
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
