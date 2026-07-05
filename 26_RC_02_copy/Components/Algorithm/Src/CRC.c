#include "CRC.h"

#include "usart.h"
#include "struct_typedef.h"
#include <string.h>
#include <stddef.h>
#include "R2_move.h"

extern UART_HandleTypeDef huart10;

ParseState BT_Uart10 = STATE_WAIT_HEADER;
uint8_t bt_data[BT_FRAME_DATA_LEN];
uint8_t data_index = 0U;
uint8_t checksum = 0U;
volatile uint8_t bt_parse_ok = 0U;
uint8_t btReceiveData = 0U;

CRC_Debug_t crc_dbg = {0};

volatile uint8_t USB_Task_flag = 0U;
volatile uint8_t USART_Task_flag = 1U;
uint8_t UU_flag = 0U;

uint8_t climb_enable_flag = 0U;
uint8_t climb_step_flag = 0U;
uint8_t climb_auto_flag = 0U;

static volatile uint8_t s_usart_control_timeout = 0U;
static volatile uint32_t s_usart_control_timeout_tick = 0U;

static float read_float_le(const uint8_t *buf);

void Control_SetSource(uint8_t source)
{
    if (source == CONTROL_SOURCE_USB)
    {
        USB_Task_flag = 1U;
        USART_Task_flag = 0U;
    }
    else
    {
        USART_Task_flag = 1U;
        USB_Task_flag = 0U;
    }
}

void UART10_Receive(uint8_t receiveData)
{
    switch (BT_Uart10)
    {
    case STATE_WAIT_HEADER:
        if (receiveData == 0xA5U)
        {
            BT_Uart10 = STATE_RECV_DATA;
            data_index = 0U;
            memset(bt_data, 0, sizeof(bt_data));
        }
        break;

    case STATE_RECV_DATA:
        bt_data[data_index++] = receiveData;
        if (data_index >= BT_FRAME_DATA_LEN)
        {
            BT_Uart10 = STATE_RECV_CHECKSUM;
        }
        break;

    case STATE_RECV_CHECKSUM:
        checksum = receiveData;
        BT_Uart10 = STATE_RECV_TAIL;
        break;

    case STATE_RECV_TAIL:
        if (receiveData == 0x5AU)
        {
            uint8_t calc_checksum = 0U;
            uint8_t i;

            for (i = 0U; i < BT_FRAME_DATA_LEN; i++)
            {
                calc_checksum += bt_data[i];
            }

            crc_dbg.checksum_calc = calc_checksum;
            crc_dbg.checksum_recv = checksum;

            if (calc_checksum == checksum)
            {
                crc_dbg.last_frame_tick = HAL_GetTick();
                crc_dbg.checksum_ok = 1U;
                bt_parse_ok = 1U;
            }
            else
            {
                crc_dbg.checksum_fail_count++;
                crc_dbg.last_checksum_fail_tick = HAL_GetTick();
                crc_dbg.checksum_ok = 0U;
            }
        }

        BT_Uart10 = STATE_WAIT_HEADER;
        data_index = 0U;
        break;

    default:
        BT_Uart10 = STATE_WAIT_HEADER;
        data_index = 0U;
        break;
    }
}

static float read_float_le(const uint8_t *buf)
{
    union { uint8_t b[4]; float f; } u;
    u.b[0] = buf[0];
    u.b[1] = buf[1];
    u.b[2] = buf[2];
    u.b[3] = buf[3];
    return u.f;
}

void R2_Chassis_Process(R2_Move_Ctrl_t *ctrl, uint8_t mode,
                        float p1, float p2, float p3)
{
    R2_MoveMode_t m;

    if (ctrl == NULL)
    {
        return;
    }

    if (mode > 7U)
    {
        if (R2_Move_IsVelMode(ctrl->mode))
        {
            R2_Move_SetVel(ctrl, 0.0f, 0.0f, 0.0f);
        }
        else
        {
            R2_Move_Stop(ctrl);
        }
        return;
    }

    m = (R2_MoveMode_t)mode;
    if (ctrl->mode != m)
    {
        R2_Move_SetMode(ctrl, m);
    }

    if (R2_Move_IsNoYawMode(m))
    {
        if (R2_Move_IsWorldMode(m))
        {
            R2_Move_SetWorldLockYaw(ctrl, p3 * 0.0174533f);
        }
        else
        {
            R2_Move_SetRobotLockYaw(ctrl, p3 * 0.0174533f);
        }

        if (R2_Move_IsVelMode(m))
        {
            R2_Move_SetVel(ctrl, p1, p2, 0.0f);
        }
        else
        {
            (void)R2_Move_SetDist(ctrl, p1, p2, 0.0f);
        }
        return;
    }

    if (R2_Move_IsVelMode(m))
    {
        R2_Move_SetVel(ctrl, p1, p2, p3);
    }
    else
    {
        (void)R2_Move_SetDist(ctrl, p1, p2, p3);
    }
}

void BT_Data_MAC_Process(float *V_x, float *V_y, float *V_w, int8_t *cmd)
{
    uint8_t frame[BT_FRAME_DATA_LEN];
    uint8_t mode;
    float chs_p1;
    float chs_p2;
    float chs_p3;

    (void)cmd;
    (void)V_x;
    (void)V_y;
    (void)V_w;

    if (bt_parse_ok == 0U)
    {
        return;
    }

    __disable_irq();
    memcpy(frame, bt_data, sizeof(frame));
    bt_parse_ok = 0U;
    __enable_irq();

    {
        uint8_t i;
        mode = 0xFFU;
        for (i = 0U; i < 8U; i++)
        {
            if (frame[i] == 1U)
            {
                mode = i;
                break;
            }
        }
    }

    UU_flag = frame[9];
    climb_enable_flag = frame[13];
    climb_step_flag = frame[14];
    climb_auto_flag = frame[15];

    chs_p1 = read_float_le(&frame[BT_FRAME_FLOAT_OFFSET + 0U]);
    chs_p2 = read_float_le(&frame[BT_FRAME_FLOAT_OFFSET + 4U]);
    chs_p3 = read_float_le(&frame[BT_FRAME_FLOAT_OFFSET + 8U]);

    crc_dbg.mode = mode;
    crc_dbg.chs_p1 = chs_p1;
    crc_dbg.chs_p2 = chs_p2;
    crc_dbg.chs_p3 = chs_p3;
    crc_dbg.source_flag = UU_flag;
    crc_dbg.climb_enable = climb_enable_flag;
    crc_dbg.climb_step = climb_step_flag;
    crc_dbg.climb_auto = climb_auto_flag;
    crc_dbg.reserved8 = frame[8];
    crc_dbg.reserved10 = frame[10];
    crc_dbg.reserved11 = frame[11];
    crc_dbg.reserved12 = frame[12];
    crc_dbg.frame_count++;
    crc_dbg.last_frame_tick = HAL_GetTick();
    crc_dbg.checksum_ok = 1U;
    crc_dbg.control_timeout = 0U;
    s_usart_control_timeout = 0U;

    Control_SetSource((UU_flag == 1U) ? CONTROL_SOURCE_USB : CONTROL_SOURCE_USART);

    if (USART_Task_flag == 1U)
    {
        R2_Chassis_Process(&g_r2_ctrl_usart, mode, chs_p1, chs_p2, chs_p3);
        R2_Climb_SetInput(&g_r2_climb_usart,
                          climb_enable_flag,
                          climb_step_flag,
                          climb_auto_flag);
    }
    else
    {
        R2_Move_Stop(&g_r2_ctrl_usart);
        R2_Climb_Stop(&g_r2_climb_usart);
    }
}

void USART_ControlWatchdog_Check(void)
{
    uint32_t now_tick;
    uint32_t last_tick;
    uint8_t need_stop = 0U;

    now_tick = HAL_GetTick();

    __disable_irq();
    last_tick = crc_dbg.last_frame_tick;
    if ((USART_Task_flag != 0U) &&
        (last_tick != 0U) &&
        (s_usart_control_timeout == 0U) &&
        ((now_tick - last_tick) > USART_CONTROL_TIMEOUT_MS))
    {
        s_usart_control_timeout = 1U;
        s_usart_control_timeout_tick = now_tick;
        crc_dbg.control_timeout = 1U;
        need_stop = 1U;
    }
    __enable_irq();

    if (need_stop != 0U)
    {
        R2_Move_Stop(&g_r2_ctrl_usart);
        R2_Climb_Stop(&g_r2_climb_usart);
    }
}

uint8_t USART_ControlWatchdog_IsTimeout(void)
{
    uint8_t timeout;

    __disable_irq();
    timeout = s_usart_control_timeout;
    __enable_irq();

    return timeout;
}
