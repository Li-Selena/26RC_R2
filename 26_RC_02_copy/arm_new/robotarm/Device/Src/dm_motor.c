#include "dm_motor.h"

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static uint32_t dm_float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x - x_min;

    if (x < x_min)
    {
        offset = 0.0f;
    }
    else if (x > x_max)
    {
        offset = span;
    }

    return (uint32_t)((offset * ((float)((1 << bits) - 1))) / span);
}

uint8_t dm8006_enter_motor_mode(FDCAN_HandleTypeDef *hcan, uint16_t motor_id)
{
    uint8_t cmd[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    return canx_send_data(hcan, (uint16_t)(motor_id | MIT_MODE), cmd, 8U);
}

uint8_t dm8006_exit_motor_mode(FDCAN_HandleTypeDef *hcan, uint16_t motor_id)
{
    uint8_t cmd[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    return canx_send_data(hcan, (uint16_t)(motor_id | MIT_MODE), cmd, 8U);
}

uint8_t dm8006_set_zero_position(FDCAN_HandleTypeDef *hcan, uint16_t motor_id)
{
    uint8_t cmd[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    return canx_send_data(hcan, (uint16_t)(motor_id | MIT_MODE), cmd, 8U);
}

uint8_t dm8006_send_mit_command(FDCAN_HandleTypeDef *hcan, uint16_t motor_id,
                                float pos, float vel, float kp, float kd, float tor)
{
    uint32_t p_int = dm_float_to_uint(pos, DM8006_P_MIN, DM8006_P_MAX, 16);
    uint32_t v_int = dm_float_to_uint(vel, DM8006_V_MIN, DM8006_V_MAX, 12);
    uint32_t kp_int = dm_float_to_uint(kp, DM8006_KP_MIN, DM8006_KP_MAX, 12);
    uint32_t kd_int = dm_float_to_uint(kd, DM8006_KD_MIN, DM8006_KD_MAX, 12);
    uint32_t t_int = dm_float_to_uint(tor, DM8006_T_MIN, DM8006_T_MAX, 12);
    uint8_t cmd[8];

    cmd[0] = (uint8_t)(p_int >> 8);
    cmd[1] = (uint8_t)(p_int & 0xFF);
    cmd[2] = (uint8_t)(v_int >> 4);
    cmd[3] = (uint8_t)(((v_int & 0x0F) << 4) | (kp_int >> 8));
    cmd[4] = (uint8_t)(kp_int & 0xFF);
    cmd[5] = (uint8_t)(kd_int >> 4);
    cmd[6] = (uint8_t)(((kd_int & 0x0F) << 4) | (t_int >> 8));
    cmd[7] = (uint8_t)(t_int & 0xFF);

    return canx_send_data(hcan, (uint16_t)(motor_id | MIT_MODE), cmd, 8U);
}

void dm8006_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len)
{
    if (data_len == FDCAN_DLC_BYTES_8)
    {
        motor->para.id = (rx_data[0]) & 0x0F;
        motor->para.state = (rx_data[0]) >> 4;
        motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
        motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
        motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
        motor->para.pos = uint_to_float(motor->para.p_int, DM8006_P_MIN, DM8006_P_MAX, 16);
        motor->para.vel = uint_to_float(motor->para.v_int, DM8006_V_MIN, DM8006_V_MAX, 12);
        motor->para.tor = uint_to_float(motor->para.t_int, DM8006_T_MIN, DM8006_T_MAX, 12);
        motor->para.Tmos = (float)(rx_data[6]);
        motor->para.Tcoil = (float)(rx_data[7]);
    }
}

void dm6215_fbdata(Wheel_Motor_t *motor, uint8_t *rx_data, uint32_t data_len)
{
    if (data_len == FDCAN_DLC_BYTES_8)
    {
        motor->para.id = (rx_data[0]) & 0x0F;
        motor->para.state = (rx_data[0]) >> 4;
        motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
        motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
        motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
        motor->para.pos = uint_to_float(motor->para.p_int, DM6215_P_MIN, DM6215_P_MAX, 16);
        motor->para.vel = uint_to_float(motor->para.v_int, DM6215_V_MIN, DM6215_V_MAX, 12);
        motor->para.tor = uint_to_float(motor->para.t_int, DM6215_T_MIN, DM6215_T_MAX, 12);
        motor->para.Tmos = (float)(rx_data[6]);
        motor->para.Tcoil = (float)(rx_data[7]);
    }
}
