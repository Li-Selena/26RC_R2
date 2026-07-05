#ifndef ROBOTARM_DM_MOTOR_H
#define ROBOTARM_DM_MOTOR_H

#include "bsp_fdcan.h"
#include "fdcan.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MIT_MODE   0x000
#define POS_MODE   0x100
#define SPEED_MODE 0x200

#define DM8006_P_MIN  -12.5f
#define DM8006_P_MAX   12.5f
#define DM8006_V_MIN  -45.0f
#define DM8006_V_MAX   45.0f
#define DM8006_KP_MIN   0.0f
#define DM8006_KP_MAX 500.0f
#define DM8006_KD_MIN   0.0f
#define DM8006_KD_MAX   5.0f
#define DM8006_T_MIN  -20.0f
#define DM8006_T_MAX   20.0f

#define DM6215_P_MIN  -12.5f
#define DM6215_P_MAX   12.5f
#define DM6215_V_MIN  -45.0f
#define DM6215_V_MAX   45.0f
#define DM6215_KP_MIN   0.0f
#define DM6215_KP_MAX 500.0f
#define DM6215_KD_MIN   0.0f
#define DM6215_KD_MAX   5.0f
#define DM6215_T_MIN  -10.0f
#define DM6215_T_MAX   10.0f

typedef struct
{
    uint16_t id;
    uint16_t state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;
} motor_fbpara_t;

typedef struct
{
    uint16_t mode;
    motor_fbpara_t para;
} Joint_Motor_t;

typedef struct
{
    uint16_t mode;
    float wheel_T;
    motor_fbpara_t para;
} Wheel_Motor_t;

float uint_to_float(int x_int, float x_min, float x_max, int bits);
uint8_t dm8006_enter_motor_mode(FDCAN_HandleTypeDef *hcan, uint16_t motor_id);
uint8_t dm8006_exit_motor_mode(FDCAN_HandleTypeDef *hcan, uint16_t motor_id);
uint8_t dm8006_set_zero_position(FDCAN_HandleTypeDef *hcan, uint16_t motor_id);
uint8_t dm8006_enter_pos_speed_mode(FDCAN_HandleTypeDef *hcan, uint16_t motor_id);
uint8_t dm8006_exit_pos_speed_mode(FDCAN_HandleTypeDef *hcan, uint16_t motor_id);
uint8_t dm8006_set_zero_position_pos_speed(FDCAN_HandleTypeDef *hcan, uint16_t motor_id);
uint8_t dm8006_send_mit_command(FDCAN_HandleTypeDef *hcan, uint16_t motor_id,
                                float pos, float vel, float kp, float kd, float tor);
uint8_t dm8006_send_pos_speed_command(FDCAN_HandleTypeDef *hcan, uint16_t motor_id,
                                      float pos, float vel);
void dm8006_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len);
void dm6215_fbdata(Wheel_Motor_t *motor, uint8_t *rx_data, uint32_t data_len);

#ifdef __cplusplus
}
#endif

#endif
