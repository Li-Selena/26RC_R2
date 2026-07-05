#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include "bsp_mcu.h"
#include "include.h"
#include "robotarm_mixed.h"
#include "bsp_tick.h"
#include "mecanum_classic.h"
#include "chassis_move.h"
#include "CRC.h"
#include "bsp_uart.h"
#include "usbd_cdc_if.h"
#include "R2_move.h"
#include "R2_climb.h"
#include "INS_Task.h"
#include <stdint.h>

typedef struct
{
    uint32_t tick_ms;
    uint8_t enc_inited;
    uint8_t imu_yaw_valid;
    uint8_t active_source;
    uint8_t reserved;

    int32_t current_enc[CHASSIS_MOTOR_COUNT];
    int32_t last_enc[CHASSIS_MOTOR_COUNT];
    int32_t enc_delta[CHASSIS_MOTOR_COUNT];
    float wheel_delta_m[CHASSIS_MOTOR_COUNT];

    float robot_dx_m;
    float robot_dy_m;
    float robot_dyaw_rad;
    float odom_vx_mps;
    float odom_vy_mps;
    float odom_wz_radps;

    float imu_yaw_rad;
    float active_odom_x;
    float active_odom_y;
    float active_odom_yaw;
} R2_DebugOdom_t;

extern R2_Move_Ctrl_t g_r2_ctrl_usart;
extern R2_Move_Ctrl_t g_r2_ctrl_usb;
extern R2_Climb_Ctrl_t g_r2_climb_usart;
extern R2_Climb_Ctrl_t g_r2_climb_usb;
extern R2_DebugOdom_t g_r2_debug_odom;
extern uint32_t g_r2_tick_ms;
extern int32_t g_r2_last_enc[CHASSIS_MOTOR_COUNT];
extern uint8_t g_r2_enc_inited;

void Mecanum_task_USB(ChassisVel_t *chassis_user,
                      MecanumParam_t *param_user,
                      WheelSpeed_t *speed_user);
void control_tim1mscallback(void);

#endif
