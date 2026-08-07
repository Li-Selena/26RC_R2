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

typedef enum
{
    R2_TASK_FLOW_NONE = 0,
    R2_TASK_FLOW_S1_UP_V1 = 1,
    R2_TASK_FLOW_S1_DOWN_V1 = 2,
    R2_TASK_FLOW_RESERVED_3 = 3,
    R2_TASK_FLOW_S1_UP_S2_DOWN_V1 = 4,
    R2_TASK_FLOW_S1_DOWN_S2_UP_V1 = 5,
    R2_TASK_FLOW_S1_DOWN_S2_DOWN_V1 = 6,
    R2_TASK_FLOW_WEAPON_GRAB_V1 = 7,
    R2_TASK_FLOW_THROW_BLOCK_V1 = 8,
    R2_TASK_FLOW_WEAPON_DOCK_TEST_V1 = 9,
    R2_TASK_FLOW_S1_UP_V2 = R2_TASK_FLOW_S1_UP_V1,
    R2_TASK_FLOW_S1_DOWN_V2 = R2_TASK_FLOW_S1_DOWN_V1,
} R2_TaskFlowId_t;

typedef enum
{
    R2_TASK_FLOW_STATE_IDLE = 0,
    R2_TASK_FLOW_STATE_ISSUE = 1,
    R2_TASK_FLOW_STATE_WAIT = 2,
    R2_TASK_FLOW_STATE_DONE = 3,
    R2_TASK_FLOW_STATE_ERROR = 4,
} R2_TaskFlowState_t;

typedef enum
{
    R2_TASK_FLOW_ERR_NONE = 0,
    R2_TASK_FLOW_ERR_BAD_FLOW = 1,
    R2_TASK_FLOW_ERR_ARM = 2,
    R2_TASK_FLOW_ERR_CLIMB = 3,
    R2_TASK_FLOW_ERR_POSTURE = 4,
    R2_TASK_FLOW_ERR_PRECONDITION = 5,
} R2_TaskFlowError_t;

typedef enum
{
    R2_TASK_FLOW_S1_END_NONE = 0,
    R2_TASK_FLOW_S1_END_UP = 1,
    R2_TASK_FLOW_S1_END_DOWN = 2,
} R2_TaskFlowS1End_t;

typedef struct
{
    uint8_t flow_id;
    uint8_t state;
    uint8_t error;
    uint8_t current_op;
    uint16_t entry_index;
    uint16_t entry_count;
    uint16_t repeat_index;
    uint16_t repeat_count;
    uint8_t completed_s1_end;
    uint8_t flow_arg;
    uint8_t host_checkpoint_pending;
    uint8_t host_checkpoint_ack;
    uint16_t arm_reached_ms;
    uint32_t step_start_ms;
    uint32_t last_update_ms;
    uint32_t completed_steps;
} R2_TaskFlow_Ctrl_t;

extern R2_Move_Ctrl_t g_r2_ctrl_usart;
extern R2_Move_Ctrl_t g_r2_ctrl_usb;
extern R2_Climb_Ctrl_t g_r2_climb_usart;
extern R2_Climb_Ctrl_t g_r2_climb_usb;
extern R2_TaskFlow_Ctrl_t g_r2_task_flow_usb;
extern R2_DebugOdom_t g_r2_debug_odom;
extern uint32_t g_r2_tick_ms;
extern int32_t g_r2_last_enc[CHASSIS_MOTOR_COUNT];
extern uint8_t g_r2_enc_inited;

void Mecanum_task_USB(ChassisVel_t *chassis_user,
                      MecanumParam_t *param_user,
                      WheelSpeed_t *speed_user);
void R2_TaskFlow_Init(R2_TaskFlow_Ctrl_t *ctrl);
void R2_TaskFlow_Stop(R2_TaskFlow_Ctrl_t *ctrl);
void R2_TaskFlow_Request(R2_TaskFlow_Ctrl_t *ctrl,
                         uint8_t flow_id,
                         uint32_t now_ms);
void R2_TaskFlow_RequestWithArg(R2_TaskFlow_Ctrl_t *ctrl,
                                uint8_t flow_id,
                                uint8_t flow_arg,
                                uint32_t now_ms);
void R2_TaskFlow_ConfirmHostCheckpoint(R2_TaskFlow_Ctrl_t *ctrl,
                                       uint8_t checkpoint);
uint8_t R2_TaskFlow_IsActive(const R2_TaskFlow_Ctrl_t *ctrl);
void control_tim1mscallback(void);

#endif
