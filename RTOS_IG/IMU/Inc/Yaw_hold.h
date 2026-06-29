#pragma once
#ifndef __YAW_HOLD_H
#define __YAW_HOLD_H

#include "include.h"

#define YAW_HOLD_MOVE_DEADBAND   0.05f    // 位置偏差死区，小于该值不执行调节
#define YAW_HOLD_OMEGA_DEADBAND  0.05f    // 角速度偏差死区，小于该值不执行调节
#define YAW_HOLD_KP              0.6f     // 航向保持PID 比例系数
#define YAW_HOLD_KI              0.001f    // 航向保持PID 积分系数
#define YAW_HOLD_KD              0.03f     // 航向保持PID 微分系数
#define YAW_HOLD_I_MAX           100.0f    // 航向保持积分限幅
#define YAW_HOLD_OMEGA_MAX       0.6f     // 航向保持输出最大角速度限制 m/s

typedef struct
{
    float target_yaw_deg;  // 目标航向角
    uint8_t active;        // 航向保持功能使能标志，1启用/0关闭
    float i_term;          // PID积分项缓存值
} YawHold_t;
       
extern YawHold_t g_yaw_hold;

void YawHold_Init(void);
float YawHold_Update(float vx_cmd, float vy_cmd, float omega_user, float yaw_now_deg, float gyro_z_dps, float dt);

#endif // !__YAW_HOLD_H

