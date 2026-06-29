#pragma once
#ifndef SWERVE_CTRL_H
#define SWERVE_CTRL_H
/*0号轮(右前 FR) : X > 0, Y < 0
   • 1号轮(左前 FL) : X > 0, Y > 0
   • 2号轮(左后 BL) : X < 0, Y > 0
   • 3号轮(右后 BR) : X < 0, Y < 0 (注：旋转正方向为逆时针)
   */

#include "include.h" 
#include <math.h>

#define PI 3.14159265358979323846f

#define CHASSIS_L  0.440f  // 前后轮距的一半 (X轴方向)
#define CHASSIS_W  0.445f  // 左右轮距的一半 (Y轴方向)

// 速度死区，防止摇杆回中时的抖动
#define DEAD_ZONE  0.05f 
#define SWERVE_FLIP_MARGIN_DEG   8.0f   // 可调：8~15 都行，越大越稳但响应略慢

typedef struct
{
    // --- 输入/状态 ---
    fp32 wheel_x_offset; // 轮子相对于中心的 X 坐标
    fp32 wheel_y_offset; // 轮子相对于中心的 Y 坐标
    fp32 last_target_angle; // 上一次的目标角度
    /* 状态反馈 */
    fp32 current_angle_deg; // 真实的角度    

    fp32 target_speed;     // 3508 目标速度
    fp32 target_angle_deg; // 6020 目标角度 (度)
    fp32 target_encoder;
    
    uint8_t steering_flipped;   // 0:不翻转  1:180°翻转解
} SwerveModule;


extern SwerveModule swerve_modules[4];

void Swerve_Init(void);
void Swerve_Update(fp32 vx, fp32 vy, fp32 omega, fp32 yaw_rad);
void Swerve_Execute(float dt);
void Chassis_Control(float vx_cmd, float vy_cmd, float omega_user, float dt);
void Swerve_Stop(void);

#endif
