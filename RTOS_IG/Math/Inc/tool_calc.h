#pragma once
#ifndef __TOOL_CALC_H
#define __TOOL_CALC_H

/* --- 宏定义补充 --- */
#define RC_MAX_VALUE       660.0f  // 遥控器最大读数
#define RC_DEADZONE_RATIO  0.07f   // 5% 的死区 (大约对应整数的33)
#define RC_EXPO_RATIO      0.3f    // 曲线强度 (0.0是线性，1.0是纯三次方，建议0.2~0.4)
#define RC_EXPO_RATIO_OMEGA   0.5f 

#include "include.h"

typedef struct {
    float prev_filtered_norm_val; 
} rc_channel_filter_state_t;

// PID配置结构体
typedef struct
{
    // 梯形规划终点吸附窗口
    float arrival_window;
    // 静摩擦补偿启动阈值
    float static_friction_threshold;
    // 静摩擦补偿电流值
    float static_friction_current;
    // 可变积分缩放系数
    float vint_scale_factor;
    // 可变积分scale上限
    float vint_max_scale;
    // D项低通滤波系数
    float d_filter_alpha;
} PID_ConfigTypeDef;

#define PID_DEFAULT_CONFIG \
{           \
     10.0f, \
     3.0f,  \
     20.0f, \
     10.0f, \
     10.0f, \
     0.5f   \
}
extern PID_ConfigTypeDef pid_config;

// 为每个通道创建一个静态实例，这样它们的状态会被保持
static rc_channel_filter_state_t rc_channel_states[3] = {0}; // 数组大小根据你实际用的通道数来定

float apply_deadzone(float v, float dz);
float apply_expo(float v, float expo);
float rc_process_channel(int16_t raw_input, float max_phys_val, float set_expo, float lpf_alpha, uint8_t channel_idx);

fp32 Calc_Steering_Penalty(fp32 err_deg);
fp32 Get_Max_Steering_Error(void);

float circular_error(float a, float b, float half_range, float full_range);
void update_trapezoidal_planner(pid_type_def *pid, float set, float dt,float half_range, float full_range);
float compute_pid_output(pid_type_def *pid, float ref, float dt,float half_range, float full_range) ;


#endif // !__TOOL_CALC_H
