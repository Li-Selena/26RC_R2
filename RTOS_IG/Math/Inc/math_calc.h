#pragma once
#ifndef __MATH_CALC_H
#define __MATH_CALC_H
#include "include.h"

#define PI 3.14159265358979323846f

fp32 rad2deg(fp32 rad);
fp32 deg2rad(fp32 angle_deg);
fp32 Angle_To_Encoder(fp64 theta);
fp32 Encoder_To_Angle(fp32 encoder);
fp32 Angle_Calc(fp32 vx, fp32 vy);
 
 fp32 wrap_diff_deg(fp32 target, fp32 current);
 fp32 normalize_deg(fp32 deg);
 float sign(float x);

 float linear_speed_to_motor_rpm(float wheel_linear_speed);
// float linear_speed_to_encoder_ticks_per_s(float wheel_linear_speed_m_s);
// float approach_float(float current, float target, int32_t max_delta);
int32_t encoder_wrap_target(int32_t current, int32_t target);

#endif // !__MATH_CALC_H
