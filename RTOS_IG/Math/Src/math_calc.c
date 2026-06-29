#include "math_calc.h"
#include <math.h>
/**
 * @brief  弧度转换为角度
 * @param  rad 弧度值
 * @retval deg 角度值
 * @attention
 */
fp32 rad2deg(fp32 rad)
{
    fp32 deg;
    deg = (fp32)(rad * (180.0 / PI));
    return deg;
}

//角度转弧度
fp32 deg2rad(fp32 angle_deg)
{
    return angle_deg * (PI / 180.0f);
}

//角度换算编码值
fp32 Angle_To_Encoder(fp64 theta)
{
//    while (theta < 0.0)   theta += 360.0;
//    while (theta >= 360.0) theta -= 360.0;
//    return (fp32)(theta / 360.0 * 8191.0);   
   if (theta < 0){theta += 360.0;}

   fp32 encoder = 0;
   encoder = (theta / 360.0) * 8191.0;
   return encoder;
}

//编码值换算角度(弧度制)
fp32 Encoder_To_Angle(fp32 encoder)
{
    fp32 angle = 0.0;
    angle = (encoder / 8191.0) * 2.0 * PI;
    return angle;
}

/**
 * @brief  舵电机角度计算，最后输出舵电机要转过的角度
 * @param  vx 每个轮子的x方向速度
 * @param  vy 每个轮子的y方向速度
 * @retval angle 角度值（0°-360°）
 * @attention 此函数是将角度规范在-180°-180°
 */
fp32 Angle_Calc(fp32 vx, fp32 vy)
{
   const fp32 EPS = 1e-6f;

    if(fabsf(vx) < EPS && fabsf(vy) < EPS)
  {
        return 0.0f;
    }
    /* 返回范围 [-180, 180] */
    fp32 theta = atan2f(vy, vx);
    fp32 deg = rad2deg(theta);   

    /* 将 -180 視作 +180 */
    if (deg <= -180.0f + EPS) deg = 180.0f;

    return deg;
}

/*符号判断*/
float sign(float x)
{
    if (x > 0) return 1.0f;
    if (x < 0) return -1.0f;
    return 0.0f;
}

fp32 wrap_diff_deg(fp32 target, fp32 current)
{
    fp32 diff = target - current;
    if (diff > 180.0f) diff -= 360.0f;
    else if (diff < -180.0f) diff += 360.0f;
    return diff;
}


/* 将角度规范到 [-180,180] */
 fp32 normalize_deg(fp32 deg)
{
    /* 归一化到 (-180, 180] */
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

/*轮子线速度转为电机转速rpm*/
 float linear_speed_to_motor_rpm(float wheel_linear_speed)
 {
     float wheel_rev_s = wheel_linear_speed / (2 * PI * 0.062f);
     float motor_rev_s = wheel_rev_s * 19;
     float motor_rpm = motor_rev_s * 60;

     return motor_rpm;
 }
 
// /*轮子线速度转为电机ticks/s*/
// float linear_speed_to_encoder_ticks_per_s(float wheel_linear_speed_m_s)
// {
//    float motor_rev_s = (wheel_linear_speed_m_s / (2.0f* PI *0.062f))*19.0f;
//    return motor_rev_s * (float)(1024*4);
// }
  
///*限制步长*/
// float approach_float(float current, float target, int32_t max_delta)
//{   
//    if (max_delta <= 0) return target; 
//    float diff = target - current;
//    if (diff > (float)max_delta)  return current + (float)max_delta;
//    if (diff < -(float)max_delta) return current - (float)max_delta;
//    return target;
//}
static  int32_t encoder_wrap_diff(int32_t target, int32_t current)
{
    int32_t diff = target - current;
    if (diff > 4096)       diff -= 4096;
    else if (diff < -8192) diff += 8192;
    return diff;
}

// 把目标换成“离当前最近”的等价目标
int32_t encoder_wrap_target(int32_t current, int32_t target)
{
    return current + encoder_wrap_diff(target, current);
}

