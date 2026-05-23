#include "mecanum_classic.h"
//#include "pid_user.h"

ChassisVel_t total_vel = {0,0,0};
WheelSpeed_t total_speed = {0,0,0,0};
TrapezoidMecanumParam_t mecParam = {
    .wheel_radius = 0.150f,       // 轮子半径：150mm = 0.150m
    
    .L_front = 0.190f,            // 前轴到中心的距离：380mm / 2 = 190mm = 0.190m
    .L_rear = 0.190f,             // 后轴到中心的距离：380mm / 2 = 190mm = 0.190m
    
    .W_front = 0.26752f,          // 前轮半轮距：535.04mm / 2 = 267.52mm = 0.26752m
    .W_rear = 0.169725f,          // 后轮半轮距：339.45mm / 2 = 169.725mm = 0.169725m
    
    .max_wheel_speed = 523.6f     // 最大轮速：5000 rpm 转换为 rad/s 约等于 523.6
};

static float abs_f(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float max_f(float a, float b)
{
    return (a > b) ? a : b;
}

/**
 * @brief �����ķ�����˶�ѧ
 */


void Mecanum_Calc(
    const ChassisVel_t *chassis,
    const TrapezoidMecanumParam_t *param,
    WheelSpeed_t *wheel)
{
    float r = param->wheel_radius; 

    float chassis_vx = chassis->vx;  
    float chassis_vy = chassis->vy;  
    float chassis_vw = -chassis->vw; // 保持原有坐标系习惯

    // 1. 分别计算 4 个轮子各自的旋转系数
    float k_fl = (param->L_front + param->W_front) * chassis_vw;
    float k_fr = (param->L_front + param->W_front) * chassis_vw;
    float k_bl = (param->L_rear  + param->W_rear)  * chassis_vw;
    float k_br = (param->L_rear  + param->W_rear)  * chassis_vw;

    // 2. 结合速度解算（注意：因为轮子安装姿态未变，vx 和 vy 的正负号与原版完全一致）
    wheel->fl = (chassis_vy + chassis_vx - k_fl) / r;  // 左前
    wheel->fr = (chassis_vy - chassis_vx + k_fr) / r;  // 右前
    wheel->bl = (chassis_vy - chassis_vx - k_bl) / r;  // 左后
    wheel->br = (chassis_vy + chassis_vx + k_br) / r;  // 右后

    // 3. 速度归一化限速保护（保持原样）
    float max_val = 0.0f;
    max_val = max_f(max_val, abs_f(wheel->fl));
    max_val = max_f(max_val, abs_f(wheel->fr));
    max_val = max_f(max_val, abs_f(wheel->bl));
    max_val = max_f(max_val, abs_f(wheel->br));

    if (max_val > param->max_wheel_speed)
    {
        float scale = param->max_wheel_speed / max_val;
        wheel->fl *= scale;
        wheel->fr *= scale;
        wheel->bl *= scale;
        wheel->br *= scale;
    }
}