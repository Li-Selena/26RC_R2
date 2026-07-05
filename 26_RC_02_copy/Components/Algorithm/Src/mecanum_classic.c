#include "mecanum_classic.h"
//#include "pid_user.h"

ChassisVel_t total_vel = {0,0,0};
WheelSpeed_t total_speed = {0,0,0,0};

MecanumParam_t mecParam = {
    .wheel_radius = 0.076f,       // 轮子半径：150mm = 0.150m
    
    .L = 0.338f,                  // 前后轴到中心的距离：380mm / 2 = 190mm = 0.190m
    .W = 0.375f,                  // 左右半轮距：(!!! 请根据你实际的长方形底盘轮距修改此值 !!!)
    
    .max_wheel_speed = MEC_DEBUG_WHEEL_LIMIT_MPS  /* debug limit; physical max is MEC_WHEEL_PHYSICAL_MAX_MPS */
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
 * @brief 麦克纳姆轮底盘运动学解算（长方形）
 */
void Mecanum_Calc(
    const ChassisVel_t *chassis,
    const MecanumParam_t *param,
    WheelSpeed_t *wheel)
{
    /* Project robot frame: +X is right, +Y is physical front, +yaw is CCW. */
    float chassis_vx = MEC_RIGHT_SIGN * chassis->vx;
    float chassis_vy = MEC_FORWARD_SIGN * chassis->vy;
    float chassis_vw = chassis->vw;  // +vw follows the project yaw convention: CCW is positive.

    // 1. 由于长方形底盘对称，旋转切向速度的系数 4 个轮子完全相同
    float k = (param->L + param->W) * chassis_vw;

    // 2. 合成 4 个轮子的目标物理线速度 (单位: m/s)
    float v_fl = chassis_vy + chassis_vx - k;
    float v_fr = chassis_vy - chassis_vx + k;
    float v_bl = chassis_vy - chassis_vx - k;
    float v_br = chassis_vy + chassis_vx + k;

    // 3. 找出当前计算出的最大轮线速度绝对值
    float max_val = 0.0f;
    max_val = max_f(max_val, abs_f(v_fl));
    max_val = max_f(max_val, abs_f(v_fr));
    max_val = max_f(max_val, abs_f(v_bl));
    max_val = max_f(max_val, abs_f(v_br));

    // 4. 速度归一化限速保护
    // 如果最大的轮子线速度超出了最大限速，则等比例缩小所有轮子的速度，确保行驶轨迹不走歪
    if (max_val > param->max_wheel_speed)
    {
        float scale = param->max_wheel_speed / max_val;
        v_fl *= scale;
        v_fr *= scale;
        v_bl *= scale;
        v_br *= scale;
    }

    // 5. 最终输出赋值
    // -------------------------------------------------------------------------
    // 【方案 A】直接输出线速度 (m/s) —— [当前默认激活]
    // -------------------------------------------------------------------------
    wheel->fl = v_fl;
    wheel->fr = v_fr;
    wheel->bl = v_bl;
    wheel->br = v_br;

    // -------------------------------------------------------------------------
    // 【方案 B】输出角速度 (rad/s)
    // -------------------------------------------------------------------------
    /*
    float r = param->wheel_radius;
    wheel->fl = v_fl / r;
    wheel->fr = v_fr / r;
    wheel->bl = v_bl / r;
    wheel->br = v_br / r;
    */
}
