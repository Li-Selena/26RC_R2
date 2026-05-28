#include "mecanum_classic.h"
//#include "pid_user.h"

ChassisVel_t total_vel = {0,0,0};
WheelSpeed_t total_speed = {0,0,0,0};
TrapezoidMecanumParam_t mecParam = {
    .wheel_radius = 0.075f,       // 轮子半径：150mm = 0.150m
    
    .L_front = 0.190f,            // 前轴到中心的距离：380mm / 2 = 190mm = 0.190m
    .L_rear = 0.190f,             // 后轴到中心的距离：380mm / 2 = 190mm = 0.190m
    
    .W_front = 0.26752f,          // 前轮半轮距：535.04mm / 2 = 267.52mm = 0.26752m
    .W_rear = 0.169725f,          // 后轮半轮距：339.45mm / 2 = 169.725mm = 0.169725m
    
    .max_wheel_speed = 0.68f     // 最大轮速：5000 rpm 转换为 rad/s 约等于 523.6
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
    float chassis_vx = chassis->vx;  
    float chassis_vy = chassis->vy;  
    float chassis_vw = -chassis->vw; // 保持原有坐标系习惯

    // 1. 分别计算 4 个轮子各自的旋转切向速度 (单位: m/s)
    float k_fl = (param->L_front + param->W_front) * chassis_vw;
    float k_fr = (param->L_front + param->W_front) * chassis_vw;
    float k_bl = (param->L_rear  + param->W_rear)  * chassis_vw;
    float k_br = (param->L_rear  + param->W_rear)  * chassis_vw;

    // 2. 合成 4 个轮子的目标物理线速度 (单位: m/s)
    // 【注意】这里先不除以 r，确保所有物理量都在 m/s 维度，避免单位混乱
    float v_fl = chassis_vy + chassis_vx - k_fl;
    float v_fr = chassis_vy - chassis_vx + k_fr;
    float v_bl = chassis_vy - chassis_vx - k_bl;
    float v_br = chassis_vy + chassis_vx + k_br;

    // 3. 找出当前计算出的最大轮线速度绝对值
    float max_val = 0.0f;
    max_val = max_f(max_val, abs_f(v_fl));
    max_val = max_f(max_val, abs_f(v_fr));
    max_val = max_f(max_val, abs_f(v_bl));
    max_val = max_f(max_val, abs_f(v_br));

    // 4. 速度归一化限速保护
    // 如果最大的轮子线速度超出了 3.68 m/s，则等比例缩小所有轮子的速度，确保行驶轨迹不走歪
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
    // 此时 wheel->fl 等变量内的数值将严格约束在 3.68 以内。
    // -------------------------------------------------------------------------
    wheel->fl = v_fl;
    wheel->fr = v_fr;
    wheel->bl = v_bl;
    wheel->br = v_br;

    // -------------------------------------------------------------------------
    // 【方案 B】输出角速度 (rad/s) —— [如果你的后续PID需要rad/s，请取消下方注释并注释掉上方方案A]
    // 约束完 3.68 m/s 后再除以 r。此时数值最大会达到 49.06，但小车物理速度依然被完美限死在 3.68m/s。
    // -------------------------------------------------------------------------
    /*
    float r = param->wheel_radius;
    wheel->fl = v_fl / r;
    wheel->fr = v_fr / r;
    wheel->bl = v_bl / r;
    wheel->br = v_br / r;
    */
}
