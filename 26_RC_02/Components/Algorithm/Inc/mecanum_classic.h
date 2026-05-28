#ifndef MECANUM_KINEMATICS_H
#define MECANUM_KINEMATICS_H

#define MEC_R  0.075f   // m 轮子半径
#define RPM_TO_MS  (2.0f * 3.1415926f * MEC_R / 60.0f) // rpm 转换为 m/s 的系数
#define MOTOR_IN2OUT (187.0f / 3591.0f) // 电机转速到轮子转速的转换系数（根据实际测量）

#define MEC_REMOTE_VX_MIN_RPM      -9000.0f * MOTOR_IN2OUT * RPM_TO_MS   // 3.68m/s 
#define MEC_REMOTE_VX_MAX_RPM       9000.0f * MOTOR_IN2OUT * RPM_TO_MS   // m/s
#define MEC_REMOTE_VY_MIN_RPM      -9000.0f * MOTOR_IN2OUT * RPM_TO_MS   // m/s
#define MEC_REMOTE_VY_MAX_RPM       9000.0f * MOTOR_IN2OUT * RPM_TO_MS   // m/s
#define MEC_REMOTE_VW_MIN_RAD_S    -3.1415926f * 2.0f / 20.0f  // rad/s
#define MEC_REMOTE_VW_MAX_RAD_S     3.1415926f * 2.0f / 20.0f  // rad/s

typedef struct
{
    float vx;   // m/s
    float vy;   // m/s
    float vw;   // rad/s
} ChassisVel_t;

typedef struct
{
    float fl;
    float fr;
    float bl;
    float br;
} WheelSpeed_t;


typedef struct
{
    float wheel_radius;     // r (m) 轮子半径
    
    // 长方形底盘参数（均为主轴中心到轮子中心的距离）
    float L;                // 前后轴到中心的距离 (m)
    float W;                // 左右轮距的一半 (m)
    
    float max_wheel_speed;  // 最大轮速
} MecanumParam_t;

extern ChassisVel_t total_vel ;
extern WheelSpeed_t total_speed;
extern MecanumParam_t mecParam;

void Mecanum_Calc(
    const ChassisVel_t *chassis,
    const MecanumParam_t *param,
    WheelSpeed_t *wheel);

#endif
