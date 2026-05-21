#ifndef MECANUM_KINEMATICS_H
#define MECANUM_KINEMATICS_H

#define MEC_REMOTE_VX_MIN_RPM      -9000.0f   // rpm
#define MEC_REMOTE_VX_MAX_RPM       9000.0f   // rpm
#define MEC_REMOTE_VY_MIN_RPM      -9000.0f   // rpm
#define MEC_REMOTE_VY_MAX_RPM       9000.0f   // rpm
#define MEC_REMOTE_VW_MIN_RAD_S    -3.14f/2.0f*250.0f  // rad/s
#define MEC_REMOTE_VW_MAX_RAD_S     3.14f/2.0f*250.0f  // rad/s   

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
    float wheel_radius;     // r  (m)
    float wheel_base;       // L  (m) ǰ���־�
    float wheel_track;      // W  (m) �����־�
    float max_wheel_speed;  // ������� (rad/s �� rpm�������һ��)
} MecanumParam_t;

extern ChassisVel_t total_vel ;
extern WheelSpeed_t total_speed;
extern MecanumParam_t mecParam;

void Mecanum_Calc(
    const ChassisVel_t *chassis,
    const MecanumParam_t *param,
    WheelSpeed_t *wheel);

#endif
