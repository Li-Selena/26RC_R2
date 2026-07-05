#ifndef ROBOTARM_STATE_H
#define ROBOTARM_STATE_H

#include "dm_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    Joint_Motor_t joint_motor[4];
    Wheel_Motor_t wheel_motor[2];

    float v_set;
    float x_set;
    float target_v;

    float turn_set;
    float roll_set;
    float roll_x;
    float phi_set;
    float theta_set;

    float leg_set;
    float last_leg_set;

    float v_filter;
    float x_filter;

    float myPithR;
    float myPithGyroR;
    float myPithL;
    float myPithGyroL;
    float roll;
    float total_yaw;
    float theta_err;

    float turn_T;
    float roll_f0;

    float leg_tp;

    uint8_t start_flag;
    uint8_t jump_flag;
    uint8_t jump_flag2;
    uint8_t prejump_flag;
    uint8_t recover_flag;
} chassis_t;

extern chassis_t chassis_move;

#ifdef __cplusplus
}
#endif

#endif
