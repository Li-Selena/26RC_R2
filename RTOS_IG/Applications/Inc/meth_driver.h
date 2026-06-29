#pragma once
#ifndef MECH_DRIVER_H
#define MECH_DRIVER_H 

#include "include.h"

#define POS_UP_Two  (-8192.0f*560.0f)
#define POS_UP_One    (-8192.0f * 380.0f) 
#define POS_DOWN  (0.0f)

// 定义你的气动阀状态
typedef enum {
    VALVE_OFF = 0,
    VALVE_ON
} Valve_State_e;

void Servo_Init_All(void);
void Servo_Set_Angle_Dual(float angle1, float angle2); // 同时设置两个舵机


void Pneumatic_Gripper_Ctrl(Valve_State_e state);   // 夹爪气缸 
void Pneumatic_Suction_Ctrl(Valve_State_e state);   // 吸盘气路 

void Mech_Servo_Logic(float input_val,float roll_val);
void Mech_PneuGripper3_Logic(float input_val, float roll_val);
void Mech_Suction_Logic(float input_val,float input_pn,float roll_val);
void Mech_Lift_Logic(float input_val, float roll_val);

#endif

