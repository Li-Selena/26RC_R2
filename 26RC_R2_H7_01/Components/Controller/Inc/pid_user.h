#ifndef __PID_USER_H
#define __PID_USER_H
#include "pid.h"
#include "include.h"

void PID_devices_Init(void);

float PID_velocity_realize_1(float set_speed,int i);
float PID_position_realize_1(float set_pos,int i);
float pid_call_1(float position,int i);

float PID_velocity_realize_2(float set_speed,int i);
float PID_position_realize_2(float set_pos,int i);
float pid_call_2(float position,int i);

float PID_velocity_realize_3(float set_speed,int i);
float PID_position_realize_3(float set_pos,int i);
float pid_call_3(float position,int i);


void PID_Yaw_Init(void);

float Chassis_Yaw_Robot_Frame_Ctrl(float target_yaw_rate);
float Chassis_Yaw_World_Frame_Ctrl(float target_yaw_angle);
void Chassis_Yaw_PID_Clear(void);


#endif























