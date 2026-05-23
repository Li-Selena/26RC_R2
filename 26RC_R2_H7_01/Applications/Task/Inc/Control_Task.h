#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include "bsp_mcu.h"
#include "include.h"
#include "dm_motor_ctrl.h"
#include "bsp_tick.h"
#include "leg.h"
#include "mecanum_classic.h"
#include "CRC.h"
#include "bsp_uart.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include "bsp_usb.h"
#include "arm_ik_3r_safe_stm32h7.h"
#include "arm_user.h"
#include "arm_echo_uart10.h"
#include "arm_tools.h"

extern float ctrl_j1,ctrl_j2,ctrl_j3;
extern float model_theta1,model_theta2,model_theta3;
extern float ctrl_J_USB[4];
extern float model_J_USB[4];
extern float ctrl_J_USART[4];
extern float model_J_USART[4];

void Mecanum_task_USB(ChassisVel_t *chassis_user, TrapezoidMecanumParam_t *param_user, WheelSpeed_t *speed_user);    //麦克纳姆轮底盘控制处理，专门给USB数据解析调用的接口
void LEG_task_USB(float legx,float legy,float h);
void Arm_task_USB(float x,float y,float z);


#endif /* __CONTROL_TASK_H */
