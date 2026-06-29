#pragma once
/* 舵机控制模块 - servo.h */
#ifndef __SERVO_H
#define __SERVO_H

#include "include.h"

/*舵机*/
#define SERVO_SPEED_STEP 0.5f  // 摇杆推满时，每10ms角度变化的步长
#define STATE_HOME      0   
#define STATE_WORK_ON   1   
#define STATE_WORK_OFF  2   

#define TRIGGER_THRESHOLD  0.5f   

// 舵机结构体定义
typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel;           // PWM通道
    
    /*状态参数*/
    float current_angle;        // 当前角度
    float set_angle;            // 目标角度
    
    /*限制参数*/
    uint16_t min_pulse;         // 最小脉冲宽度(us)
    uint16_t max_pulse;         // 最大脉冲宽度(us)
    float min_angle;            // 最小角度(度)
    float max_angle;            // 最大角度(度)
    float s_val;
    float get_pulse;
    
} Servo_t;

extern Servo_t servo_gimbal;
extern Servo_t servo_claw;


/* 函数声明 */
void Servo_Init(Servo_t* servo,TIM_HandleTypeDef* htim,uint32_t channel);
void Servo_SetAngle(Servo_t* servo, float angle);
float Servo_GetAngle(Servo_t* servo);
void Servo_Update_Angle(void);


#endif /* __SERVO_H */
