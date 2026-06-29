#include "servo.h"
#include "usart.h"
/**
 * @brief 初始化舵机
 * @param servo: 舵机结构体指针
 * @param channel: PWM通道
 */
void Servo_Init(Servo_t* servo,TIM_HandleTypeDef* htim,uint32_t channel)
{
    servo->htim=htim;
    servo->channel = channel;
    servo->current_angle = servo->set_angle = 0.0f;
    servo->s_val=0.0f;
    servo->get_pulse=0.0f;
    
    servo->min_pulse = 500;     // 0.5ms 0度
    servo->max_pulse = 2500;    // 2.5ms 180度
    servo->min_angle = 0.0f;
    servo->max_angle = 180.0f;
    
    // 初始位置设为0度
    Servo_SetAngle(servo, 0.0f);    

    HAL_TIM_PWM_Start(servo->htim, servo->channel);


}

/**
 * @brief 设置舵机角度
 * @param servo: 舵机结构体指针
 * @param angle: 目标角度(度)
 * @param pulse_us: 脉冲宽度(微秒)
 */
void Servo_SetAngle(Servo_t* servo, float angle)
{
    // 角度限幅
    if (angle < servo->min_angle) angle = servo->min_angle;
    if (angle > servo->max_angle) angle = servo->max_angle;

    servo->current_angle = angle;

    // 计算脉冲宽度(线性映射)
     float pulse_us = servo->min_pulse +
        (angle - servo->min_angle) *
        (servo->max_pulse - servo->min_pulse) /
        (servo->max_angle - servo->min_angle);
    
    // 脉冲宽度限幅
    if (pulse_us < servo->min_pulse) pulse_us = servo->min_pulse;
    if (pulse_us > servo->max_pulse) pulse_us = servo->max_pulse;
     servo->get_pulse=pulse_us;
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel,(uint16_t)pulse_us);

}

/**
 * @brief 获取当前角度
 * @param servo: 舵机结构体指针
 * @return 当前角度
 */
float Servo_GetAngle(Servo_t* servo)
{
    return servo->current_angle;
}

