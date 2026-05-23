#include "arm_tools.h"
#include "bsp_tick.h"
#include "fdcan_receive.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

//工具切换电机，只有电机motor_fdcan3[4]，两个工具固定在电机上
extern motor_measure_t motor_fdcan3[8];

//工具句柄
clamp_Handle_t clamp;
chuck_Handle_t chuck;

//工具初始化
void clamp_init(clamp_Handle_t *clamp)
{
    clamp->state = CLAMP_CLOSE;
    clamp->control_source[0] = 0U;          //USART
    clamp->control_source[1] = 0U;          //USB
    clamp->real_angle = motor_fdcan3[3].total_angle;
    clamp->target_angle = 0.0f;
    clamp->safe_flag = 0U;
}

void chuck_init(chuck_Handle_t *chuck)
{
    chuck->state = CHUCK_CLOSE;
    chuck->control_source[0] = 0U;          //USART
    chuck->control_source[1] = 0U;          //USB
    chuck->real_angle = motor_fdcan3[3].total_angle;
    chuck->target_angle = 0.0f;
    chuck->safe_flag = 0U;
}
//设置工具控制信号源
void set_clamp_controlSource(clamp_Handle_t *clamp, uint8_t source)
{
    if(source == TOOL_USART_SOURCE)
    {
        clamp->control_source[0] = 1U;
        clamp->control_source[1] = 0U;
    }
    else if(source == TOOL_USB_SOURCE)
    {
        clamp->control_source[0] = 0U;
        clamp->control_source[1] = 1U;
    }
}

void set_chuck_controlSource(chuck_Handle_t *chuck, uint8_t source)
{
    if(source == TOOL_USART_SOURCE)
    {
        chuck->control_source[0] = 1U;
        chuck->control_source[1] = 0U;
    }
    else if(source == TOOL_USB_SOURCE)
    {
        chuck->control_source[0] = 0U;
        chuck->control_source[1] = 1U;
    }
}
//获取电机实际角度
int update_clamp_real_angle(clamp_Handle_t *clamp, float angle)
{
    clamp->real_angle = angle;
    return 1;
}

int update_chuck_real_angle(chuck_Handle_t *chuck, float angle)
{
    chuck->real_angle = angle;
    return 1;
}
//获取安全标志（是否达到目标角度）
int get_clamp_safe_flag(clamp_Handle_t *clamp)
{
    update_clamp_real_angle(clamp, motor_fdcan3[3].total_angle);
    if(fabsf(clamp->real_angle - clamp->target_angle) < 1.0f)
    {
        clamp->safe_flag = 1U;
        return clamp->safe_flag;
    }
    else
    {
        clamp->safe_flag = 0U;
        return clamp->safe_flag;
    }
}

int get_chuck_safe_flag(chuck_Handle_t *chuck)
{
    update_chuck_real_angle(chuck, motor_fdcan3[3].total_angle);
    if(fabsf(chuck->real_angle - chuck->target_angle) < 1.0f)
    {
        chuck->safe_flag = 1U;
        return chuck->safe_flag;
    }
    else
    {
        chuck->safe_flag = 0U;
        return chuck->safe_flag;
    }
}
//更新控制目标角度
void set_clamp_target_angle(clamp_Handle_t *clamp, float angle)
{
    clamp->target_angle = angle;
}

void set_chuck_target_angle(chuck_Handle_t *chuck, float angle)
{
    chuck->target_angle = angle;
}
//更新使用控制状态
// int update_clamp_control_state(clamp_Handle_t *clamp,uint8_t state)
// {
//     if(state == CLAMP_OPEN)
//         set_clamp_target_angle(clamp, CLAMP_TARGET_ANGLE);
//     else if(state == CLAMP_CLOSE)
//         set_clamp_target_angle(clamp, 0.0f);
//     if(get_clamp_safe_flag(clamp) == SAFE_YES)
//     {
//         clamp->state = state;
//         return 1;
//     }
//     else
//         return 0;
// }
// int update_chuck_control_state(chuck_Handle_t *chuck,uint8_t state)
// {
//     if(state == CHUCK_OPEN)
//         set_chuck_target_angle(chuck, CHUCK_TARGET_ANGLE);
//     else if(state == CHUCK_CLOSE)
//         set_chuck_target_angle(chuck, 0.0f);

//     Delay_ms(1000);

//     if(get_chuck_safe_flag(chuck) == SAFE_YES)
//     {
//         chuck->state = state;
//         return 1;
//     }
//     else
//         return 0;
// }

void trigger_clamp_action(clamp_Handle_t *clamp, uint8_t target_state)
{
    // 防御性编程：如果正在运动中，可以拒绝新指令，或者重置超时时间
    if(clamp->run_status == TOOL_STATUS_MOVING) {
        return; 
    }

    // 1. 下发目标角度
    if(target_state == CLAMP_OPEN)
        set_clamp_target_angle(clamp, CLAMP_TARGET_ANGLE);
    else if(target_state == CLAMP_CLOSE)
        set_clamp_target_angle(clamp, 0.0f);

    // 2. 更新状态机变量
    clamp->pending_state = target_state;             // 暂存目标状态
    clamp->start_tick = xTaskGetTickCount();         // 获取当前 FreeRTOS 系统 Tick
    clamp->run_status = TOOL_STATUS_MOVING;          // 引擎启动，切入运动状态
}

// 状态机步进函数（放在主循环或周期任务中高频调用）
void clamp_state_machine_run(clamp_Handle_t *clamp)
{
    switch(clamp->run_status)
    {
        case TOOL_STATUS_IDLE:
            // 空闲状态，电机已到位，无需任何处理
            break;

        case TOOL_STATUS_MOVING:
            // 1. 检查是否已经安全到达目标位置
            if(get_clamp_safe_flag(clamp) == SAFE_YES)
            {
                clamp->state = clamp->pending_state;  // 真正确认状态已改变
                clamp->run_status = TOOL_STATUS_IDLE; // 回归空闲状态
                
                // 【拓展】可以在这里发送一帧串口数据给上位机，告知“执行完毕”
            }
            // 2. 超时检测：如果超过 1000ms 还没到位（比如夹到硬物卡死）
            else if((xTaskGetTickCount() - clamp->start_tick) > pdMS_TO_TICKS(1000))
            {
                clamp->run_status = TOOL_STATUS_ERROR; // 切入错误状态
                // 【拓展】可以在这里将 target_angle 设回当前真实角度，让电机卸力
            }
            break;

        case TOOL_STATUS_ERROR:
            // 发生错误，等待系统复位或新的干预指令
            // 如果收到复位指令，可以将 run_status 重新设为 IDLE
            break;
            
        default:
            clamp->run_status = TOOL_STATUS_IDLE;
            break;
    }
}

void trigger_chuck_action(chuck_Handle_t *chuck, uint8_t target_state)
{
    // 防御性编程：如果正在运动中，可以拒绝新指令，或者重置超时时间
    if(chuck->run_status == TOOL_STATUS_MOVING) {
        return; 
    }

    // 1. 下发目标角度
    if(target_state == CHUCK_OPEN)
        set_chuck_target_angle(chuck, CHUCK_TARGET_ANGLE);
    else if(target_state == CHUCK_CLOSE)
        set_chuck_target_angle(chuck, 0.0f);

    // 2. 更新状态机变量
    chuck->pending_state = target_state;             // 暂存目标状态
    chuck->start_tick = xTaskGetTickCount();         // 获取当前 FreeRTOS 系统 Tick
    chuck->run_status = TOOL_STATUS_MOVING;          // 引擎启动，切入运动状态
}

// 状态机步进函数（放在主循环或周期任务中高频调用）
void chuck_state_machine_run(chuck_Handle_t *chuck)
{
    switch(chuck->run_status)
    {
        case TOOL_STATUS_IDLE:
            // 空闲状态，电机已到位，无需任何处理
            break;

        case TOOL_STATUS_MOVING:
            // 1. 检查是否已经安全到达目标位置
            if(get_chuck_safe_flag(chuck) == SAFE_YES)
            {
                chuck->state = chuck->pending_state;  // 真正确认状态已改变
                chuck->run_status = TOOL_STATUS_IDLE; // 回归空闲状态
                
                // 【拓展】可以在这里发送一帧串口数据给上位机，告知“执行完毕”
            }
            // 2. 超时检测：如果超过 1000ms 还没到位（比如夹到硬物卡死）
            else if((xTaskGetTickCount() - chuck->start_tick) > pdMS_TO_TICKS(1000))
            {
                chuck->run_status = TOOL_STATUS_ERROR; // 切入错误状态
                // 【拓展】可以在这里将 target_angle 设回当前真实角度，让电机卸力
            }
            break;

        case TOOL_STATUS_ERROR:
            // 发生错误，等待系统复位或新的干预指令
            // 如果收到复位指令，可以将 run_status 重新设为 IDLE
            break;
            
        default:
            chuck->run_status = TOOL_STATUS_IDLE;
            break;
    }
}
