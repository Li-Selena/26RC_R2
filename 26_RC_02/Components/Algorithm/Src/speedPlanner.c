#include "speedPlanner.h"
#include "math.h"
#include "stm32h7xx_hal.h"



speedPlanner_t SP_user;
float dt_flag = 0;


void speedPlanner_Init(speedPlanner_t *planner, float accel, float decel, float cycle_time)
{
    planner->accel = accel;
    planner->decel = decel;
    planner->dt = cycle_time; 
    planner->target_v = 0.0f;
    planner->current_v = 0.0f;
}
void speedPlanner_SetTarget(speedPlanner_t *planner, float target_v)
{
    planner->target_v = target_v;
}

float speedPlanner_Update(speedPlanner_t *planner)
{
    float step;

    // 计算目标差
    float error = planner->target_v - planner->current_v;

    // 误差极小 → 直接等于目标
    if (fabsf(error) < 0.001f) {
        planner->current_v = planner->target_v;
        return planner->current_v;
    }

    // 正确加减速判断
    if (error > 0) {
        step = planner->accel * planner->dt;
    } else {
        step = planner->decel * planner->dt;
    }

    // 梯度逼近
    if (error > 0) {
        planner->current_v += step;
        if (planner->current_v > planner->target_v)
            planner->current_v = planner->target_v;
    } else {
        planner->current_v -= step;
        if (planner->current_v < planner->target_v)
            planner->current_v = planner->target_v;
    }

    return planner->current_v;
}


// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//   if (htim->Instance == TIM3) {

//     speedPlanner_Update(&SP_user);
//     dt_flag ++;

//   }
// }

