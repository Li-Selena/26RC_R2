#include "arm_tools.h"
#include "fdcan_receive.h"
#include <stdint.h>

extern motor_measure_t motor_fdcan3[8];

clamp_Handle_t clamp;
chuck_Handle_t chuck;

void clamp_init(clamp_Handle_t *clamp)
{
    clamp->state[0] = CLAMP_CLOSE;//USART
    clamp->state[1] = CLAMP_CLOSE;//USB
    clamp->flag[0] = 0U;          //USART
    clamp->flag[1] = 0U;          //USB
    clamp->real_angle = motor_fdcan3[3].total_angle;
    clamp->target_angle = CLAMP_TANGLE;
    clamp->safe_flag = 0U;
}

void chuck_init(chuck_Handle_t *chuck)
{
    chuck->state[0] = CHUCK_CLOSE;
    chuck->state[1] = CHUCK_CLOSE;
    chuck->flag[0] = 0U;
    chuck->flag[1] = 0U;
    chuck->real_angle = motor_fdcan3[3].total_angle;
    chuck->target_angle = CHUCK_TANGLE;
    chuck->safe_flag = 0U;
}
