#ifndef __PID_H
#define __PID_H
#include "stm32f4xx_hal.h"
#include "include.h"

#define PID_VINT_SCALE_FACTOR   10.0f  /* 变速积分缩放因子（越大 -> 误差对积分抑制越明显）*/
#define PID_D_FILTER_ALPHA      0.5f    /*微分滤波系数*/

enum PID_MODE
{
    PID_POSITION = 0,
    PID_VARY_INT,
    PID_SUCTION,
    PID_VARY_D_ON_INCOM_D
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;
    float K_ff;
    
    float max_out;  //最大输出n
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float FF_out;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次
    
    float prev_fdb;       /* 上一次反馈值*/
    float prev_set;
    uint8_t prev_fdb_init;/* 是否已初始化  */
    float I_separation_limit;
    uint8_t variable_I_enable;

    float planned_pos;      // 当前规划到的位置
    float planned_vel;      // 当前规划的速度
    float max_plan_vel;     // 最大规划速度 (单位: 编码器数值/秒)
    float max_plan_acc;     // 最大规划加速度
} pid_type_def;

extern void PID_init(pid_type_def *pid, uint8_t mode, const float PID[4], float max_out, float max_iout);

extern float PID_calc(pid_type_def *pid, float ref, float set,float dt);

extern void PID_clear(pid_type_def *pid);

#endif



