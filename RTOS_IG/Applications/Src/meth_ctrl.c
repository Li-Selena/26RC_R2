#include "robot_def.h"
#include "include.h"
#include "tool_calc.h"

#define ACTIVE_MECH 2

static uint8_t servo_trigger_lock = 1;  // 扳机保险
static uint8_t servo_trigger_flag = 1; 

//static uint8_t suction_initialized = 0;
//static float suction_home_pos = 0;
//static float suction_target_pos = 0;
static uint8_t suction_flag=1;
static uint8_t suction_state=1;

#if ACTIVE_MECH == 1
/* --- 舵机/夹爪控制 --- */
void Mech_Servo_Logic(float input_val,float roll_val)
{
    float stick_val = apply_deadzone(input_val / 660.0f, 0.1f);
    float roll_stick_val = apply_deadzone(roll_val / 660.0f, 0.1f);

    // 状态机复位条件：摇杆回中
    if (stick_val == 0.0f&&roll_stick_val==0.0f) {
        servo_trigger_lock = 1;
    }

    // 触发夹取 
    if (stick_val > 0.5f && servo_trigger_lock)
    {
        Servo_Set_Angle_Dual(180.0f, 170.0f); // 动作
        Pneumatic_Gripper_Ctrl(VALVE_ON);  

        valve_closed = 1;   
        gimbal_at_work = 1; 
        claw_at_work = 1; 
        
        servo_trigger_lock = 0;  // 锁住，防连发
    }
    // 触发归位 
    else if (stick_val < -0.5f && servo_trigger_lock)
    {
       if (valve_closed) 
        {
            Pneumatic_Gripper_Ctrl(VALVE_OFF); 
            valve_closed = 0; 
        }
        
        else if (gimbal_at_work)
        {
            Servo_SetAngle(&servo_gimbal, 25.0f);
            gimbal_at_work = 0; // 云台回位
        }
            servo_trigger_lock = 0;
       
    }
    
    else if (roll_stick_val > 0.5f && servo_trigger_lock)
    {
        if (claw_at_work)
        {
            Servo_SetAngle(&servo_claw, 80.0f);
            claw_at_work = 0; // 夹爪回位
        }
        
        servo_trigger_lock = 0;
    }
}
#elif ACTIVE_MECH == 2
/* --- 三气动夹爪控制 --- */
void Mech_PneuGripper3_Logic(float input_val, float roll_val)
{
    static float pneu_target_pos = POS_DOWN;
 
    float stick_val = apply_deadzone(input_val / 660.0f, 0.1f);
    float roll_stick_val = apply_deadzone(roll_val / 660.0f, 0.1f);
  
    // 目标位置状态机切换
    if (stick_val < -0.5f) {
        pneu_target_pos = POS_DOWN;
    }
    else if (stick_val > 0.5f) {
        pneu_target_pos = -8192.0f*29.0f; 
    }
    
    if (roll_stick_val == 0.0f) {
        servo_trigger_lock  = 1;
    }

    if (roll_stick_val > 0.5f && servo_trigger_lock  == 1)
    {
        if (servo_trigger_flag  == 1)
        {
            Pneumatic_Gripper_Ctrl(VALVE_ON);
            servo_trigger_flag = 2; // 更新状态为开
        }
        else if (suction_flag == 2)
        {
           Pneumatic_Gripper_Ctrl(VALVE_OFF);
            servo_trigger_flag  = 1;

        }
        servo_trigger_lock  = 0;
    }
    

// int16_t out_motor = (int16_t)pid_call_1(pneu_target_pos,1, 0.003f);
//// can1_tx_table_2006.target_current[0] = out_motor;
// can1_tx_table_2006.update_flag = 1;

}
#endif

/* --- 云台/吸盘控制 --- */
void Mech_Suction_Logic(float input_val,float input_pn,float roll_val)
{
    float stick_val = apply_deadzone(input_val / 660.0f, 0.1f);
    float stick_pn = apply_deadzone(input_pn / 660.0f, 0.1f);
    float stick_roll = apply_deadzone(roll_val / 660.0f, 0.1f);

    if (stick_val > 0.5f) // 正拨
    {
        // 目标：零点 + 90度 
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 1000);
    }
    else if (stick_val < -0.5f)
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 1290);
    }

    if (stick_pn > 0.5f)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    }
    else if (stick_pn < -0.5f)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    }

    //扳机保险
    if (stick_roll == 0.0f) {
        suction_state = 1;
    }

    if (stick_roll > 0.5f && suction_state == 1)
    {
        if (suction_flag == 1)
        {
            Pneumatic_Suction_Ctrl(VALVE_ON);
            suction_flag = 2; // 更新状态为开
        }
        else if (suction_flag == 2)
        {
            Pneumatic_Suction_Ctrl(VALVE_OFF);
            suction_flag = 1;

        }
        suction_state = 0;
    }
}


/* 抬升机构控制逻辑 */
void Mech_Lift_Logic(float input_val, float roll_val)
{
    static float lift_target_pos = POS_DOWN; 
    
    // 归一化输入并处理死区
    float stick_lift = apply_deadzone(input_val / 660.0f, 0.1f);
    float stick_roll = apply_deadzone(roll_val / 660.0f, 0.1f);

    // 目标位置状态机切换
    if (stick_lift < -0.5f) {
        lift_target_pos = POS_DOWN;
    }
    else if (stick_lift > 0.5f) {
        lift_target_pos = POS_UP_One;
    }
    else if (stick_roll > 0.5f) {
        lift_target_pos = POS_UP_Two;
    }

    // 重力补偿
    int16_t gravity_comp = 500;

    int16_t out_motor5 = (int16_t)pid_call_2(lift_target_pos, 5, 0.003f) + gravity_comp;
    int16_t out_motor6 = (int16_t)pid_call_2(lift_target_pos, 6, 0.003f) + gravity_comp;

    can2_tx_table.target_current[4] = out_motor5;
    can2_tx_table.target_current[5] = out_motor6;
    can2_tx_table.update_flag = 1;
}


//void Mech_Lift_Logic(float input_val, float roll_val)
//{
//    static float lift_target_pos = POS_DOWN;
//    static uint8_t lift_initialized = 0;
//    static uint8_t last_stick_active = 0;

//    float stick_lift = apply_deadzone(input_val / 660.0f, 0.1f);

//    float fb_pos = 0.5f * (motor_can2[4].total_angle + motor_can2[5].total_angle);

//    /* 上电初始化 */
//    if (lift_initialized == 0 && motor_can2[4].msg_cnt > 50 && motor_can2[5].msg_cnt > 50)
//    {
//        lift_target_pos = fb_pos;
//        lift_initialized = 1;
//        last_stick_active = 0;
//    }

//    if (lift_initialized)
//    {
//        const float MAX_SPEED = 8000.0f; 
//        uint8_t stick_active = (fabsf(stick_lift) > 0.02f);

//        if (stick_active)
//        {
//            lift_target_pos += stick_lift * MAX_SPEED;
//            last_stick_active = 1;
//        }
//        else
//        {
//            /* 松手瞬间，把目标钉在当前位置，避免继续追远目标 */
//            if (last_stick_active)
//            {
//                lift_target_pos = fb_pos;

//                last_stick_active = 0;
//            }
//        }
//    }

//    int16_t gravity_comp = 500;

//    int16_t out_motor5 = (int16_t)pid_call_2(lift_target_pos, 5, 0.003f) + gravity_comp;
//    int16_t out_motor6 = (int16_t)pid_call_2(lift_target_pos, 6, 0.003f) + gravity_comp;

//    can2_tx_table.target_current[4] = out_motor5;
//    can2_tx_table.target_current[5] = out_motor6;
//    can2_tx_table.update_flag = 1;
//}
