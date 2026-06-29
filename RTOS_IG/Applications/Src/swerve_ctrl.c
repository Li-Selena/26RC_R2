#include "swerve_ctrl.h"
#include "tool_calc.h"
#include "robot_def.h"

SwerveModule swerve_modules[4];
Chassis_Cmd_t chassis_input;
//float pos[4]={0};
 float val[4]={0};
/**
 * @brief 舵轮总初始化
 */
void Swerve_Init()
{
    // 初始化每个轮子的几何位置
    swerve_modules[0].wheel_x_offset = CHASSIS_L; swerve_modules[0].wheel_y_offset = -CHASSIS_W; // 左前 (LR)
    swerve_modules[1].wheel_x_offset = CHASSIS_L; swerve_modules[1].wheel_y_offset =  CHASSIS_W;  // 右前 (FR)
    swerve_modules[2].wheel_x_offset = -CHASSIS_L; swerve_modules[2].wheel_y_offset = CHASSIS_W; // 右后 (FL)
    swerve_modules[3].wheel_x_offset = -CHASSIS_L; swerve_modules[3].wheel_y_offset = -CHASSIS_W;// 左后 (LL)
    
//    chassis_input.vx=0.0f;chassis_input.vy=0.0f;chassis_input.omega=0.0f;
    for (int i = 0; i < 4; i++) {
        swerve_modules[i].last_target_angle = 0.0f;
        swerve_modules[i].target_speed = 0.0f;
        swerve_modules[i].target_angle_deg = 0.0f;
        swerve_modules[i].target_encoder = 0.0f;
        swerve_modules[i].steering_flipped = 0;
    } 
    
}

/**
 * @brief 单个舵轮模块的最优解算 (就近转动逻辑) 
 * @param module 模块结构体指针
 * @param input_vx 轮子 X 轴速度分量
 * @param input_vy 轮子 Y 轴速度分量
 */
static void Optimize_Module(SwerveModule* module, fp32 input_vx, fp32 input_vy)
{
    // 计算目标速度大小
    fp32 target_velocity =  hypotf(input_vx, input_vy);

    // 速度死区保护
    if (target_velocity < DEAD_ZONE)
    {
        module->target_speed = 0.0f;
        // 保持上一帧的目标角度，防止原地抖动
        module->target_angle_deg = module->last_target_angle;
        return;
    }

    // 原始目标角
    fp32 base_angle = normalize_deg(rad2deg(atan2f(input_vy, input_vx)));
    fp32 flip_angle = normalize_deg(base_angle + 180.0f);

    // 当前角与两种候选解的误差
    fp32 d_base = fabsf(wrap_diff_deg(base_angle, module->current_angle_deg));
    fp32 d_flip = fabsf(wrap_diff_deg(flip_angle, module->current_angle_deg));

    // 滞回选择：优先维持上一状态，除非另一解明显更优
    uint8_t use_flip = module->steering_flipped;
    
     if (!use_flip)
    {
        if (d_flip + SWERVE_FLIP_MARGIN_DEG < d_base)
            use_flip = 1;
    }
    else
    {
        if (d_base + SWERVE_FLIP_MARGIN_DEG < d_flip)
            use_flip = 0;
    }

    module->steering_flipped = use_flip;

    if (use_flip)
    {
        module->target_angle_deg = flip_angle;
        module->target_speed = -target_velocity;
    }
    else
    {
        module->target_angle_deg = base_angle;
        module->target_speed = target_velocity;
    }

    module->target_encoder = Angle_To_Encoder(module->target_angle_deg);
    module->last_target_angle = module->target_angle_deg;
    
    //计算目标角度
//    fp32 target_angle = rad2deg(atan2f(input_vy, input_vx));

//    // 就近转动优化
//    fp32 current_real_angle = module->current_angle_deg;
//    fp32 delta_angle = normalize_deg(target_angle - current_real_angle);

//    if (fabsf(delta_angle) > 90.0f)
//    {
//        target_angle = normalize_deg(target_angle + 180.0f);
//        target_velocity = -target_velocity;
//    }
//    module->target_angle_deg = target_angle;
//    module->target_encoder = Angle_To_Encoder(module->target_angle_deg);

//    module->target_speed = target_velocity;
//    module->last_target_angle = target_angle;

}

/**
 * @brief 核心控制循环：运动学解算 + 舵向优先惩罚 (世界坐标系)
 * @param vx 世界坐标系 X 轴速度 (遥控器推杆)
 * @param vy 世界坐标系 Y 轴速度 (遥控器推杆)
 * @param omega 机器人角速度
 * @param yaw_rad 底盘当前绝对偏航角 (弧度)
 */
void Swerve_Update(fp32 vx, fp32 vy, fp32 omega, fp32 yaw_rad)
{
//    // 加入二维旋转矩阵，将世界推杆速度转为底盘局部速度
//    fp32 local_vx = vx * cosf(yaw_rad) + vy * sinf(yaw_rad);
//    fp32 local_vy = -vx * sinf(yaw_rad) + vy * cosf(yaw_rad);
    fp32 local_vx = vx;
    fp32 local_vy = vy;

    // 计算理想目标值 (Kinematics)
    for (uint8_t i = 0; i < 4; i++)
    {
        // 矢量叠加(平移 V + 旋转 Omega x R)
        fp32 v_rot_x = -omega * swerve_modules[i].wheel_y_offset;
        fp32 v_rot_y = omega * swerve_modules[i].wheel_x_offset;
        fp32 final_vx = local_vx + v_rot_x;
        fp32 final_vy = local_vy + v_rot_y;

        // 单轮优化(计算 target_angle 和 target_speed，包含倒车优化)
        Optimize_Module(&swerve_modules[i], final_vx, final_vy);
    }
    //找出最大误差
    fp32 max_err = Get_Max_Steering_Error();

    //计算全局速度缩放系数
    fp32 speed_scale =Calc_Steering_Penalty(max_err);

    //应用系数并准备下发
    for (int i = 0; i < 4; i++)
    {
        // 将速度乘以惩罚系数，保证所有轮子同步减速，维持轨迹
        swerve_modules[i].target_speed *= speed_scale;
    }
}
/**
 * @brief 执行 PID 并发送 (新版 Swerve_Move)
 * @param dt 控制周期 (秒)
 */
void Swerve_Execute(float dt)
{
    for (int i = 0; i < 4; i++)
    {
        
        can1_tx_table.target_current[i] = (int16_t)pid_call_6020(swerve_modules[i].target_encoder, i + 1 , dt);

        float target_rpm = linear_speed_to_motor_rpm(swerve_modules[i].target_speed);
        can2_tx_table.target_current[i] = (int16_t)PID_velocity_realize_2(target_rpm, i + 1, dt);
    }
    can1_tx_table.update_flag = 1;
    can2_tx_table.update_flag = 1;
}

/**
 * @brief 底盘控制总入口
 * @param vx_cmd 世界坐标系X方向速度 m/s
 * @param vy_cmd 世界坐标系Y方向速度 m/s
 * @param omega_user 用户手动旋转角速度
 * @param dt 控制周期 s
 */
void Chassis_Control(float vx_cmd, float vy_cmd, float omega_user, float dt)
{
    //航向保持计算，得到最终的旋转角速度
    float omega_final = YawHold_Update(vx_cmd, vy_cmd, omega_user,imu_data.yaw_angle_deg, imu_data.gyro_cal, dt);
    
    // 传入原有运动学解算
    Swerve_Update(vx_cmd, vy_cmd, omega_final, imu_data.yaw_rad);
}

/**
 * @brief 底盘强制停止
 * @note  切换到其他模式时，必须调用此函数让底盘停下
 */
void Swerve_Stop(void)
{
    chassis_input.vx = 0;
    chassis_input.vy = 0;
    chassis_input.omega = 0;
    for (int i = 0; i < 4; i++){val[i] = 0;swerve_modules[i].target_speed = 0.0f;} 
}
