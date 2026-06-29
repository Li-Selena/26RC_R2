#include "tool_calc.h"
#include "swerve_ctrl.h"
#include <math.h>

rc_channel_filter_state_t state[3];
  PID_ConfigTypeDef pid_config = PID_DEFAULT_CONFIG;


/*死区处理*/
float apply_deadzone(float v, float dz)
{
    if (fabsf(v) < dz) return 0.0f;
    if (v > 0.0f) return (v - dz) / (1.0f- dz);
    else          return (v + dz) / (1.0f- dz);
}

/*三次方混合响应变换*/
float apply_expo(float v, float expo)
 {
    float s = (v >= 0.0f) ? 1.0f : -1.0f;
    float a = fabsf(v);
    float out = (1.0f- expo) * a + expo * a * a * a;
    return s * out;
 }
 

/**
 * @brief 遥控器通道通用处理函数
 * @param raw_input 遥控器原始整数
 * @param max_phys_val 该通道对应的物理最大值
 * @param set_expo 指数曲线系数
 * @param lpf_alpha 低通滤波系数 (0.0f - 1.0f, 越小滤波越强，延迟越大)
 * @param channel_idx 通道索引 (0 for ch0, 1 for ch1, ...)
 * @return 处理后的物理目标值
 */
float rc_process_channel(int16_t raw_input, float max_phys_val, float set_expo, float lpf_alpha, uint8_t channel_idx)
{
    // 归一化：将 [-660, 660] 转换为 [-1.0f, 1.0f]
    // 确保 RC_MAX_VALUE 是 float 类型，例如 660.0f
    float norm_val = (float)raw_input / RC_MAX_VALUE; 

    // 简单的限幅保护
    if (norm_val > 1.0f) norm_val = 1.0f;
    if (norm_val < -1.0f) norm_val = -1.0f;

    // 应用死区处理 (输出依然是 -1.0f 到 1.0f 范围，死区内为0)
    float val_after_dz = apply_deadzone(norm_val, RC_DEADZONE_RATIO);

    // 应用指数曲线 (手感优化)
    float val_after_expo = apply_expo(val_after_dz, set_expo);

    //一阶低通滤波 ---
    float filtered_norm_val;
    // 获取当前通道的滤波状态
    rc_channel_filter_state_t *state = &rc_channel_states[channel_idx];

    // 为了避免启动时的瞬间跳变或延迟过大，当第一次出现非零值时，直接赋值
        filtered_norm_val = lpf_alpha * val_after_expo + (1.0f - lpf_alpha) * state->prev_filtered_norm_val;
    // 更新保存的值
    state->prev_filtered_norm_val = filtered_norm_val;
    
    if((filtered_norm_val * max_phys_val<=1e-4f)&&(filtered_norm_val * max_phys_val>=-1e-4f))
        return 0;
    else
        return filtered_norm_val * max_phys_val;// 映射回物理单位 (比如 mm/s, rad/s)
}

/**
 * @brief 舵项优先惩罚系数计算函数
 * @param max_err_deg 最大舵向误差 (绝对值)
 * @return 0.0 ~ 1.0 的系数 (1.0代表全速，0.0代表停车)
 */
fp32 Calc_Steering_Penalty(fp32 err_deg)
{
    err_deg = fabsf(err_deg);
    
    const fp32 STOP_THRESHOLD = 45.0f;  
    const fp32 FULL_SPEED_THRESHOLD = 10.0f; 
    const fp32 SWERVE_MIN_SPEED_SCALE = 0.15f;
    
    if (err_deg >= STOP_THRESHOLD)
    {
        return SWERVE_MIN_SPEED_SCALE; // 偏差太大，减速
    }
    else if (err_deg <= FULL_SPEED_THRESHOLD)
    {
        return 1.0f; // 偏差很小，全速
    }
    else
    {
        fp32 t= (err_deg - FULL_SPEED_THRESHOLD) / (STOP_THRESHOLD - FULL_SPEED_THRESHOLD);
        
         return 1.0f - t * (1.0f - SWERVE_MIN_SPEED_SCALE);
    }
}

/**
 * @brief 找出所有轮子中，实际角度与目标角度的最大偏差
 * @return 最大偏差角度 (绝对值)
 */
 fp32 Get_Max_Steering_Error()
{
    fp32 max_err = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        //关键点：如果轮子速度接近0，忽略其角度误差，防止停车时锁死
        if (fabsf(swerve_modules[i].target_speed) < DEAD_ZONE) {
           continue;
            
        }

        // 目标角度已在 Optimize_Module 中处理过翻向逻辑
        fp32 target = swerve_modules[i].target_angle_deg;
        // 实际角度来自电机反馈
        fp32 current = motor_6020[i].current_angle_deg;

        // 计算误差并归一化到 [-180, 180]
        fp32 err = normalize_deg(target - current);
        err = fabsf(err);

        if (err > max_err)
        {
            max_err = err;
        }
    }
//    if (max_err > 180.0f) max_err = 180.0f;
    return max_err;
}
/*pid*/
/* 工具函数：最短环绕误差（a - b）并归一化到 [-half, half)              */
   float circular_error(float a, float b, float half_range, float full_range) {
    float err = a - b;
    if (err > half_range)       err -= full_range;
    else if (err < -half_range) err += full_range;
    return err;
}

/* 梯形规划器：更新 planned_pos / planned_vel                         */
 void update_trapezoidal_planner(pid_type_def *pid, float set, float dt,
                                       float half_range, float full_range) {
    // 剩余距离
    float dist = circular_error(set, pid->planned_pos, half_range, full_range);

    //  刹车距离与加/减速决策
    float stop_dist = (pid->planned_vel * pid->planned_vel) / (2.0f * pid->max_plan_acc);
    float acc = 0.0f;

    bool moving_towards = (dist * pid->planned_vel > 0);
    bool need_brake = moving_towards && (fabsf(dist) <= stop_dist);

    if (need_brake) {
        acc = -sign(pid->planned_vel) * pid->max_plan_acc;   // 全力减速
    } else {
        acc = sign(dist) * pid->max_plan_acc;                // 向目标加速
    }

    // 更新速度、限幅
    pid->planned_vel += acc * dt;
    pid->planned_vel = LIMIT(pid->planned_vel, -pid->max_plan_vel, pid->max_plan_vel);

    // 吸附逻辑（小步长直接到位）
    if (fabsf(dist) < pid_config.arrival_window &&fabsf(pid->planned_vel) < (pid->max_plan_acc * dt)) {
        pid->planned_pos = set;
        pid->planned_vel = 0.0f;
    } else {
        pid->planned_pos += pid->planned_vel * dt;
    }

    //  位置环绕标准化
    if (pid->planned_pos >= full_range)      pid->planned_pos -= full_range;
    else if (pid->planned_pos < 0.0f)        pid->planned_pos += full_range;
}

/* 核心 PID + 前馈计算  */
float compute_pid_output(pid_type_def *pid, float ref, float dt,
                                float half_range, float full_range) {
    // ---- 误差与环绕 ----
    float error = circular_error(pid->set, ref, half_range, full_range);  // pid->set 已为 planned_pos
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = error;

    // ---- P ----
    float Pout = pid->Kp * error;

    // ---- I（带分离限幅与可变强度）----
    float Iout = 0.0f;
    if (fabsf(error) < pid->I_separation_limit) {
        float scale = 1.0f;
        if (pid->variable_I_enable) {
            float abs_err = fabsf(error);
            if (abs_err < 1e-3f) abs_err = 1e-3f;
            scale = 1.0f / (abs_err * pid_config.vint_scale_factor);
            if (scale > pid_config.vint_max_scale) scale = pid_config.vint_max_scale;
        }
        float trapezoid = 0.5f * (error + pid->error[1]) * dt;
        pid->Iout += pid->Ki * trapezoid * scale;
        pid->Iout = LIMIT(pid->Iout, -pid->max_iout, pid->max_iout);
    } else {
        pid->Iout = 0.0f;
    }
    Iout = pid->Iout;

    // ---- D（测量值微分 + 低通滤波）----
    float delta_ref = circular_error(ref, pid->prev_fdb, half_range, full_range);
    float raw_deriv = delta_ref / dt;
    pid->Dbuf[0] = pid_config.d_filter_alpha * raw_deriv +(1.0f - pid_config.d_filter_alpha) * pid->Dbuf[1];
    float Dout = -pid->Kd * pid->Dbuf[0];
    pid->Dbuf[1] = pid->Dbuf[0];

    // ---- 前馈 ----
    float viscous_ff = pid->K_ff * pid->planned_vel;

    float static_ff = 0.0f;
    if (fabsf(error) > pid_config.static_friction_threshold) {
        static_ff = sign(error) * pid_config.static_friction_current;
    }

    return Pout + Iout + Dout + viscous_ff + static_ff;
}
