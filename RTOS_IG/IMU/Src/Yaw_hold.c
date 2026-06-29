#include "Yaw_hold.h"

YawHold_t g_yaw_hold;


void YawHold_Init(void)
{ 
    g_yaw_hold.target_yaw_deg = 0.0f;
    g_yaw_hold.active = 0;
    g_yaw_hold.i_term = 0.0f;
}

float YawHold_Update(float vx_cmd, float vy_cmd, float omega_user,float yaw_now_deg, float gyro_z_dps, float dt)
{
    float trans_mag = hypotf(vx_cmd, vy_cmd);

    // 有人工旋转输入：直接放行，并释放锁定
    if (fabsf(omega_user) > YAW_HOLD_OMEGA_DEADBAND)
    {
        g_yaw_hold.active = 0;
        g_yaw_hold.i_term = 0.0f;
        return omega_user;
    }

    // 有平移、无旋转：进入航向保持
    if (trans_mag > YAW_HOLD_MOVE_DEADBAND)
    {
        if (!g_yaw_hold.active)
        {
            g_yaw_hold.active = 1;
            g_yaw_hold.target_yaw_deg = yaw_now_deg;   // 进入时锁当前航向
            g_yaw_hold.i_term = 0.0f;
        }

        float err = wrap_diff_deg(g_yaw_hold.target_yaw_deg, yaw_now_deg);

        // PI + 角速度阻尼
        g_yaw_hold.i_term += YAW_HOLD_KI * err * dt;
        g_yaw_hold.i_term = LIMIT(g_yaw_hold.i_term, -YAW_HOLD_I_MAX, YAW_HOLD_I_MAX);

        float omega_hold = YAW_HOLD_KP * err + g_yaw_hold.i_term - YAW_HOLD_KD * gyro_z_dps;
        omega_hold = LIMIT(omega_hold, -YAW_HOLD_OMEGA_MAX, YAW_HOLD_OMEGA_MAX);

        return omega_hold;
    }

    // 无平移、无旋转：不强行保持
    g_yaw_hold.active = 0;
    g_yaw_hold.i_term = 0.0f;
    return 0.0f;
}
