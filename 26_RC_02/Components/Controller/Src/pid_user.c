#include "pid_user.h"
#include "INS_Task.h"

extern motor_measure_t motor_fdcan1[8];  /* 底盘电机 */
extern motor_measure_t motor_fdcan2[8];  /* 抬升电机 */
extern motor_measure_t motor_fdcan3[8];  /* 机械臂电机 */

pid_type_def pid_v_1[8],pid_pos_1[8];   /* FDCAN1: 底盘 */
pid_type_def pid_v_2[8],pid_pos_2[8];   /* FDCAN2: 抬升 */
pid_type_def pid_v_3[8],pid_pos_3[8];   /* FDCAN3: 机械臂 */

float motor_speed_3508_pid_1[8]    = {5, 0.02, 0.1, 0.2};//3508 底盘速度环
float motor_position_3508_pid_1[8] = {0.2, 0, 1, 0};
float motor_speed_2006_pid_1[8]    = {9, 0.1, 0, 0.3};//2006
float motor_position_2006_pid_1[8] = {0.2, 0, 0, 0};

float motor_speed_3508_pid_2[8]    = {5, 0.02, 0.1, 0.2};//3508 抬升
float motor_position_3508_pid_2[8] = {0.2, 0, 1, 0};
float motor_speed_2006_pid_2[8]    = {9, 0.1, 0, 0.3};//2006
float motor_position_2006_pid_2[8] = {0.2, 0, 0, 0};

float motor_speed_3508_pid_3[8]    = {5, 0.02, 0.1, 0.2};//3508 机械臂
float motor_position_3508_pid_3[8] = {0.2, 0, 1, 0};
float motor_speed_2006_pid_3[8]    = {9, 0.1, 0, 0.3};//2006
float motor_position_2006_pid_3[8] = {0.2, 0, 0, 0};

// 实例化 Yaw 轴控制的 PID 结构体
pid_type_def pid_yaw_angle; // 世界坐标系：角度外环
pid_type_def pid_yaw_rate;  // 机器人/世界坐标系：角速度内环

// PID 参数数组: {Kp, Ki, Kd, Kf(前馈)} 
// 注意：以下参数需根据底盘实际重量和动力情况进行整定
const float PID_YAW_ANGLE_PARAM[4] = {2.0f, 0.0f, 0.05f, 0.0f};  // 角度环 Kp/Ki/Kd/Kf
const float PID_YAW_RATE_PARAM[4]  = {0.5f, 0.02f, 0.2f, 0.0f};  // 角速度环，Kd 用于抑制振荡


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


//PID��ʼ��
void PID_devices_Init(void)
{
	for(int i=0;i<4;i++)
	{
		/* FDCAN1: 底盘电机 */
		PID_init(&pid_v_1[i], PID_POSITION, motor_speed_3508_pid_1, 10000, 6000);
		PID_init(&pid_pos_1[i], PID_POSITION, motor_position_3508_pid_1, 1000, 300);

		/* FDCAN2: 抬升电机 */
		PID_init(&pid_v_2[i], PID_POSITION, motor_speed_3508_pid_2, 10000, 6000);
		PID_init(&pid_pos_2[i], PID_POSITION, motor_position_3508_pid_2, 1000, 300);

		/* FDCAN3: 机械臂电机 */
		PID_init(&pid_v_3[i], PID_POSITION, motor_speed_3508_pid_3, 10000, 6000);
		PID_init(&pid_pos_3[i], PID_POSITION, motor_position_3508_pid_3, 1000, 300);
	}
	for(int i=4;i<8;i++)
	{
		/* FDCAN1: 底盘辅助电机 */
		PID_init(&pid_v_1[i], PID_POSITION, motor_speed_3508_pid_1, 10000, 6000);
		PID_init(&pid_pos_1[i], PID_POSITION, motor_position_3508_pid_1, 1000, 300);

		/* FDCAN2: motors 5..6 are rear M2006 drive wheels. */
		if ((i == 4) || (i == 5)) {
			PID_init(&pid_v_2[i], PID_POSITION, motor_speed_2006_pid_2, 10000, 6000);
			PID_init(&pid_pos_2[i], PID_POSITION, motor_position_2006_pid_2, 1000, 300);
		} else {
			PID_init(&pid_v_2[i], PID_POSITION, motor_speed_3508_pid_2, 10000, 6000);
			PID_init(&pid_pos_2[i], PID_POSITION, motor_position_3508_pid_2, 1000, 300);
		}

		/* FDCAN3: 机械臂辅助 */
		PID_init(&pid_v_3[i], PID_POSITION, motor_speed_3508_pid_3, 10000, 6000);
		PID_init(&pid_pos_3[i], PID_POSITION, motor_position_3508_pid_3, 1000, 300);
	}

    /* IMU yaw 轴 PID 初始化 */
    PID_init(&pid_yaw_angle, PID_POSITION, PID_YAW_ANGLE_PARAM, 180, 80);
    PID_init(&pid_yaw_rate,  PID_POSITION, PID_YAW_RATE_PARAM,  360, 60);
}


/* ── FDCAN1: 底盘电机 PID ── */
float PID_velocity_realize_1(float set_speed,int i)
{
		PID_calc(&pid_v_1[i-1],motor_fdcan1[i-1].speed_rpm , set_speed);
		return pid_v_1[i-1].out;
}

float PID_position_realize_1(float set_pos,int i)
{
		PID_calc(&pid_pos_1[i-1],motor_fdcan1[i-1].total_angle , set_pos);
		return pid_pos_1[i-1].out;
}

float pid_call_1(float position,int i)
{
		return PID_velocity_realize_1(PID_position_realize_1(position,i),i);
}

/* ── FDCAN2: 抬升电机 PID ── */
float PID_velocity_realize_2(float set_speed,int i)
{
		PID_calc(&pid_v_2[i-1],motor_fdcan2[i-1].speed_rpm , set_speed);
		return pid_v_2[i-1].out;
}

float PID_position_realize_2(float set_pos,int i)
{

		PID_calc(&pid_pos_2[i-1],motor_fdcan2[i-1].total_angle , set_pos);
		return pid_pos_2[i-1].out;

}

float pid_call_2(float position,int i)
{
		return PID_velocity_realize_2(PID_position_realize_2(position,i),i);
}



float PID_velocity_realize_3(float set_speed,int i)
{
		PID_calc(&pid_v_3[i-1],motor_fdcan3[i-1].speed_rpm , set_speed);
		return pid_v_3[i-1].out;
}

float PID_position_realize_3(float set_pos,int i)
{

		PID_calc(&pid_pos_3[i-1],motor_fdcan3[i-1].total_angle , set_pos);
		return pid_pos_3[i-1].out;

}

float pid_call_3(float position,int i)
{
		return PID_velocity_realize_3(PID_position_realize_3(position,i),i);
}

/**
 * @brief  角度归一化处理（极其重要）
 * @param  angle 原始角度误差
 * @return 处理后的最短路径误差 (-180 到 180)
 */
static float Format_Yaw_Angle(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief  模式1：机器人坐标系 (相对防跑偏)
 * @param  target_yaw_rate 目标旋转速度 (单位: °/s)。直线平移时传 0.0f。
 * @return 旋转补偿输出量 (vw, °/s)
 *
 * 取反 INS gyro_z 是因为 IMU 的 gyro 正方向与
 * Mecanum_Calc 内部 chassis_vw = -vw 的坐标系约定相反。
 * 若不取反，PID 对扰动的纠正会变成正反馈，推一把就失控自旋。
 */
float Chassis_Yaw_Robot_Frame_Ctrl(float target_yaw_rate)
{
    INS_NavState_t ins_state;

    INS_GetState(&ins_state);
    return PID_calc(&pid_yaw_rate, -ins_state.gyro_z_dps, target_yaw_rate);
}

/**
 * @brief  模式2：世界坐标系 (绝对方向锁定)
 * @param  target_yaw_angle 目标绝对角度 (单位: °)
 * @return 旋转补偿输出量 (vw, °/s)
 *
 * 内环 gyro_z 同样取反，保持与外环 yaw 角方向一致。
 */
float Chassis_Yaw_World_Frame_Ctrl(float target_yaw_angle)
{
    INS_NavState_t ins_state;

    INS_GetState(&ins_state);

    // 1. 计算最短路径角度误差
    float error_angle = target_yaw_angle - ins_state.yaw_deg;
    error_angle = Format_Yaw_Angle(error_angle);

    // 2. 角度环计算 (外环)
    float target_yaw_rate = PID_calc(&pid_yaw_angle, 0.0f, error_angle);

    // 3. 角速度环计算 (内环) — gyro_z 取反
    return PID_calc(&pid_yaw_rate, -ins_state.gyro_z_dps, target_yaw_rate);
}

/**
 * @brief  清除 Yaw 轴 PID 历史积分与状态 (切换模式时调用)
 */
void Chassis_Yaw_PID_Clear(void)
{
    PID_clear(&pid_yaw_angle);
    PID_clear(&pid_yaw_rate);
}




