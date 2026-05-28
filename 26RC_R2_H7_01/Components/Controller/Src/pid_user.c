#include "pid_user.h"
#include "imu.h"

extern motor_measure_t motor_fdcan2[8];
extern motor_measure_t motor_fdcan3[8];

pid_type_def pid_v_2[8],pid_pos_2[8];
pid_type_def pid_v_3[8],pid_pos_3[8];

float motor_speed_3508_pid_2[8]    = {5, 0.02, 0.1, 0.2};//3508����
float motor_position_3508_pid_2[8] = {0.2, 0, 1, 0};
float motor_speed_2006_pid_2[8]    = {9, 0.1, 0, 0.3};//2006
float motor_position_2006_pid_2[8] = {0.2, 0, 0, 0};

float motor_speed_3508_pid_3[8]    = {5, 0.02, 0.1, 0.2};//3508
float motor_position_3508_pid_3[8] = {0.2, 0, 1, 0};
float motor_speed_2006_pid_3[8]    = {9, 0.1, 0, 0.3};//2006
float motor_position_2006_pid_3[8] = {0.2, 0, 0, 0};



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

		
		PID_init(&pid_v_2[i], PID_POSITION, motor_speed_3508_pid_2, 10000, 6000);
		PID_init(&pid_pos_2[i], PID_POSITION, motor_position_3508_pid_2, 1000, 300);
		
		PID_init(&pid_v_3[i], PID_POSITION, motor_speed_3508_pid_3, 10000, 6000);
		PID_init(&pid_pos_3[i], PID_POSITION, motor_position_3508_pid_3, 1000, 300);
		
		
	}
		for(int i=4;i<8;i++)
	{

		
		PID_init(&pid_v_2[i], PID_POSITION, motor_speed_3508_pid_2, 10000, 6000);
		PID_init(&pid_pos_2[i], PID_POSITION, motor_position_3508_pid_2, 1000, 300);
		
		PID_init(&pid_v_3[i], PID_POSITION, motor_speed_3508_pid_3, 10000, 6000);
		PID_init(&pid_pos_3[i], PID_POSITION, motor_position_3508_pid_3, 1000, 300);
		
		
	}
	


}


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



extern IMU_Data_t imu_data; // 来自 IMU_Task.c 的全局 IMU 数据结构

// 实例化 Yaw 轴控制的 PID 结构体
pid_type_def pid_yaw_angle; // 世界坐标系：角度外环
pid_type_def pid_yaw_rate;  // 机器人/世界坐标系：角速度内环

// PID 参数数组: {Kp, Ki, Kd, Kf(前馈)} 
// 注意：以下参数需根据底盘实际重量和动力情况进行整定
const float PID_YAW_ANGLE_PARAM[4] = {3.5f, 0.0f, 0.1f, 0.0f};  // 角度环通常纯P即可，微调D
const float PID_YAW_RATE_PARAM[4]  = {2.0f, 0.1f, 0.0f, 0.0f};  // 角速度环P要大，加一点I消除稳态误差

void PID_Yaw_Init(void)
{
    /* Yaw 轴陀螺仪纠偏 PID 初始化 */
    PID_init(&pid_yaw_angle, PID_POSITION, PID_YAW_ANGLE_PARAM, 180.0f, 50.0f);
    PID_init(&pid_yaw_rate,  PID_POSITION, PID_YAW_RATE_PARAM,  180.0f, 50.0f);
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
 * @return 旋转补偿输出量 (vw)
 */
float Chassis_Yaw_Robot_Frame_Ctrl(float target_yaw_rate)
{
    // 直接闭环 Z 轴角速度 (imu_data.gyro_z)
    return PID_calc(&pid_yaw_rate, imu_data.gyro_z, target_yaw_rate);
}

/**
 * @brief  模式2：世界坐标系 (绝对方向锁定)
 * @param  target_yaw_angle 目标绝对角度 (单位: °)
 * @return 旋转补偿输出量 (vw)
 */
float Chassis_Yaw_World_Frame_Ctrl(float target_yaw_angle)
{
    // 1. 计算最短路径角度误差
    float error_angle = target_yaw_angle - imu_data.yaw;
    error_angle = Format_Yaw_Angle(error_angle);

    // 2. 角度环计算 (外环)
    // 技巧：这里将反馈 ref 设为 0，目标 set 设为 error_angle。
    // 这样完美绕过了 PID_calc 内部缺乏过零处理的 set - ref 逻辑，不破坏原有库代码。
    float target_yaw_rate = PID_calc(&pid_yaw_angle, 0.0f, error_angle);

    // 3. 角速度环计算 (内环)
    // 将角度环的输出作为角速度环的目标值
    return PID_calc(&pid_yaw_rate, imu_data.gyro_z, target_yaw_rate);
}

/**
 * @brief  清除 Yaw 轴 PID 历史积分与状态 (切换模式时调用)
 */
void Chassis_Yaw_PID_Clear(void)
{
    PID_clear(&pid_yaw_angle);
    PID_clear(&pid_yaw_rate);
}




