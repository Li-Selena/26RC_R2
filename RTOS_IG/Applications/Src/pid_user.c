#include "pid_user.h"
//#include "swerve_ctrl.h"


 pid_type_def pid_v_1[8],pid_pos_1[8];
pid_type_def pid_v_2[8],pid_pos_2[8];
pid_type_def pid_v_6020[8],pid_pos_6020[8];

float motor_speed_3508_pid[4] = {1.2,0.006,0,0};//3508参数
float motor_position_3508_pid[4] = {0.2,0,0.3,0};

float motor_speed_2006_pid[4] = {1.4,0.0,0,0};//2006参数
float motor_position_2006_pid[4] = {0.01,0,0.4,0};

float motor_speed_6020_pid[4]={35,0,0.02,0};		//底盘6020参数绝对值
float motor_position_6020_pid[4]={0.4,0.001,0.0,0.001};


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


//PID初始化
void PID_devices_Init(void)
{
	for(int i=0;i<4;i++)
	{
    PID_init(&pid_v_1[i],PID_VARY_INT, motor_speed_2006_pid, 10000, 6000);
		PID_init(&pid_pos_1[i],  PID_POSITION, motor_position_2006_pid, 10000, 300);
		
		PID_init(&pid_v_2[i],  PID_POSITION, motor_speed_3508_pid, 10000, 6000);
		PID_init(&pid_pos_2[i],  PID_POSITION, motor_position_3508_pid, 10000, 300);
        
        PID_init(&pid_v_6020[i],PID_VARY_D_ON_INCOM_D, motor_speed_6020_pid, 20000, 10000);
	PID_init(&pid_pos_6020[i], PID_VARY_D_ON_INCOM_D, motor_position_6020_pid, 10000, 100);
//        PID_init(&pid_v_6020_2[i], PID_VARY_INT, motor_speed_6020_pid_2, 20000, 10000);
//	PID_init(&pid_pos_6020_2[i], PID_SUCTION, motor_position_6020_pid_2, 10000, 100);


	}
	
	for(int i=4;i<8;i++)
	{		
    PID_init(&pid_v_1[i],PID_VARY_INT, motor_speed_2006_pid, 10000, 6000);
		PID_init(&pid_pos_1[i],PID_POSITION, motor_position_2006_pid,3000, 300);
		
		PID_init(&pid_v_2[i], PID_POSITION, motor_speed_3508_pid, 10000, 6000);
		PID_init(&pid_pos_2[i], PID_POSITION, motor_position_3508_pid,3000, 300);
    PID_init(&pid_v_6020[i],PID_VARY_D_ON_INCOM_D, motor_speed_6020_pid, 20000, 10000);
	PID_init(&pid_pos_6020[i], PID_VARY_D_ON_INCOM_D, motor_position_6020_pid, 10000, 100);

	}
}


float PID_velocity_realize_1(float set_speed,int i,float dt)
{
		PID_calc(&pid_v_1[i-1],motor_can1[i-1].speed_rpm , set_speed,dt);
		return pid_v_1[i-1].out;
}

float PID_position_realize_1(float set_pos,int i,float dt)
{

		PID_calc(&pid_pos_1[i-1],motor_can1[i-1].total_angle , set_pos,dt);
		return pid_pos_1[i-1].out;

}

float pid_call_1(float position,int i,float dt)
{
		return PID_velocity_realize_1(PID_position_realize_1(position,i,dt),i,dt);
}






float PID_velocity_realize_2(float set_speed,int i,float dt)
{
		PID_calc(&pid_v_2[i-1],motor_can2[i-1].speed_rpm , set_speed,dt);
		return pid_v_2[i-1].out;
}

float PID_position_realize_2(float set_pos,int i,float dt)
{

		PID_calc(&pid_pos_2[i-1],motor_can2[i-1].total_angle , set_pos,dt);
		return pid_pos_2[i-1].out;

}


float pid_call_2(float position,int i,float dt)
{
		return PID_velocity_realize_2(PID_position_realize_2(position,i,dt),i,dt);
}

//6020

float PID_velocity_realize_6020(float set_speed,int i,float dt)
{
		PID_calc(&pid_v_6020[i-1],motor_6020[i-1].speed_rpm , set_speed,dt);
		return pid_v_6020[i-1].out;
}

float PID_position_realize_6020(float set_pos,int i,float dt)
{

		PID_calc(&pid_pos_6020[i-1],motor_6020[i-1].total_angle , set_pos,dt);
		return pid_pos_6020[i-1].out;

}
//float PID_position_realize_6020(float set_pos, int i, float dt)
//{
//    int32_t cur = (int32_t)motor_6020[i - 1].angle;      // 原始 0~8191
//    // 目标也是 0~8191
//    int32_t set_equiv = encoder_wrap_target(cur, (int32_t)set_pos);

//    // 这里让 PID 看见的是“短路径误差”
//    PID_calc(&pid_pos_6020[i - 1], (float)cur, (float)set_equiv, dt);
//    return pid_pos_6020[i - 1].out;
//}

float pid_call_6020(float position,int i,float dt)
{
		return PID_velocity_realize_6020(PID_position_realize_6020(position,i,dt)/*60.0f/(8192.0f*1.0f)*/,i,dt);
}
