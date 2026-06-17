/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : INS_Task.c
  * @brief          : INS task — full strapdown inertial navigation system
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v2.0
  ******************************************************************************
  * @attention      : Attitude EKF + strapdown position/velocity integration
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "INS_Task.h"
#include "bmi088.h"
#include "lpf.h"
#include "pid.h"
#include "config.h"
#include "tim.h"
#include "Quaternion.h"
#include "bsp_pwm.h"

/**
  * @brief the structure that contains the information for the INS.
  */
INS_Info_Typedef INS_Info;

/**
  * @brief the array that contains the data of LPF2p coefficients.
  */
static float INS_LPF2p_Alpha[3]={1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

/**
  * @brief the structure that contains the Information of accel LPF2p.
  */
LowPassFilter2p_Info_TypeDef INS_AccelPF2p[3];

/**
  * @brief the Initialize data of state transition matrix.
  */
static float QuaternionEKF_A_Data[36]={1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};

/**
  * @brief the Initialize data of posteriori covariance matrix.
  */
static float QuaternionEKF_P_Data[36]= {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                        0.1, 0.1, 0.1,   0.1,  100, 0.1,
                                        0.1, 0.1, 0.1,   0.1,  0.1, 100};

/**
  * @brief the Initialize data of Temperature Control PID.
  */
static float TemCtrl_PID_Param[PID_PARAMETER_NUM]={1200,20,0,0,0,0,2000};

/**
  * @brief the structure that contains the Information of Temperature Control PID.
  */
PID_Info_TypeDef TempCtrl_PID;

/**
  * @brief INS update interval (seconds), matched to osDelayUntil(1) at 1 kHz.
  */
#define INS_DT  0.001f

/**
  * @brief Stationary threshold for gyro norm (rad/s).
  *        When gyro norm < this value, the robot is considered stationary.
  */
#define INS_STATIONARY_GYRO_THRESH  0.3f

/**
  * @brief Velocity decay factor applied during zero-velocity updates (0..1).
  *        0.95f means velocity decays by 5% per iteration when stationary.
  */
#define INS_ZUPT_DECAY  0.95f

/**
 * @brief Initializes the INS_Task.
 */
static void INS_Task_Init(void);

/**
  * @brief  Control the BMI088 temperature
  */
static void BMI088_Temp_Control(float temp);

/**
  * @brief  Perform one step of strapdown inertial navigation.
  * @note   Rotates body-frame acceleration to world frame, subtracts gravity,
  *         and integrates to velocity and position.
  */
static void INS_Strapdown_Update(void);

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the StartINSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  TickType_t INS_Task_SysTick = 0;


	/* Initializes the INS_Task. */
	INS_Task_Init();

  /* Infinite loop */
  for(;;)
  {
    INS_Task_SysTick = osKernelSysTick();

			/* Update the BMI088 measurement */
	    BMI088_Info_Update(&BMI088_Info);

	    /* Accel measurement LPF2p */
	    INS_Info.Accel[0]   =   LowPassFilter2p_Update(&INS_AccelPF2p[0],BMI088_Info.Accel[0]);
	    INS_Info.Accel[1]   =   LowPassFilter2p_Update(&INS_AccelPF2p[1],BMI088_Info.Accel[1]);
	    INS_Info.Accel[2]   =   LowPassFilter2p_Update(&INS_AccelPF2p[2],BMI088_Info.Accel[2]);

	    /* Update the INS gyro in radians */
			INS_Info.Gyro[0]   =   BMI088_Info.Gyro[0];
	    INS_Info.Gyro[1]   =   BMI088_Info.Gyro[1];
	    INS_Info.Gyro[2]   =   BMI088_Info.Gyro[2];

			/* Update the QuaternionEKF */
	    QuaternionEKF_Update(&Quaternion_Info,INS_Info.Gyro,INS_Info.Accel,INS_DT);

	    memcpy(INS_Info.Angle,Quaternion_Info.EulerAngle,sizeof(INS_Info.Angle));

			/* Update the Euler angle in degrees. */
	    INS_Info.Pitch_Angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_PITCH]*RadiansToDegrees;
	    INS_Info.Yaw_Angle   = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_YAW]   *RadiansToDegrees;
	    INS_Info.Roll_Angle  = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_ROLL]*RadiansToDegrees;

			/* Update the yaw total angle */
			if(INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle < -180.f)
			{
				INS_Info.YawRoundCount++;
			}
			else if(INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle > 180.f)
			{
				INS_Info.YawRoundCount--;
			}
			INS_Info.Last_Yaw_Angle = INS_Info.Yaw_Angle;

			INS_Info.Yaw_TolAngle = INS_Info.Yaw_Angle + INS_Info.YawRoundCount*360.f;

	    /* Update the INS gyro in degrees */
	    INS_Info.Pitch_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_PITCH]*RadiansToDegrees;
	    INS_Info.Yaw_Gyro   = INS_Info.Gyro[IMU_GYRO_INDEX_YAW]*RadiansToDegrees;
	    INS_Info.Roll_Gyro  = INS_Info.Gyro[IMU_GYRO_INDEX_ROLL]*RadiansToDegrees;

			/* ===== Strapdown Inertial Navigation ===== */
			INS_Strapdown_Update();

			if(INS_Task_SysTick%5 == 0)
			{
				BMI088_Temp_Control(BMI088_Info.Temperature);
			}

	    osDelayUntil(&INS_Task_SysTick,1);

	  }
	  /* USER CODE END INS_Task */
}
//------------------------------------------------------------------------------
/**
 * @brief Initializes the INS_Task.
 */
static void INS_Task_Init(void)
{
  /* Initializes the Second order lowpass filter  */
  LowPassFilter2p_Init(&INS_AccelPF2p[0],INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AccelPF2p[1],INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AccelPF2p[2],INS_LPF2p_Alpha);

  /* Initializes the Temperature Control PID  */
	PID_Init(&TempCtrl_PID,PID_POSITION,TemCtrl_PID_Param);

  /* Initializes the Quaternion EKF */
	QuaternionEKF_Init(&Quaternion_Info,10.f, 0.001f, 1000000.f,QuaternionEKF_A_Data,QuaternionEKF_P_Data);

	/* Reset INS navigation state to zero */
	memset(INS_Info.WorldAccel, 0, sizeof(INS_Info.WorldAccel));
	memset(INS_Info.Velocity,   0, sizeof(INS_Info.Velocity));
	memset(INS_Info.Position,   0, sizeof(INS_Info.Position));
}
//------------------------------------------------------------------------------
/**
  * @brief  Perform one step of strapdown inertial navigation.
  * @note   Rotates body-frame acceleration to world frame using the current
  *         attitude quaternion, subtracts gravity, and integrates to velocity
  *         and position. Applies zero-velocity update (ZUPT) when the robot
  *         is near-stationary to suppress drift.
  * @retval None
  */
static void INS_Strapdown_Update(void)
{
	float q0, q1, q2, q3;
	float R11, R12, R13;
	float R21, R22, R23;
	float R31, R32, R33;
	float ax, ay, az;       /* body-frame acceleration */
	float ax_w, ay_w, az_w;  /* world-frame acceleration */
	bool  is_stationary;

	/* ---- 1. Extract current attitude quaternion ---- */
	q0 = Quaternion_Info.quat[0];
	q1 = Quaternion_Info.quat[1];
	q2 = Quaternion_Info.quat[2];
	q3 = Quaternion_Info.quat[3];

	/* ---- 2. Compute body-to-world rotation matrix (DCM) ----
	 * R = | 1-2(q2^2+q3^2)    2(q1q2-q0q3)    2(q1q3+q0q2) |
	 *     | 2(q1q2+q0q3)    1-2(q1^2+q3^2)    2(q2q3-q0q1) |
	 *     | 2(q1q3-q0q2)    2(q2q3+q0q1)    1-2(q1^2+q2^2) |
	 */
	R11 = 1.f - 2.f * (q2 * q2 + q3 * q3);
	R12 = 2.f * (q1 * q2 - q0 * q3);
	R13 = 2.f * (q1 * q3 + q0 * q2);

	R21 = 2.f * (q1 * q2 + q0 * q3);
	R22 = 1.f - 2.f * (q1 * q1 + q3 * q3);
	R23 = 2.f * (q2 * q3 - q0 * q1);

	R31 = 2.f * (q1 * q3 - q0 * q2);
	R32 = 2.f * (q2 * q3 + q0 * q1);
	R33 = 1.f - 2.f * (q1 * q1 + q2 * q2);

	/* ---- 3. Rotate body-frame accel to world frame ---- */
	ax = INS_Info.Accel[0];
	ay = INS_Info.Accel[1];
	az = INS_Info.Accel[2];

	ax_w = R11 * ax + R12 * ay + R13 * az;
	ay_w = R21 * ax + R22 * ay + R23 * az;
	az_w = R31 * ax + R32 * ay + R33 * az;

	/* ---- 4. Subtract gravity (world Z-axis is up) ---- */
	az_w -= GravityAccel;

	/* Store world-frame acceleration for external use */
	INS_Info.WorldAccel[0] = ax_w;
	INS_Info.WorldAccel[1] = ay_w;
	INS_Info.WorldAccel[2] = az_w;

	/* ---- 5. Stationary detection (ZUPT gate) ----
	 * Use the gyro norm from the EKF (Quaternion_Info.GyroInvNorm).
	 * 1/GyroInvNorm = gyro_norm. If < threshold, robot is near-stationary.
	 */
	is_stationary = (Quaternion_Info.GyroInvNorm > 0.f)
	             && ((1.f / Quaternion_Info.GyroInvNorm) < INS_STATIONARY_GYRO_THRESH);

	if (is_stationary)
	{
		/* ---- ZUPT: decay velocity toward zero ---- */
		INS_Info.Velocity[0] *= INS_ZUPT_DECAY;
		INS_Info.Velocity[1] *= INS_ZUPT_DECAY;
		INS_Info.Velocity[2] *= INS_ZUPT_DECAY;

		/* Clamp very small velocities to exact zero */
		if (fabsf(INS_Info.Velocity[0]) < 0.001f) INS_Info.Velocity[0] = 0.f;
		if (fabsf(INS_Info.Velocity[1]) < 0.001f) INS_Info.Velocity[1] = 0.f;
		if (fabsf(INS_Info.Velocity[2]) < 0.001f) INS_Info.Velocity[2] = 0.f;
	}
	else
	{
		/* ---- Integrate world acceleration to velocity (Euler) ---- */
		INS_Info.Velocity[0] += ax_w * INS_DT;
		INS_Info.Velocity[1] += ay_w * INS_DT;
		INS_Info.Velocity[2] += az_w * INS_DT;
	}

	/* ---- 6. Integrate velocity to position (Euler) ---- */
	INS_Info.Position[0] += INS_Info.Velocity[0] * INS_DT;
	INS_Info.Position[1] += INS_Info.Velocity[1] * INS_DT;
	INS_Info.Position[2] += INS_Info.Velocity[2] * INS_DT;
}
//------------------------------------------------------------------------------
/**
  * @brief  Reset the INS position to specified values.
  * @param  pos_x  Position X (m)
  * @param  pos_y  Position Y (m)
  * @param  pos_z  Position Z (m)
  * @retval None
  */
void INS_Reset_Position(float pos_x, float pos_y, float pos_z)
{
	INS_Info.Position[0] = pos_x;
	INS_Info.Position[1] = pos_y;
	INS_Info.Position[2] = pos_z;
}
//------------------------------------------------------------------------------
/**
  * @brief  Reset the INS velocity to specified values.
  * @param  vel_x  Velocity X (m/s)
  * @param  vel_y  Velocity Y (m/s)
  * @param  vel_z  Velocity Z (m/s)
  * @retval None
  */
void INS_Reset_Velocity(float vel_x, float vel_y, float vel_z)
{
	INS_Info.Velocity[0] = vel_x;
	INS_Info.Velocity[1] = vel_y;
	INS_Info.Velocity[2] = vel_z;
}
//------------------------------------------------------------------------------
/**
  * @brief  Zero the INS velocity (set all axes to 0).
  * @retval None
  */
void INS_Zero_Velocity(void)
{
	INS_Info.Velocity[0] = 0.f;
	INS_Info.Velocity[1] = 0.f;
	INS_Info.Velocity[2] = 0.f;
}
//------------------------------------------------------------------------------
/**
  * @brief  Check if the robot is near-stationary based on gyro norm.
  * @retval true if stationary, false otherwise
  */
bool INS_Is_Stationary(void)
{
	if (Quaternion_Info.GyroInvNorm <= 0.f)
	{
		return false;
	}
	return ((1.f / Quaternion_Info.GyroInvNorm) < INS_STATIONARY_GYRO_THRESH);
}
//------------------------------------------------------------------------------
/**
  * @brief  Control the BMI088 temperature
  * @param  temp  measure of the BMI088 temperature
  * @retval none
  */
static void BMI088_Temp_Control(float Temp)
{
	PID_Calculate(&TempCtrl_PID,40.f,Temp);

	VAL_LIMIT(TempCtrl_PID.Output,-TempCtrl_PID.Param.LimitOutput,TempCtrl_PID.Param.LimitOutput);

	Heat_Power_Control((uint16_t)(TempCtrl_PID.Output));
}
//------------------------------------------------------------------------------
