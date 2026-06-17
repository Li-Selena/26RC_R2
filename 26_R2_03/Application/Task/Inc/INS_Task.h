/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : INS_Task.h
  * @brief          : INS task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INS_TASK_H
#define INS_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for the INS.
 */
typedef struct
{
	/* Attitude (degrees) */
	float Pitch_Angle;
	float Yaw_Angle;
	float Yaw_TolAngle;
	float Roll_Angle;

	/* Gyro rates (degrees/s) */
  float Pitch_Gyro;
  float Yaw_Gyro;
  float Roll_Gyro;

	/* Raw sensor data (Euler angles in rad, gyro in rad/s, accel in m/s^2) */
  float Angle[3];
	float Gyro[3];
	float Accel[3];       /* Body-frame, low-pass filtered accel (m/s^2) */

	/* World-frame acceleration (m/s^2), gravity-compensated */
	float WorldAccel[3];

	/* World-frame velocity (m/s) */
	float Velocity[3];

	/* World-frame position (m) */
	float Position[3];

	/* Yaw unwinding */
	float Last_Yaw_Angle;
	int16_t YawRoundCount;

}INS_Info_Typedef;

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Reset the INS position to specified values.
  * @param  pos_x  Position X (m)
  * @param  pos_y  Position Y (m)
  * @param  pos_z  Position Z (m)
  * @retval None
  */
void INS_Reset_Position(float pos_x, float pos_y, float pos_z);

/**
  * @brief  Reset the INS velocity to specified values.
  * @param  vel_x  Velocity X (m/s)
  * @param  vel_y  Velocity Y (m/s)
  * @param  vel_z  Velocity Z (m/s)
  * @retval None
  */
void INS_Reset_Velocity(float vel_x, float vel_y, float vel_z);

/**
  * @brief  Zero the INS velocity (set all axes to 0).
  * @retval None
  */
void INS_Zero_Velocity(void);

/**
  * @brief  Check if the robot is near-stationary based on gyro norm.
  * @retval true if stationary, false otherwise
  */
bool INS_Is_Stationary(void);

/* Externs---------------------------------------------------------*/
extern INS_Info_Typedef INS_Info;

#endif //INS_TASK_H
