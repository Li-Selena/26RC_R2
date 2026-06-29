#pragma once
#ifndef __IMU_FUSION_H
#define __IMU_FUSION_H
#include "include.h"

#define GYRO_STILL_THRESH    0.1f    // 静止判定阈值(°/s)，低于此值认为传感器静止
#define STILL_UPDATE_CNT     100     // 连续静止多少次后开始更新零偏（100Hz采样≈1秒）
#define BIAS_UPDATE_RATE     0.008f   // 零偏更新速率(0~1)，越小越慢、越平稳


#ifdef __cplusplus
extern "C" {
#endif

#ifndef IST8310_MAG_OFFSET_X
#define IST8310_MAG_OFFSET_X   0.0f
#endif

#ifndef IST8310_MAG_OFFSET_Y
#define IST8310_MAG_OFFSET_Y   0.0f
#endif

#ifndef IST8310_MAG_OFFSET_Z
#define IST8310_MAG_OFFSET_Z   0.0f
#endif

#ifndef PI
#define PI 3.14159265358979323846f
#endif

     int IMU_Init(void);
	void IMU_Update(float dt);

	void BMI088_Update(float dt);

#ifdef __cplusplus
}
#endif



#endif // !__IMU_FUSION_H

