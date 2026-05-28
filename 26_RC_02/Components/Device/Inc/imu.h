#ifndef __IMU_H
#define __IMU_H

#include "main.h"  // 包含CubeMX生成的huart7声明
#include "wit_c_sdk.h"
#include "bsp_tick.h"


/* IMU数据结构体：存储解算后的加速度和角度 */
typedef struct {
    float acc_x;  // X轴加速度 (单位：g)
    float acc_y;  // Y轴加速度 (单位：g)
    float acc_z;  // Z轴加速度 (单位：g)
	  float gyro_x; // X轴角加速度（陀螺仪） (单位：°/s)
    float gyro_y; // Y轴角加速度（陀螺仪） (单位：°/s)
    float gyro_z; // Z轴角加速度（陀螺仪） (单位：°/s)
    float roll;   // 横滚角 (单位：°)
    float pitch;  // 俯仰角 (单位：°)
    float yaw;    // 偏航角 (单位：°)

    uint8_t update_flag; // 数据更新标志：1-新数据，0-无更新
} IMU_Data_t;

/* 全局IMU数据（供业务层访问） */
extern IMU_Data_t imu_data;
extern uint8_t imu_rx_byte;

/* 函数声明 */
void IMU_Init(void);          // IMU初始化（开启中断+SDK注册）
void IMU_ParseData(void);     // 数据解析辅助函数（可选）

#endif /* __IMU_H */