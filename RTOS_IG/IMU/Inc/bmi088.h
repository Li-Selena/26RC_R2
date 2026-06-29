#pragma once
#ifndef BMI088_H
#define BMI088_H

#include "main.h" 

/* 外部SPI句柄声明 */
extern SPI_HandleTypeDef hspi1;
#define BMI088_SPI hspi1

/* 片选引脚宏定义 */
#define BMI088_ACCEL_CS_L() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define BMI088_ACCEL_CS_H() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define BMI088_GYRO_CS_L()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define BMI088_GYRO_CS_H()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)

/* BMI088 陀螺仪相关寄存器地址  */
#define BMI088_GYRO_CHIP_ID         0x00U
#define BMI088_GYRO_DATA             0x02U
#define BMI088_GYRO_RANGE            0x0FU
#define BMI088_GYRO_BANDWIDTH        0x10U
#define BMI088_GYRO_LPM1             0x11U
#define BMI088_GYRO_SOFTRESET        0x14U

/* BMI088 加速度计相关寄存器地址  */
#define BMI088_ACCEL_CHIP_ID         0x00U
#define BMI088_ACCEL_DATA             0x12U
#define BMI088_ACCEL_CONF             0x40U
#define BMI088_ACCEL_RANGE            0x41U
#define BMI088_ACCEL_PWR_CTRL         0x7DU
#define BMI088_ACCEL_SOFTRESET        0x7EU

/* BMI088 ID 值 */
#define BMI088_ACCEL_CHIP_ID_VALUE    0x1EU
#define BMI088_GYRO_CHIP_ID_VALUE     0x0FU

/* BMI088 加速度计模块内置温度传感器 */
#define BMI088_ACCEL_TEMP_L    0x22U  // 温度数据低字节（加速度计模块）
#define BMI088_ACCEL_TEMP_H    0x23U  // 温度数据高字节（加速度计模块）
/* IMU数据结构体，用于存储原始数据*/
typedef struct {
    float accel[3];         // 加速度
    float gyro[3];          // 角速度 (度/秒)

    float gyro_offset[3];   // 陀螺仪零偏数据
    float yaw_angle_deg;    // 积分得到的偏航角 (度)
    float yaw_rad;          // 转换后的偏航角 (弧度)
    
     float temp_init;            // 初始化时的温度
    float temp_comp_coeff;      // 温度补偿系数 (°/s/℃)
    float temp_current;
    float gyro_cal;
    
} imu_t;

extern imu_t imu_data;

/* 公共函数声明 */
uint8_t BMI088_Init(void);
void BMI088_GyroRead(float gyro[3]);
void BMI088_AccelRead(float accel[3]);
float BMI088_ReadTemperature(void);
void BMI088_Calibrate_Gyro(void);
//void BMI088_Update(float dt); // 增加时间间隔参数用于积分

#endif

