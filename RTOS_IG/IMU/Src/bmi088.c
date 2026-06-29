#include "bmi088.h"
#include <math.h>

imu_t imu_data = {0};

// 基础 SPI 收发一个字节
static uint8_t SPI_TransmitReceive(uint8_t tx_data)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&BMI088_SPI, &tx_data, &rx_data, 1, 10);
    return rx_data;
}

// 读寄存器（CS 由调用者控制）
static uint8_t BMI088_ReadReg(uint8_t reg, uint8_t cs_pin)
{
    uint8_t data;
    if (cs_pin == 0) BMI088_ACCEL_CS_L();
    else BMI088_GYRO_CS_L();

    SPI_TransmitReceive(reg | 0x80); // 发送读地址，忽略返回的无效字节
    data = SPI_TransmitReceive(0xFF); // 发送0xFF，同时读取有效数据

    if (cs_pin == 0) BMI088_ACCEL_CS_H();
    else BMI088_GYRO_CS_H();
    return data;
}

// 写寄存器
static void BMI088_WriteReg(uint8_t reg, uint8_t data, uint8_t cs_pin)
{
    if (cs_pin == 0) BMI088_ACCEL_CS_L();
    else BMI088_GYRO_CS_L();

    SPI_TransmitReceive(reg & 0x7F); // 发送写地址
    SPI_TransmitReceive(data);       // 发送数据

    if (cs_pin == 0) BMI088_ACCEL_CS_H();
    else BMI088_GYRO_CS_H();
}

/**
 * @brief  初始化加速度计
 * @retval 0: 成功, 1: 失败
 */
static uint8_t BMI088_AccelInit(void)
{
    uint8_t id = 0;
/* bmi088加速度计上电默认是 I2C 模式,通过一次 Dummy Read 切换至 SPI 模式 */
    BMI088_ReadReg(BMI088_ACCEL_CHIP_ID, 0); 
    HAL_Delay(1);

    /* 读取加速度计ID，验证通信 */
    id = BMI088_ReadReg(BMI088_ACCEL_CHIP_ID, 0);
    if (id != BMI088_ACCEL_CHIP_ID_VALUE) {
        return 1;
    }

    /* 软复位加速度计 */
    BMI088_WriteReg(BMI088_ACCEL_SOFTRESET, 0xB6, 0);
    HAL_Delay(50); // 复位后需要等待 

    /* 配置加速度计为正常模式 */
    BMI088_WriteReg(BMI088_ACCEL_PWR_CTRL, 0x04, 0); // 0x04 使能加速度计
    HAL_Delay(5); // 等待模式切换 

    /*配置加速度计量程和带宽 */
    BMI088_WriteReg(BMI088_ACCEL_RANGE, 0x00, 0); // 0x00 = ±3g
    BMI088_WriteReg(BMI088_ACCEL_CONF, 0x0A, 0);  // 0x0A = 100Hz, 正常模式

    HAL_Delay(10);
    return 0;
}

/**
 * @brief  初始化陀螺仪
 * @retval 0: 成功, 1: 失败
 */
static uint8_t BMI088_GyroInit(void)
{
    uint8_t id = 0;
    
    
    /* 读取陀螺仪ID，验证通信 */
    id = BMI088_ReadReg(BMI088_GYRO_CHIP_ID, 1);
    if (id != BMI088_GYRO_CHIP_ID_VALUE) {
        return 1;
    }

    /*  软复位陀螺仪 */
    BMI088_WriteReg(BMI088_GYRO_SOFTRESET, 0xB6, 1);
    HAL_Delay(20);

    /* 配置陀螺仪为正常模式 */
    BMI088_WriteReg(BMI088_GYRO_LPM1, 0x00, 1); // 正常模式

    /*  配置陀螺仪带宽和量程 */
    BMI088_WriteReg(BMI088_GYRO_RANGE, 0x00, 1);      // 0x00 = ±2000°/s
    BMI088_WriteReg(BMI088_GYRO_BANDWIDTH, 0x02, 1);  // 0x02 = 116Hz

    HAL_Delay(10);
    return 0;
}

/**
 * @brief  BMI088 总初始化函数
 * @retval 0: 成功, 1: 失败
 */
uint8_t BMI088_Init(void)
{
    if (BMI088_AccelInit() != 0) return 1;
    if (BMI088_GyroInit() != 0) return 1;
    return 0;
}

/**
 * @brief  读取陀螺仪三轴数据
 * @param  gyro: 用于存储陀螺仪数据的浮点数组（单位：°/s）
 */
void BMI088_GyroRead(float gyro[3])
{
    uint8_t buf[6];
    int16_t raw[3];

    BMI088_GYRO_CS_L();

    /* 发送陀螺仪数据起始地址 (0x02)，并读取6字节数据 */
    HAL_SPI_Transmit(&BMI088_SPI, (uint8_t*)"\x82", 1,10); // 地址0x02 | 0x80
    HAL_SPI_Receive(&BMI088_SPI, buf, 6, 10);

    BMI088_GYRO_CS_H();

    /* 数据拼接 (BMI088 数据为小端格式) */
    raw[0] = (int16_t)((buf[1] << 8) | buf[0]);
    raw[1] = (int16_t)((buf[3] << 8) | buf[2]);
    raw[2] = (int16_t)((buf[5] << 8) | buf[4]);

    /* 转换为实际值，根据量程(±2000°/s)和16位分辨率计算。 2000 / 32768 = 0.061035 */
    gyro[0] = raw[0] * 0.061035f;
    gyro[1] = raw[1] * 0.061035f;
    gyro[2] = raw[2] * 0.061035f;
}

/**
 * @brief  读取加速度计三轴数据
 * @param  accel: 用于存储加速度计数据的浮点数组（单位：g）
 */
void BMI088_AccelRead(float accel[3])
{
    uint8_t buf[7];
    int16_t raw[3];

    BMI088_ACCEL_CS_L();

    /* 发送加速度计数据起始地址 (0x12)，并读取6字节数据 */
    HAL_SPI_Transmit(&BMI088_SPI, (uint8_t*)"\x92", 1,10); // 地址0x12 | 0x80
    HAL_SPI_Receive(&BMI088_SPI, buf, 7, 10);

    BMI088_ACCEL_CS_H();

    /* 数据拼接 */
    raw[0] = (int16_t)((buf[2] << 8) | buf[1]);
    raw[1] = (int16_t)((buf[4] << 8) | buf[3]);
    raw[2] = (int16_t)((buf[6] << 8) | buf[5]);


    /* 转换为实际值，根据量程(±3g)和16位分辨率计算。 3 / 32768 = 0.000091552 */
    accel[0] = raw[0] * 0.000091552f;
    accel[1] = raw[1] * 0.000091552f;
    accel[2] = raw[2] * 0.000091552f;

}

///**
// * @brief  读取陀螺仪温度（单位：℃）
// * @retval 温度值（摄氏度）
// */
// float BMI088_ReadTemperature(void)
//{
//    uint8_t temp_l, temp_h;
//    int16_t temp_raw;

//    /* 读低字节 (地址 0x20) */
//    temp_l = BMI088_ReadReg(BMI088_GYRO_TEMP_X_L, 1);  // 1 = 陀螺仪片选
//    /* 读高字节 (地址 0x21) */
//    temp_h = BMI088_ReadReg(BMI088_GYRO_TEMP_X_H, 1);

//    /* 小端拼接：低字节在前，高字节在后 */
//    temp_raw = (int16_t)((temp_h << 8) | temp_l);

//    /* BMI088 数据手册：温度(℃) = temp_raw * 0.01 + 23.0 */
//    return (float)temp_raw * 0.01f + 23.0f;
//}

/**
 * @brief  从指定寄存器读取多个字节
 * @param  reg:     起始寄存器地址
 * @param  buf:     数据缓存区
 * @param  len:     读取字节数
 * @param  cs_pin:  片选引脚 (0: 加速度计, 1: 陀螺仪)
 */
static void BMI088_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len, uint8_t cs_pin)
{
    if (cs_pin == 0) BMI088_ACCEL_CS_L();
    else BMI088_GYRO_CS_L();

    /* 发送读地址（最高位1） */
    HAL_SPI_Transmit(&BMI088_SPI, (uint8_t[]){reg | 0x80}, 1, 10);
    /* 连续读取 len 个字节 */
    HAL_SPI_Receive(&BMI088_SPI, buf, len, 10);

    if (cs_pin == 0) BMI088_ACCEL_CS_H();
    else BMI088_GYRO_CS_H();
}

/**
 * @brief  读取陀螺仪温度（单位：℃）
 * @retval 温度值（摄氏度）
 */
float BMI088_ReadTemperature(void)
{
    uint8_t buf[2];
    uint16_t temp_uint11;
    int16_t temp_int11;

    // 读取温度寄存器
    BMI088_ReadRegs(BMI088_ACCEL_TEMP_L, buf, 2, 0);  // 0 = 加速度计片选

    // 拼接11位原始值
    temp_uint11 = (buf[1] << 3) | (buf[0] >> 5);

    // 补码转有符号数
    if (temp_uint11 > 1023) {
        temp_int11 = (int16_t)(temp_uint11 - 2048);
    } else {
        temp_int11 = (int16_t)temp_uint11;
    }

    // 换算成实际温度
    return (float)temp_int11 * 0.125f + 23.0f;
}

/**
 * @brief 陀螺仪静态校准：开机静止 1 秒获取零偏
 */
void BMI088_Calibrate_Gyro(void) {
    float sum = 0;
    float temp_gyro[3];
    for (int i = 0; i < 1000; i++) {
        BMI088_GyroRead(temp_gyro);
        sum += temp_gyro[2]; // 只校准 Z 轴角速度
        HAL_Delay(1);
    }
    imu_data.gyro_offset[2] = sum / 1000.0f; // 记录零偏
}

///**
// * @brief 1ms 周期调用更新：积分获取 Yaw
// */
//void BMI088_Update(float dt) {

//    BMI088_AccelRead(imu_data.accel);
//    BMI088_GyroRead(imu_data.gyro);

//    // 扣除零偏并进行死区滤波（防止静止时角度漂移）
//    float gz = imu_data.gyro[2] - imu_data.gyro_offset[2];
//    if (fabsf(gz) < 0.05f) gz = 0.0f;

//    // 欧拉积分：角度 = 角度 + 角速度 * 时间
//    imu_data.yaw_angle_deg += gz * dt;
//    // 转换为弧度
//    imu_data.yaw_rad = imu_data.yaw_angle_deg * (3.14159265f / 180.0f);
//}
