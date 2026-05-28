#include "imu.h"

extern UART_HandleTypeDef huart7;
/* 全局变量定义 */
IMU_Data_t imu_data = {0};
uint8_t imu_rx_byte = 0;  // UART7中断接收缓存（单字节）


/* 寄存器地址宏定义（若wit_c_sdk.h未定义则补充） */
#ifndef AX
#define AX          0x00    // 加速度X轴寄存器地址
#define AY          0x01    // 加速度Y轴寄存器地址
#define AZ          0x02    // 加速度Z轴寄存器地址
#define GX          0x03    // X轴陀螺仪寄存器地址
#define GY          0x04    // Y轴陀螺仪寄存器地址
#define GZ          0x05    // Z轴陀螺仪寄存器地址
#define Roll        0x14    // 横滚角寄存器地址
#define Pitch       0x15    // 俯仰角寄存器地址
#define Yaw         0x16    // 偏航角寄存器地址

#endif

/* SDK回调函数声明（静态函数，仅本文件使用） */
static void IMU_SerialWrite(uint8_t *pData, uint32_t len);
static void IMU_DelayMs(uint32_t ms);
static void IMU_RegUpdateCallback(uint32_t uiReg, uint32_t uiLen);

/**
 * @brief  IMU初始化（基于CubeMX配置的UART7）
 * @note   1. 开启UART7接收中断 2. 注册SDK回调 3. 初始化SDK
 */
void IMU_Init(void)
{
    /* 1. 开启UART7第一次接收中断 */
    if (HAL_UART_Receive_IT(&huart7, &imu_rx_byte, 1) != HAL_OK)
    {
        Error_Handler();
    }

    /* 2. 注册SDK回调函数 */
    WitSerialWriteRegister(IMU_SerialWrite);    
    WitRegisterCallBack(IMU_RegUpdateCallback); 
    WitDelayMsRegister((DelaymsCb)IMU_DelayMs);

    /* 3. 初始化Wit SDK */
    if (WitInit(WIT_PROTOCOL_NORMAL, 0xFF) != WIT_HAL_OK)
    {
        Error_Handler();
    }

    /* 4. 配置IMU参数：输出 加速度+陀螺仪+角度 */
    WitSetOutputRate(RRATE_100HZ);
    WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE); // 关键：添加RSW_GYRO
}



/**
 * @brief  SDK串口发送函数（适配CubeMX的UART7）
 * @param  pData: 发送数据缓冲区
 * @param  len: 发送长度
 */
static void IMU_SerialWrite(uint8_t *pData, uint32_t len)
{
    /* 非阻塞发送（中断模式），避免阻塞中断上下文 */
    HAL_UART_Transmit_IT(&huart7, pData, len);
}

/**
 * @brief  SDK延时函数（复用CubeMX的HAL_Delay）
 * @param  ms: 延时毫秒数
 */
static void IMU_DelayMs(uint32_t ms)
{
    Delay_ms(ms);
}

/**
 * @brief  SDK数据更新回调（解析加速度/角度）
 * @param  uiReg: 寄存器起始地址
 * @param  uiLen: 寄存器长度
 */
static void IMU_RegUpdateCallback(uint32_t uiReg, uint32_t uiLen)
{
    /* 临界区保护：防止主循环读取时数据错乱 */
    __disable_irq();

    switch(uiReg)
    {
        case AX: // 加速度寄存器
            imu_data.acc_x = (float)sReg[AX] / 32768.0f * 16.0f;
            imu_data.acc_y = (float)sReg[AY] / 32768.0f * 16.0f;
            imu_data.acc_z = (float)sReg[AZ] / 32768.0f * 16.0f;
            imu_data.update_flag = 1;
            break;
				 case GX: // 陀螺仪（角加速度）寄存器
            // 转换为实际角加速度（量程±2000°/s）
            imu_data.gyro_x = (float)sReg[GX] / 32768.0f * 2000.0f;
            imu_data.gyro_y = (float)sReg[GY] / 32768.0f * 2000.0f;
            imu_data.gyro_z = (float)sReg[GZ] / 32768.0f * 2000.0f;
            imu_data.update_flag = 1;
            break;

        case Roll: // 角度寄存器
            imu_data.roll  = (float)sReg[Roll] / 32768.0f * 180.0f;
            imu_data.pitch = (float)sReg[Pitch] / 32768.0f * 180.0f;
            imu_data.yaw   = (float)sReg[Yaw] / 32768.0f * 180.0f;
            imu_data.update_flag = 1;
            break;

        default:
            break;
    }

    __enable_irq(); // 恢复中断
}

///**
// * @brief  IMU数据解析辅助函数（主循环调用）
// * @note   非必需，仅用于封装业务逻辑
// */
//void IMU_ParseData(void)
//{
//    if (imu_data.update_flag == 1)
//    {
//        /* 示例：打印数据（需初始化调试串口，如UART1） */
//        char buf[128] = {0};
//        sprintf(buf, "Acc:%.2f,%.2f,%.2f | Angle:%.2f,%.2f,%.2f\r\n",
//                imu_data.acc_x, imu_data.acc_y, imu_data.acc_z,
//                imu_data.roll, imu_data.pitch, imu_data.yaw);
//        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);

//        imu_data.update_flag = 0; // 清零更新标志
//    }
//}