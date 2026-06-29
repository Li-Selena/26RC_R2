#include "Imu_fusion.h"

/* ========== PI融合参数（可根据实际效果微调） ========== */
#define KP_MAG    0.05f   // 比例增益：越大航向收敛越快，但磁力计噪声越明显
#define KI_MAG    0.006f  // 积分增益：越大零偏估计越快，太大会导致零偏抖动
#define DEAD_ZONE 0.05f   // 角速度死区，减小静止抖动
static float IMU_WrapDeg(float angle)
{
    return normalize_deg(angle);
}

/* 倾斜补偿后的磁航向，默认你的坐标系：X前、Y左、Z上 */
static uint8_t IST8310_GetHeadingTiltComp(float* heading_deg)
{
    float mag[3];
    float ax, ay, az;
    float mx, my, mz;
    float roll, pitch;
    float mx2, my2;

    if (IST8310_ReadMag(mag) != 0)
        return 1;

    ax = imu_data.accel[0];
    ay = imu_data.accel[1];
    az = imu_data.accel[2];

    mx = mag[0] - IST8310_MAG_OFFSET_X;
    my = mag[1] - IST8310_MAG_OFFSET_Y;
    mz = mag[2] - IST8310_MAG_OFFSET_Z;

    /* 由加速度估计姿态 */
    roll = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    /* 磁力计倾斜补偿 */
    mx2 = mx * cosf(pitch) + mz * sinf(pitch);
    my2 = mx * sinf(roll) * sinf(pitch)+ my * cosf(roll)- mz * sinf(roll) * cosf(pitch);

    /* 如果你的航向方向反了，把这里改成 atan2f(-my2, mx2) */
    *heading_deg = IMU_WrapDeg(atan2f(my2, mx2) * 180.0f / PI);
    return 0;
}

int IMU_Init()
{
    float yaw0;

    if (BMI088_Init() != 0)
    {
        return 1;
    }

    IST8310_Init();

    BMI088_Calibrate_Gyro();
    
    imu_data.temp_init = BMI088_ReadTemperature();
    imu_data.temp_comp_coeff = 0.008f;   // 温度补偿系数，自行标定
    
    if (IST8310_GetHeadingTiltComp(&yaw0) == 0)
    {
        imu_data.yaw_angle_deg = yaw0;
        imu_data.yaw_rad = yaw0 * PI / 180.0f;
    }
    else
    {
        imu_data.yaw_angle_deg = 0.0f;
        imu_data.yaw_rad = 0.0f;
    }
    
    return 0;
}


void IMU_Update(float dt)
{
    float gz;
    float mag_heading;
    float yaw_err;
//    static uint8_t temp_read_cnt = 0;
    
    BMI088_AccelRead(imu_data.accel);  // 仅读取，不参与融合，保留给外部使用
    BMI088_GyroRead(imu_data.gyro);
   
     imu_data.temp_current = BMI088_ReadTemperature(); 

    /* 原有温度补偿保留，作为零偏的粗补偿 */
    float delta_temp = imu_data.temp_current - imu_data.temp_init;
    float temp_offset = delta_temp * imu_data.temp_comp_coeff;    

    /* 陀螺角速度：减去静态零偏 + 温度补偿 */
    gz = imu_data.gyro[2] - imu_data.gyro_offset[2] - temp_offset;
    imu_data.gyro_cal = gz;
    /* 小死区，减少静止抖动 */
    if (fabsf(gz) < DEAD_ZONE)
        gz = 0.0f;

    /* 陀螺积分得到航向 */
    imu_data.yaw_angle_deg = IMU_WrapDeg(imu_data.yaw_angle_deg + gz * dt);

    if (IST8310_GetHeadingTiltComp(&mag_heading) == 0)
    {
        // 计算航向误差（处理0-360°跳变）
        yaw_err = IMU_WrapDeg(mag_heading - imu_data.yaw_angle_deg);

        // P项：比例修正航向，快速缩小误差
        imu_data.yaw_angle_deg = IMU_WrapDeg(imu_data.yaw_angle_deg + KP_MAG * yaw_err);

        // I项：积分误差，在线更新陀螺零偏（核心：拉住零漂的关键）
        // 稳态时误差会被积分项完全抵消，零偏被自动估计出来
        imu_data.gyro_offset[2] -= KI_MAG * yaw_err * dt;
    }

    imu_data.yaw_rad = imu_data.yaw_angle_deg * PI / 180.0f;
}

