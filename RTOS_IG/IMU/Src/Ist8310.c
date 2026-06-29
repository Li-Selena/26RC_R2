#include "ist8310.h"

static HAL_StatusTypeDef IST8310_WriteReg(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c3,IST8310_DEVICE_ADDR, reg,I2C_MEMADD_SIZE_8BIT, &data,1,10);
}

static HAL_StatusTypeDef IST8310_ReadReg(uint8_t reg, uint8_t* data)
{
    return HAL_I2C_Mem_Read(&hi2c3, IST8310_DEVICE_ADDR, reg,I2C_MEMADD_SIZE_8BIT, data,1,10);
}

static float IST8310_WrapAngleDeg(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f)  angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 初始化 IST8310
 * @retval 0: 成功, 1: 失败
 */
uint8_t IST8310_Init(void)
{
    uint8_t id = 0;

    /* 复位脚 */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(10);

    if (IST8310_ReadReg(IST8310_WHO_AM_I, &id) != HAL_OK)
    {
        return 1;
    }

    if (id != IST8310_WHO_AM_I_VAL)
    {
        return 1;
    }

    /* 软复位 */
    if (IST8310_WriteReg(IST8310_CTRL1, 0x0D) != HAL_OK)
    {
        return 1;
    }
    HAL_Delay(10);

    /* 正常工作模式 */
    if (IST8310_WriteReg(IST8310_CTRL1, 0x01) != HAL_OK)
    {
        return 1;
    }
    HAL_Delay(5);

    return 0;
}


/**
 * @brief 读取三轴磁力计数据 (单位: uT)
 * @param mag [x, y, z]
 * @retval 0 成功, 1 失败
 */
uint8_t IST8310_ReadMag(float mag[3])
{
    uint8_t buf[6];
    int16_t raw[3];

    if (HAL_I2C_Mem_Read(&hi2c3,IST8310_DEVICE_ADDR,IST8310_DATA_X_L,I2C_MEMADD_SIZE_8BIT,buf,6,10) != HAL_OK)
    {
        return 1;
    }

    raw[0] = (int16_t)((buf[1] << 8) | buf[0]);   /* X */
    raw[1] = (int16_t)((buf[3] << 8) | buf[2]);   /* Y */
    raw[2] = (int16_t)((buf[5] << 8) | buf[4]);   /* Z */

    mag[0] = ((float)raw[0] / IST8310_SENS_XY) * 100.0f;
    mag[1] = ((float)raw[1] / IST8310_SENS_XY) * 100.0f;
    mag[2] = ((float)raw[2] / IST8310_SENS_Z) * 100.0f;

    return 0;
}

/**
 * @brief 根据磁力计计算航向角
 * @retval 航向角（度，范围 -180 ~ 180）
 */
float IST8310_GetHeading(const float mag[3])
{
    float heading = atan2f(mag[1], mag[0]) * 180.0f / 3.14159265f;
    return IST8310_WrapAngleDeg(heading);
}

