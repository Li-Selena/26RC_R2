#pragma once
#ifndef __IST8310_H
#define __IST8310_H

#include "include.h"

#define IST8310_DEVICE_ADDR    (0x07 << 1) // 7-bit地址0x0C，左移一位

// 寄存器地址
#define IST8310_WHO_AM_I       0x00
#define IST8310_STATUS         0x01
#define IST8310_CTRL1          0x02
#define IST8310_DATA_X_L       0x03
#define IST8310_DATA_X_H       0x04
#define IST8310_DATA_Y_L       0x05
#define IST8310_DATA_Y_H       0x06
#define IST8310_DATA_Z_L       0x07
#define IST8310_DATA_Z_H       0x08

// ID 值
#define IST8310_WHO_AM_I_VAL   0x10

#define IST8310_SENS_XY   3.3f   // LSB/uT
#define IST8310_SENS_Z    3.3f   // LSB/uT
extern I2C_HandleTypeDef IST8310_I2C;

// 函数声明
uint8_t IST8310_Init(void);
uint8_t IST8310_ReadMag(float mag[3]);
float IST8310_GetHeading(const float mag[3]);


#endif // !__IST8310_H
