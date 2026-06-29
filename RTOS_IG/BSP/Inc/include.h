#pragma once
#ifndef __INCLUDE_H_
#define __INCLUDE_H_

#include "main.h"

#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "can.h"
#include "i2c.h"
#include "bsp_can.h"
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "pid.h"
#include "pid_user.h"
#include "DT7_control.h"
#include "swerve_ctrl.h"
#include "math_calc.h"
#include "meth_driver.h"
#include "servo.h"

#include "bmi088.h"
#include "ist8310.h"
#include "Yaw_hold.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>

/* ¸¨Öúºê */
#define ABS(x) ((x) > 0 ? (x) : -(x))
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#endif

