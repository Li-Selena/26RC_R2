#ifndef __DATA_ANALYSIS_H__
#define __DATA_ANALYSIS_H__

#include "CAN_Task.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "bsp_usb.h"

/* 命令表 */
//整体控制
#define USB_CMD_ALL_ENABLE        0x01U//使能
#define USB_CMD_ALL_DISABLE       0x00U//失能
#define USB_CMD_ALL_MODE_SWITCH   0x02U//模式选择
#define USB_CMD_ALL_STOP          0x03U//急停
#define USB_CMD_ALL_GET_STATUS    0x04U//状态回传
//麦克纳姆底盘控制
#define USB_CMD_MEC_ENABLE        0x11U
#define USB_CMD_MEC_DISABLE       0x10U
#define USB_CMD_MEC_SET_TARGET    0x12U//目标点下发
#define USB_CMD_MEC_STOP          0x13U
#define USB_CMD_MEC_GET_STATUS    0x14U
//腿部控制
#define USB_CMD_LEG_ENABLE        0x21U
#define USB_CMD_LEG_DISABLE       0x20U
#define USB_CMD_LEG_SET_TARGET    0x22U
#define USB_CMD_LEG_STOP          0x23U
#define USB_CMD_LEG_GET_STATUS    0x24U
//机械臂控制
#define USB_CMD_ARM_ENABLE        0x31U
#define USB_CMD_ARM_DISABLE       0x30U
#define USB_CMD_ARM_SET_TARGET    0x32U
#define USB_CMD_ARM_STOP          0x33U
#define USB_CMD_ARM_GET_STATUS    0x34U

//使能标志位
extern uint8_t Mecanum_control_flag ;
extern uint8_t LEG_control_flag ;
extern uint8_t Arm_control_flag ;

//麦克纳姆底盘参数
extern TrapezoidMecanumParam_t mecParam;
extern ChassisVel_t total_vel_USB;
extern WheelSpeed_t total_speed_USB;

//腿部位置参数
extern float legx_USB ;
extern float legy_USB ;
extern float leghtheta_USB ;

//机械臂位置参数
extern float ARM_setX_USB ;
extern float ARM_setY_USB ;
extern float ARM_setZ_USB ;

void Data_Analysis(uint8_t cmd, const uint8_t* datas, uint8_t len);





#endif



