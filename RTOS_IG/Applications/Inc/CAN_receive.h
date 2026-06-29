/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef __CAN_RECEIVE_H
#define __CAN_RECEIVE_H

#include "robot_def.h"
#include "include.h"

#define ENCODER_RESOLUTION 8192.0f // 编码器分辨率
#define ENCODER_OFFSET_COUNT 50 // 校准次数

/* CAN send and receive ID */
typedef enum
{
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
    CAN_3508_M5_ID = 0x205,
    CAN_3508_M6_ID = 0x206,
    CAN_3508_M7_ID = 0x207,
    
    CAN_2006_M1_ID = 0x201,
    CAN_2006_M2_ID = 0x202,
    CAN_2006_M3_ID = 0x203,
    CAN_2006_M4_ID = 0x204,

    CAN_6020_M1_ID = 0x205,
	CAN_6020_M2_ID = 0x206,
	CAN_6020_M3_ID = 0x207,
	CAN_6020_M4_ID = 0x208,
    CAN_6020_M5_ID = 0x209,
    CAN_6020_M6_ID = 0x20A
} can_msg_id_e;

typedef enum {
    OFFSET_AUTO_ON_STARTUP = 0, // 上电自动归零
    OFFSET_FIXED_VALUE,         // 使用固定标定值
} offset_style_e;

#define MOTOR_MAX_NUM 7



typedef struct
{
 uint16_t angle;
 int16_t speed_rpm;
 int16_t given_current;
 uint8_t temperature;
 int16_t last_angle;
 int32_t total_angle;
 int32_t round_cnt;
 uint16_t offset_angle;
 uint32_t msg_cnt;
  fp32 current_angle_deg;  
    
} motor_measure_t;

//电机实例配置结构体
typedef struct {
    CAN_HandleTypeDef *hcan;  // 挂载在哪条 CAN 总线上 (hcan1 或 hcan2)
    uint32_t rx_id;           // 接收 ID (StdId)
    motor_measure_t *data;    // 指向上面数据结构体的指针
    offset_style_e offset_style; // 零点策略
    uint16_t fixed_offset;       // 固定的机械零点值 
} motor_instance_t;



extern motor_measure_t motor_6020[8];
extern motor_measure_t motor_can1[8];
extern motor_measure_t motor_can2[8];

void get_motor_measure(motor_measure_t *ptr,uint8_t data[]);
void get_motor_offset(motor_measure_t *ptr, uint8_t data[]);
void DJI_Motor_Decode(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t *rx_data);

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

void CAN_Send(CAN_HandleTypeDef* hcan, uint32_t std_id, int16_t m1, int16_t m2, int16_t m3, int16_t m4);
void Send_Motor_Commands(CAN_HandleTypeDef* hcan,CAN_Command_Table_t* table);

#endif
