#pragma once
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

//#include "include.h"
#include "struct_typedef.h"

typedef enum {
    MODE_LOCK_ALL = 0, 
    MODE_CHASSIS_FULL, 
    MODE_SERVO_2D,   
    MODE_RELAY_2D, 
    MODE_SHANGWEIJI
} Robot_Mode_e;

//  CAN 接收帧 
//typedef struct {
//    CAN_HandleTypeDef* hcan;
//    uint32_t StdId;
//    uint8_t  Data[8];
//} CAN_Rx_Frame_t;

//CAN发送帧
typedef struct {
    int16_t target_current[8]; 
    uint8_t update_flag; 
    uint32_t std_id[3];
} CAN_Command_Table_t;

extern CAN_Command_Table_t can1_tx_table;
extern CAN_Command_Table_t can2_tx_table; 
extern CAN_Command_Table_t can1_tx_table_2006;

// 底盘控制指令
typedef struct {
    float vx;        // m/s
    float vy;        // m/s
    float omega;     // rad/s
    uint8_t mode;    // 0: Lock, 1: Unlock
} Chassis_Cmd_t;

//  机构控制指令 
typedef struct {
    Robot_Mode_e mode; // 机构也需要知道模式
    float ch3_raw;     
    float ch2_raw;       
    float lift_speed;
    // 可以在这里加 sw1, sw2 的原始值
} Mech_Cmd_t;

//IMU数据
typedef struct {
    float yaw_rad;       // 当前偏航角 (弧度) [-PI, PI]
    float gyro_z_rads;   // 当前Z轴角速度 (弧度/秒)
    uint8_t is_calibrated; // 标记位：0-未校准/校准中, 1-校准完成
} Imu_Data_t;



#define MAX_SPEED_VX    3.0f    // X方向最大速度 m/s
#define MAX_SPEED_VY    3.0f    // Y方向最大速度 m/s
#define MAX_SPEED_OMEGA 3.0f    // 最大旋转角速度 rad/s

/* 遥控器开关状态定义 */
#define RC_SW_UP          1       // 拨杆上
#define RC_SW_DOWN        2       // 拨杆下
#define RC_SW_MID         3       // 拨杆中

// 低通滤波系数。0.0f-1.0f。值越小滤波越强，输出越平滑，但响应延迟越大。
#define LPF_RC_ALPHA_VX     0.25f // X轴速度通道
#define LPF_RC_ALPHA_VY     0.25f // Y轴速度通道
#define LPF_RC_ALPHA_OMEGA  0.3f // 角速度通道可以稍微强一点，通常对平稳性要求高

#define RC_CH_VY_IDX        0 
#define RC_CH_VX_IDX        1  
#define RC_CH_OMEGA_IDX     2 

#endif
