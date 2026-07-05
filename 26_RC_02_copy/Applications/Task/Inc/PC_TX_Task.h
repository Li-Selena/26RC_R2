#ifndef __PC_TX_TASK_H
#define __PC_TX_TASK_H

#include <stdint.h>
#include "Control_Task.h"
#include "Data_Analysis.h"
#include "R2_arm.h"
#include "robotarm_mixed.h"

#define ROBOT_STATUS_VIEW_USB_DATA_LEN     16U
#define ROBOT_STATUS_VIEW_FLOAT_COUNT      4U
#define ROBOT_STATUS_VIEW_CHASSIS_WHEELS   4U
#define ROBOT_STATUS_VIEW_CHASSIS_PARAMS   3U
#define ROBOT_STATUS_VIEW_CLIMB_LEGS       4U
#define ROBOT_STATUS_VIEW_CLIMB_DRIVES     2U
#define ROBOT_STATUS_VIEW_LASER_COUNT      3U

typedef struct
{
    uint32_t tick_ms;
    uint8_t active_source;
    uint8_t enable_flags;       /* bit0=chassis, bit3=climb, bit4=arm */
    uint8_t executing_flags;    /* bit0=chassis, bit1=pos, bit4=climb, bit5=yaw_tune, bit6=arm, bit7=any */
    uint8_t error_flags;
    uint8_t online_flags;       /* bit0=USB, bit1=USART, bit2=IMU, bit3=chassis, bit4=arm, bit5=climb, bit6=laser */
    uint8_t timeout_flags;      /* bit0=USB chassis, bit3=USART */
    uint8_t active_source_stale;
} RobotStatusSummaryView_t;

typedef struct
{
    uint32_t usb_count;
    uint32_t usb_last_tick;
    uint8_t usb_last_cmd;
    uint8_t usb_last_len;
    uint8_t usb_payload_valid;
    uint8_t usb_recent;
    uint8_t usb_last_data[ROBOT_STATUS_VIEW_USB_DATA_LEN];
    float usb_last_f[ROBOT_STATUS_VIEW_FLOAT_COUNT];

    uint32_t usart_frame_count;
    uint32_t usart_last_tick;
    uint32_t usart_checksum_fail_count;
    uint8_t usart_checksum_ok;
    uint8_t usart_control_timeout;
    uint8_t usart_recent;
    uint8_t usart_mode;
    float usart_chassis_param[ROBOT_STATUS_VIEW_CHASSIS_PARAMS];
    uint8_t usart_source_flag;
    uint8_t usart_climb_enable;
    uint8_t usart_climb_step;
    uint8_t usart_climb_auto;
} RobotStatusRxView_t;

typedef struct
{
    uint8_t enabled;
    uint8_t mode;
    uint8_t pos_state;
    uint8_t emergency_stop;
    uint8_t moving;
    uint8_t motor_online;
    float target_vx;
    float target_vy;
    float target_vw;
    float target_dx;
    float target_dy;
    float target_dyaw;
    float vel_vx;
    float vel_vy;
    float vel_vw;
    float odom_x;
    float odom_y;
    float odom_yaw;
    float nav_x;
    float nav_y;
    float nav_yaw;
    float nav_vx;
    float nav_vy;
    float nav_wz;
    float wheel_speed[ROBOT_STATUS_VIEW_CHASSIS_WHEELS];
    float pos_progress;
    float pos_err_x;
    float pos_err_y;
    float pos_err_yaw;
} RobotStatusChassisView_t;

typedef struct
{
    uint8_t source;
    uint8_t enabled;
    uint8_t state;
    uint8_t auto_run;
    uint8_t state_done;
    uint8_t error_flags;
    uint8_t flow;
    uint8_t pending_step;
    uint8_t pending_auto;
    uint8_t pending_test_action;
    uint8_t test_action;
    uint8_t test_active;
    uint8_t test_chassis_active;
    uint8_t motor_active;
    uint8_t motor_online;
    uint32_t elapsed_ms;
    uint32_t last_update_ms;
    float leg_pos_mm[ROBOT_STATUS_VIEW_CLIMB_LEGS];
    float leg_target_mm[ROBOT_STATUS_VIEW_CLIMB_LEGS];
    float drive_pos_mm[ROBOT_STATUS_VIEW_CLIMB_DRIVES];
    float drive_target_mm[ROBOT_STATUS_VIEW_CLIMB_DRIVES];
} RobotStatusClimbView_t;

typedef struct
{
    uint8_t valid_flags;
    uint8_t online_flags;
    uint8_t waiting_flags;
    uint8_t all_valid;
    uint8_t all_online;
    uint8_t reserved[3];
    uint32_t update_tick;
    int32_t distance_mm[ROBOT_STATUS_VIEW_LASER_COUNT];
    int32_t raw_distance_mm[ROBOT_STATUS_VIEW_LASER_COUNT];
    int32_t offset_mm[ROBOT_STATUS_VIEW_LASER_COUNT];
    uint32_t last_update_tick[ROBOT_STATUS_VIEW_LASER_COUNT];
    uint32_t timeout_count[ROBOT_STATUS_VIEW_LASER_COUNT];
    uint32_t crc_error_count[ROBOT_STATUS_VIEW_LASER_COUNT];
    uint32_t parse_error_count[ROBOT_STATUS_VIEW_LASER_COUNT];
    uint32_t uart_error_count[ROBOT_STATUS_VIEW_LASER_COUNT];
} RobotStatusLaserView_t;

typedef struct
{
    uint8_t enabled;
    uint8_t has_target;
    uint8_t output_enabled;
    uint8_t state;
    uint8_t status_flags;
    uint8_t error_flags;
    uint8_t ik_status;
    uint8_t ik_reason;
    uint32_t last_command_ms;
    uint32_t last_update_ms;
    uint32_t output_apply_count;
    uint8_t tool;
    uint8_t tool_state;
    uint8_t reserved[2];
    float target_z_mm;
    float approach_yaw_rad;
    float theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    float tool_world_mm[3];
} RobotStatusArmView_t;

typedef struct
{
    uint8_t state;
    uint8_t segment_index;
    uint8_t segment_count;
    uint8_t pass_index;
    uint8_t pass_count;
    uint8_t fail_reason;
    uint8_t active_mode;
    uint8_t phase;
    uint32_t tick_ms;
    uint32_t segment_elapsed_ms;
    float yaw_error_deg;
    float yaw_error_abs_max_deg;
    float yaw_rate_error_rms_dps;
    float gyro_z_abs_max_dps;
    float wheel_rpm_abs_max;
    float score;
    float last_adjust;
    float angle_kp;
    float angle_ki;
    float angle_kd;
    float rate_kp;
    float rate_ki;
    float rate_kd;
    float pos_kp_yaw;
} RobotStatusYawTuneView_t;

typedef struct
{
    RobotStatusSummaryView_t summary;
    RobotStatusRxView_t rx;
    RobotStatusChassisView_t chassis;
    RobotStatusArmView_t arm;
    RobotStatusClimbView_t climb;
    RobotStatusLaserView_t laser;
    RobotStatusYawTuneView_t yaw_tune;
} RobotStatusView_t;

extern volatile RobotStatusView_t g_robot_status_view;
void RobotStatusView_Update(void);

void PC_TX_ReqSysStatus(void);
void PC_TX_ReqChsStatus(void);
void PC_TX_ReqArmStatus(void);
void PC_TX_ReqToolStatus(void);
void PC_TX_ReqRobotStatus(void);
void PC_TX_ReqClimbStatus(void);
void PC_TX_ReqYawTuneStatus(void);

#endif
