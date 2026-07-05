#include "PC_TX_Task.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_usb.h"
#include "R2_move.h"
#include "R2_laser_user.h"
#include "R2_yaw_autotune.h"
#include "fdcan_receive.h"
#include "INS_Task.h"
#include "CRC.h"

#include <stddef.h>

extern motor_measure_t motor_fdcan1[8];
extern motor_measure_t motor_fdcan2[8];
extern R2_Move_Ctrl_t g_r2_ctrl_usb;
extern R2_Move_Ctrl_t g_r2_ctrl_usart;
extern volatile uint8_t USB_Task_flag;
extern volatile uint8_t USART_Task_flag;
extern uint8_t Mecanum_control_flag;

volatile RobotStatusView_t g_robot_status_view;

#define PC_TX_STATUS_QUEUE_LEN  16U
#define CLIMB_MOTOR_ONLINE_COUNT 8U

static volatile uint8_t s_status_queue[PC_TX_STATUS_QUEUE_LEN];
static volatile uint8_t s_status_q_head = 0U;
static volatile uint8_t s_status_q_tail = 0U;
static volatile uint8_t s_status_q_count = 0U;
static volatile uint32_t s_status_q_drop_count = 0U;

static void PC_TX_EnqueueStatus(uint8_t cmd);
static uint8_t PC_TX_DequeueStatus(uint8_t *cmd);
static void PackFloatLE(float v, uint8_t *buf, uint16_t offset);
static void PackU32LE(uint32_t v, uint8_t *buf, uint16_t offset);
static void PackI32LE(int32_t v, uint8_t *buf, uint16_t offset);
static float AbsF(float x);
static uint8_t MotorOnline(const motor_measure_t *m, uint8_t n);
static uint8_t ClimbMotorOnline(void);
static void SendSysStatus(void);
static void SendChsStatus(void);
static void SendArmStatus(uint8_t reply_cmd);
static void SendClimbStatus(void);
static void SendYawTuneStatus(void);
static void SendRobotStatus(void);

void PC_TX_ReqSysStatus(void) { PC_TX_EnqueueStatus(USB_CMD_SYS_GET_STATUS); }
void PC_TX_ReqChsStatus(void) { PC_TX_EnqueueStatus(USB_CMD_CHS_GET_STATUS); }
void PC_TX_ReqArmStatus(void) { PC_TX_EnqueueStatus(USB_CMD_ARM_GET_STATUS); }
void PC_TX_ReqToolStatus(void) { PC_TX_EnqueueStatus(USB_CMD_TOOL_GET_STATUS); }
void PC_TX_ReqRobotStatus(void) { PC_TX_EnqueueStatus(USB_CMD_ROBOT_GET_STATUS); }
void PC_TX_ReqClimbStatus(void) { PC_TX_EnqueueStatus(USB_CMD_CLIMB_GET_STATUS); }
void PC_TX_ReqYawTuneStatus(void) { PC_TX_EnqueueStatus(USB_CMD_YAW_TUNE_GET_STATUS); }

static void PC_TX_EnqueueStatus(uint8_t cmd)
{
    taskENTER_CRITICAL();
    if (s_status_q_count < PC_TX_STATUS_QUEUE_LEN)
    {
        s_status_queue[s_status_q_tail] = cmd;
        s_status_q_tail++;
        if (s_status_q_tail >= PC_TX_STATUS_QUEUE_LEN)
        {
            s_status_q_tail = 0U;
        }
        s_status_q_count++;
    }
    else
    {
        s_status_q_drop_count++;
    }
    taskEXIT_CRITICAL();
}

static uint8_t PC_TX_DequeueStatus(uint8_t *cmd)
{
    uint8_t ok = 0U;

    taskENTER_CRITICAL();
    if (s_status_q_count != 0U)
    {
        *cmd = s_status_queue[s_status_q_head];
        s_status_q_head++;
        if (s_status_q_head >= PC_TX_STATUS_QUEUE_LEN)
        {
            s_status_q_head = 0U;
        }
        s_status_q_count--;
        ok = 1U;
    }
    taskEXIT_CRITICAL();

    return ok;
}

static void PackFloatLE(float v, uint8_t *buf, uint16_t offset)
{
    union { float f; uint8_t b[4]; } u;
    u.f = v;
    buf[offset] = u.b[0];
    buf[offset + 1U] = u.b[1];
    buf[offset + 2U] = u.b[2];
    buf[offset + 3U] = u.b[3];
}

static void PackU32LE(uint32_t v, uint8_t *buf, uint16_t offset)
{
    buf[offset] = (uint8_t)(v & 0xFFU);
    buf[offset + 1U] = (uint8_t)((v >> 8) & 0xFFU);
    buf[offset + 2U] = (uint8_t)((v >> 16) & 0xFFU);
    buf[offset + 3U] = (uint8_t)((v >> 24) & 0xFFU);
}

static void PackI32LE(int32_t v, uint8_t *buf, uint16_t offset)
{
    PackU32LE((uint32_t)v, buf, offset);
}

static float AbsF(float x)
{
    return (x < 0.0f) ? -x : x;
}

static uint8_t MotorOnline(const motor_measure_t *m, uint8_t n)
{
    uint8_t cnt = 0U;
    uint8_t i;

    for (i = 0U; i < n; i++)
    {
        if (m[i].msg_cnt > 50U)
        {
            cnt++;
        }
    }
    return cnt;
}

static uint8_t ClimbMotorOnline(void)
{
    uint8_t cnt = MotorOnline(motor_fdcan2, 6U);

    if (motor_fdcan1[4U].msg_cnt > 50U)
    {
        cnt++;
    }
    if (motor_fdcan1[5U].msg_cnt > 50U)
    {
        cnt++;
    }

    return cnt;
}

#define PC_TX_CHASSIS_VEL_MOVE_TOL  0.01f
#define PC_TX_CHASSIS_WZ_MOVE_TOL   0.01f

static void SendSysStatus(void)
{
    uint8_t buf[8];

    buf[0] = USB_Task_flag;
    buf[1] = USART_Task_flag;
    buf[2] = Mecanum_control_flag;
    buf[3] = 0U;
    buf[4] = g_r2_arm_usb.enabled;
    buf[5] = MotorOnline(motor_fdcan1, CHASSIS_MOTOR_COUNT);
    buf[6] = USB_ControlWatchdog_TimeoutFlags();
    buf[7] = 0U;

    Send_Cmd_Data(USB_CMD_SYS_GET_STATUS, buf, 8U);
}

#define CHS_STATUS_LEN  112U

static void SendChsStatus(void)
{
    uint8_t buf[CHS_STATUS_LEN];
    R2_Move_Ctrl_t snapshot;
    const R2_Move_Ctrl_t *c = &snapshot;
    INS_NavState_t nav;
    uint8_t chassis_motor_online;
    uint8_t pos_running;
    uint8_t moving;
    uint8_t timeout_flags;
    uint8_t status_flags;
    uint8_t error_flags;

    taskENTER_CRITICAL();
    snapshot = g_r2_ctrl_usb;
    taskEXIT_CRITICAL();

    INS_GetState(&nav);

    chassis_motor_online = MotorOnline(motor_fdcan1, CHASSIS_MOTOR_COUNT);
    pos_running = (c->pos_state == R2_POS_RUNNING) ? 1U : 0U;
    moving = ((pos_running != 0U) ||
              (AbsF(c->robot_vel.vx) > PC_TX_CHASSIS_VEL_MOVE_TOL) ||
              (AbsF(c->robot_vel.vy) > PC_TX_CHASSIS_VEL_MOVE_TOL) ||
              (AbsF(c->robot_vel.vw) > PC_TX_CHASSIS_WZ_MOVE_TOL)) ? 1U : 0U;

    timeout_flags = USB_ControlWatchdog_TimeoutFlags();
    status_flags =
        ((Mecanum_control_flag != 0U) ? 0x01U : 0U) |
        ((R2_Move_IsVelMode(c->mode) != 0U) ? 0x02U : 0U) |
        ((R2_Move_IsPosMode(c->mode) != 0U) ? 0x04U : 0U) |
        ((pos_running != 0U) ? 0x08U : 0U) |
        ((moving != 0U) ? 0x10U : 0U) |
        ((c->pos_state == R2_POS_DONE) ? 0x20U : 0U) |
        ((nav.imu_online != 0U) ? 0x40U : 0U) |
        ((chassis_motor_online >= CHASSIS_MOTOR_COUNT) ? 0x80U : 0U);
    error_flags =
        (((timeout_flags & 0x01U) != 0U) ? 0x01U : 0U) |
        ((nav.imu_online == 0U) ? 0x02U : 0U) |
        ((c->emergency_stop != 0U) ? 0x04U : 0U) |
        ((chassis_motor_online == 0U) ? 0x08U : 0U) |
        (((chassis_motor_online > 0U) &&
          (chassis_motor_online < CHASSIS_MOTOR_COUNT)) ? 0x10U : 0U);

    buf[0] = (uint8_t)c->mode;
    buf[1] = (uint8_t)c->pos_state;
    buf[2] = c->emergency_stop;
    buf[3] = 0U;
    PackFloatLE(c->robot_vel.vx, buf, 4);
    PackFloatLE(c->robot_vel.vy, buf, 8);
    PackFloatLE(c->robot_vel.vw, buf, 12);
    PackFloatLE(c->odom_x, buf, 16);
    PackFloatLE(c->odom_y, buf, 20);
    PackFloatLE(c->odom_yaw, buf, 24);
    PackFloatLE(c->v_max, buf, 28);
    PackFloatLE(c->a_max, buf, 32);
    PackFloatLE(c->j_max, buf, 36);
    PackFloatLE(c->pos_progress, buf, 40);
    PackFloatLE(c->pos_err_x, buf, 44);
    PackFloatLE(c->pos_err_y, buf, 48);
    PackFloatLE(c->pos_err_yaw, buf, 52);
    PackFloatLE(nav.x_m, buf, 56);
    PackFloatLE(nav.y_m, buf, 60);
    PackFloatLE(nav.yaw_rad, buf, 64);
    PackFloatLE(nav.yaw_total_rad, buf, 68);
    PackFloatLE(nav.vx_mps, buf, 72);
    PackFloatLE(nav.vy_mps, buf, 76);
    PackFloatLE(nav.wz_radps, buf, 80);
    buf[84] = nav.imu_online;
    buf[85] = timeout_flags;
    buf[86] = status_flags;
    buf[87] = error_flags;
    PackFloatLE(c->target_vx, buf, 88);
    PackFloatLE(c->target_vy, buf, 92);
    PackFloatLE(c->target_vw, buf, 96);
    PackFloatLE(c->target_dx, buf, 100);
    PackFloatLE(c->target_dy, buf, 104);
    PackFloatLE(c->target_dyaw, buf, 108);

    Send_Cmd_Data(USB_CMD_CHS_GET_STATUS, buf, CHS_STATUS_LEN);
}

#define ARM_STATUS_LEN  64U

static void SendArmStatus(uint8_t reply_cmd)
{
    uint8_t buf[ARM_STATUS_LEN];
    uint8_t i;
    R2_ArmStatus_t arm;

    for (i = 0U; i < ARM_STATUS_LEN; i++)
    {
        buf[i] = 0U;
    }

    taskENTER_CRITICAL();
    R2_Arm_GetStatus(&g_r2_arm_usb, &arm);
    taskEXIT_CRITICAL();

    buf[0] = arm.enabled;
    buf[1] = arm.has_target;
    buf[2] = arm.output_enabled;
    buf[3] = arm.state;
    buf[4] = arm.status_flags;
    buf[5] = arm.error_flags;
    buf[6] = arm.last_ik_status;
    buf[7] = arm.last_ik_reason;
    PackU32LE(arm.last_command_ms, buf, 8);
    PackU32LE(arm.last_update_ms, buf, 12);
    PackU32LE(arm.output_apply_count, buf, 16);
    buf[20] = (uint8_t)arm.request.tool;
    buf[21] = (uint8_t)arm.request.state;
    PackFloatLE(arm.request.target_z_mm, buf, 24);
    PackFloatLE(arm.request.approach_yaw_rad, buf, 28);
    PackFloatLE(arm.result.theta_rad[0], buf, 32);
    PackFloatLE(arm.result.theta_rad[1], buf, 36);
    PackFloatLE(arm.result.theta_rad[2], buf, 40);
    PackFloatLE(arm.result.tool_world_mm.x, buf, 44);
    PackFloatLE(arm.result.tool_world_mm.y, buf, 48);
    PackFloatLE(arm.result.tool_world_mm.z, buf, 52);
    PackFloatLE(arm.result.j4_world_mm.z, buf, 56);
    PackFloatLE(arm.request.target_z_mm - arm.result.tool_world_mm.z, buf, 60);

    Send_Cmd_Data(reply_cmd, buf, ARM_STATUS_LEN);
}

#define CLIMB_STATUS_LEN  68U

static uint8_t ClimbValueReached(float pos, float target, float tol)
{
    float err = pos - target;

    if (err < 0.0f)
    {
        err = -err;
    }

    return (err <= tol) ? 1U : 0U;
}

static uint8_t ClimbLegReachedMask(const R2_Climb_Ctrl_t *climb)
{
    uint8_t i;
    uint8_t mask = 0U;

    if (climb == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < 4U; i++)
    {
        if (ClimbValueReached(climb->leg_pos_mm[i],
                              climb->leg_target_mm[i],
                              R2_CLIMB_LEG_TOL_MM) != 0U)
        {
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}

static uint8_t ClimbDriveReachedMask(const R2_Climb_Ctrl_t *climb)
{
    uint8_t i;
    uint8_t mask = 0U;
    uint8_t drive_group_mask;

    if (climb == NULL)
    {
        return 0U;
    }

    drive_group_mask = climb->drive_group_mask &
                       (R2_CLIMB_DRIVE_GROUP_FRONT |
                        R2_CLIMB_DRIVE_GROUP_REAR);
    if (drive_group_mask == 0U)
    {
        drive_group_mask = R2_CLIMB_DRIVE_GROUP_ALL;
    }

    for (i = 0U; i < 2U; i++)
    {
        uint8_t reached = 1U;

        if (((drive_group_mask & R2_CLIMB_DRIVE_GROUP_FRONT) != 0U) &&
            (ClimbValueReached(climb->front_drive_pos_mm[i],
                               climb->drive_target_mm[i],
                               R2_CLIMB_DRIVE_TOL_MM) == 0U))
        {
            reached = 0U;
        }

        if (((drive_group_mask & R2_CLIMB_DRIVE_GROUP_REAR) != 0U) &&
            (ClimbValueReached(climb->rear_drive_pos_mm[i],
                               climb->drive_target_mm[i],
                               R2_CLIMB_DRIVE_TOL_MM) == 0U))
        {
            reached = 0U;
        }

        if (reached != 0U)
        {
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}

static void SendClimbStatus(void)
{
    uint8_t buf[CLIMB_STATUS_LEN];
    uint8_t i;
    uint8_t active_source;
    uint8_t leg_reached_mask;
    uint8_t drive_reached_mask;
    uint8_t status_flags;
    uint8_t leg_busy;
    uint8_t drive_busy;
    uint8_t ready_for_next;
    R2_Climb_Ctrl_t climb;
    uint32_t elapsed_ms = 0U;

    for (i = 0U; i < CLIMB_STATUS_LEN; i++)
    {
        buf[i] = 0U;
    }

    taskENTER_CRITICAL();
    active_source = (USB_Task_flag != 0U) ? CONTROL_SOURCE_USB :
                    ((USART_Task_flag != 0U) ? CONTROL_SOURCE_USART : CONTROL_SOURCE_NONE);
    climb = (active_source == CONTROL_SOURCE_USB) ? g_r2_climb_usb : g_r2_climb_usart;
    taskEXIT_CRITICAL();

    if ((climb.state_start_ms != 0U) &&
        (climb.last_update_ms >= climb.state_start_ms))
    {
        elapsed_ms = climb.last_update_ms - climb.state_start_ms;
    }

    leg_reached_mask = ClimbLegReachedMask(&climb);
    drive_reached_mask = ClimbDriveReachedMask(&climb);
    leg_busy = ((leg_reached_mask & 0x0FU) != 0x0FU) ? 1U : 0U;
    drive_busy = ((drive_reached_mask & 0x03U) != 0x03U) ? 1U : 0U;
    ready_for_next =
        ((((climb.state_done != 0U) ||
           (climb.state == R2_CLIMB_STATE_IDLE) ||
           (climb.state == R2_CLIMB_STATE_DONE)) &&
          (climb.pending_step == 0U) &&
          (climb.pending_auto == 0U) &&
          (climb.pending_test_action == 0U) &&
          (climb.test_chassis_active == 0U) &&
          (leg_busy == 0U) &&
          (drive_busy == 0U)) ? 1U : 0U);
    status_flags =
        ((R2_Climb_IsMotorActive(&climb) != 0U) ? 0x01U : 0U) |
        ((leg_busy != 0U) ? 0x02U : 0U) |
        ((drive_busy != 0U) ? 0x04U : 0U) |
        ((climb.test_chassis_active != 0U) ? 0x08U : 0U) |
        ((climb.pending_step != 0U) ? 0x10U : 0U) |
        ((climb.pending_auto != 0U) ? 0x20U : 0U) |
        ((climb.pending_test_action != 0U) ? 0x40U : 0U) |
        ((ready_for_next != 0U) ? 0x80U : 0U);

    buf[0] = (uint8_t)climb.state;
    buf[1] = climb.enabled;
    buf[2] = climb.auto_run;
    buf[3] = climb.state_done;
    buf[4] = climb.error_flags;
    buf[5] = active_source;
    buf[6] = ClimbMotorOnline();
    buf[7] = climb.test_action;
    PackU32LE(elapsed_ms, buf, 8);
    PackU32LE(climb.last_update_ms, buf, 12);
    PackFloatLE(climb.leg_pos_mm[0], buf, 16);
    PackFloatLE(climb.leg_pos_mm[1], buf, 20);
    PackFloatLE(climb.leg_pos_mm[2], buf, 24);
    PackFloatLE(climb.leg_pos_mm[3], buf, 28);
    PackFloatLE(climb.leg_target_mm[0], buf, 32);
    PackFloatLE(climb.leg_target_mm[1], buf, 36);
    PackFloatLE(climb.leg_target_mm[2], buf, 40);
    PackFloatLE(climb.leg_target_mm[3], buf, 44);
    PackFloatLE(climb.drive_pos_mm[0], buf, 48);
    PackFloatLE(climb.drive_pos_mm[1], buf, 52);
    PackFloatLE(climb.drive_target_mm[0], buf, 56);
    PackFloatLE(climb.drive_target_mm[1], buf, 60);
    buf[64] = climb.flow;
    buf[65] = status_flags;
    buf[66] = leg_reached_mask;
    buf[67] = drive_reached_mask;

    Send_Cmd_Data(USB_CMD_CLIMB_GET_STATUS, buf, CLIMB_STATUS_LEN);
}

#define YAW_TUNE_STATUS_LEN  64U

static void SendYawTuneStatus(void)
{
    uint8_t buf[YAW_TUNE_STATUS_LEN];
    R2_YawAutoTuneStatus_t s;
    uint8_t i;

    for (i = 0U; i < YAW_TUNE_STATUS_LEN; i++)
    {
        buf[i] = 0U;
    }

    R2_YawAutoTune_GetStatus(&s);

    buf[0] = s.state;
    buf[1] = s.segment_index;
    buf[2] = s.segment_count;
    buf[3] = s.fail_reason;
    PackU32LE(s.tick_ms, buf, 4);
    PackU32LE(s.segment_elapsed_ms, buf, 8);
    PackFloatLE(s.yaw_error_deg, buf, 12);
    PackFloatLE(s.yaw_error_abs_max_deg, buf, 16);
    PackFloatLE(s.yaw_rate_error_rms_dps, buf, 20);
    PackFloatLE(s.gyro_z_abs_max_dps, buf, 24);
    PackFloatLE(s.score, buf, 28);
    PackFloatLE(s.angle_kp, buf, 32);
    PackFloatLE(s.angle_kd, buf, 36);
    PackFloatLE(s.rate_kp, buf, 40);
    PackFloatLE(s.rate_ki, buf, 44);
    PackFloatLE(s.rate_kd, buf, 48);
    PackFloatLE(s.pos_kp_yaw, buf, 52);
    PackFloatLE(s.last_adjust, buf, 56);
    buf[60] = s.pass_index;
    buf[61] = s.pass_count;
    buf[62] = s.active_mode;
    buf[63] = s.phase;

    Send_Cmd_Data(USB_CMD_YAW_TUNE_GET_STATUS, buf, YAW_TUNE_STATUS_LEN);
}

#define ROBOT_STATUS_PROTOCOL_VERSION 4U
#define ROBOT_STATUS_LEN        240U
#define ROBOT_CMD_RECENT_MS     500U
#define ROBOT_VEL_MOVE_TOL      0.01f
#define ROBOT_WZ_MOVE_TOL       0.01f

void RobotStatusView_Update(void)
{
    uint8_t i;
    uint8_t active_source;
    uint8_t timeout_flags;
    uint8_t usb_recent;
    uint8_t usart_recent;
    uint8_t active_source_stale;
    uint8_t chassis_moving;
    uint8_t chassis_pos_running;
    uint8_t climb_motor_active;
    uint8_t arm_motor_active;
    uint8_t chassis_motor_online;
    uint8_t climb_motor_online;
    uint8_t yaw_tune_running;
    uint8_t enable_flags = 0U;
    uint8_t executing_flags = 0U;
    uint8_t error_flags = 0U;
    uint8_t online_flags = 0U;
    uint32_t now_tick;
    uint32_t climb_elapsed_ms = 0U;
    USB_CommandRxState_t usb_rx;
    CRC_Debug_t usart_rx;
    R2_Move_Ctrl_t chs;
    R2_Climb_Ctrl_t climb;
    R2_ArmStatus_t arm;
    R2_LaserMeasure_t laser;
    R2_YawAutoTuneStatus_t yaw_tune;
    INS_NavState_t nav;
    volatile RobotStatusView_t *view = &g_robot_status_view;

    now_tick = HAL_GetTick();
    USB_GetCommandRxState(&usb_rx);
    INS_GetState(&nav);
    R2_LaserUser_GetMeasure(&laser);
    R2_YawAutoTune_GetStatus(&yaw_tune);

    taskENTER_CRITICAL();
    active_source = (USB_Task_flag != 0U) ? CONTROL_SOURCE_USB :
                    ((USART_Task_flag != 0U) ? CONTROL_SOURCE_USART : CONTROL_SOURCE_NONE);
    chs = (active_source == CONTROL_SOURCE_USB) ? g_r2_ctrl_usb : g_r2_ctrl_usart;
    climb = (active_source == CONTROL_SOURCE_USB) ? g_r2_climb_usb : g_r2_climb_usart;
    R2_Arm_GetStatus(&g_r2_arm_usb, &arm);
    usart_rx = crc_dbg;
    taskEXIT_CRITICAL();

    usb_recent = ((usb_rx.last_tick != 0U) &&
                  ((now_tick - usb_rx.last_tick) <= ROBOT_CMD_RECENT_MS)) ? 1U : 0U;
    usart_recent = ((usart_rx.last_frame_tick != 0U) &&
                    ((now_tick - usart_rx.last_frame_tick) <= ROBOT_CMD_RECENT_MS)) ? 1U : 0U;

    active_source_stale = 0U;
    if ((active_source == CONTROL_SOURCE_USB) && (usb_recent == 0U))
    {
        active_source_stale = 1U;
    }
    else if ((active_source == CONTROL_SOURCE_USART) && (usart_recent == 0U))
    {
        active_source_stale = 1U;
    }
    else if (active_source == CONTROL_SOURCE_NONE)
    {
        active_source_stale = 1U;
    }

    chassis_pos_running = (chs.pos_state == R2_POS_RUNNING) ? 1U : 0U;
    chassis_moving = ((chassis_pos_running != 0U) ||
                      (AbsF(chs.robot_vel.vx) > ROBOT_VEL_MOVE_TOL) ||
                      (AbsF(chs.robot_vel.vy) > ROBOT_VEL_MOVE_TOL) ||
                      (AbsF(chs.robot_vel.vw) > ROBOT_WZ_MOVE_TOL)) ? 1U : 0U;

    climb_motor_active = R2_Climb_IsMotorActive(&climb);
    arm_motor_active = R2_Arm_IsMotorActive(&g_r2_arm_usb);
    if ((climb.state_start_ms != 0U) &&
        (climb.last_update_ms >= climb.state_start_ms))
    {
        climb_elapsed_ms = climb.last_update_ms - climb.state_start_ms;
    }

    chassis_motor_online = MotorOnline(motor_fdcan1, CHASSIS_MOTOR_COUNT);
    climb_motor_online = ClimbMotorOnline();
    yaw_tune_running = (yaw_tune.state == (uint8_t)R2_YAW_AUTOTUNE_RUNNING) ? 1U : 0U;

    if (Mecanum_control_flag != 0U) enable_flags |= 0x01U;
    if (climb.enabled != 0U) enable_flags |= 0x08U;
    if (arm.enabled != 0U) enable_flags |= 0x10U;

    if (chassis_moving != 0U) executing_flags |= 0x01U;
    if (chassis_pos_running != 0U) executing_flags |= 0x02U;
    if (climb_motor_active != 0U) executing_flags |= 0x10U;
    if (yaw_tune_running != 0U) executing_flags |= 0x20U;
    if (arm_motor_active != 0U) executing_flags |= 0x40U;
    if (executing_flags != 0U) executing_flags |= 0x80U;

    timeout_flags = USB_ControlWatchdog_TimeoutFlags();
    if (usart_rx.control_timeout != 0U) timeout_flags |= 0x08U;
    if ((timeout_flags & 0x01U) != 0U) error_flags |= 0x01U;
    if (nav.imu_online == 0U) error_flags |= 0x08U;
    if (chs.emergency_stop != 0U) error_flags |= 0x10U;
    if (arm.error_flags != 0U) error_flags |= 0x20U;
    if (active_source_stale != 0U) error_flags |= 0x80U;

    if (usb_recent != 0U) online_flags |= 0x01U;
    if (usart_recent != 0U) online_flags |= 0x02U;
    if (nav.imu_online != 0U) online_flags |= 0x04U;
    if (chassis_motor_online >= CHASSIS_MOTOR_COUNT) online_flags |= 0x08U;
    if (arm.last_update_ms != 0U) online_flags |= 0x10U;
    if (climb_motor_online >= CLIMB_MOTOR_ONLINE_COUNT) online_flags |= 0x20U;
    if (laser.all_online != 0U) online_flags |= 0x40U;

    view->summary.tick_ms = now_tick;
    view->summary.active_source = active_source;
    view->summary.enable_flags = enable_flags;
    view->summary.executing_flags = executing_flags;
    view->summary.error_flags = error_flags;
    view->summary.online_flags = online_flags;
    view->summary.timeout_flags = timeout_flags;
    view->summary.active_source_stale = active_source_stale;

    view->rx.usb_count = usb_rx.count;
    view->rx.usb_last_tick = usb_rx.last_tick;
    view->rx.usb_last_cmd = usb_rx.last_cmd;
    view->rx.usb_last_len = usb_rx.last_len;
    view->rx.usb_payload_valid = usb_rx.last_payload_valid;
    view->rx.usb_recent = usb_recent;
    for (i = 0U; i < ROBOT_STATUS_VIEW_USB_DATA_LEN; i++)
    {
        view->rx.usb_last_data[i] = usb_rx.last_data[i];
    }
    for (i = 0U; i < ROBOT_STATUS_VIEW_FLOAT_COUNT; i++)
    {
        view->rx.usb_last_f[i] = usb_rx.last_f[i];
    }

    view->rx.usart_frame_count = usart_rx.frame_count;
    view->rx.usart_last_tick = usart_rx.last_frame_tick;
    view->rx.usart_checksum_fail_count = usart_rx.checksum_fail_count;
    view->rx.usart_checksum_ok = usart_rx.checksum_ok;
    view->rx.usart_control_timeout = usart_rx.control_timeout;
    view->rx.usart_recent = usart_recent;
    view->rx.usart_mode = usart_rx.mode;
    view->rx.usart_chassis_param[0] = usart_rx.chs_p1;
    view->rx.usart_chassis_param[1] = usart_rx.chs_p2;
    view->rx.usart_chassis_param[2] = usart_rx.chs_p3;
    view->rx.usart_source_flag = usart_rx.source_flag;
    view->rx.usart_climb_enable = usart_rx.climb_enable;
    view->rx.usart_climb_step = usart_rx.climb_step;
    view->rx.usart_climb_auto = usart_rx.climb_auto;

    view->chassis.enabled = Mecanum_control_flag;
    view->chassis.mode = (uint8_t)chs.mode;
    view->chassis.pos_state = (uint8_t)chs.pos_state;
    view->chassis.emergency_stop = chs.emergency_stop;
    view->chassis.moving = chassis_moving;
    view->chassis.motor_online = chassis_motor_online;
    view->chassis.target_vx = chs.target_vx;
    view->chassis.target_vy = chs.target_vy;
    view->chassis.target_vw = chs.target_vw;
    view->chassis.target_dx = chs.target_dx;
    view->chassis.target_dy = chs.target_dy;
    view->chassis.target_dyaw = chs.target_dyaw;
    view->chassis.vel_vx = chs.robot_vel.vx;
    view->chassis.vel_vy = chs.robot_vel.vy;
    view->chassis.vel_vw = chs.robot_vel.vw;
    view->chassis.odom_x = chs.odom_x;
    view->chassis.odom_y = chs.odom_y;
    view->chassis.odom_yaw = chs.odom_yaw;
    view->chassis.nav_x = nav.x_m;
    view->chassis.nav_y = nav.y_m;
    view->chassis.nav_yaw = nav.yaw_total_rad;
    view->chassis.nav_vx = nav.vx_mps;
    view->chassis.nav_vy = nav.vy_mps;
    view->chassis.nav_wz = nav.wz_radps;
    view->chassis.wheel_speed[0] = chs.wheel_speed.fl;
    view->chassis.wheel_speed[1] = chs.wheel_speed.fr;
    view->chassis.wheel_speed[2] = chs.wheel_speed.bl;
    view->chassis.wheel_speed[3] = chs.wheel_speed.br;
    view->chassis.pos_progress = chs.pos_progress;
    view->chassis.pos_err_x = chs.pos_err_x;
    view->chassis.pos_err_y = chs.pos_err_y;
    view->chassis.pos_err_yaw = chs.pos_err_yaw;

    view->arm.enabled = arm.enabled;
    view->arm.has_target = arm.has_target;
    view->arm.output_enabled = arm.output_enabled;
    view->arm.state = arm.state;
    view->arm.status_flags = arm.status_flags;
    view->arm.error_flags = arm.error_flags;
    view->arm.ik_status = arm.last_ik_status;
    view->arm.ik_reason = arm.last_ik_reason;
    view->arm.last_command_ms = arm.last_command_ms;
    view->arm.last_update_ms = arm.last_update_ms;
    view->arm.output_apply_count = arm.output_apply_count;
    view->arm.tool = (uint8_t)arm.request.tool;
    view->arm.tool_state = (uint8_t)arm.request.state;
    view->arm.reserved[0] = 0U;
    view->arm.reserved[1] = 0U;
    view->arm.target_z_mm = arm.request.target_z_mm;
    view->arm.approach_yaw_rad = arm.request.approach_yaw_rad;
    for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
    {
        view->arm.theta_rad[i] = arm.result.theta_rad[i];
    }
    view->arm.tool_world_mm[0] = arm.result.tool_world_mm.x;
    view->arm.tool_world_mm[1] = arm.result.tool_world_mm.y;
    view->arm.tool_world_mm[2] = arm.result.tool_world_mm.z;

    view->climb.source = active_source;
    view->climb.enabled = climb.enabled;
    view->climb.state = (uint8_t)climb.state;
    view->climb.auto_run = climb.auto_run;
    view->climb.state_done = climb.state_done;
    view->climb.error_flags = climb.error_flags;
    view->climb.flow = climb.flow;
    view->climb.pending_step = climb.pending_step;
    view->climb.pending_auto = climb.pending_auto;
    view->climb.pending_test_action = climb.pending_test_action;
    view->climb.test_action = climb.test_action;
    view->climb.test_active = climb.test_active;
    view->climb.test_chassis_active = climb.test_chassis_active;
    view->climb.motor_active = climb_motor_active;
    view->climb.motor_online = climb_motor_online;
    view->climb.elapsed_ms = climb_elapsed_ms;
    view->climb.last_update_ms = climb.last_update_ms;
    for (i = 0U; i < ROBOT_STATUS_VIEW_CLIMB_LEGS; i++)
    {
        view->climb.leg_pos_mm[i] = climb.leg_pos_mm[i];
        view->climb.leg_target_mm[i] = climb.leg_target_mm[i];
    }
    for (i = 0U; i < ROBOT_STATUS_VIEW_CLIMB_DRIVES; i++)
    {
        view->climb.drive_pos_mm[i] = climb.drive_pos_mm[i];
        view->climb.drive_target_mm[i] = climb.drive_target_mm[i];
    }

    view->laser.valid_flags =
        ((laser.x_pos_valid != 0U) ? 0x01U : 0U) |
        ((laser.y_pos_valid != 0U) ? 0x02U : 0U) |
        ((laser.height_valid != 0U) ? 0x04U : 0U);
    view->laser.online_flags =
        ((laser.x_pos_online != 0U) ? 0x01U : 0U) |
        ((laser.y_pos_online != 0U) ? 0x02U : 0U) |
        ((laser.height_online != 0U) ? 0x04U : 0U);
    view->laser.waiting_flags =
        ((laser.channel[0].waiting_response != 0U) ? 0x01U : 0U) |
        ((laser.channel[1].waiting_response != 0U) ? 0x02U : 0U) |
        ((laser.channel[2].waiting_response != 0U) ? 0x04U : 0U);
    view->laser.all_valid = laser.all_valid;
    view->laser.all_online = laser.all_online;
    view->laser.reserved[0] = 0U;
    view->laser.reserved[1] = 0U;
    view->laser.reserved[2] = 0U;
    view->laser.update_tick = laser.update_tick;
    view->laser.distance_mm[0] = laser.x_pos_mm;
    view->laser.distance_mm[1] = laser.y_pos_mm;
    view->laser.distance_mm[2] = laser.height_mm;
    view->laser.raw_distance_mm[0] = laser.x_pos_raw_mm;
    view->laser.raw_distance_mm[1] = laser.y_pos_raw_mm;
    view->laser.raw_distance_mm[2] = laser.height_raw_mm;
    for (i = 0U; i < ROBOT_STATUS_VIEW_LASER_COUNT; i++)
    {
        view->laser.offset_mm[i] = laser.channel[i].offset_mm;
        view->laser.last_update_tick[i] = laser.channel[i].last_update_tick;
        view->laser.timeout_count[i] = laser.channel[i].timeout_count;
        view->laser.crc_error_count[i] = laser.channel[i].crc_error_count;
        view->laser.parse_error_count[i] = laser.channel[i].parse_error_count;
        view->laser.uart_error_count[i] = laser.channel[i].uart_error_count;
    }

    view->yaw_tune.state = yaw_tune.state;
    view->yaw_tune.segment_index = yaw_tune.segment_index;
    view->yaw_tune.segment_count = yaw_tune.segment_count;
    view->yaw_tune.pass_index = yaw_tune.pass_index;
    view->yaw_tune.pass_count = yaw_tune.pass_count;
    view->yaw_tune.fail_reason = yaw_tune.fail_reason;
    view->yaw_tune.active_mode = yaw_tune.active_mode;
    view->yaw_tune.phase = yaw_tune.phase;
    view->yaw_tune.tick_ms = yaw_tune.tick_ms;
    view->yaw_tune.segment_elapsed_ms = yaw_tune.segment_elapsed_ms;
    view->yaw_tune.yaw_error_deg = yaw_tune.yaw_error_deg;
    view->yaw_tune.yaw_error_abs_max_deg = yaw_tune.yaw_error_abs_max_deg;
    view->yaw_tune.yaw_rate_error_rms_dps = yaw_tune.yaw_rate_error_rms_dps;
    view->yaw_tune.gyro_z_abs_max_dps = yaw_tune.gyro_z_abs_max_dps;
    view->yaw_tune.wheel_rpm_abs_max = yaw_tune.wheel_rpm_abs_max;
    view->yaw_tune.score = yaw_tune.score;
    view->yaw_tune.last_adjust = yaw_tune.last_adjust;
    view->yaw_tune.angle_kp = yaw_tune.angle_kp;
    view->yaw_tune.angle_ki = yaw_tune.angle_ki;
    view->yaw_tune.angle_kd = yaw_tune.angle_kd;
    view->yaw_tune.rate_kp = yaw_tune.rate_kp;
    view->yaw_tune.rate_ki = yaw_tune.rate_ki;
    view->yaw_tune.rate_kd = yaw_tune.rate_kd;
    view->yaw_tune.pos_kp_yaw = yaw_tune.pos_kp_yaw;
}

static void SendRobotStatus(void)
{
    uint8_t buf[ROBOT_STATUS_LEN];
    uint8_t i;
    const volatile RobotStatusView_t *view = &g_robot_status_view;

    for (i = 0U; i < ROBOT_STATUS_LEN; i++)
    {
        buf[i] = 0U;
    }

    RobotStatusView_Update();

    buf[0] = ROBOT_STATUS_PROTOCOL_VERSION;
    buf[1] = view->summary.active_source;
    buf[2] = view->summary.enable_flags;
    buf[3] = view->summary.executing_flags;
    buf[4] = view->summary.error_flags;
    buf[5] = view->summary.online_flags;
    buf[6] = view->rx.usb_last_cmd;
    buf[7] = view->rx.usb_last_len;
    PackU32LE(view->rx.usb_count, buf, 8);
    PackU32LE(view->rx.usb_last_tick, buf, 12);
    PackU32LE(view->rx.usart_frame_count, buf, 16);
    PackU32LE(view->rx.usart_last_tick, buf, 20);
    PackU32LE(view->rx.usart_checksum_fail_count, buf, 24);
    buf[28] = view->rx.usart_checksum_ok;
    buf[29] = view->rx.usart_mode;
    buf[30] = 0U;
    buf[31] = 0U;
    PackFloatLE(view->chassis.nav_x, buf, 32);
    PackFloatLE(view->chassis.nav_y, buf, 36);
    PackFloatLE(view->chassis.nav_yaw, buf, 40);
    PackFloatLE(view->chassis.nav_vx, buf, 44);
    PackFloatLE(view->chassis.nav_vy, buf, 48);
    PackFloatLE(view->chassis.nav_wz, buf, 52);
    PackFloatLE(view->chassis.odom_x, buf, 56);
    PackFloatLE(view->chassis.odom_y, buf, 60);
    PackFloatLE(view->chassis.odom_yaw, buf, 64);
    PackFloatLE(view->chassis.vel_vx, buf, 68);
    PackFloatLE(view->chassis.vel_vy, buf, 72);
    PackFloatLE(view->chassis.vel_vw, buf, 76);
    buf[94] = view->chassis.pos_state;
    buf[95] = view->summary.timeout_flags;

    buf[96] = view->climb.source;
    buf[97] = view->climb.state;
    buf[98] = view->climb.enabled;
    buf[99] = view->climb.auto_run;
    buf[100] = view->climb.state_done;
    buf[101] = view->climb.error_flags;
    buf[102] = view->climb.pending_step;
    buf[103] = view->climb.pending_auto;
    buf[104] = view->climb.motor_active;
    buf[105] = view->climb.motor_online;
    buf[106] = view->climb.test_action;
    buf[107] = ((view->climb.test_active != 0U) ? 0x01U : 0U) |
               ((view->climb.test_chassis_active != 0U) ? 0x02U : 0U) |
               ((view->climb.flow == (uint8_t)R2_CLIMB_FLOW_DOWNSTAIRS) ? 0x04U : 0U);
    PackU32LE(view->climb.elapsed_ms, buf, 108);
    PackU32LE(view->climb.last_update_ms, buf, 112);

    buf[116] = view->laser.valid_flags;
    buf[117] = view->laser.online_flags;
    buf[118] = view->laser.waiting_flags;
    buf[119] = view->laser.all_valid;
    buf[120] = view->laser.all_online;
    PackU32LE(view->laser.update_tick, buf, 124);
    PackI32LE(view->laser.distance_mm[0], buf, 128);
    PackI32LE(view->laser.distance_mm[1], buf, 132);
    PackI32LE(view->laser.distance_mm[2], buf, 136);

    buf[140] = view->yaw_tune.state;
    buf[141] = view->yaw_tune.fail_reason;
    buf[142] = view->yaw_tune.phase;
    buf[143] = view->yaw_tune.active_mode;
    PackU32LE(view->yaw_tune.tick_ms, buf, 144);
    PackU32LE(view->yaw_tune.segment_elapsed_ms, buf, 148);
    PackFloatLE(view->yaw_tune.yaw_error_deg, buf, 152);
    PackFloatLE(view->yaw_tune.score, buf, 156);

    PackFloatLE(view->chassis.target_vx, buf, 160);
    PackFloatLE(view->chassis.target_vy, buf, 164);
    PackFloatLE(view->chassis.target_vw, buf, 168);
    PackFloatLE(view->chassis.target_dx, buf, 172);
    PackFloatLE(view->chassis.target_dy, buf, 176);
    PackFloatLE(view->chassis.target_dyaw, buf, 180);

    buf[184] = view->arm.enabled;
    buf[185] = view->arm.has_target;
    buf[186] = view->arm.output_enabled;
    buf[187] = view->arm.state;
    buf[188] = view->arm.status_flags;
    buf[189] = view->arm.error_flags;
    buf[190] = view->arm.ik_status;
    buf[191] = view->arm.ik_reason;
    PackFloatLE(view->arm.target_z_mm, buf, 192);
    PackFloatLE(view->arm.approach_yaw_rad, buf, 196);
    PackFloatLE(view->arm.theta_rad[0], buf, 200);
    PackFloatLE(view->arm.theta_rad[1], buf, 204);
    PackFloatLE(view->arm.theta_rad[2], buf, 208);
    PackFloatLE(view->arm.tool_world_mm[0], buf, 212);
    PackFloatLE(view->arm.tool_world_mm[1], buf, 216);
    PackFloatLE(view->arm.tool_world_mm[2], buf, 220);
    PackU32LE(view->arm.last_update_ms, buf, 224);
    PackU32LE(view->arm.output_apply_count, buf, 228);
    buf[232] = view->arm.tool;
    buf[233] = view->arm.tool_state;

    buf[238] = view->climb.error_flags;
    buf[239] = view->summary.active_source_stale;

    Send_Cmd_Data(USB_CMD_ROBOT_GET_STATUS, buf, ROBOT_STATUS_LEN);
}

void PC_TX_Task(void const *argument)
{
    uint8_t status_cmd;
    uint32_t last_laser_poll_tick;

    (void)argument;

    R2_LaserUser_Init();
    last_laser_poll_tick = osKernelSysTick();

    for (;;)
    {
        uint32_t now_tick = osKernelSysTick();

        if ((now_tick - last_laser_poll_tick) >= R2_LASER_POLL_PERIOD_MS)
        {
            last_laser_poll_tick = now_tick;
            R2_LaserUser_Poll10ms();
        }

        RobotStatusView_Update();

        if (PC_TX_DequeueStatus(&status_cmd) != 0U)
        {
            switch (status_cmd)
            {
            case USB_CMD_SYS_GET_STATUS:
                SendSysStatus();
                break;
            case USB_CMD_CHS_GET_STATUS:
                SendChsStatus();
                break;
            case USB_CMD_ARM_GET_STATUS:
                SendArmStatus(USB_CMD_ARM_GET_STATUS);
                break;
            case USB_CMD_TOOL_GET_STATUS:
                SendArmStatus(USB_CMD_TOOL_GET_STATUS);
                break;
            case USB_CMD_ROBOT_GET_STATUS:
                SendRobotStatus();
                break;
            case USB_CMD_CLIMB_GET_STATUS:
                SendClimbStatus();
                break;
            case USB_CMD_YAW_TUNE_GET_STATUS:
                SendYawTuneStatus();
                break;
            default:
                break;
            }
        }

        osDelay(5);
    }
}
