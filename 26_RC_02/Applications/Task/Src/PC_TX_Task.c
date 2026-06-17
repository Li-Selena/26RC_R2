#include "PC_TX_Task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_usb.h"
#include "R2_move.h"
#include "arm_user.h"
#include "arm_tools.h"
#include "fdcan_receive.h"
#include "INS_Task.h"
#include "CRC.h"

extern motor_measure_t motor_fdcan2[8];

/* ── 外部引用 ── */

/* 底盘控制器（USB 通道） */
extern R2_Move_Ctrl_t g_r2_ctrl_usb;
extern R2_Move_Ctrl_t g_r2_ctrl_usart;

/* 任务/模块标志 */
extern volatile uint8_t USB_Task_flag;
extern volatile uint8_t USART_Task_flag;
extern uint8_t Mecanum_control_flag;
extern uint8_t Arm_control_flag;
extern uint8_t Tool_control_flag;

/* 电机数据 */
extern motor_measure_t motor_fdcan1[8];  /* 底盘，在线检测用 */
extern motor_measure_t motor_fdcan3[8];  /* 机械臂，实际编码器回传 */


/* ═══════════════════════════════════════════════════════
 *  状态请求标志（Data_Analysis 设置，本任务消费）
 * ═══════════════════════════════════════════════════════ */

#define PC_TX_STATUS_QUEUE_LEN  16U

static volatile uint8_t  s_status_queue[PC_TX_STATUS_QUEUE_LEN];
static volatile uint8_t  s_status_q_head = 0U;
static volatile uint8_t  s_status_q_tail = 0U;
static volatile uint8_t  s_status_q_count = 0U;
static volatile uint32_t s_status_q_drop_count = 0U;

static void PC_TX_EnqueueStatus(uint8_t cmd)
{
    taskENTER_CRITICAL();
    if (s_status_q_count < PC_TX_STATUS_QUEUE_LEN) {
        s_status_queue[s_status_q_tail] = cmd;
        s_status_q_tail++;
        if (s_status_q_tail >= PC_TX_STATUS_QUEUE_LEN) {
            s_status_q_tail = 0U;
        }
        s_status_q_count++;
    } else {
        s_status_q_drop_count++;
    }
    taskEXIT_CRITICAL();
}

static uint8_t PC_TX_DequeueStatus(uint8_t *cmd)
{
    uint8_t ok = 0U;

    taskENTER_CRITICAL();
    if (s_status_q_count != 0U) {
        *cmd = s_status_queue[s_status_q_head];
        s_status_q_head++;
        if (s_status_q_head >= PC_TX_STATUS_QUEUE_LEN) {
            s_status_q_head = 0U;
        }
        s_status_q_count--;
        ok = 1U;
    }
    taskEXIT_CRITICAL();

    return ok;
}


/* ── Data_Analysis 调用的请求接口 ── */

void PC_TX_ReqSysStatus(void)   { PC_TX_EnqueueStatus(USB_CMD_SYS_GET_STATUS); }
void PC_TX_ReqChsStatus(void)   { PC_TX_EnqueueStatus(USB_CMD_CHS_GET_STATUS); }
void PC_TX_ReqArmStatus(void)   { PC_TX_EnqueueStatus(USB_CMD_ARM_GET_STATUS); }
void PC_TX_ReqToolStatus(void)  { PC_TX_EnqueueStatus(USB_CMD_TOOL_GET_STATUS); }
void PC_TX_ReqRobotStatus(void) { PC_TX_EnqueueStatus(USB_CMD_ROBOT_GET_STATUS); }
void PC_TX_ReqClimbStatus(void) { PC_TX_EnqueueStatus(USB_CMD_CLIMB_GET_STATUS); }


/* ═══════════════════════════════════════════════════════
 *  工具函数
 * ═══════════════════════════════════════════════════════ */

/* float → 4 字节小端，写入 buf[offset..offset+3] */
static void PackFloatLE(float v, uint8_t *buf, uint8_t offset)
{
    union { float f; uint8_t b[4]; } u;
    u.f = v;
    buf[offset    ] = u.b[0];
    buf[offset + 1] = u.b[1];
    buf[offset + 2] = u.b[2];
    buf[offset + 3] = u.b[3];
}

static void PackU32LE(uint32_t v, uint8_t *buf, uint8_t offset)
{
    buf[offset    ] = (uint8_t)( v        & 0xFFU);
    buf[offset + 1] = (uint8_t)((v >>  8) & 0xFFU);
    buf[offset + 2] = (uint8_t)((v >> 16) & 0xFFU);
    buf[offset + 3] = (uint8_t)((v >> 24) & 0xFFU);
}

static float AbsF(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* 电机在线计数（msg_cnt > 50 视为在线） */
static uint8_t MotorOnline(const motor_measure_t *m, uint8_t n)
{
    uint8_t cnt = 0U;
    uint8_t i;
    for (i = 0U; i < n; i++) {
        if (m[i].msg_cnt > 50U) cnt++;
    }
    return cnt;
}


/* ═══════════════════════════════════════════════════════
 *  System 状态回传 (cmd=0x06) — LEN=8
 * ═══════════════════════════════════════════════════════
 *
 *  [0] USB_Task_flag
 *  [1] USART_Task_flag
 *  [2] Mecanum_control_flag
 *  [3] Arm_control_flag
 *  [4] Tool_control_flag
 *  [5] 底盘电机在线数 (0~4)
 *  [6..7] reserved
 */
static void SendSysStatus(void)
{
    uint8_t buf[8];
    buf[0] = USB_Task_flag;
    buf[1] = USART_Task_flag;
    buf[2] = Mecanum_control_flag;
    buf[3] = Arm_control_flag;
    buf[4] = Tool_control_flag;
    buf[5] = MotorOnline(motor_fdcan1, 4U);
    buf[6] = USB_ControlWatchdog_TimeoutFlags();
    buf[7] = 0U;
    Send_Cmd_Data(USB_CMD_SYS_GET_STATUS, buf, 8U);
}


/* ═══════════════════════════════════════════════════════
 *  Chassis 状态回传 (cmd=0x16) — LEN=56
 * ═══════════════════════════════════════════════════════
 *
 *  当前模式 / 运动状态 / 实时速度 / 实时里程计 / 运动限制 / 世界系朝向
 *  / POS 诊断（进度 + 位置误差，供上位机判断真实到位）
 *
 *  [0]     mode            (uint8, 0~7)
 *  [1]     pos_state       (uint8, 0=IDLE 1=RUNNING 2=DONE)
 *  [2]     emergency_stop  (uint8, 0/1)
 *  [3]     reserved
 *  [4..7]  robot_vel.vx    (float LE, m/s)
 *  [8..11] robot_vel.vy    (float LE, m/s)
 *  [12..15]robot_vel.vw    (float LE, rad/s)
 *  [16..19]odom_x          (float LE, m)
 *  [20..23]odom_y          (float LE, m)
 *  [24..27]odom_yaw        (float LE, rad)
 *  [28..31]v_max           (float LE, m/s)
 *  [32..35]a_max           (float LE, m/s²)
 *  [36..39]j_max           (float LE, m/s³)
 *  [40..43]pos_progress    (float LE, 0~1,  当前运动进度)
 *  [44..47]pos_err_x       (float LE, m,    位置误差)
 *  [48..51]pos_err_y       (float LE, m)
 *  [52..55]pos_err_yaw     (float LE, rad,  yaw 误差)
 */
#define CHS_STATUS_LEN  88U

static void SendChsStatus(void)
{
    uint8_t buf[CHS_STATUS_LEN];
    R2_Move_Ctrl_t snapshot;
    const R2_Move_Ctrl_t *c = &snapshot;
    INS_NavState_t nav;

    taskENTER_CRITICAL();
    snapshot = g_r2_ctrl_usb;
    taskEXIT_CRITICAL();
    INS_GetState(&nav);

    buf[0] = (uint8_t)c->mode;
    buf[1] = (uint8_t)c->pos_state;
    buf[2] = c->emergency_stop;
    buf[3] = 0U;   /* reserved */

    PackFloatLE(c->robot_vel.vx, buf,  4);
    PackFloatLE(c->robot_vel.vy, buf,  8);
    PackFloatLE(c->robot_vel.vw, buf, 12);
    PackFloatLE(c->odom_x,       buf, 16);
    PackFloatLE(c->odom_y,       buf, 20);
    PackFloatLE(c->odom_yaw,     buf, 24);
    PackFloatLE(c->v_max,        buf, 28);
    PackFloatLE(c->a_max,        buf, 32);
    PackFloatLE(c->j_max,        buf, 36);
    PackFloatLE(c->pos_progress, buf, 40);
    PackFloatLE(c->pos_err_x,    buf, 44);
    PackFloatLE(c->pos_err_y,    buf, 48);
    PackFloatLE(c->pos_err_yaw,  buf, 52);
    PackFloatLE(nav.x_m,         buf, 56);
    PackFloatLE(nav.y_m,         buf, 60);
    PackFloatLE(nav.yaw_rad,     buf, 64);
    PackFloatLE(nav.yaw_total_rad, buf, 68);
    PackFloatLE(nav.vx_mps,      buf, 72);
    PackFloatLE(nav.vy_mps,      buf, 76);
    PackFloatLE(nav.wz_radps,    buf, 80);
    buf[84] = nav.imu_online;
    buf[85] = USB_ControlWatchdog_TimeoutFlags();
    buf[86] = 0U;
    buf[87] = 0U;

    Send_Cmd_Data(USB_CMD_CHS_GET_STATUS, buf, CHS_STATUS_LEN);
}


/* ═══════════════════════════════════════════════════════
 *  Arm 状态回传 (cmd=0x26) — LEN=40
 * ═══════════════════════════════════════════════════════
 *
 *  逆解结果 / 命令采纳 / 电机目标值 / 电机实际编码器（物理闭环）
 *
 *  [0]     has_last_valid   (uint8)
 *  [1]     last_status_code (uint8, 0=OK 1=UNREACHABLE 2=UNSAFE 3=PARAM_ERR)
 *  [2]     last_action_code (uint8, 0=APPLY_NEW 1=HOLD_LAST 2=KEEP_CURRENT)
 *  [3]     reserved
 *  [4..7]  model_theta1     (float LE, rad) — 逆解结果
 *  [8..11] model_theta2     (float LE, rad)
 *  [12..15]model_theta3     (float LE, rad)
 *  [16..19]motor_j1_target  (float LE, deg) — 电机目标值
 *  [20..23]motor_j2_target  (float LE, deg)
 *  [24..27]motor_j3_target  (float LE, deg)
 *  [28..31]actual_j1_deg    (float LE, deg) — 编码器实际值（物理闭环）
 *  [32..35]actual_j2_deg    (float LE, deg)
 *  [36..39]actual_j3_deg    (float LE, deg)
 */
#define ARM_STATUS_LEN  40U

/*
 * 编码器 → 关节角换算系数（与 CAN_Task 一致）。
 * 符号方向：pid_call_3 中 J1 传入负值，此处回传同样取反。
 */
#define ARM_TNUM1   0.0002464f   /* 360/8192/36/94*19, J1 编码器→deg */
#define ARM_TNUM23  0.0004577f   /* 360/8192/3591*187/5, J2/J3 编码器→deg */

static void SendArmStatus(void)
{
    uint8_t buf[ARM_STATUS_LEN];
    const ArmIK_AppState_t *app = ArmIK_GetAppState();
    float actual_j1, actual_j2, actual_j3;

    if (app == NULL) {
        for (uint8_t i = 0U; i < ARM_STATUS_LEN; i++) buf[i] = 0U;
        Send_Cmd_Data(USB_CMD_ARM_GET_STATUS, buf, ARM_STATUS_LEN);
        return;
    }

    buf[0] = app->has_last_valid;
    buf[1] = app->last_status_code;
    buf[2] = app->last_action_code;
    buf[3] = 0U;

    /* 逆解结果 — 模型控制角 (rad) */
    PackFloatLE(app->active_model.theta1, buf,  4);
    PackFloatLE(app->active_model.theta2, buf,  8);
    PackFloatLE(app->active_model.theta3, buf, 12);

    /* 电机目标值 (deg) */
    PackFloatLE(app->active_motor_deg.j1_deg, buf, 16);
    PackFloatLE(app->active_motor_deg.j2_deg, buf, 20);
    PackFloatLE(app->active_motor_deg.j3_deg, buf, 24);

    /* 编码器实际值 → 关节角 (deg)，符号与 CAN_Task pid_call_3 一致 */
    actual_j1 = -(float)motor_fdcan3[0].total_angle * ARM_TNUM1;
    actual_j2 =  (float)motor_fdcan3[1].total_angle * ARM_TNUM23;
    actual_j3 =  (float)motor_fdcan3[2].total_angle * ARM_TNUM23;

    PackFloatLE(actual_j1, buf, 28);
    PackFloatLE(actual_j2, buf, 32);
    PackFloatLE(actual_j3, buf, 36);

    Send_Cmd_Data(USB_CMD_ARM_GET_STATUS, buf, ARM_STATUS_LEN);
}


/* ═══════════════════════════════════════════════════════
 *  Tool 状态回传 (cmd=0x36) — LEN=16
 * ═══════════════════════════════════════════════════════
 *
 *  当前状态 / 实体状态 / 真实角度 / 安全标志
 *
 *  [0]     tool_dev        (uint8, 0=吸盘 1=夹爪)
 *  [1]     clamp.state     (uint8, 0=CLOSE 1=OPEN)
 *  [2]     clamp.run_status(uint8, 0=IDLE 1=MOVING 2=ERROR)
 *  [3]     clamp.safe_flag (uint8, 0=UNSAFE 1=SAFE)
 *  [4..7]  clamp.real_angle(float LE)
 *  [8]     chuck.state     (uint8)
 *  [9]     chuck.run_status(uint8)
 *  [10]    chuck.safe_flag (uint8)
 *  [11]    active_source   (uint8, 0=USART 1=USB)
 *  [12..15]chuck.real_angle(float LE)
 */
#define TOOL_STATUS_LEN  16U

static void SendToolStatus(void)
{
    uint8_t buf[TOOL_STATUS_LEN];
    uint8_t source;
    uint8_t selected_dev;
    clamp_Handle_t clamp_snapshot;
    chuck_Handle_t chuck_snapshot;

    taskENTER_CRITICAL();
    source = Tool_GetActiveSource();
    selected_dev = Tool_GetSelectedDev(source);
    clamp_snapshot = *Tool_GetClamp(source);
    chuck_snapshot = *Tool_GetChuck(source);
    taskEXIT_CRITICAL();

    buf[0] = selected_dev;
    buf[1] = clamp_snapshot.state;
    buf[2] = (uint8_t)clamp_snapshot.run_status;
    buf[3] = clamp_snapshot.safe_flag;
    PackFloatLE(clamp_snapshot.real_angle, buf, 4);

    buf[8]  = chuck_snapshot.state;
    buf[9]  = (uint8_t)chuck_snapshot.run_status;
    buf[10] = chuck_snapshot.safe_flag;
    buf[11] = source;
    PackFloatLE(chuck_snapshot.real_angle, buf, 12);

    Send_Cmd_Data(USB_CMD_TOOL_GET_STATUS, buf, TOOL_STATUS_LEN);
}

#define CLIMB_STATUS_LEN  64U

static void SendClimbStatus(void)
{
    uint8_t buf[CLIMB_STATUS_LEN];
    uint8_t i;
    uint8_t active_source;
    R2_Climb_Ctrl_t climb;
    uint32_t elapsed_ms = 0U;

    for (i = 0U; i < CLIMB_STATUS_LEN; i++) {
        buf[i] = 0U;
    }

    taskENTER_CRITICAL();
    active_source = (USB_Task_flag != 0U) ? TOOL_USB_SOURCE :
                    ((USART_Task_flag != 0U) ? TOOL_USART_SOURCE : 2U);
    climb = (active_source == TOOL_USB_SOURCE) ?
            g_r2_climb_usb : g_r2_climb_usart;
    taskEXIT_CRITICAL();

    if ((climb.state_start_ms != 0U) &&
        (climb.last_update_ms >= climb.state_start_ms)) {
        elapsed_ms = climb.last_update_ms - climb.state_start_ms;
    }

    buf[0] = (uint8_t)climb.state;
    buf[1] = climb.enabled;
    buf[2] = climb.auto_run;
    buf[3] = climb.state_done;
    buf[4] = climb.error_flags;
    buf[5] = active_source;
    buf[6] = MotorOnline(motor_fdcan2, 6U);
    buf[7] = 0U;

    PackU32LE(elapsed_ms,           buf,  8);
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

    Send_Cmd_Data(USB_CMD_CLIMB_GET_STATUS, buf, CLIMB_STATUS_LEN);
}


/* ═══════════════════════════════════════════════════════
 *  PC_TX 主任务 — 5ms 轮询，消费状态请求标志
 * ═══════════════════════════════════════════════════════ */

#define ROBOT_STATUS_LEN        96U
#define ROBOT_CMD_RECENT_MS     500U
#define ROBOT_ARM_MOVE_TOL_DEG  2.0f
#define ROBOT_VEL_MOVE_TOL      0.01f
#define ROBOT_WZ_MOVE_TOL       0.01f

static void SendRobotStatus(void)
{
    uint8_t buf[ROBOT_STATUS_LEN];
    uint8_t i;
    uint8_t active_source;
    uint8_t selected_dev;
    uint8_t timeout_flags;
    uint8_t usb_recent;
    uint8_t usart_recent;
    uint8_t active_source_stale;
    uint8_t chassis_moving;
    uint8_t chassis_pos_running;
    uint8_t arm_moving;
    uint8_t tool_moving;
    uint8_t tool_error;
    uint8_t enable_flags = 0U;
    uint8_t executing_flags = 0U;
    uint8_t error_flags = 0U;
    uint8_t online_flags = 0U;
    uint32_t now_tick;
    USB_CommandRxState_t usb_rx;
    CRC_Debug_t usart_rx;
    R2_Move_Ctrl_t chs;
    INS_NavState_t nav;
    const ArmIK_AppState_t *app;
    clamp_Handle_t clamp_snapshot;
    chuck_Handle_t chuck_snapshot;
    float actual_j1;
    float actual_j2;
    float actual_j3;
    float arm_err_j1 = 0.0f;
    float arm_err_j2 = 0.0f;
    float arm_err_j3 = 0.0f;

    for (i = 0U; i < ROBOT_STATUS_LEN; i++) {
        buf[i] = 0U;
    }

    USB_GetCommandRxState(&usb_rx);
    INS_GetState(&nav);
    app = ArmIK_GetAppState();
    now_tick = HAL_GetTick();

    taskENTER_CRITICAL();
    active_source = (USB_Task_flag != 0U) ? TOOL_USB_SOURCE :
                    ((USART_Task_flag != 0U) ? TOOL_USART_SOURCE : 2U);
    chs = (active_source == TOOL_USB_SOURCE) ? g_r2_ctrl_usb : g_r2_ctrl_usart;
    usart_rx = crc_dbg;
    selected_dev = (active_source <= TOOL_USB_SOURCE) ? Tool_GetSelectedDev(active_source) : 0U;
    clamp_snapshot = *Tool_GetClamp((active_source == TOOL_USB_SOURCE) ? TOOL_USB_SOURCE : TOOL_USART_SOURCE);
    chuck_snapshot = *Tool_GetChuck((active_source == TOOL_USB_SOURCE) ? TOOL_USB_SOURCE : TOOL_USART_SOURCE);
    taskEXIT_CRITICAL();

    usb_recent = ((usb_rx.last_tick != 0U) &&
                  ((now_tick - usb_rx.last_tick) <= ROBOT_CMD_RECENT_MS)) ? 1U : 0U;
    usart_recent = ((usart_rx.last_frame_tick != 0U) &&
                    ((now_tick - usart_rx.last_frame_tick) <= ROBOT_CMD_RECENT_MS)) ? 1U : 0U;

    active_source_stale = 0U;
    if ((active_source == TOOL_USB_SOURCE) && (usb_recent == 0U)) {
        active_source_stale = 1U;
    } else if ((active_source == TOOL_USART_SOURCE) && (usart_recent == 0U)) {
        active_source_stale = 1U;
    } else if (active_source > TOOL_USB_SOURCE) {
        active_source_stale = 1U;
    }

    chassis_pos_running = (chs.pos_state == R2_POS_RUNNING) ? 1U : 0U;
    chassis_moving = ((chassis_pos_running != 0U) ||
                      (AbsF(chs.robot_vel.vx) > ROBOT_VEL_MOVE_TOL) ||
                      (AbsF(chs.robot_vel.vy) > ROBOT_VEL_MOVE_TOL) ||
                      (AbsF(chs.robot_vel.vw) > ROBOT_WZ_MOVE_TOL)) ? 1U : 0U;

    actual_j1 = -(float)motor_fdcan3[0].total_angle * ARM_TNUM1;
    actual_j2 =  (float)motor_fdcan3[1].total_angle * ARM_TNUM23;
    actual_j3 =  (float)motor_fdcan3[2].total_angle * ARM_TNUM23;

    if ((app != NULL) && (app->has_last_valid != 0U)) {
        arm_err_j1 = AbsF(actual_j1 - app->active_motor_deg.j1_deg);
        arm_err_j2 = AbsF(actual_j2 - app->active_motor_deg.j2_deg);
        arm_err_j3 = AbsF(actual_j3 - app->active_motor_deg.j3_deg);
    }
    arm_moving = (((arm_err_j1 > ROBOT_ARM_MOVE_TOL_DEG) ||
                   (arm_err_j2 > ROBOT_ARM_MOVE_TOL_DEG) ||
                   (arm_err_j3 > ROBOT_ARM_MOVE_TOL_DEG)) &&
                  (Arm_control_flag != 0U)) ? 1U : 0U;

    if (selected_dev == 0U) {
        tool_moving = (chuck_snapshot.run_status == TOOL_STATUS_MOVING) ? 1U : 0U;
        tool_error = (chuck_snapshot.run_status == TOOL_STATUS_ERROR) ? 1U : 0U;
    } else {
        tool_moving = (clamp_snapshot.run_status == TOOL_STATUS_MOVING) ? 1U : 0U;
        tool_error = (clamp_snapshot.run_status == TOOL_STATUS_ERROR) ? 1U : 0U;
    }

    if (Mecanum_control_flag != 0U) enable_flags |= 0x01U;
    if (Arm_control_flag != 0U)     enable_flags |= 0x02U;
    if (Tool_control_flag != 0U)    enable_flags |= 0x04U;

    if (chassis_moving != 0U)     executing_flags |= 0x01U;
    if (chassis_pos_running != 0U) executing_flags |= 0x02U;
    if (arm_moving != 0U)         executing_flags |= 0x04U;
    if (tool_moving != 0U)        executing_flags |= 0x08U;
    if (executing_flags != 0U)    executing_flags |= 0x10U;

    timeout_flags = USB_ControlWatchdog_TimeoutFlags();
    if (usart_rx.control_timeout != 0U) timeout_flags |= 0x08U;
    if ((timeout_flags & 0x01U) != 0U) error_flags |= 0x01U;
    if ((timeout_flags & 0x02U) != 0U) error_flags |= 0x02U;
    if ((timeout_flags & 0x04U) != 0U) error_flags |= 0x04U;
    if (nav.imu_online == 0U)          error_flags |= 0x08U;
    if (chs.emergency_stop != 0U)      error_flags |= 0x10U;
    if ((app != NULL) && (app->last_status_code != ARM_IK_RESULT_OK)) error_flags |= 0x20U;
    if (tool_error != 0U)              error_flags |= 0x40U;
    if (active_source_stale != 0U)     error_flags |= 0x80U;

    if (usb_recent != 0U)                    online_flags |= 0x01U;
    if (usart_recent != 0U)                  online_flags |= 0x02U;
    if (nav.imu_online != 0U)                online_flags |= 0x04U;
    if (MotorOnline(motor_fdcan1, 4U) >= 4U) online_flags |= 0x08U;
    if (MotorOnline(motor_fdcan3, 3U) >= 3U) online_flags |= 0x10U;

    buf[0] = 1U;
    buf[1] = active_source;
    buf[2] = enable_flags;
    buf[3] = executing_flags;
    buf[4] = error_flags;
    buf[5] = online_flags;
    buf[6] = usb_rx.last_cmd;
    buf[7] = usb_rx.last_len;
    PackU32LE(usb_rx.count,                 buf,  8);
    PackU32LE(usb_rx.last_tick,             buf, 12);
    PackU32LE(usart_rx.frame_count,         buf, 16);
    PackU32LE(usart_rx.last_frame_tick,     buf, 20);
    PackU32LE(usart_rx.checksum_fail_count, buf, 24);
    buf[28] = usart_rx.checksum_ok;
    buf[29] = usart_rx.mode;
    buf[30] = (app != NULL) ? app->last_status_code : ARM_IK_RESULT_PARAM_ERR;
    buf[31] = (app != NULL) ? app->last_action_code : ARM_IK_ACTION_KEEP_CURRENT;
    PackFloatLE(nav.x_m,           buf, 32);
    PackFloatLE(nav.y_m,           buf, 36);
    PackFloatLE(nav.yaw_total_rad, buf, 40);
    PackFloatLE(nav.vx_mps,        buf, 44);
    PackFloatLE(nav.vy_mps,        buf, 48);
    PackFloatLE(nav.wz_radps,      buf, 52);
    PackFloatLE(chs.odom_x,        buf, 56);
    PackFloatLE(chs.odom_y,        buf, 60);
    PackFloatLE(chs.odom_yaw,      buf, 64);
    PackFloatLE(chs.robot_vel.vx,  buf, 68);
    PackFloatLE(chs.robot_vel.vy,  buf, 72);
    PackFloatLE(chs.robot_vel.vw,  buf, 76);
    PackFloatLE(arm_err_j1,        buf, 80);
    PackFloatLE(arm_err_j2,        buf, 84);
    PackFloatLE(arm_err_j3,        buf, 88);
    buf[92] = selected_dev;
    buf[93] = (selected_dev == 0U) ? (uint8_t)chuck_snapshot.run_status
                                   : (uint8_t)clamp_snapshot.run_status;
    buf[94] = (uint8_t)chs.pos_state;
    buf[95] = timeout_flags;

    Send_Cmd_Data(USB_CMD_ROBOT_GET_STATUS, buf, ROBOT_STATUS_LEN);
}

void PC_TX_Task(void const * argument)
{
    uint8_t status_cmd;

    (void)argument;

    for (;;)
    {
        /* 每次只处理一个请求，避免 USB 突发拥堵 */
        if (PC_TX_DequeueStatus(&status_cmd) != 0U) {
            switch (status_cmd) {
            case USB_CMD_SYS_GET_STATUS:
                SendSysStatus();
                break;
            case USB_CMD_CHS_GET_STATUS:
                SendChsStatus();
                break;
            case USB_CMD_ARM_GET_STATUS:
                SendArmStatus();
                break;
            case USB_CMD_TOOL_GET_STATUS:
                SendToolStatus();
                break;
            case USB_CMD_ROBOT_GET_STATUS:
                SendRobotStatus();
                break;
            case USB_CMD_CLIMB_GET_STATUS:
                SendClimbStatus();
                break;
            default:
                break;
            }
        }

        osDelay(5);
    }
}
