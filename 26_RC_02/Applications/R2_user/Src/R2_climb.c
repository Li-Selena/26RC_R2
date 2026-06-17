#include "R2_climb.h"
#include "fdcan_receive.h"
#include "pid_user.h"
#include <string.h>

extern motor_measure_t motor_fdcan2[8];

#define LEG_FRONT_RIGHT_IDX 0U
#define LEG_REAR_RIGHT_IDX  1U
#define LEG_REAR_LEFT_IDX   2U
#define LEG_FRONT_LEFT_IDX  3U

#define DRIVE_LEFT_IDX      0U
#define DRIVE_RIGHT_IDX     1U

static const float s_leg_dir[4] = {
    R2_CLIMB_LEG1_DIR,
    R2_CLIMB_LEG2_DIR,
    R2_CLIMB_LEG3_DIR,
    R2_CLIMB_LEG4_DIR,
};

static const float s_drive_dir[2] = {
    R2_CLIMB_DRIVE_LEFT_DIR,
    R2_CLIMB_DRIVE_RIGHT_DIR,
};

volatile R2_ClimbDebug_t g_r2_climb_debug_usart;
volatile R2_ClimbDebug_t g_r2_climb_debug_usb;
volatile R2_ClimbDebug_t g_r2_climb_debug_active;

static void UpdatePositions(R2_Climb_Ctrl_t *ctrl);
static const char *StateName(R2_ClimbState_t state);
static void BuildDebugView(const R2_Climb_Ctrl_t *ctrl,
                           uint8_t source,
                           const volatile R2_ClimbDebug_t *prev,
                           R2_ClimbDebug_t *view);
static void StoreDebugCurrent(volatile R2_ClimbDebug_t *view,
                              const R2_ClimbMotorCmd_t *cmd);

static float AbsF(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float SafeScale(float scale)
{
    return (scale > 0.001f) ? scale : 1.0f;
}

static uint8_t ParamsReady(void)
{
    if (R2_CLIMB_PARAM_CONFIGURED == 0U) {
        return 0U;
    }

    if (R2_CLIMB_LEG_COUNT_PER_MM <= 0.001f) {
        return 0U;
    }

    if (R2_CLIMB_DRIVE_COUNT_PER_MM <= 0.001f) {
        return 0U;
    }

    return 1U;
}

static uint32_t StateTimeoutMs(R2_ClimbState_t state)
{
    switch (state) {
    case R2_CLIMB_STATE_RAISE_ALL:
        return R2_CLIMB_RAISE_TIMEOUT_MS;
    case R2_CLIMB_STATE_FIRST_PUSH:
        return R2_CLIMB_FIRST_PUSH_TIMEOUT_MS;
    case R2_CLIMB_STATE_LOWER_ALL_TO_STEP:
        return R2_CLIMB_LOWER_TIMEOUT_MS;
    case R2_CLIMB_STATE_RETRACT_FRONT_LEGS:
        return R2_CLIMB_FRONT_RETRACT_TIMEOUT_MS;
    case R2_CLIMB_STATE_SECOND_PUSH:
        return R2_CLIMB_SECOND_PUSH_TIMEOUT_MS;
    case R2_CLIMB_STATE_RETRACT_REAR_LEGS:
        return R2_CLIMB_REAR_RETRACT_TIMEOUT_MS;
    default:
        return 0U;
    }
}

static R2_ClimbState_t NextState(R2_ClimbState_t state)
{
    switch (state) {
    case R2_CLIMB_STATE_RAISE_ALL:
        return R2_CLIMB_STATE_FIRST_PUSH;
    case R2_CLIMB_STATE_FIRST_PUSH:
        return R2_CLIMB_STATE_LOWER_ALL_TO_STEP;
    case R2_CLIMB_STATE_LOWER_ALL_TO_STEP:
        return R2_CLIMB_STATE_RETRACT_FRONT_LEGS;
    case R2_CLIMB_STATE_RETRACT_FRONT_LEGS:
        return R2_CLIMB_STATE_SECOND_PUSH;
    case R2_CLIMB_STATE_SECOND_PUSH:
        return R2_CLIMB_STATE_RETRACT_REAR_LEGS;
    case R2_CLIMB_STATE_RETRACT_REAR_LEGS:
        return R2_CLIMB_STATE_DONE;
    default:
        return R2_CLIMB_STATE_DONE;
    }
}

static const char *StateName(R2_ClimbState_t state)
{
    switch (state) {
    case R2_CLIMB_STATE_IDLE:
        return "IDLE";
    case R2_CLIMB_STATE_RAISE_ALL:
        return "RAISE_ALL";
    case R2_CLIMB_STATE_FIRST_PUSH:
        return "FIRST_PUSH";
    case R2_CLIMB_STATE_LOWER_ALL_TO_STEP:
        return "LOWER_ALL_TO_STEP";
    case R2_CLIMB_STATE_RETRACT_FRONT_LEGS:
        return "RETRACT_FRONT_LEGS";
    case R2_CLIMB_STATE_SECOND_PUSH:
        return "SECOND_PUSH";
    case R2_CLIMB_STATE_RETRACT_REAR_LEGS:
        return "RETRACT_REAR_LEGS";
    case R2_CLIMB_STATE_DONE:
        return "DONE";
    case R2_CLIMB_STATE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

static void CaptureLegZero(R2_Climb_Ctrl_t *ctrl)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++) {
        ctrl->leg_zero[i] = motor_fdcan2[i].total_angle;
    }
}

static void CaptureDriveSegmentStart(R2_Climb_Ctrl_t *ctrl)
{
    ctrl->drive_segment_start[DRIVE_LEFT_IDX] =
        motor_fdcan2[4U].total_angle;
    ctrl->drive_segment_start[DRIVE_RIGHT_IDX] =
        motor_fdcan2[5U].total_angle;
}

static void SetAllLegTarget(R2_Climb_Ctrl_t *ctrl, float target_mm)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++) {
        ctrl->leg_target_mm[i] = target_mm;
    }
}

static void SetFrontLegTarget(R2_Climb_Ctrl_t *ctrl, float target_mm)
{
    ctrl->leg_target_mm[LEG_FRONT_RIGHT_IDX] = target_mm;
    ctrl->leg_target_mm[LEG_FRONT_LEFT_IDX] = target_mm;
}

static void SetRearLegTarget(R2_Climb_Ctrl_t *ctrl, float target_mm)
{
    ctrl->leg_target_mm[LEG_REAR_RIGHT_IDX] = target_mm;
    ctrl->leg_target_mm[LEG_REAR_LEFT_IDX] = target_mm;
}

static void SetDriveTarget(R2_Climb_Ctrl_t *ctrl, float target_mm)
{
    ctrl->drive_target_mm[DRIVE_LEFT_IDX] = target_mm;
    ctrl->drive_target_mm[DRIVE_RIGHT_IDX] = target_mm;
}

static void BeginState(R2_Climb_Ctrl_t *ctrl,
                       R2_ClimbState_t state,
                       uint32_t now_ms)
{
    if (ctrl == 0) {
        return;
    }

    ctrl->state = state;
    ctrl->state_done = 0U;
    ctrl->state_start_ms = now_ms;

    switch (state) {
    case R2_CLIMB_STATE_RAISE_ALL:
        CaptureLegZero(ctrl);
        CaptureDriveSegmentStart(ctrl);
        SetAllLegTarget(ctrl, R2_CLIMB_LIFT_SAFE_MM);
        SetDriveTarget(ctrl, 0.0f);
        break;

    case R2_CLIMB_STATE_FIRST_PUSH:
        CaptureDriveSegmentStart(ctrl);
        SetAllLegTarget(ctrl, R2_CLIMB_LIFT_SAFE_MM);
        SetDriveTarget(ctrl, R2_CLIMB_FIRST_PUSH_MM);
        break;

    case R2_CLIMB_STATE_LOWER_ALL_TO_STEP:
        SetAllLegTarget(ctrl, R2_CLIMB_LIFT_STEP_MM);
        break;

    case R2_CLIMB_STATE_RETRACT_FRONT_LEGS:
        SetFrontLegTarget(ctrl, 0.0f);
        SetRearLegTarget(ctrl, R2_CLIMB_LIFT_STEP_MM);
        break;

    case R2_CLIMB_STATE_SECOND_PUSH:
        CaptureDriveSegmentStart(ctrl);
        SetFrontLegTarget(ctrl, 0.0f);
        SetRearLegTarget(ctrl, R2_CLIMB_LIFT_STEP_MM);
        SetDriveTarget(ctrl, R2_CLIMB_SECOND_PUSH_MM);
        break;

    case R2_CLIMB_STATE_RETRACT_REAR_LEGS:
        SetAllLegTarget(ctrl, 0.0f);
        break;

    case R2_CLIMB_STATE_DONE:
        ctrl->auto_run = 0U;
        ctrl->state_done = 1U;
        SetAllLegTarget(ctrl, 0.0f);
        SetDriveTarget(ctrl, 0.0f);
        break;

    default:
        break;
    }

    UpdatePositions(ctrl);
}

static void UpdatePositions(R2_Climb_Ctrl_t *ctrl)
{
    uint8_t i;
    float leg_scale = SafeScale(R2_CLIMB_LEG_COUNT_PER_MM);
    float drive_scale = SafeScale(R2_CLIMB_DRIVE_COUNT_PER_MM);

    for (i = 0U; i < 4U; i++) {
        ctrl->leg_pos_mm[i] =
            s_leg_dir[i] *
            ((float)(motor_fdcan2[i].total_angle - ctrl->leg_zero[i])) /
            leg_scale;
    }

    ctrl->drive_pos_mm[DRIVE_LEFT_IDX] =
        s_drive_dir[DRIVE_LEFT_IDX] *
        ((float)(motor_fdcan2[4U].total_angle -
                 ctrl->drive_segment_start[DRIVE_LEFT_IDX])) /
        drive_scale;

    ctrl->drive_pos_mm[DRIVE_RIGHT_IDX] =
        s_drive_dir[DRIVE_RIGHT_IDX] *
        ((float)(motor_fdcan2[5U].total_angle -
                 ctrl->drive_segment_start[DRIVE_RIGHT_IDX])) /
        drive_scale;
}

static uint8_t LegsReached(const R2_Climb_Ctrl_t *ctrl, uint8_t mask)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++) {
        if ((mask & (uint8_t)(1U << i)) == 0U) {
            continue;
        }
        if (AbsF(ctrl->leg_pos_mm[i] - ctrl->leg_target_mm[i]) >
            R2_CLIMB_LEG_TOL_MM) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t DrivesReached(const R2_Climb_Ctrl_t *ctrl)
{
    if (AbsF(ctrl->drive_pos_mm[DRIVE_LEFT_IDX] -
             ctrl->drive_target_mm[DRIVE_LEFT_IDX]) >
        R2_CLIMB_DRIVE_TOL_MM) {
        return 0U;
    }

    if (AbsF(ctrl->drive_pos_mm[DRIVE_RIGHT_IDX] -
             ctrl->drive_target_mm[DRIVE_RIGHT_IDX]) >
        R2_CLIMB_DRIVE_TOL_MM) {
        return 0U;
    }

    return 1U;
}

static uint8_t CurrentStateReached(const R2_Climb_Ctrl_t *ctrl)
{
    switch (ctrl->state) {
    case R2_CLIMB_STATE_RAISE_ALL:
        return LegsReached(ctrl, 0x0FU);
    case R2_CLIMB_STATE_FIRST_PUSH:
        return DrivesReached(ctrl);
    case R2_CLIMB_STATE_LOWER_ALL_TO_STEP:
        return LegsReached(ctrl, 0x0FU);
    case R2_CLIMB_STATE_RETRACT_FRONT_LEGS:
        return LegsReached(ctrl,
                           (uint8_t)((1U << LEG_FRONT_RIGHT_IDX) |
                                     (1U << LEG_FRONT_LEFT_IDX)));
    case R2_CLIMB_STATE_SECOND_PUSH:
        return DrivesReached(ctrl);
    case R2_CLIMB_STATE_RETRACT_REAR_LEGS:
        return LegsReached(ctrl,
                           (uint8_t)((1U << LEG_REAR_RIGHT_IDX) |
                                     (1U << LEG_REAR_LEFT_IDX)));
    default:
        return 0U;
    }
}

static uint8_t DriveHoldActive(R2_ClimbState_t state)
{
    switch (state) {
    case R2_CLIMB_STATE_FIRST_PUSH:
    case R2_CLIMB_STATE_LOWER_ALL_TO_STEP:
    case R2_CLIMB_STATE_RETRACT_FRONT_LEGS:
    case R2_CLIMB_STATE_SECOND_PUSH:
        return 1U;
    default:
        return 0U;
    }
}

static int32_t LegTargetCount(const R2_Climb_Ctrl_t *ctrl, uint8_t index)
{
    float scale = SafeScale(R2_CLIMB_LEG_COUNT_PER_MM);
    float counts = (float)ctrl->leg_zero[index] +
                   s_leg_dir[index] * ctrl->leg_target_mm[index] * scale;

    return (int32_t)counts;
}

static int32_t DriveTargetCount(const R2_Climb_Ctrl_t *ctrl, uint8_t index)
{
    float scale = SafeScale(R2_CLIMB_DRIVE_COUNT_PER_MM);
    float counts = (float)ctrl->drive_segment_start[index] +
                   s_drive_dir[index] * ctrl->drive_target_mm[index] * scale;

    return (int32_t)counts;
}

static void BuildDebugView(const R2_Climb_Ctrl_t *ctrl,
                           uint8_t source,
                           const volatile R2_ClimbDebug_t *prev,
                           R2_ClimbDebug_t *view)
{
    uint8_t i;

    if (view == 0) {
        return;
    }

    memset(view, 0, sizeof(*view));
    view->source = source;
    view->state_name = StateName(R2_CLIMB_STATE_IDLE);
    view->param_ready = ParamsReady();
    view->leg_count_per_mm = R2_CLIMB_LEG_COUNT_PER_MM;
    view->drive_count_per_mm = R2_CLIMB_DRIVE_COUNT_PER_MM;
    for (i = 0U; i < 4U; i++) {
        view->leg_dir[i] = s_leg_dir[i];
    }
    for (i = 0U; i < 2U; i++) {
        view->drive_dir[i] = s_drive_dir[i];
    }

    if (prev != 0) {
        for (i = 0U; i < 4U; i++) {
            view->leg_current[i] = prev->leg_current[i];
        }
        for (i = 0U; i < 2U; i++) {
            view->drive_current[i] = prev->drive_current[i];
        }
    }

    if (ctrl == 0) {
        return;
    }

    view->state = (uint8_t)ctrl->state;
    view->enabled = ctrl->enabled;
    view->auto_run = ctrl->auto_run;
    view->state_done = ctrl->state_done;
    view->error_flags = ctrl->error_flags;
    view->pending_step = ctrl->pending_step;
    view->pending_auto = ctrl->pending_auto;
    view->state_start_ms = ctrl->state_start_ms;
    view->last_update_ms = ctrl->last_update_ms;
    view->state_name = StateName(ctrl->state);
    view->is_motor_active = R2_Climb_IsMotorActive(ctrl);

    if ((ctrl->state == R2_CLIMB_STATE_IDLE) ||
        (ctrl->state == R2_CLIMB_STATE_DONE)) {
        view->elapsed_ms = 0U;
    } else {
        view->elapsed_ms = ctrl->last_update_ms - ctrl->state_start_ms;
    }

    for (i = 0U; i < 4U; i++) {
        view->leg_pos_mm[i] = ctrl->leg_pos_mm[i];
        view->leg_target_mm[i] = ctrl->leg_target_mm[i];
        view->leg_zero[i] = ctrl->leg_zero[i];
    }

    for (i = 0U; i < 2U; i++) {
        view->drive_pos_mm[i] = ctrl->drive_pos_mm[i];
        view->drive_target_mm[i] = ctrl->drive_target_mm[i];
        view->drive_segment_start[i] = ctrl->drive_segment_start[i];
    }
}

static void StoreDebugCurrent(volatile R2_ClimbDebug_t *view,
                              const R2_ClimbMotorCmd_t *cmd)
{
    uint8_t i;

    if ((view == 0) || (cmd == 0)) {
        return;
    }

    for (i = 0U; i < 4U; i++) {
        view->leg_current[i] = cmd->leg[i];
    }
    for (i = 0U; i < 2U; i++) {
        view->drive_current[i] = cmd->drive[i];
    }
}

void R2_Climb_Init(R2_Climb_Ctrl_t *ctrl)
{
    if (ctrl == 0) {
        return;
    }

    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->state = R2_CLIMB_STATE_IDLE;
}

void R2_Climb_Stop(R2_Climb_Ctrl_t *ctrl)
{
    if (ctrl == 0) {
        return;
    }

    R2_Climb_Init(ctrl);
}

void R2_Climb_RequestStep(R2_Climb_Ctrl_t *ctrl)
{
    if (ctrl == 0) {
        return;
    }

    if (ctrl->enabled == 0U) {
        ctrl->enabled = 1U;
    }
    ctrl->pending_step = 1U;
}

void R2_Climb_RequestAuto(R2_Climb_Ctrl_t *ctrl)
{
    if (ctrl == 0) {
        return;
    }

    if (ctrl->enabled == 0U) {
        ctrl->enabled = 1U;
    }
    ctrl->pending_auto = 1U;
}

void R2_Climb_SetInput(R2_Climb_Ctrl_t *ctrl,
                       uint8_t enable_level,
                       uint8_t step_level,
                       uint8_t auto_level)
{
    uint8_t step_edge;
    uint8_t auto_edge;

    if (ctrl == 0) {
        return;
    }

    if (enable_level == 0U) {
        R2_Climb_Stop(ctrl);
        ctrl->last_step_level = step_level;
        ctrl->last_auto_level = auto_level;
        return;
    }

    if (ctrl->enabled == 0U) {
        ctrl->enabled = 1U;
        ctrl->state = R2_CLIMB_STATE_IDLE;
        ctrl->error_flags = 0U;
    }

    step_edge = ((step_level != 0U) && (ctrl->last_step_level == 0U)) ? 1U : 0U;
    auto_edge = ((auto_level != 0U) && (ctrl->last_auto_level == 0U)) ? 1U : 0U;

    if (step_edge != 0U) {
        R2_Climb_RequestStep(ctrl);
    }

    if (auto_edge != 0U) {
        R2_Climb_RequestAuto(ctrl);
    }

    ctrl->last_step_level = step_level;
    ctrl->last_auto_level = auto_level;
}

void R2_Climb_Update(R2_Climb_Ctrl_t *ctrl, uint32_t now_ms)
{
    uint8_t reached;
    uint8_t step;
    uint8_t auto_req;

    if (ctrl == 0) {
        return;
    }

    ctrl->last_update_ms = now_ms;
    UpdatePositions(ctrl);

    if (ctrl->enabled == 0U) {
        return;
    }

    if (ParamsReady() == 0U) {
        ctrl->error_flags |= R2_CLIMB_ERR_PARAM_NOT_CONFIGURED;
        ctrl->auto_run = 0U;
        ctrl->state_done = 0U;
        ctrl->pending_step = 0U;
        ctrl->pending_auto = 0U;
        ctrl->state = R2_CLIMB_STATE_ERROR;
        return;
    }

    step = ctrl->pending_step;
    auto_req = ctrl->pending_auto;
    ctrl->pending_step = 0U;
    ctrl->pending_auto = 0U;

    if (auto_req != 0U) {
        ctrl->auto_run = 1U;
        if (ctrl->state == R2_CLIMB_STATE_IDLE) {
            BeginState(ctrl, R2_CLIMB_STATE_RAISE_ALL, now_ms);
        }
        step = 0U;
    }

    if (step != 0U) {
        ctrl->auto_run = 0U;
        if (ctrl->state == R2_CLIMB_STATE_IDLE) {
            BeginState(ctrl, R2_CLIMB_STATE_RAISE_ALL, now_ms);
        } else if ((ctrl->state != R2_CLIMB_STATE_DONE) &&
                   (ctrl->state != R2_CLIMB_STATE_ERROR)) {
            BeginState(ctrl, NextState(ctrl->state), now_ms);
        }
    }

    if ((ctrl->state == R2_CLIMB_STATE_IDLE) ||
        (ctrl->state == R2_CLIMB_STATE_DONE) ||
        (ctrl->state == R2_CLIMB_STATE_ERROR) ||
        (ctrl->state_done != 0U)) {
        return;
    }

    if (StateTimeoutMs(ctrl->state) != 0U) {
        if ((now_ms - ctrl->state_start_ms) > StateTimeoutMs(ctrl->state)) {
            ctrl->error_flags |= R2_CLIMB_ERR_TIMEOUT;
            ctrl->auto_run = 0U;
            ctrl->state_done = 0U;
            ctrl->state = R2_CLIMB_STATE_ERROR;
            return;
        }
    }

    reached = CurrentStateReached(ctrl);
    if (reached == 0U) {
        return;
    }

    if (ctrl->auto_run != 0U) {
        BeginState(ctrl, NextState(ctrl->state), now_ms);
    } else {
        ctrl->state_done = 1U;
    }
}

uint8_t R2_Climb_IsMotorActive(const R2_Climb_Ctrl_t *ctrl)
{
    if ((ctrl == 0) || (ctrl->enabled == 0U)) {
        return 0U;
    }

    if ((ctrl->state == R2_CLIMB_STATE_IDLE) ||
        (ctrl->state == R2_CLIMB_STATE_DONE) ||
        (ctrl->state == R2_CLIMB_STATE_ERROR)) {
        return 0U;
    }

    return 1U;
}

void R2_Climb_GetMotorCurrent(const R2_Climb_Ctrl_t *ctrl,
                              R2_ClimbMotorCmd_t *cmd)
{
    uint8_t i;
    uint8_t active;

    if (cmd == 0) {
        return;
    }

    for (i = 0U; i < 4U; i++) {
        cmd->leg[i] = 0;
        cmd->drive[i] = 0;
    }

    active = R2_Climb_IsMotorActive(ctrl);

    if (active == 0U) {
        for (i = 0U; i < 4U; i++) {
            cmd->leg[i] = (int16_t)PID_velocity_realize_2(0.0f, (int)i + 1);
        }
        cmd->drive[0] = (int16_t)PID_velocity_realize_2(0.0f, 5);
        cmd->drive[1] = (int16_t)PID_velocity_realize_2(0.0f, 6);
        return;
    }

    for (i = 0U; i < 4U; i++) {
        cmd->leg[i] = (int16_t)pid_call_2((float)LegTargetCount(ctrl, i),
                                          (int)i + 1);
    }

    if (DriveHoldActive(ctrl->state) != 0U) {
        cmd->drive[0] =
            (int16_t)pid_call_2((float)DriveTargetCount(ctrl, DRIVE_LEFT_IDX), 5);
        cmd->drive[1] =
            (int16_t)pid_call_2((float)DriveTargetCount(ctrl, DRIVE_RIGHT_IDX), 6);
    } else {
        cmd->drive[0] = (int16_t)PID_velocity_realize_2(0.0f, 5);
        cmd->drive[1] = (int16_t)PID_velocity_realize_2(0.0f, 6);
    }
}

void R2_Climb_UpdateDebugViews(const R2_Climb_Ctrl_t *usart_ctrl,
                               const R2_Climb_Ctrl_t *usb_ctrl,
                               uint8_t active_source)
{
    R2_ClimbDebug_t usart_view;
    R2_ClimbDebug_t usb_view;
    R2_ClimbDebug_t active_view;

    BuildDebugView(usart_ctrl,
                   R2_CLIMB_DEBUG_SOURCE_USART,
                   &g_r2_climb_debug_usart,
                   &usart_view);
    BuildDebugView(usb_ctrl,
                   R2_CLIMB_DEBUG_SOURCE_USB,
                   &g_r2_climb_debug_usb,
                   &usb_view);

    g_r2_climb_debug_usart = usart_view;
    g_r2_climb_debug_usb = usb_view;

    if (active_source == R2_CLIMB_DEBUG_SOURCE_USB) {
        active_view = usb_view;
    } else if (active_source == R2_CLIMB_DEBUG_SOURCE_USART) {
        active_view = usart_view;
    } else {
        BuildDebugView(0,
                       R2_CLIMB_DEBUG_SOURCE_NONE,
                       &g_r2_climb_debug_active,
                       &active_view);
    }

    g_r2_climb_debug_active = active_view;
}

void R2_Climb_SetDebugMotorCurrent(uint8_t source,
                                   const R2_ClimbMotorCmd_t *cmd)
{
    R2_ClimbMotorCmd_t zero_cmd;

    if (cmd == 0) {
        return;
    }

    if (source == R2_CLIMB_DEBUG_SOURCE_USART) {
        StoreDebugCurrent(&g_r2_climb_debug_usart, cmd);
        if (g_r2_climb_debug_active.source == R2_CLIMB_DEBUG_SOURCE_USART) {
            StoreDebugCurrent(&g_r2_climb_debug_active, cmd);
        }
    } else if (source == R2_CLIMB_DEBUG_SOURCE_USB) {
        StoreDebugCurrent(&g_r2_climb_debug_usb, cmd);
        if (g_r2_climb_debug_active.source == R2_CLIMB_DEBUG_SOURCE_USB) {
            StoreDebugCurrent(&g_r2_climb_debug_active, cmd);
        }
    } else {
        memset(&zero_cmd, 0, sizeof(zero_cmd));
        StoreDebugCurrent(&g_r2_climb_debug_usart, &zero_cmd);
        StoreDebugCurrent(&g_r2_climb_debug_usb, &zero_cmd);
        StoreDebugCurrent(&g_r2_climb_debug_active, &zero_cmd);
    }
}
