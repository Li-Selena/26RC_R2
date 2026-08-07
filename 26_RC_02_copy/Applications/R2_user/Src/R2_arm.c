#include "R2_arm.h"

#include "include.h"
#include "tim.h"

#include <math.h>
#include <stddef.h>

#define R2_ARM_DEFAULT_TOOL       ROBOTARM_TOOL_S1
#define R2_ARM_DEFAULT_STATE      ROBOTARM_TOOL_STATE_S1_PREPARE
#define R2_ARM_STATUS_ENABLED     0x01U
#define R2_ARM_STATUS_HAS_TARGET  0x02U
#define R2_ARM_STATUS_OUTPUT      0x04U
#define R2_ARM_STATUS_IK_OK       0x08U
#define R2_ARM_STATUS_ACTIVE      0x10U
#define R2_ARM_ERR_IK             0x01U
#define R2_ARM_ERR_LIMIT          0x02U
#define R2_ARM_ERR_UNREACHABLE    0x04U
#define R2_ARM_ERR_BAD_PARAM      0x08U
#define R2_ARM_ERR_UNSAFE         0x10U
#define R2_ARM_ERR_UNSUPPORTED    0x20U
#define R2_ARM_JOINT_POS_SPEED_RAD_S 2.0f
#define R2_ARM_J2_POS_SPEED_RAD_S    2.5f
#define R2_ARM_J3_POS_SPEED_RAD_S    3.0f
#define R2_ARM_DEG_TO_RAD         (ROBOTARM_KIN_PI / 180.0f)
#define R2_ARM_JOINT_LIMIT_EPS_RAD 1.0e-4f
#define R2_ARM_LONG_LINK_LIMIT_TOL_RAD (2.0f * R2_ARM_DEG_TO_RAD)
#define R2_ARM_ENABLE_ALPHA_RAD   (90.0f * R2_ARM_DEG_TO_RAD)
#define R2_ARM_YAW_REACHED_TOL_RAD (2.0f * R2_ARM_DEG_TO_RAD)
#define R2_ARM_POWERON_YAW_RAD    0.0f
#define R2_ARM_POWERON_LONG_ALPHA_RAD (-30.0f * R2_ARM_DEG_TO_RAD)
#define R2_ARM_POWERON_FRAME4_PHI_RAD (-0.5f * ROBOTARM_KIN_PI)
#define R2_ARM_DM_RUNNING_STATE   1U
#define R2_ARM_ENABLE_RETRY_MS    250U
#define R2_ARM_ENABLE_STARTUP_PHASE_NONE 0U
#define R2_ARM_ENABLE_STARTUP_PHASE_J2   1U
#define R2_ARM_ENABLE_STARTUP_PHASE_J1   2U
#define R2_ARM_WORKSPACE_SWITCH_PHASE_NONE       0U
#define R2_ARM_WORKSPACE_SWITCH_PHASE_ALPHA_MAX  1U
#define R2_ARM_WORKSPACE_SWITCH_PHASE_J1_Y_POS   2U
#define R2_ARM_WORKSPACE_SWITCH_PHASE_J1         3U
#define R2_ARM_WORKSPACE_SWITCH_PHASE_RESTORE_J2 4U
#define R2_ARM_GRIPPER_PWM_MIN_US 500.0f
#define R2_ARM_GRIPPER_PWM_MAX_US 2500.0f
#define R2_ARM_GRIPPER_TIMER_HZ   1000000.0f

#ifndef R2_ARM_GRIPPER_OPEN_DEG
#define R2_ARM_GRIPPER_OPEN_DEG 120.0f
#endif

#ifndef R2_ARM_GRIPPER_CLOSE_DEG
#define R2_ARM_GRIPPER_CLOSE_DEG 0.0f
#endif

#ifndef R2_ARM_J1_MOTOR_SIGN
#define R2_ARM_J1_MOTOR_SIGN 1.0f
#endif

/* Map the power-on model yaw to motor zero for normal angle-path control. */
#ifndef R2_ARM_J1_MOTOR_ZERO_OFFSET_RAD
#define R2_ARM_J1_MOTOR_ZERO_OFFSET_RAD (-(R2_ARM_J1_MOTOR_SIGN) * R2_ARM_POWERON_YAW_RAD)
#endif

/*
 * J2's physical output axis is -X0 while the solver theta2 axis is +X0; this
 * cancels the gear reversal. J3 uses the solver +X0 axis, so only the gear
 * reversal remains.
 */
#ifndef R2_ARM_J2_MOTOR_SIGN
#define R2_ARM_J2_MOTOR_SIGN 1.0f
#endif

#ifndef R2_ARM_J2_MOTOR_PER_JOINT_RAD
#define R2_ARM_J2_MOTOR_PER_JOINT_RAD (55.0f / 30.0f)
#endif

#ifndef R2_ARM_J3_MOTOR_SIGN
#define R2_ARM_J3_MOTOR_SIGN (-1.0f)
#endif

#ifndef R2_ARM_J3_MOTOR_PER_JOINT_RAD
#define R2_ARM_J3_MOTOR_PER_JOINT_RAD 2.0f
#endif

/* Active-high suction 2 output is fixed to the board's PE13 label. */
#ifndef R2_ARM_S2_SUCTION_GPIO_PORT
#define R2_ARM_S2_SUCTION_GPIO_PORT SUCTION_2_GPIO_Port
#endif
#ifndef R2_ARM_S2_SUCTION_GPIO_PIN
#define R2_ARM_S2_SUCTION_GPIO_PIN SUCTION_2_Pin
#endif

R2_Arm_Ctrl_t g_r2_arm_usb;
static uint8_t s_r2_arm_motor_hw_enabled = 0U;
static uint32_t s_r2_arm_motor_enable_retry_ms = 0U;

static void R2_Arm_UpdateFlags(R2_Arm_Ctrl_t *ctrl);
static void R2_Arm_SetPowerOnHoldState(R2_Arm_Ctrl_t *ctrl);
static RobotArmIKStatus_t R2_Arm_StartEnableStartup(R2_Arm_Ctrl_t *ctrl,
                                                    uint32_t now_ms);
static RobotArmIKStatus_t R2_Arm_SetEnableStartupPose(R2_Arm_Ctrl_t *ctrl,
                                                       uint8_t phase,
                                                       uint32_t now_ms);
static void R2_Arm_UpdateEnableStartup(R2_Arm_Ctrl_t *ctrl,
                                       uint32_t now_ms);
static uint8_t R2_Arm_EnableStartupTargetReached(
    const R2_Arm_Ctrl_t *ctrl,
    uint8_t phase);
static uint8_t R2_Arm_EnableStartupOutputMask(uint8_t phase);
static uint8_t R2_Arm_EnableStartupReachedMask(uint8_t phase);
static void R2_Arm_CancelEnableStartup(R2_Arm_Ctrl_t *ctrl);
static RobotArmIKStatus_t R2_Arm_StartWorkspaceSwitch(
    R2_Arm_Ctrl_t *ctrl,
    RobotArmWorkDirection_t direction,
    uint32_t now_ms);
static RobotArmIKStatus_t R2_Arm_SetWorkspaceSwitchPhase(
    R2_Arm_Ctrl_t *ctrl,
    uint8_t phase,
    uint32_t now_ms);
static void R2_Arm_UpdateWorkspaceSwitch(R2_Arm_Ctrl_t *ctrl,
                                         uint32_t now_ms);
static uint8_t R2_Arm_WorkspaceSwitchTargetReached(
    const R2_Arm_Ctrl_t *ctrl);
static uint8_t R2_Arm_WorkspaceSwitchJointMask(uint8_t phase);
static uint8_t R2_Arm_WorkspaceSwitchNeedsYPosPhase(
    const R2_Arm_Ctrl_t *ctrl);
static void R2_Arm_BuildWorkspaceSwitchTheta(
    const R2_Arm_Ctrl_t *ctrl,
    uint8_t phase,
    float theta_rad[ROBOTARM_KIN_JOINT_COUNT]);
static RobotArmIKStatus_t R2_Arm_ValidateWorkspaceSwitchTheta(
    const R2_Arm_Ctrl_t *ctrl,
    const float theta_rad[ROBOTARM_KIN_JOINT_COUNT],
    uint8_t phase,
    RobotArmIKReason_t *reason);
static void R2_Arm_CancelWorkspaceSwitch(R2_Arm_Ctrl_t *ctrl);
static void R2_Arm_SetRequestCurrent(R2_Arm_Ctrl_t *ctrl,
                                     RobotArmIKRequest_t *request);
static void R2_Arm_CancelIndependentJointJog(R2_Arm_Ctrl_t *ctrl);
static void R2_Arm_OutputApplyIndependentJointTargetsRad(
    const float theta_rad[ROBOTARM_KIN_JOINT_COUNT],
    uint8_t selected_mask);
static void R2_Arm_CommitResultIfOk(R2_Arm_Ctrl_t *ctrl,
                                    RobotArmIKStatus_t ik_status);
static void R2_Arm_CancelIKTestFlow(R2_Arm_Ctrl_t *ctrl);
static RobotArmIKStatus_t R2_Arm_SetCommandError(R2_Arm_Ctrl_t *ctrl,
                                                 RobotArmIKStatus_t status,
                                                 RobotArmIKReason_t reason,
                                                 uint32_t now_ms);
static RobotArmIKStatus_t R2_Arm_SetToolTargetXYZDirected(
    R2_Arm_Ctrl_t *ctrl,
    RobotArmTool_t tool,
    RobotArmToolState_t state,
    const RobotArmVec3_t *target_xyz_mm,
    uint8_t target_direction_valid,
    RobotArmWorkDirection_t target_direction,
    uint32_t now_ms);
static uint8_t R2_Arm_IsDirectionValid(RobotArmWorkDirection_t direction);
static float R2_Arm_LongToChordOffsetRad(void);
static float R2_Arm_AlphaToTheta2Rad(float alpha_rad);
static float R2_Arm_PowerOnTheta2Rad(void);
static float R2_Arm_PowerOnTheta3Rad(void);
static float R2_Arm_ModelJ1ToMotorRad(float model_yaw_rad);
static float R2_Arm_ModelJ2ToMotorRad(float model_theta2_rad);
static float R2_Arm_ModelJ3ToMotorRad(float model_theta3_rad);
static float R2_Arm_MotorJ1ToModelRad(float motor_yaw_rad);
static float R2_Arm_MotorJ2ToModelRad(float motor_theta2_rad);
static float R2_Arm_MotorJ3ToModelRad(float motor_theta3_rad);
static void R2_Arm_GetMotorFeedbackRad(float feedback_rad[ROBOTARM_KIN_JOINT_COUNT],
                                       uint8_t *ok_mask);
static uint8_t R2_Arm_GetFeedbackModelTheta(float theta_rad[ROBOTARM_KIN_JOINT_COUNT],
                                            uint8_t *ok_mask);
static float R2_Arm_NormalizePi(float x);
static float R2_Arm_NormalizeNear(float x, float reference);
static void R2_Arm_UpdateMotorHardwareState(const R2_Arm_Ctrl_t *ctrl);
static uint8_t R2_Arm_DMFeedbackNeedsEnableRetry(void);
static void R2_Arm_ResolveLinkedToolPosture(R2_Arm_Ctrl_t *ctrl,
                                            RobotArmIKRequest_t *request);
static void R2_Arm_UpdateSharedToolState(R2_Arm_Ctrl_t *ctrl,
                                         RobotArmTool_t tool,
                                         RobotArmToolState_t state,
                                         RobotArmIKStatus_t ik_status);
static void R2_Arm_FillLegacyTargetXY(RobotArmIKRequest_t *request,
                                      float approach_yaw_rad);
static float R2_Arm_DefaultTargetZ(RobotArmTool_t tool,
                                   RobotArmToolState_t state);
static void R2_Arm_SetSuction2Output(uint8_t enabled);
static void R2_Arm_SetGripperOutput(uint8_t enabled);
static void R2_Arm_SetGripperAngleDeg(float angle_deg);
static float R2_Arm_ClampJointLimitTolerance(const R2_Arm_Ctrl_t *ctrl,
                                             uint8_t index,
                                             float theta_rad);
static float R2_Arm_ClampFloat(float x, float lo, float hi);

void R2_Arm_Init(R2_Arm_Ctrl_t *ctrl)
{
    uint8_t i;

    if (ctrl == NULL)
    {
        return;
    }

    ctrl->enabled = 0U;
    ctrl->has_target = 0U;
    ctrl->output_enabled = 0U;
    ctrl->state = (uint8_t)R2_ARM_STATE_IDLE;
    ctrl->status_flags = 0U;
    ctrl->error_flags = 0U;
    ctrl->last_ik_status = (uint8_t)ROBOTARM_IK_OK;
    ctrl->last_ik_reason = (uint8_t)ROBOTARM_IK_REASON_NONE;
    ctrl->last_command_ms = 0U;
    ctrl->last_update_ms = 0U;
    ctrl->output_apply_count = 0U;
    ctrl->ik_test_active = 0U;
    ctrl->ik_test_step = 0U;
    ctrl->ik_test_done = 0U;
    ctrl->ik_test_error = 0U;
    ctrl->ik_test_step_start_ms = 0U;
    ctrl->ik_test_step_period_ms = R2_ARM_IK_TEST_DEFAULT_STEP_MS;
    ctrl->joint_jog_active = 0U;
    ctrl->joint_jog_index = 0U;
    ctrl->enable_startup_active = 0U;
    ctrl->enable_startup_phase = R2_ARM_ENABLE_STARTUP_PHASE_NONE;
    ctrl->workspace_switch_active = 0U;
    ctrl->workspace_switch_phase = R2_ARM_WORKSPACE_SWITCH_PHASE_NONE;
    ctrl->workspace_switch_target_direction = ROBOTARM_WORK_DIR_Y_POS;
    for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
    {
        ctrl->workspace_switch_saved_theta_rad[i] = 0.0f;
    }
    RobotArm_Mixed_Init();
    s_r2_arm_motor_hw_enabled = 0U;
    s_r2_arm_motor_enable_retry_ms = 0U;
    RobotArmKinematics_DefaultConfig(&ctrl->kinematics_config);
    ctrl->current_direction = ROBOTARM_WORK_DIR_Y_POS;
    ctrl->current_alpha_rad = R2_ARM_POWERON_LONG_ALPHA_RAD;
    ctrl->current_theta_rad[0] = R2_ARM_POWERON_YAW_RAD;
    ctrl->current_theta_rad[1] = R2_Arm_PowerOnTheta2Rad();
    ctrl->current_theta_rad[2] = R2_Arm_PowerOnTheta3Rad();
    ctrl->next_gripper_motion_state = ROBOTARM_TOOL_STATE_GRIPPER_UP;
    ctrl->shared_s2_state = ROBOTARM_TOOL_STATE_S2_SUCTION_DOWN;

    R2_Arm_SetPowerOnHoldState(ctrl);
    R2_Arm_UpdateFlags(ctrl);
}

void R2_Arm_Enable(R2_Arm_Ctrl_t *ctrl)
{
    RobotArmIKStatus_t ik_status;

    if (ctrl == NULL)
    {
        return;
    }

    if (ctrl->enabled == 0U)
    {
        R2_Arm_SetPowerOnHoldState(ctrl);
    }
    R2_Arm_CancelIKTestFlow(ctrl);
    R2_Arm_CancelIndependentJointJog(ctrl);
    R2_Arm_CancelWorkspaceSwitch(ctrl);
    ctrl->enabled = 1U;
    ik_status = R2_Arm_StartEnableStartup(ctrl, HAL_GetTick());
    if (ik_status != ROBOTARM_IK_OK)
    {
        ctrl->output_enabled = 0U;
        ctrl->state = (uint8_t)R2_ARM_STATE_ERROR;
    }
    R2_Arm_UpdateFlags(ctrl);
}

void R2_Arm_Disable(R2_Arm_Ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->enabled = 0U;
    ctrl->output_enabled = 0U;
    ctrl->state = (uint8_t)R2_ARM_STATE_IDLE;
    R2_Arm_CancelEnableStartup(ctrl);
    R2_Arm_CancelIKTestFlow(ctrl);
    R2_Arm_CancelIndependentJointJog(ctrl);
    R2_Arm_CancelWorkspaceSwitch(ctrl);
    R2_Arm_SetPowerOnHoldState(ctrl);
    R2_Arm_UpdateFlags(ctrl);
}

void R2_Arm_Stop(R2_Arm_Ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->output_enabled = 0U;
    ctrl->state = (uint8_t)R2_ARM_STATE_STOPPED;
    R2_Arm_CancelEnableStartup(ctrl);
    R2_Arm_CancelIKTestFlow(ctrl);
    R2_Arm_CancelIndependentJointJog(ctrl);
    R2_Arm_CancelWorkspaceSwitch(ctrl);
    R2_Arm_UpdateFlags(ctrl);
}

RobotArmIKStatus_t R2_Arm_SetToolTarget(R2_Arm_Ctrl_t *ctrl,
                                        RobotArmTool_t tool,
                                        RobotArmToolState_t state,
                                        float target_z_mm,
                                        float approach_yaw_rad,
                                        uint32_t now_ms)
{
    if (ctrl == NULL)
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    (void)tool;
    (void)state;
    (void)target_z_mm;
    (void)approach_yaw_rad;
    return R2_Arm_SetCommandError(ctrl,
                                  ROBOTARM_IK_ERR_UNSUPPORTED_STATE,
                                  ROBOTARM_IK_REASON_UNSUPPORTED_STATE,
                                  now_ms);
}

RobotArmIKStatus_t R2_Arm_SetToolTargetXYZ(R2_Arm_Ctrl_t *ctrl,
                                           RobotArmTool_t tool,
                                           RobotArmToolState_t state,
                                           const RobotArmVec3_t *target_xyz_mm,
                                           uint32_t now_ms)
{
    return R2_Arm_SetToolTargetXYZDirected(ctrl,
                                           tool,
                                           state,
                                           target_xyz_mm,
                                           0U,
                                           ROBOTARM_WORK_DIR_Y_POS,
                                           now_ms);
}

static RobotArmIKStatus_t R2_Arm_SetToolTargetXYZDirected(
    R2_Arm_Ctrl_t *ctrl,
    RobotArmTool_t tool,
    RobotArmToolState_t state,
    const RobotArmVec3_t *target_xyz_mm,
    uint8_t target_direction_valid,
    RobotArmWorkDirection_t target_direction,
    uint32_t now_ms)
{
    if ((ctrl == NULL) || (target_xyz_mm == NULL))
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    if ((target_direction_valid != 0U) &&
        (!R2_Arm_IsDirectionValid(target_direction)))
    {
        return R2_Arm_SetCommandError(ctrl,
                                      ROBOTARM_IK_ERR_BAD_DIRECTION,
                                      ROBOTARM_IK_REASON_BAD_DIRECTION,
                                      now_ms);
    }

    (void)tool;
    (void)state;
    (void)target_direction_valid;
    (void)target_direction;
    return R2_Arm_SetCommandError(ctrl,
                                  ROBOTARM_IK_ERR_UNSUPPORTED_STATE,
                                  ROBOTARM_IK_REASON_UNSUPPORTED_STATE,
                                  now_ms);
}

RobotArmIKStatus_t R2_Arm_SetWorkDirection(R2_Arm_Ctrl_t *ctrl,
                                           RobotArmWorkDirection_t direction,
                                           uint32_t now_ms)
{
    if (ctrl == NULL)
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    if (!R2_Arm_IsDirectionValid(direction))
    {
        return R2_Arm_SetCommandError(ctrl,
                                      ROBOTARM_IK_ERR_BAD_DIRECTION,
                                      ROBOTARM_IK_REASON_BAD_DIRECTION,
                                      now_ms);
    }

    return R2_Arm_StartWorkspaceSwitch(ctrl, direction, now_ms);
}

RobotArmIKStatus_t R2_Arm_JogToolHeight(R2_Arm_Ctrl_t *ctrl,
                                        float delta_z_mm,
                                        uint32_t now_ms)
{
    if (ctrl == NULL)
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    if ((delta_z_mm != delta_z_mm) ||
        (delta_z_mm > 3.4028234e38f) ||
        (delta_z_mm < -3.4028234e38f))
    {
        return R2_Arm_SetCommandError(ctrl,
                                      ROBOTARM_IK_ERR_BAD_PARAM,
                                      ROBOTARM_IK_REASON_BAD_FLOAT,
                                      now_ms);
    }

    (void)delta_z_mm;
    return R2_Arm_SetCommandError(ctrl,
                                  ROBOTARM_IK_ERR_UNSUPPORTED_STATE,
                                  ROBOTARM_IK_REASON_UNSUPPORTED_STATE,
                                  now_ms);
}

RobotArmIKStatus_t R2_Arm_MoveToolHeightLimit(R2_Arm_Ctrl_t *ctrl,
                                              uint8_t use_max,
                                              uint32_t now_ms)
{
    if (ctrl == NULL)
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    (void)use_max;
    return R2_Arm_SetCommandError(ctrl,
                                  ROBOTARM_IK_ERR_UNSUPPORTED_STATE,
                                  ROBOTARM_IK_REASON_UNSUPPORTED_STATE,
                                  now_ms);
}

RobotArmIKStatus_t R2_Arm_SetToolPosture(R2_Arm_Ctrl_t *ctrl,
                                         RobotArmTool_t tool,
                                         RobotArmToolState_t state,
                                         uint32_t now_ms)
{
    if (ctrl == NULL)
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    (void)tool;
    (void)state;
    return R2_Arm_SetCommandError(ctrl,
                                  ROBOTARM_IK_ERR_UNSUPPORTED_STATE,
                                  ROBOTARM_IK_REASON_UNSUPPORTED_STATE,
                                  now_ms);
}

RobotArmIKStatus_t R2_Arm_JogJointActual(R2_Arm_Ctrl_t *ctrl,
                                         uint8_t joint,
                                         float delta_deg,
                                         uint32_t now_ms)
{
    uint8_t index;
    uint8_t i;
    uint8_t feedback_ok_mask = 0U;
    float theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    float feedback_theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    RobotArmIKResult_t result;
    RobotArmIKRequest_t request;
    RobotArmIKStatus_t fk_status;
    RobotArmIKReason_t limit_reason;

    if (ctrl == NULL)
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    if ((delta_deg != delta_deg) ||
        (delta_deg > 3.4028234e38f) ||
        (delta_deg < -3.4028234e38f))
    {
        return R2_Arm_SetCommandError(ctrl,
                                      ROBOTARM_IK_ERR_BAD_PARAM,
                                      ROBOTARM_IK_REASON_BAD_FLOAT,
                                      now_ms);
    }

    if (joint == 2U)
    {
        index = 1U;
        limit_reason = ROBOTARM_IK_REASON_J2_LIMIT;
    }
    else if (joint == 3U)
    {
        index = 2U;
        limit_reason = ROBOTARM_IK_REASON_J3_LIMIT;
    }
    else
    {
        return R2_Arm_SetCommandError(ctrl,
                                      ROBOTARM_IK_ERR_BAD_PARAM,
                                      ROBOTARM_IK_REASON_BAD_FLOAT,
                                      now_ms);
    }

    for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
    {
        theta_rad[i] = ctrl->current_theta_rad[i];
    }
    if (R2_Arm_GetFeedbackModelTheta(feedback_theta_rad,
                                     &feedback_ok_mask) != 0U)
    {
        for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
        {
            if ((feedback_ok_mask & (uint8_t)(1U << i)) != 0U)
            {
                theta_rad[i] = feedback_theta_rad[i];
            }
        }
    }

    theta_rad[index] += delta_deg * R2_ARM_DEG_TO_RAD;
    theta_rad[index] = R2_Arm_ClampJointLimitTolerance(ctrl,
                                                       index,
                                                       theta_rad[index]);
    if ((theta_rad[index] < ctrl->kinematics_config.joint_min_rad[index]) ||
        (theta_rad[index] > ctrl->kinematics_config.joint_max_rad[index]))
    {
        return R2_Arm_SetCommandError(ctrl,
                                      ROBOTARM_IK_ERR_JOINT_LIMIT,
                                      limit_reason,
                                      now_ms);
    }

    fk_status = RobotArmKinematics_ForwardState(ctrl->request.tool,
                                                ctrl->request.state,
                                                theta_rad,
                                                &result);
    if (fk_status != ROBOTARM_IK_OK)
    {
        return R2_Arm_SetCommandError(ctrl,
                                      fk_status,
                                      result.reason,
                                      now_ms);
    }

    if ((result.alpha_rad <
         (ctrl->kinematics_config.alpha_min_rad -
          R2_ARM_LONG_LINK_LIMIT_TOL_RAD)) ||
        (result.alpha_rad >
         (ctrl->kinematics_config.alpha_max_rad +
          R2_ARM_LONG_LINK_LIMIT_TOL_RAD)))
    {
        return R2_Arm_SetCommandError(ctrl,
                                      ROBOTARM_IK_ERR_LONG_LINK_LIMIT,
                                      ROBOTARM_IK_REASON_LONG_LINK_LIMIT,
                                      now_ms);
    }

    R2_Arm_CancelEnableStartup(ctrl);
    R2_Arm_CancelIKTestFlow(ctrl);
    R2_Arm_CancelWorkspaceSwitch(ctrl);

    request = ctrl->request;
    request.tool = result.tool;
    request.state = result.state;
    request.target_xyz_mm = result.tool_world_mm;
    request.target_z_mm = result.tool_world_mm.z;
    request.approach_yaw_rad = result.approach_yaw_rad;
    request.target_direction_valid = 1U;
    request.target_direction = result.target_direction;
    request.posture_source_valid = 0U;
    request.posture_source_tool = request.tool;
    request.posture_source_state = request.state;
    R2_Arm_SetRequestCurrent(ctrl, &request);

    ctrl->request = request;
    ctrl->result = result;
    ctrl->has_target = 1U;
    ctrl->last_command_ms = now_ms;
    ctrl->last_ik_status = (uint8_t)ROBOTARM_IK_OK;
    ctrl->last_ik_reason = (uint8_t)ROBOTARM_IK_REASON_NONE;
    ctrl->state = (uint8_t)R2_ARM_STATE_TARGET_VALID;
    ctrl->output_enabled = (ctrl->enabled != 0U) ? 1U : 0U;
    R2_Arm_CommitResultIfOk(ctrl, ROBOTARM_IK_OK);
    ctrl->joint_jog_active = 1U;
    ctrl->joint_jog_index = index;
    R2_Arm_UpdateSharedToolState(ctrl,
                                 ctrl->request.tool,
                                 ctrl->request.state,
                                 ROBOTARM_IK_OK);
    R2_Arm_UpdateMotorHardwareState(ctrl);
    if (ctrl->output_enabled != 0U)
    {
        R2_Arm_OutputApplyIndependentJointTargetsRad(
            ctrl->result.theta_rad,
            (uint8_t)(1U << index));
        ctrl->output_apply_count++;
    }
    R2_Arm_UpdateFlags(ctrl);
    return ROBOTARM_IK_OK;
}

void R2_Arm_StartIKTestFlow(R2_Arm_Ctrl_t *ctrl,
                            uint32_t now_ms,
                            uint32_t step_period_ms)
{
    if (ctrl == NULL)
    {
        return;
    }

    (void)step_period_ms;
    ctrl->ik_test_active = 0U;
    ctrl->ik_test_step = 0U;
    ctrl->ik_test_done = 0U;
    ctrl->ik_test_error = 1U;
    ctrl->ik_test_step_start_ms = now_ms;
    ctrl->last_command_ms = now_ms;
    R2_Arm_CancelEnableStartup(ctrl);
    R2_Arm_CancelIndependentJointJog(ctrl);
    R2_Arm_CancelWorkspaceSwitch(ctrl);
    ctrl->last_ik_status = (uint8_t)ROBOTARM_IK_ERR_UNSUPPORTED_STATE;
    ctrl->last_ik_reason = (uint8_t)ROBOTARM_IK_REASON_UNSUPPORTED_STATE;
    ctrl->state = (uint8_t)R2_ARM_STATE_ERROR;
    ctrl->output_enabled = 0U;
    R2_Arm_UpdateFlags(ctrl);
}

void R2_Arm_StopIKTestFlow(R2_Arm_Ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    R2_Arm_CancelEnableStartup(ctrl);
    R2_Arm_CancelIKTestFlow(ctrl);
    R2_Arm_CancelIndependentJointJog(ctrl);
    R2_Arm_CancelWorkspaceSwitch(ctrl);
    ctrl->output_enabled = 0U;
    ctrl->state = (uint8_t)R2_ARM_STATE_STOPPED;
    R2_Arm_UpdateFlags(ctrl);
}

void R2_Arm_Update(R2_Arm_Ctrl_t *ctrl, uint32_t now_ms)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->last_update_ms = now_ms;

    if (ctrl->enable_startup_active != 0U)
    {
        R2_Arm_UpdateEnableStartup(ctrl, now_ms);
        return;
    }

    if (ctrl->ik_test_active != 0U)
    {
        (void)R2_Arm_SetCommandError(ctrl,
                                     ROBOTARM_IK_ERR_UNSUPPORTED_STATE,
                                     ROBOTARM_IK_REASON_UNSUPPORTED_STATE,
                                     now_ms);
        return;
    }

    if (ctrl->workspace_switch_active != 0U)
    {
        R2_Arm_UpdateWorkspaceSwitch(ctrl, now_ms);
        return;
    }

    if (ctrl->joint_jog_active != 0U)
    {
        if ((ctrl->enabled == 0U) ||
            (ctrl->has_target == 0U) ||
            (ctrl->state == (uint8_t)R2_ARM_STATE_STOPPED))
        {
            ctrl->joint_jog_active = 0U;
            ctrl->joint_jog_index = 0U;
            ctrl->output_enabled = 0U;
            R2_Arm_UpdateMotorHardwareState(ctrl);
            R2_Arm_UpdateFlags(ctrl);
            return;
        }

        ctrl->output_enabled = 1U;
        R2_Arm_UpdateMotorHardwareState(ctrl);
        R2_Arm_UpdateFlags(ctrl);
        return;
    }

    if ((ctrl->enabled == 0U) ||
        (ctrl->has_target == 0U) ||
        (ctrl->state == (uint8_t)R2_ARM_STATE_STOPPED))
    {
        ctrl->output_enabled = 0U;
        R2_Arm_UpdateMotorHardwareState(ctrl);
        R2_Arm_UpdateFlags(ctrl);
        return;
    }

    if ((ctrl->last_ik_status == (uint8_t)ROBOTARM_IK_OK) &&
        (ctrl->result.status == ROBOTARM_IK_OK))
    {
        ctrl->state = (uint8_t)R2_ARM_STATE_TARGET_VALID;
        ctrl->output_enabled = 1U;
        R2_Arm_UpdateMotorHardwareState(ctrl);
    }
    else
    {
        ctrl->state = (uint8_t)R2_ARM_STATE_ERROR;
        ctrl->output_enabled = 0U;
        R2_Arm_UpdateMotorHardwareState(ctrl);
    }

    R2_Arm_UpdateFlags(ctrl);
}

void R2_Arm_GetStatus(const R2_Arm_Ctrl_t *ctrl, R2_ArmStatus_t *status)
{
    if ((ctrl == NULL) || (status == NULL))
    {
        return;
    }

    status->enabled = ctrl->enabled;
    status->has_target = ctrl->has_target;
    status->output_enabled = ctrl->output_enabled;
    status->state = ctrl->state;
    status->status_flags = ctrl->status_flags;
    status->error_flags = ctrl->error_flags;
    status->last_ik_status = ctrl->last_ik_status;
    status->last_ik_reason = ctrl->last_ik_reason;
    status->last_command_ms = ctrl->last_command_ms;
    status->last_update_ms = ctrl->last_update_ms;
    status->output_apply_count = ctrl->output_apply_count;
    status->ik_test_active = ctrl->ik_test_active;
    status->ik_test_step = ctrl->ik_test_step;
    status->ik_test_done = ctrl->ik_test_done;
    status->ik_test_error = ctrl->ik_test_error;
    status->ik_test_step_start_ms = ctrl->ik_test_step_start_ms;
    status->ik_test_step_period_ms = ctrl->ik_test_step_period_ms;
    status->request = ctrl->request;
    status->result = ctrl->result;
    R2_Arm_GetMotorFeedbackRad(status->motor_feedback_rad,
                               &status->motor_feedback_ok_mask);
}

uint8_t R2_Arm_IsMotorActive(const R2_Arm_Ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
        return 0U;
    }

    return ((ctrl->enabled != 0U) &&
            (ctrl->output_enabled != 0U) &&
            (ctrl->last_ik_status == (uint8_t)ROBOTARM_IK_OK)) ? 1U : 0U;
}

uint8_t R2_Arm_IsJointTargetReached(const R2_Arm_Ctrl_t *ctrl,
                                    uint8_t joint,
                                    float tolerance_deg)
{
    float feedback_theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    float error_rad;
    uint8_t feedback_ok_mask = 0U;
    uint8_t index;

    if ((ctrl == NULL) || (tolerance_deg < 0.0f) ||
        (tolerance_deg != tolerance_deg))
    {
        return 0U;
    }

    if (joint == 2U)
    {
        index = 1U;
    }
    else if (joint == 3U)
    {
        index = 2U;
    }
    else
    {
        return 0U;
    }

    if (R2_Arm_GetFeedbackModelTheta(feedback_theta_rad,
                                     &feedback_ok_mask) == 0U)
    {
        return 0U;
    }
    if ((feedback_ok_mask & (uint8_t)(1U << index)) == 0U)
    {
        return 0U;
    }

    error_rad = feedback_theta_rad[index] - ctrl->result.theta_rad[index];
    if (index != 2U)
    {
        error_rad = R2_Arm_NormalizePi(error_rad);
    }

    return (fabsf(error_rad) <=
            (tolerance_deg * R2_ARM_DEG_TO_RAD)) ? 1U : 0U;
}

void R2_Arm_SetToolActuator(RobotArmTool_t tool, uint8_t enabled)
{
    if (tool == ROBOTARM_TOOL_S2)
    {
        R2_Arm_SetSuction2Output(enabled);
    }
    else if (tool == ROBOTARM_TOOL_GRIPPER)
    {
        R2_Arm_SetGripperOutput(enabled);
    }
}

static void R2_Arm_OutputApplyIndependentJointTargetsRad(
    const float theta_rad[ROBOTARM_KIN_JOINT_COUNT],
    uint8_t selected_mask)
{
    RobotArm_MixedCommand_t cmd;
    RobotArm_FDCAN3Feedback_t feedback;
    float motor_target[ROBOTARM_KIN_JOINT_COUNT];

    if (theta_rad == NULL)
    {
        return;
    }

    motor_target[0] = R2_Arm_ModelJ1ToMotorRad(theta_rad[0]);
    motor_target[1] = R2_Arm_ModelJ2ToMotorRad(theta_rad[1]);
    motor_target[2] = R2_Arm_ModelJ3ToMotorRad(theta_rad[2]);

    RobotArm_Mixed_GetFeedbackSnapshot(&feedback);
    if (((selected_mask & 0x01U) == 0U) &&
        (feedback.dm8006_1.feedback_ok != 0U))
    {
        motor_target[0] = feedback.dm8006_1.pos;
    }
    if (((selected_mask & 0x02U) == 0U) &&
        (feedback.dm8006_2.feedback_ok != 0U))
    {
        motor_target[1] = feedback.dm8006_2.pos;
    }
    if (((selected_mask & 0x04U) == 0U) &&
        (feedback.el05_3.feedback_ok != 0U))
    {
        motor_target[2] = feedback.el05_3.pos;
    }

    cmd.dm8006_1.mode = ROBOTARM_COMM_POS_SPEED;
    cmd.dm8006_1.pos = motor_target[0];
    cmd.dm8006_1.vel = R2_ARM_JOINT_POS_SPEED_RAD_S;
    cmd.dm8006_1.kp = 0.0f;
    cmd.dm8006_1.kd = 0.0f;
    cmd.dm8006_1.tor = 0.0f;

    cmd.dm8006_2.mode = ROBOTARM_COMM_POS_SPEED;
    cmd.dm8006_2.pos = motor_target[1];
    cmd.dm8006_2.vel = R2_ARM_J2_POS_SPEED_RAD_S;
    cmd.dm8006_2.kp = 0.0f;
    cmd.dm8006_2.kd = 0.0f;
    cmd.dm8006_2.tor = 0.0f;

    cmd.el05.mode = ROBOTARM_COMM_POS_SPEED;
    cmd.el05.pos = motor_target[2];
    cmd.el05.vel = R2_ARM_J3_POS_SPEED_RAD_S;
    cmd.el05.kp = 0.0f;
    cmd.el05.kd = 0.0f;
    cmd.el05.tor = 0.0f;

    RobotArm_Mixed_SetControlCommand(&cmd);
}

__weak void R2_Arm_OutputApplyJointTargetsRad(const RobotArmIKResult_t *target)
{
    RobotArm_MixedCommand_t cmd;
    RobotArm_FDCAN3Feedback_t feedback;
    float live_theta3_rad;
    uint8_t j2_feedback_ready;

    if ((target == NULL) || (target->status != ROBOTARM_IK_OK))
    {
        return;
    }

    live_theta3_rad = target->theta_rad[2];
    RobotArm_Mixed_GetFeedbackSnapshot(&feedback);

    j2_feedback_ready = ((feedback.dm8006_2.feedback_ok != 0U) &&
                         (fabsf(R2_ARM_J2_MOTOR_SIGN) > 1.0e-6f) &&
                         (fabsf(R2_ARM_J2_MOTOR_PER_JOINT_RAD) > 1.0e-6f)) ?
                        1U : 0U;
    if (j2_feedback_ready == 0U)
    {
        /*
         * Tool pose hold is a coupled J2/J3 motion. If J2 feedback is not
         * available, sending the final J3 target can make the wrist unwind
         * before J2 has moved. Keep the previous command until live J2
         * position is available.
         */
        return;
    }

    {
        float actual_theta2_rad = R2_Arm_MotorJ2ToModelRad(feedback.dm8006_2.pos);
        float theta2_delta_rad = actual_theta2_rad - target->theta_rad[1];
        float reference_theta3_rad = target->theta_rad[2];

        if ((feedback.el05_3.feedback_ok != 0U) &&
            (fabsf(R2_ARM_J3_MOTOR_SIGN) > 1.0e-6f) &&
            (fabsf(R2_ARM_J3_MOTOR_PER_JOINT_RAD) > 1.0e-6f))
        {
            reference_theta3_rad = R2_Arm_MotorJ3ToModelRad(feedback.el05_3.pos);
        }

        /*
         * Keep Frame4/tool posture fixed in model space first:
         * J3 follows the opposite of live J2 error, then maps to the real
         * EL05 motor through R2_Arm_ModelJ3ToMotorRad().
         */
        live_theta3_rad = R2_Arm_NormalizeNear(target->theta_rad[2] -
                                               theta2_delta_rad,
                                               reference_theta3_rad);
    }

    cmd.dm8006_1.mode = ROBOTARM_COMM_POS_SPEED;
    cmd.dm8006_1.pos = R2_Arm_ModelJ1ToMotorRad(target->theta_rad[0]);
    cmd.dm8006_1.vel = R2_ARM_JOINT_POS_SPEED_RAD_S;
    cmd.dm8006_1.kp = 0.0f;
    cmd.dm8006_1.kd = 0.0f;
    cmd.dm8006_1.tor = 0.0f;

    cmd.dm8006_2.mode = ROBOTARM_COMM_POS_SPEED;
    cmd.dm8006_2.pos = R2_Arm_ModelJ2ToMotorRad(target->theta_rad[1]);
    cmd.dm8006_2.vel = R2_ARM_J2_POS_SPEED_RAD_S;
    cmd.dm8006_2.kp = 0.0f;
    cmd.dm8006_2.kd = 0.0f;
    cmd.dm8006_2.tor = 0.0f;

    cmd.el05.mode = ROBOTARM_COMM_POS_SPEED;
    cmd.el05.pos = R2_Arm_ModelJ3ToMotorRad(live_theta3_rad);
    cmd.el05.vel = R2_ARM_J3_POS_SPEED_RAD_S;
    cmd.el05.kp = 0.0f;
    cmd.el05.kd = 0.0f;
    cmd.el05.tor = 0.0f;

    RobotArm_Mixed_SetControlCommand(&cmd);
}

static void R2_Arm_CancelIKTestFlow(R2_Arm_Ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->ik_test_active = 0U;
    ctrl->ik_test_step = 0U;
    ctrl->ik_test_done = 0U;
    ctrl->ik_test_error = 0U;
    ctrl->ik_test_step_start_ms = 0U;
}

static RobotArmIKStatus_t R2_Arm_SetCommandError(R2_Arm_Ctrl_t *ctrl,
                                                 RobotArmIKStatus_t status,
                                                 RobotArmIKReason_t reason,
                                                 uint32_t now_ms)
{
    uint8_t hold_output;

    if (ctrl == NULL)
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    hold_output = ((ctrl->enabled != 0U) &&
                   (ctrl->has_target != 0U) &&
                   (ctrl->result.status == ROBOTARM_IK_OK)) ? 1U : 0U;
    R2_Arm_CancelEnableStartup(ctrl);
    R2_Arm_CancelIKTestFlow(ctrl);
    R2_Arm_CancelWorkspaceSwitch(ctrl);
    ctrl->last_ik_status = (uint8_t)status;
    ctrl->last_ik_reason = (uint8_t)reason;
    ctrl->result.status = status;
    ctrl->result.reason = reason;
    ctrl->last_command_ms = now_ms;
    ctrl->state = (uint8_t)R2_ARM_STATE_ERROR;
    ctrl->output_enabled = hold_output;
    R2_Arm_UpdateFlags(ctrl);
    return status;
}

static uint8_t R2_Arm_IsDirectionValid(RobotArmWorkDirection_t direction)
{
    return ((direction == ROBOTARM_WORK_DIR_Y_POS) ||
            (direction == ROBOTARM_WORK_DIR_X_POS) ||
            (direction == ROBOTARM_WORK_DIR_X_NEG)) ? 1U : 0U;
}

static float R2_Arm_LongToChordOffsetRad(void)
{
    float adjacent = ROBOTARM_KIN_LONG_LINK_MM +
                     ROBOTARM_KIN_SHORT_LINK_MM * cosf(ROBOTARM_KIN_BEND_RAD);
    float opposite = ROBOTARM_KIN_SHORT_LINK_MM * sinf(ROBOTARM_KIN_BEND_RAD);

    return atan2f(opposite, adjacent);
}

static float R2_Arm_AlphaToTheta2Rad(float alpha_rad)
{
    return R2_Arm_NormalizePi(alpha_rad -
                              R2_Arm_LongToChordOffsetRad() -
                              (0.5f * ROBOTARM_KIN_PI));
}

static float R2_Arm_PowerOnTheta2Rad(void)
{
    return R2_Arm_AlphaToTheta2Rad(R2_ARM_POWERON_LONG_ALPHA_RAD);
}

static float R2_Arm_PowerOnTheta3Rad(void)
{
    return R2_Arm_NormalizePi(R2_ARM_POWERON_FRAME4_PHI_RAD -
                              R2_Arm_PowerOnTheta2Rad());
}

static float R2_Arm_ModelJ1ToMotorRad(float model_yaw_rad)
{
    return R2_ARM_J1_MOTOR_ZERO_OFFSET_RAD +
           R2_ARM_J1_MOTOR_SIGN * model_yaw_rad;
}

static float R2_Arm_ModelJ2ToMotorRad(float model_theta2_rad)
{
    return R2_ARM_J2_MOTOR_SIGN *
           (model_theta2_rad - R2_Arm_PowerOnTheta2Rad()) *
           R2_ARM_J2_MOTOR_PER_JOINT_RAD;
}

static float R2_Arm_ModelJ3ToMotorRad(float model_theta3_rad)
{
    return R2_ARM_J3_MOTOR_SIGN *
           (model_theta3_rad - R2_Arm_PowerOnTheta3Rad()) *
           R2_ARM_J3_MOTOR_PER_JOINT_RAD;
}

static float R2_Arm_MotorJ1ToModelRad(float motor_yaw_rad)
{
    return R2_Arm_NormalizePi((motor_yaw_rad -
                               R2_ARM_J1_MOTOR_ZERO_OFFSET_RAD) /
                              R2_ARM_J1_MOTOR_SIGN);
}

static float R2_Arm_MotorJ2ToModelRad(float motor_theta2_rad)
{
    return R2_Arm_NormalizePi((motor_theta2_rad /
                               (R2_ARM_J2_MOTOR_SIGN *
                                R2_ARM_J2_MOTOR_PER_JOINT_RAD)) +
                              R2_Arm_PowerOnTheta2Rad());
}

static float R2_Arm_MotorJ3ToModelRad(float motor_theta3_rad)
{
    return (motor_theta3_rad /
            (R2_ARM_J3_MOTOR_SIGN * R2_ARM_J3_MOTOR_PER_JOINT_RAD)) +
           R2_Arm_PowerOnTheta3Rad();
}

static void R2_Arm_GetMotorFeedbackRad(float feedback_rad[ROBOTARM_KIN_JOINT_COUNT],
                                       uint8_t *ok_mask)
{
    RobotArm_FDCAN3Feedback_t feedback;
    uint8_t mask = 0U;

    if (feedback_rad == NULL)
    {
        return;
    }

    RobotArm_Mixed_GetFeedbackSnapshot(&feedback);
    feedback_rad[0] = feedback.dm8006_1.pos;
    feedback_rad[1] = feedback.dm8006_2.pos;
    feedback_rad[2] = feedback.el05_3.pos;

    if (feedback.dm8006_1.feedback_ok != 0U) mask |= 0x01U;
    if (feedback.dm8006_2.feedback_ok != 0U) mask |= 0x02U;
    if (feedback.el05_3.feedback_ok != 0U) mask |= 0x04U;
    if (ok_mask != NULL)
    {
        *ok_mask = mask;
    }
}

static uint8_t R2_Arm_GetFeedbackModelTheta(float theta_rad[ROBOTARM_KIN_JOINT_COUNT],
                                            uint8_t *ok_mask)
{
    float motor_rad[ROBOTARM_KIN_JOINT_COUNT];
    uint8_t mask = 0U;

    if (theta_rad == NULL)
    {
        return 0U;
    }

    R2_Arm_GetMotorFeedbackRad(motor_rad, &mask);
    if ((fabsf(R2_ARM_J1_MOTOR_SIGN) < 1.0e-6f) &&
        ((mask & 0x01U) != 0U))
    {
        mask &= (uint8_t)~0x01U;
    }
    if (((fabsf(R2_ARM_J2_MOTOR_SIGN) < 1.0e-6f) ||
         (fabsf(R2_ARM_J2_MOTOR_PER_JOINT_RAD) < 1.0e-6f)) &&
        ((mask & 0x02U) != 0U))
    {
        mask &= (uint8_t)~0x02U;
    }
    if (((fabsf(R2_ARM_J3_MOTOR_SIGN) < 1.0e-6f) ||
         (fabsf(R2_ARM_J3_MOTOR_PER_JOINT_RAD) < 1.0e-6f)) &&
        ((mask & 0x04U) != 0U))
    {
        mask &= (uint8_t)~0x04U;
    }

    theta_rad[0] = ((mask & 0x01U) != 0U) ?
        R2_Arm_MotorJ1ToModelRad(motor_rad[0]) : 0.0f;
    theta_rad[1] = ((mask & 0x02U) != 0U) ?
        R2_Arm_MotorJ2ToModelRad(motor_rad[1]) : 0.0f;
    theta_rad[2] = ((mask & 0x04U) != 0U) ?
        R2_Arm_MotorJ3ToModelRad(motor_rad[2]) : 0.0f;

    if (ok_mask != NULL)
    {
        *ok_mask = mask;
    }
    return (mask != 0U) ? 1U : 0U;
}

static float R2_Arm_NormalizePi(float x)
{
    while (x > ROBOTARM_KIN_PI)
    {
        x -= 2.0f * ROBOTARM_KIN_PI;
    }
    while (x < -ROBOTARM_KIN_PI)
    {
        x += 2.0f * ROBOTARM_KIN_PI;
    }
    return x;
}

static float R2_Arm_NormalizeNear(float x, float reference)
{
    while ((x - reference) > ROBOTARM_KIN_PI)
    {
        x -= 2.0f * ROBOTARM_KIN_PI;
    }
    while ((x - reference) < -ROBOTARM_KIN_PI)
    {
        x += 2.0f * ROBOTARM_KIN_PI;
    }
    return x;
}

static void R2_Arm_SetPowerOnHoldState(R2_Arm_Ctrl_t *ctrl)
{
    RobotArmIKStatus_t ik_status;

    if (ctrl == NULL)
    {
        return;
    }

    ctrl->current_direction = ROBOTARM_WORK_DIR_Y_POS;
    ctrl->current_alpha_rad = R2_ARM_POWERON_LONG_ALPHA_RAD;
    ctrl->current_theta_rad[0] = R2_ARM_POWERON_YAW_RAD;
    ctrl->current_theta_rad[1] = R2_Arm_PowerOnTheta2Rad();
    ctrl->current_theta_rad[2] = R2_Arm_PowerOnTheta3Rad();

    ik_status = RobotArmKinematics_Forward(R2_ARM_DEFAULT_TOOL,
                                           ctrl->current_theta_rad,
                                           &ctrl->result);

    ctrl->request.tool = R2_ARM_DEFAULT_TOOL;
    ctrl->request.state = R2_ARM_DEFAULT_STATE;
    ctrl->request.approach_yaw_rad = R2_ARM_POWERON_YAW_RAD;
    ctrl->request.target_direction_valid = 1U;
    ctrl->request.target_direction = ROBOTARM_WORK_DIR_Y_POS;
    ctrl->request.posture_source_valid = 0U;
    ctrl->request.posture_source_tool = ctrl->request.tool;
    ctrl->request.posture_source_state = ctrl->request.state;
    if (ik_status == ROBOTARM_IK_OK)
    {
        ctrl->request.target_z_mm = ctrl->result.tool_world_mm.z;
        ctrl->request.target_xyz_mm = ctrl->result.tool_world_mm;
    }
    else
    {
        ctrl->request.target_z_mm = 0.0f;
        R2_Arm_FillLegacyTargetXY(&ctrl->request,
                                  ctrl->request.approach_yaw_rad);
    }
    R2_Arm_SetRequestCurrent(ctrl, &ctrl->request);

    ctrl->last_ik_status = (uint8_t)ik_status;
    ctrl->last_ik_reason = (uint8_t)ctrl->result.reason;
    ctrl->has_target = 1U;
    ctrl->output_enabled = 0U;
    ctrl->state = (uint8_t)R2_ARM_STATE_IDLE;
}

static RobotArmIKStatus_t R2_Arm_StartEnableStartup(R2_Arm_Ctrl_t *ctrl,
                                                    uint32_t now_ms)
{
    if (ctrl == NULL)
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    ctrl->enable_startup_active = 1U;
    ctrl->enable_startup_phase = R2_ARM_ENABLE_STARTUP_PHASE_J2;
    return R2_Arm_SetEnableStartupPose(ctrl,
                                       R2_ARM_ENABLE_STARTUP_PHASE_J2,
                                       now_ms);
}

static RobotArmIKStatus_t R2_Arm_SetEnableStartupPose(R2_Arm_Ctrl_t *ctrl,
                                                       uint8_t phase,
                                                       uint32_t now_ms)
{
    RobotArmIKStatus_t ik_status;
    float theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    float current_theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    float feedback_theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    RobotArmWorkDirection_t direction;
    uint8_t feedback_ok_mask = 0U;
    uint8_t i;

    if (ctrl == NULL)
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
    {
        current_theta_rad[i] = ctrl->current_theta_rad[i];
    }
    if (R2_Arm_GetFeedbackModelTheta(feedback_theta_rad,
                                     &feedback_ok_mask) != 0U)
    {
        for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
        {
            if ((feedback_ok_mask & (uint8_t)(1U << i)) != 0U)
            {
                current_theta_rad[i] = feedback_theta_rad[i];
            }
        }
    }

    if (phase == R2_ARM_ENABLE_STARTUP_PHASE_J2)
    {
        theta_rad[0] = current_theta_rad[0];
        direction = ctrl->current_direction;
    }
    else if (phase == R2_ARM_ENABLE_STARTUP_PHASE_J1)
    {
        theta_rad[0] = R2_Arm_MotorJ1ToModelRad(0.0f);
        direction = ROBOTARM_WORK_DIR_Y_POS;
    }
    else
    {
        return ROBOTARM_IK_ERR_BAD_PARAM;
    }

    theta_rad[1] = R2_Arm_AlphaToTheta2Rad(R2_ARM_ENABLE_ALPHA_RAD);
    theta_rad[2] = R2_Arm_PowerOnTheta3Rad();

    ik_status = RobotArmKinematics_ForwardState(R2_ARM_DEFAULT_TOOL,
                                                R2_ARM_DEFAULT_STATE,
                                                theta_rad,
                                                &ctrl->result);
    ctrl->result.target_direction = direction;
    ctrl->last_ik_status = (uint8_t)ik_status;
    ctrl->last_ik_reason = (uint8_t)ctrl->result.reason;

    ctrl->request.tool = R2_ARM_DEFAULT_TOOL;
    ctrl->request.state = R2_ARM_DEFAULT_STATE;
    ctrl->request.target_xyz_mm = ctrl->result.tool_world_mm;
    ctrl->request.target_z_mm = ctrl->result.tool_world_mm.z;
    ctrl->request.approach_yaw_rad = theta_rad[0];
    ctrl->request.target_direction_valid = 1U;
    ctrl->request.target_direction = direction;
    ctrl->request.posture_source_valid = 0U;
    ctrl->request.posture_source_tool = ctrl->request.tool;
    ctrl->request.posture_source_state = ctrl->request.state;
    ctrl->request.current_direction = ctrl->current_direction;
    ctrl->request.current_alpha_rad = ctrl->current_alpha_rad;
    ctrl->request.current_theta_rad[0] = ctrl->current_theta_rad[0];
    ctrl->request.current_theta_rad[1] = ctrl->current_theta_rad[1];
    ctrl->request.current_theta_rad[2] = ctrl->current_theta_rad[2];

    ctrl->has_target = 1U;
    ctrl->last_command_ms = now_ms;

    if (ik_status == ROBOTARM_IK_OK)
    {
        ctrl->enable_startup_active = 1U;
        ctrl->enable_startup_phase = phase;
        ctrl->state = (uint8_t)R2_ARM_STATE_TARGET_VALID;
        ctrl->output_enabled = (ctrl->enabled != 0U) ? 1U : 0U;
        R2_Arm_CommitResultIfOk(ctrl, ik_status);
    }
    else
    {
        ctrl->enable_startup_active = 0U;
        ctrl->enable_startup_phase = R2_ARM_ENABLE_STARTUP_PHASE_NONE;
        ctrl->state = (uint8_t)R2_ARM_STATE_ERROR;
        ctrl->output_enabled = 0U;
    }

    return ik_status;
}

static void R2_Arm_UpdateEnableStartup(R2_Arm_Ctrl_t *ctrl,
                                       uint32_t now_ms)
{
    uint8_t selected_mask;

    if (ctrl == NULL)
    {
        return;
    }

    (void)now_ms;

    if ((ctrl->enabled == 0U) ||
        (ctrl->state == (uint8_t)R2_ARM_STATE_STOPPED))
    {
        R2_Arm_CancelEnableStartup(ctrl);
        ctrl->output_enabled = 0U;
        R2_Arm_UpdateMotorHardwareState(ctrl);
        R2_Arm_UpdateFlags(ctrl);
        return;
    }

    if ((ctrl->last_ik_status == (uint8_t)ROBOTARM_IK_OK) &&
        (ctrl->state != (uint8_t)R2_ARM_STATE_ERROR))
    {
        selected_mask =
            R2_Arm_EnableStartupOutputMask(ctrl->enable_startup_phase);
        ctrl->output_enabled = 1U;
        R2_Arm_UpdateMotorHardwareState(ctrl);
        R2_Arm_OutputApplyIndependentJointTargetsRad(ctrl->result.theta_rad,
                                                     selected_mask);
        ctrl->output_apply_count++;
    }
    else
    {
        ctrl->output_enabled = 0U;
        R2_Arm_UpdateMotorHardwareState(ctrl);
        R2_Arm_UpdateFlags(ctrl);
        return;
    }

    if (R2_Arm_EnableStartupTargetReached(
            ctrl, ctrl->enable_startup_phase) != 0U)
    {
        if (ctrl->enable_startup_phase == R2_ARM_ENABLE_STARTUP_PHASE_J2)
        {
            (void)R2_Arm_SetEnableStartupPose(
                ctrl,
                R2_ARM_ENABLE_STARTUP_PHASE_J1,
                now_ms);
            R2_Arm_UpdateFlags(ctrl);
            return;
        }

        ctrl->enable_startup_active = 0U;
        ctrl->enable_startup_phase = R2_ARM_ENABLE_STARTUP_PHASE_NONE;
        ctrl->current_direction = ctrl->result.target_direction;
        ctrl->joint_jog_active = 1U;
        ctrl->joint_jog_index = 0U;
        ctrl->state = (uint8_t)R2_ARM_STATE_TARGET_VALID;
        ctrl->output_enabled =
            ((ctrl->enabled != 0U) &&
             (ctrl->last_ik_status == (uint8_t)ROBOTARM_IK_OK)) ? 1U : 0U;
        R2_Arm_UpdateMotorHardwareState(ctrl);
        R2_Arm_UpdateFlags(ctrl);
        return;
    }

    R2_Arm_UpdateFlags(ctrl);
}

static uint8_t R2_Arm_EnableStartupTargetReached(
    const R2_Arm_Ctrl_t *ctrl,
    uint8_t phase)
{
    float feedback_theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    float feedback_motor_rad[ROBOTARM_KIN_JOINT_COUNT];
    float err_rad;
    float motor_target_rad;
    uint8_t ok_mask = 0U;
    uint8_t motor_ok_mask = 0U;
    uint8_t reached_mask;
    uint8_t i;

    if (ctrl == NULL)
    {
        return 0U;
    }

    reached_mask = R2_Arm_EnableStartupReachedMask(phase);
    if (reached_mask == 0U)
    {
        return 0U;
    }

    if (R2_Arm_GetFeedbackModelTheta(feedback_theta_rad, &ok_mask) == 0U)
    {
        return 0U;
    }
    if ((ok_mask & reached_mask) != reached_mask)
    {
        return 0U;
    }

    if ((phase == R2_ARM_ENABLE_STARTUP_PHASE_J1) &&
        ((reached_mask & 0x01U) != 0U))
    {
        R2_Arm_GetMotorFeedbackRad(feedback_motor_rad, &motor_ok_mask);
        if ((motor_ok_mask & 0x01U) == 0U)
        {
            return 0U;
        }
        motor_target_rad = R2_Arm_ModelJ1ToMotorRad(
            ctrl->result.theta_rad[0]);
        err_rad = feedback_motor_rad[0] - motor_target_rad;
        if (fabsf(err_rad) > R2_ARM_YAW_REACHED_TOL_RAD)
        {
            return 0U;
        }
        reached_mask &= (uint8_t)~0x01U;
    }

    for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
    {
        if ((reached_mask & (uint8_t)(1U << i)) == 0U)
        {
            continue;
        }

        err_rad = feedback_theta_rad[i] - ctrl->result.theta_rad[i];
        if (i != 2U)
        {
            err_rad = R2_Arm_NormalizePi(err_rad);
        }
        if (fabsf(err_rad) > R2_ARM_YAW_REACHED_TOL_RAD)
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t R2_Arm_EnableStartupOutputMask(uint8_t phase)
{
    if (phase == R2_ARM_ENABLE_STARTUP_PHASE_J2)
    {
        return 0x06U;
    }
    if (phase == R2_ARM_ENABLE_STARTUP_PHASE_J1)
    {
        return 0x05U;
    }
    return 0U;
}

static uint8_t R2_Arm_EnableStartupReachedMask(uint8_t phase)
{
    if (phase == R2_ARM_ENABLE_STARTUP_PHASE_J2)
    {
        return 0x02U;
    }
    if (phase == R2_ARM_ENABLE_STARTUP_PHASE_J1)
    {
        return 0x01U;
    }
    return 0U;
}

static void R2_Arm_CancelEnableStartup(R2_Arm_Ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->enable_startup_active = 0U;
    ctrl->enable_startup_phase = R2_ARM_ENABLE_STARTUP_PHASE_NONE;
}

static RobotArmIKStatus_t R2_Arm_StartWorkspaceSwitch(
    R2_Arm_Ctrl_t *ctrl,
    RobotArmWorkDirection_t direction,
    uint32_t now_ms)
{
    float feedback_theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    float target_yaw_rad;
    float yaw_err_rad;
    uint8_t feedback_ok_mask = 0U;
    uint8_t i;
    RobotArmIKStatus_t ik_status;

    if (ctrl == NULL)
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    if (!R2_Arm_IsDirectionValid(direction))
    {
        return R2_Arm_SetCommandError(ctrl,
                                      ROBOTARM_IK_ERR_BAD_DIRECTION,
                                      ROBOTARM_IK_REASON_BAD_DIRECTION,
                                      now_ms);
    }

    if ((ctrl->enabled == 0U) ||
        (ctrl->state == (uint8_t)R2_ARM_STATE_STOPPED))
    {
        return R2_Arm_SetCommandError(ctrl,
                                      ROBOTARM_IK_ERR_UNSUPPORTED_STATE,
                                      ROBOTARM_IK_REASON_UNSUPPORTED_STATE,
                                      now_ms);
    }

    for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
    {
        ctrl->workspace_switch_saved_theta_rad[i] = ctrl->current_theta_rad[i];
    }
    if (R2_Arm_GetFeedbackModelTheta(feedback_theta_rad,
                                     &feedback_ok_mask) != 0U)
    {
        for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
        {
            if ((feedback_ok_mask & (uint8_t)(1U << i)) != 0U)
            {
                ctrl->workspace_switch_saved_theta_rad[i] =
                    feedback_theta_rad[i];
            }
        }
    }

    R2_Arm_CancelEnableStartup(ctrl);
    R2_Arm_CancelIKTestFlow(ctrl);
    R2_Arm_CancelIndependentJointJog(ctrl);

    ctrl->workspace_switch_target_direction = direction;
    target_yaw_rad = RobotArmKinematics_DirectionYaw(direction);
    yaw_err_rad = R2_Arm_NormalizePi(
        ctrl->workspace_switch_saved_theta_rad[0] - target_yaw_rad);

    if (fabsf(yaw_err_rad) <= R2_ARM_YAW_REACHED_TOL_RAD)
    {
        ctrl->workspace_switch_saved_theta_rad[0] = target_yaw_rad;
        ctrl->workspace_switch_active = 0U;
        ctrl->workspace_switch_phase = R2_ARM_WORKSPACE_SWITCH_PHASE_NONE;
        ik_status = R2_Arm_SetWorkspaceSwitchPhase(
            ctrl,
            R2_ARM_WORKSPACE_SWITCH_PHASE_RESTORE_J2,
            now_ms);
        R2_Arm_CancelWorkspaceSwitch(ctrl);
        ctrl->current_direction = direction;
        return ik_status;
    }

    ctrl->workspace_switch_active = 1U;
    ctrl->workspace_switch_phase = R2_ARM_WORKSPACE_SWITCH_PHASE_ALPHA_MAX;
    return R2_Arm_SetWorkspaceSwitchPhase(
        ctrl,
        R2_ARM_WORKSPACE_SWITCH_PHASE_ALPHA_MAX,
        now_ms);
}

static RobotArmIKStatus_t R2_Arm_SetWorkspaceSwitchPhase(
    R2_Arm_Ctrl_t *ctrl,
    uint8_t phase,
    uint32_t now_ms)
{
    RobotArmIKRequest_t request;
    RobotArmIKStatus_t ik_status;
    RobotArmIKReason_t reason = ROBOTARM_IK_REASON_NONE;
    float theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    uint8_t i;

    if (ctrl == NULL)
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    if ((phase != R2_ARM_WORKSPACE_SWITCH_PHASE_ALPHA_MAX) &&
        (phase != R2_ARM_WORKSPACE_SWITCH_PHASE_J1_Y_POS) &&
        (phase != R2_ARM_WORKSPACE_SWITCH_PHASE_J1) &&
        (phase != R2_ARM_WORKSPACE_SWITCH_PHASE_RESTORE_J2))
    {
        return R2_Arm_SetCommandError(ctrl,
                                      ROBOTARM_IK_ERR_BAD_PARAM,
                                      ROBOTARM_IK_REASON_BAD_FLOAT,
                                      now_ms);
    }

    R2_Arm_BuildWorkspaceSwitchTheta(ctrl, phase, theta_rad);
    for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
    {
        theta_rad[i] = R2_Arm_ClampJointLimitTolerance(ctrl,
                                                       i,
                                                       theta_rad[i]);
    }
    ik_status = R2_Arm_ValidateWorkspaceSwitchTheta(ctrl,
                                                    theta_rad,
                                                    phase,
                                                    &reason);
    if (ik_status != ROBOTARM_IK_OK)
    {
        return R2_Arm_SetCommandError(ctrl, ik_status, reason, now_ms);
    }

    ik_status = RobotArmKinematics_ForwardState(ctrl->request.tool,
                                                ctrl->request.state,
                                                theta_rad,
                                                &ctrl->result);
    if (ik_status != ROBOTARM_IK_OK)
    {
        return R2_Arm_SetCommandError(ctrl,
                                      ik_status,
                                      ctrl->result.reason,
                                      now_ms);
    }

    if ((ctrl->result.alpha_rad <
         (ctrl->kinematics_config.alpha_min_rad -
          R2_ARM_LONG_LINK_LIMIT_TOL_RAD)) ||
        (ctrl->result.alpha_rad >
         (ctrl->kinematics_config.alpha_max_rad +
          R2_ARM_LONG_LINK_LIMIT_TOL_RAD)))
    {
        return R2_Arm_SetCommandError(ctrl,
                                      ROBOTARM_IK_ERR_LONG_LINK_LIMIT,
                                      ROBOTARM_IK_REASON_LONG_LINK_LIMIT,
                                      now_ms);
    }

    request = ctrl->request;
    request.tool = ctrl->result.tool;
    request.state = ctrl->result.state;
    request.target_xyz_mm = ctrl->result.tool_world_mm;
    request.target_z_mm = ctrl->result.tool_world_mm.z;
    request.approach_yaw_rad = ctrl->result.approach_yaw_rad;
    request.target_direction_valid = 1U;
    request.target_direction = ctrl->workspace_switch_target_direction;
    request.posture_source_valid = 0U;
    request.posture_source_tool = request.tool;
    request.posture_source_state = request.state;
    R2_Arm_SetRequestCurrent(ctrl, &request);

    ctrl->request = request;
    ctrl->has_target = 1U;
    ctrl->last_command_ms = now_ms;
    ctrl->last_ik_status = (uint8_t)ROBOTARM_IK_OK;
    ctrl->last_ik_reason = (uint8_t)ROBOTARM_IK_REASON_NONE;
    ctrl->state = (uint8_t)R2_ARM_STATE_TARGET_VALID;
    ctrl->output_enabled = (ctrl->enabled != 0U) ? 1U : 0U;
    ctrl->workspace_switch_phase = phase;
    R2_Arm_CommitResultIfOk(ctrl, ROBOTARM_IK_OK);
    R2_Arm_UpdateMotorHardwareState(ctrl);
    if (ctrl->output_enabled != 0U)
    {
        R2_Arm_OutputApplyIndependentJointTargetsRad(ctrl->result.theta_rad,
            R2_Arm_WorkspaceSwitchJointMask(phase));
        ctrl->output_apply_count++;
    }
    R2_Arm_UpdateFlags(ctrl);
    return ROBOTARM_IK_OK;
}

static void R2_Arm_UpdateWorkspaceSwitch(R2_Arm_Ctrl_t *ctrl,
                                         uint32_t now_ms)
{
    uint8_t next_phase = R2_ARM_WORKSPACE_SWITCH_PHASE_NONE;

    if (ctrl == NULL)
    {
        return;
    }

    if ((ctrl->enabled == 0U) ||
        (ctrl->has_target == 0U) ||
        (ctrl->state == (uint8_t)R2_ARM_STATE_STOPPED))
    {
        R2_Arm_CancelWorkspaceSwitch(ctrl);
        ctrl->output_enabled = 0U;
        R2_Arm_UpdateMotorHardwareState(ctrl);
        R2_Arm_UpdateFlags(ctrl);
        return;
    }

    if (R2_Arm_WorkspaceSwitchTargetReached(ctrl) != 0U)
    {
        if (ctrl->workspace_switch_phase ==
            R2_ARM_WORKSPACE_SWITCH_PHASE_ALPHA_MAX)
        {
            next_phase =
                (R2_Arm_WorkspaceSwitchNeedsYPosPhase(ctrl) != 0U) ?
                R2_ARM_WORKSPACE_SWITCH_PHASE_J1_Y_POS :
                R2_ARM_WORKSPACE_SWITCH_PHASE_J1;
        }
        else if (ctrl->workspace_switch_phase ==
                 R2_ARM_WORKSPACE_SWITCH_PHASE_J1_Y_POS)
        {
            next_phase = R2_ARM_WORKSPACE_SWITCH_PHASE_J1;
        }
        else if (ctrl->workspace_switch_phase ==
                 R2_ARM_WORKSPACE_SWITCH_PHASE_J1)
        {
            next_phase = R2_ARM_WORKSPACE_SWITCH_PHASE_RESTORE_J2;
        }
        else
        {
            ctrl->workspace_switch_active = 0U;
            ctrl->workspace_switch_phase =
                R2_ARM_WORKSPACE_SWITCH_PHASE_NONE;
            ctrl->current_direction =
                ctrl->workspace_switch_target_direction;
            ctrl->state = (uint8_t)R2_ARM_STATE_TARGET_VALID;
            ctrl->output_enabled =
                ((ctrl->enabled != 0U) &&
                 (ctrl->last_ik_status == (uint8_t)ROBOTARM_IK_OK)) ? 1U : 0U;
            R2_Arm_UpdateMotorHardwareState(ctrl);
            R2_Arm_UpdateFlags(ctrl);
            return;
        }

        if (R2_Arm_SetWorkspaceSwitchPhase(ctrl,
                                           next_phase,
                                           now_ms) != ROBOTARM_IK_OK)
        {
            return;
        }
        return;
    }

    if ((ctrl->last_ik_status == (uint8_t)ROBOTARM_IK_OK) &&
        (ctrl->state != (uint8_t)R2_ARM_STATE_ERROR))
    {
        ctrl->output_enabled = 1U;
        R2_Arm_UpdateMotorHardwareState(ctrl);
        R2_Arm_OutputApplyIndependentJointTargetsRad(ctrl->result.theta_rad,
            R2_Arm_WorkspaceSwitchJointMask(
                ctrl->workspace_switch_phase));
        ctrl->output_apply_count++;
    }
    else
    {
        ctrl->output_enabled = 0U;
        R2_Arm_UpdateMotorHardwareState(ctrl);
    }

    R2_Arm_UpdateFlags(ctrl);
}

static uint8_t R2_Arm_WorkspaceSwitchTargetReached(
    const R2_Arm_Ctrl_t *ctrl)
{
    float feedback_theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    float err_rad;
    uint8_t ok_mask = 0U;
    uint8_t reached_mask;
    uint8_t i;

    if (ctrl == NULL)
    {
        return 0U;
    }

    if (R2_Arm_GetFeedbackModelTheta(feedback_theta_rad, &ok_mask) == 0U)
    {
        return 0U;
    }
    reached_mask = R2_Arm_WorkspaceSwitchJointMask(
        ctrl->workspace_switch_phase);
    if ((reached_mask == 0U) ||
        ((ok_mask & reached_mask) != reached_mask))
    {
        return 0U;
    }

    for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
    {
        if ((reached_mask & (uint8_t)(1U << i)) == 0U)
        {
            continue;
        }

        err_rad = feedback_theta_rad[i] - ctrl->result.theta_rad[i];
        if (i != 2U)
        {
            err_rad = R2_Arm_NormalizePi(err_rad);
        }
        if (fabsf(err_rad) > R2_ARM_YAW_REACHED_TOL_RAD)
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t R2_Arm_WorkspaceSwitchJointMask(uint8_t phase)
{
    if (phase == R2_ARM_WORKSPACE_SWITCH_PHASE_ALPHA_MAX)
    {
        /* Only J2 moves to the safe long-link angle; J1/J3 hold feedback. */
        return 0x02U;
    }
    if ((phase == R2_ARM_WORKSPACE_SWITCH_PHASE_J1_Y_POS) ||
        (phase == R2_ARM_WORKSPACE_SWITCH_PHASE_J1) ||
        (phase == R2_ARM_WORKSPACE_SWITCH_PHASE_RESTORE_J2))
    {
        /* Keep J2 at its phase target while J1 turns or J2 is restored. */
        return 0x03U;
    }
    return 0U;
}

static uint8_t R2_Arm_WorkspaceSwitchNeedsYPosPhase(
    const R2_Arm_Ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
        return 0U;
    }

    return (((ctrl->current_direction == ROBOTARM_WORK_DIR_X_POS) &&
             (ctrl->workspace_switch_target_direction ==
              ROBOTARM_WORK_DIR_X_NEG)) ||
            ((ctrl->current_direction == ROBOTARM_WORK_DIR_X_NEG) &&
             (ctrl->workspace_switch_target_direction ==
              ROBOTARM_WORK_DIR_X_POS))) ? 1U : 0U;
}

static void R2_Arm_BuildWorkspaceSwitchTheta(
    const R2_Arm_Ctrl_t *ctrl,
    uint8_t phase,
    float theta_rad[ROBOTARM_KIN_JOINT_COUNT])
{
    uint8_t i;

    if ((ctrl == NULL) || (theta_rad == NULL))
    {
        return;
    }

    for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
    {
        theta_rad[i] = ctrl->workspace_switch_saved_theta_rad[i];
    }

    if ((phase == R2_ARM_WORKSPACE_SWITCH_PHASE_ALPHA_MAX) ||
        (phase == R2_ARM_WORKSPACE_SWITCH_PHASE_J1_Y_POS) ||
        (phase == R2_ARM_WORKSPACE_SWITCH_PHASE_J1))
    {
        theta_rad[1] =
            R2_Arm_AlphaToTheta2Rad(ctrl->kinematics_config.alpha_max_rad);
    }

    if (phase == R2_ARM_WORKSPACE_SWITCH_PHASE_J1_Y_POS)
    {
        theta_rad[0] =
            RobotArmKinematics_DirectionYaw(ROBOTARM_WORK_DIR_Y_POS);
    }
    else if ((phase == R2_ARM_WORKSPACE_SWITCH_PHASE_J1) ||
             (phase == R2_ARM_WORKSPACE_SWITCH_PHASE_RESTORE_J2))
    {
        theta_rad[0] =
            RobotArmKinematics_DirectionYaw(
                ctrl->workspace_switch_target_direction);
    }
}

static RobotArmIKStatus_t R2_Arm_ValidateWorkspaceSwitchTheta(
    const R2_Arm_Ctrl_t *ctrl,
    const float theta_rad[ROBOTARM_KIN_JOINT_COUNT],
    uint8_t phase,
    RobotArmIKReason_t *reason)
{
    uint8_t i;
    uint8_t validated_mask;
    RobotArmIKReason_t joint_reason[ROBOTARM_KIN_JOINT_COUNT] = {
        ROBOTARM_IK_REASON_J1_LIMIT,
        ROBOTARM_IK_REASON_J2_LIMIT,
        ROBOTARM_IK_REASON_J3_LIMIT
    };

    if ((ctrl == NULL) || (theta_rad == NULL))
    {
        if (reason != NULL)
        {
            *reason = ROBOTARM_IK_REASON_BAD_FLOAT;
        }
        return ROBOTARM_IK_ERR_NULL;
    }

    validated_mask = R2_Arm_WorkspaceSwitchJointMask(phase);
    if (validated_mask == 0U)
    {
        if (reason != NULL)
        {
            *reason = ROBOTARM_IK_REASON_BAD_FLOAT;
        }
        return ROBOTARM_IK_ERR_BAD_PARAM;
    }

    for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
    {
        if ((theta_rad[i] != theta_rad[i]) ||
            (theta_rad[i] > 3.4028234e38f) ||
            (theta_rad[i] < -3.4028234e38f))
        {
            if (reason != NULL)
            {
                *reason = ROBOTARM_IK_REASON_BAD_FLOAT;
            }
            return ROBOTARM_IK_ERR_BAD_PARAM;
        }

        if ((validated_mask & (uint8_t)(1U << i)) == 0U)
        {
            continue;
        }

        if ((theta_rad[i] < ctrl->kinematics_config.joint_min_rad[i]) ||
            (theta_rad[i] > ctrl->kinematics_config.joint_max_rad[i]))
        {
            if (reason != NULL)
            {
                *reason = joint_reason[i];
            }
            return ROBOTARM_IK_ERR_JOINT_LIMIT;
        }
    }

    if (reason != NULL)
    {
        *reason = ROBOTARM_IK_REASON_NONE;
    }
    return ROBOTARM_IK_OK;
}

static void R2_Arm_CancelWorkspaceSwitch(R2_Arm_Ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->workspace_switch_active = 0U;
    ctrl->workspace_switch_phase = R2_ARM_WORKSPACE_SWITCH_PHASE_NONE;
    ctrl->workspace_switch_target_direction = ctrl->current_direction;
}

static void R2_Arm_UpdateFlags(R2_Arm_Ctrl_t *ctrl)
{
    uint8_t status = 0U;
    uint8_t error = 0U;

    if (ctrl->enabled != 0U) status |= R2_ARM_STATUS_ENABLED;
    if (ctrl->has_target != 0U) status |= R2_ARM_STATUS_HAS_TARGET;
    if (ctrl->output_enabled != 0U) status |= R2_ARM_STATUS_OUTPUT;
    if (ctrl->last_ik_status == (uint8_t)ROBOTARM_IK_OK)
    {
        status |= R2_ARM_STATUS_IK_OK;
    }
    if (R2_Arm_IsMotorActive(ctrl) != 0U) status |= R2_ARM_STATUS_ACTIVE;

    if (ctrl->last_ik_status != (uint8_t)ROBOTARM_IK_OK)
    {
        error |= R2_ARM_ERR_IK;
    }
    if (ctrl->last_ik_status == (uint8_t)ROBOTARM_IK_ERR_JOINT_LIMIT)
    {
        error |= R2_ARM_ERR_LIMIT;
    }
    if (ctrl->last_ik_status == (uint8_t)ROBOTARM_IK_ERR_HEIGHT_UNREACHABLE)
    {
        error |= R2_ARM_ERR_UNREACHABLE;
    }
    if ((ctrl->last_ik_status == (uint8_t)ROBOTARM_IK_ERR_BAD_PARAM) ||
        (ctrl->last_ik_status == (uint8_t)ROBOTARM_IK_ERR_NULL) ||
        (ctrl->last_ik_status == (uint8_t)ROBOTARM_IK_ERR_BAD_DIRECTION))
    {
        error |= R2_ARM_ERR_BAD_PARAM;
    }
    if (ctrl->last_ik_status == (uint8_t)ROBOTARM_IK_ERR_LONG_LINK_LIMIT)
    {
        error |= R2_ARM_ERR_UNSAFE;
    }
    if (ctrl->last_ik_status == (uint8_t)ROBOTARM_IK_ERR_UNSUPPORTED_STATE)
    {
        error |= R2_ARM_ERR_UNSUPPORTED;
    }

    ctrl->status_flags = status;
    ctrl->error_flags = error;
}

static void R2_Arm_SetRequestCurrent(R2_Arm_Ctrl_t *ctrl,
                                     RobotArmIKRequest_t *request)
{
    uint8_t i;

    if ((ctrl == NULL) || (request == NULL))
    {
        return;
    }

    request->current_direction = ctrl->current_direction;
    request->current_alpha_rad = ctrl->current_alpha_rad;
    for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
    {
        request->current_theta_rad[i] = ctrl->current_theta_rad[i];
    }
    if (request->posture_source_valid == 0U)
    {
        request->posture_source_tool = request->tool;
        request->posture_source_state = request->state;
    }
}

static void R2_Arm_CancelIndependentJointJog(R2_Arm_Ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->joint_jog_active = 0U;
    ctrl->joint_jog_index = 0U;
}

static void R2_Arm_CommitResultIfOk(R2_Arm_Ctrl_t *ctrl,
                                    RobotArmIKStatus_t ik_status)
{
    uint8_t i;

    if ((ctrl == NULL) || (ik_status != ROBOTARM_IK_OK))
    {
        return;
    }

    ctrl->current_direction = ctrl->result.target_direction;
    ctrl->current_alpha_rad = ctrl->result.alpha_rad;
    for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
    {
        ctrl->current_theta_rad[i] = ctrl->result.theta_rad[i];
    }
}

static void R2_Arm_UpdateMotorHardwareState(const R2_Arm_Ctrl_t *ctrl)
{
    uint8_t should_enable;
    uint32_t now_ms;

    if (ctrl == NULL)
    {
        return;
    }

    should_enable = ((ctrl->enabled != 0U) &&
                     (ctrl->output_enabled != 0U)) ? 1U : 0U;

    if ((should_enable != 0U) && (s_r2_arm_motor_hw_enabled == 0U))
    {
        RobotArm_Mixed_SetCommModes(ROBOTARM_COMM_POS_SPEED,
                                    ROBOTARM_COMM_POS_SPEED,
                                    ROBOTARM_COMM_POS_SPEED,
                                    0U);
        RobotArm_Mixed_Enable();
        s_r2_arm_motor_hw_enabled = 1U;
        s_r2_arm_motor_enable_retry_ms = HAL_GetTick();
    }
    else if ((should_enable != 0U) &&
             (s_r2_arm_motor_hw_enabled != 0U) &&
             (R2_Arm_DMFeedbackNeedsEnableRetry() != 0U))
    {
        now_ms = HAL_GetTick();
        if ((now_ms - s_r2_arm_motor_enable_retry_ms) >=
            R2_ARM_ENABLE_RETRY_MS)
        {
            RobotArm_Mixed_Enable();
            s_r2_arm_motor_enable_retry_ms = now_ms;
        }
    }
    else if ((should_enable == 0U) && (s_r2_arm_motor_hw_enabled != 0U))
    {
        RobotArm_Mixed_Disable();
        s_r2_arm_motor_hw_enabled = 0U;
        s_r2_arm_motor_enable_retry_ms = 0U;
    }
}

static uint8_t R2_Arm_DMFeedbackNeedsEnableRetry(void)
{
    RobotArm_FDCAN3Feedback_t feedback;

    RobotArm_Mixed_GetFeedbackSnapshot(&feedback);

    if ((feedback.dm8006_1.feedback_ok != 0U) &&
        (feedback.dm8006_1.state != R2_ARM_DM_RUNNING_STATE))
    {
        return 1U;
    }
    if ((feedback.dm8006_2.feedback_ok != 0U) &&
        (feedback.dm8006_2.state != R2_ARM_DM_RUNNING_STATE))
    {
        return 1U;
    }

    return 0U;
}

static void R2_Arm_ResolveLinkedToolPosture(R2_Arm_Ctrl_t *ctrl,
                                            RobotArmIKRequest_t *request)
{
    if ((ctrl == NULL) || (request == NULL))
    {
        return;
    }

    if ((request->tool == ROBOTARM_TOOL_S1) &&
        (request->state == ROBOTARM_TOOL_STATE_S1_CARRY_BY_S2))
    {
        request->posture_source_valid = 1U;
        request->posture_source_tool = ROBOTARM_TOOL_S2;
        request->posture_source_state = ctrl->shared_s2_state;
    }
}

static void R2_Arm_UpdateSharedToolState(R2_Arm_Ctrl_t *ctrl,
                                         RobotArmTool_t tool,
                                         RobotArmToolState_t state,
                                         RobotArmIKStatus_t ik_status)
{
    if ((ctrl == NULL) || (ik_status != ROBOTARM_IK_OK))
    {
        return;
    }

    if (tool == ROBOTARM_TOOL_S2)
    {
        ctrl->shared_s2_state = state;
    }
}

static void R2_Arm_FillLegacyTargetXY(RobotArmIKRequest_t *request,
                                      float approach_yaw_rad)
{
    float yaw;

    if (request == NULL)
    {
        return;
    }

    yaw = approach_yaw_rad;
    request->target_xyz_mm.x = -sinf(yaw);
    request->target_xyz_mm.y = cosf(yaw);
    request->target_xyz_mm.z = request->target_z_mm;
}

static float R2_Arm_DefaultTargetZ(RobotArmTool_t tool,
                                   RobotArmToolState_t state)
{
    return RobotArmKinematics_ToolTargetZFromAlpha(tool,
                                                   state,
                                                   R2_ARM_ENABLE_ALPHA_RAD);
}

static void R2_Arm_SetSuction2Output(uint8_t enabled)
{
    GPIO_PinState pin_state = (enabled != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    HAL_GPIO_WritePin(R2_ARM_S2_SUCTION_GPIO_PORT,
                      R2_ARM_S2_SUCTION_GPIO_PIN,
                      pin_state);
}

static void R2_Arm_SetGripperOutput(uint8_t enabled)
{
    R2_Arm_SetGripperAngleDeg((enabled != 0U) ?
                              R2_ARM_GRIPPER_OPEN_DEG :
                              R2_ARM_GRIPPER_CLOSE_DEG);
}

static void R2_Arm_SetGripperAngleDeg(float angle_deg)
{
    float clamped_angle = R2_Arm_ClampFloat(angle_deg, 0.0f, 180.0f);
    float pulse_us = R2_ARM_GRIPPER_PWM_MIN_US +
        (R2_ARM_GRIPPER_PWM_MAX_US - R2_ARM_GRIPPER_PWM_MIN_US) *
        (clamped_angle / 180.0f);
    uint32_t pulse_ticks =
        (uint32_t)((pulse_us * (R2_ARM_GRIPPER_TIMER_HZ / 1000000.0f)) + 0.5f);

    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_ticks);
}

static float R2_Arm_ClampJointLimitTolerance(const R2_Arm_Ctrl_t *ctrl,
                                             uint8_t index,
                                             float theta_rad)
{
    float min_rad;
    float max_rad;

    if ((ctrl == NULL) || (index >= ROBOTARM_KIN_JOINT_COUNT))
    {
        return theta_rad;
    }

    min_rad = ctrl->kinematics_config.joint_min_rad[index];
    max_rad = ctrl->kinematics_config.joint_max_rad[index];

    if ((theta_rad < min_rad) &&
        (theta_rad >= (min_rad - R2_ARM_JOINT_LIMIT_EPS_RAD)))
    {
        return min_rad;
    }
    if ((theta_rad > max_rad) &&
        (theta_rad <= (max_rad + R2_ARM_JOINT_LIMIT_EPS_RAD)))
    {
        return max_rad;
    }

    return theta_rad;
}

static float R2_Arm_ClampFloat(float x, float lo, float hi)
{
    if (x < lo)
    {
        return lo;
    }
    if (x > hi)
    {
        return hi;
    }
    return x;
}
