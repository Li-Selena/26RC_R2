#include "R2_arm.h"

#include "include.h"

#include <stddef.h>

#define R2_ARM_DEFAULT_TOOL       ROBOTARM_TOOL_S1
#define R2_ARM_DEFAULT_STATE      ROBOTARM_TOOL_STATE_STOW
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

R2_Arm_Ctrl_t g_r2_arm_usb;

static void R2_Arm_UpdateFlags(R2_Arm_Ctrl_t *ctrl);
static void R2_Arm_SetRequestCurrent(R2_Arm_Ctrl_t *ctrl,
                                     RobotArmIKRequest_t *request);
static void R2_Arm_CommitResultIfOk(R2_Arm_Ctrl_t *ctrl,
                                    RobotArmIKStatus_t ik_status);
static void R2_Arm_FillLegacyTargetXY(RobotArmIKRequest_t *request,
                                      float approach_yaw_rad);
static float R2_Arm_DefaultTargetZ(RobotArmTool_t tool,
                                   RobotArmToolState_t state);

void R2_Arm_Init(R2_Arm_Ctrl_t *ctrl)
{
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
    RobotArmKinematics_DefaultConfig(&ctrl->kinematics_config);
    ctrl->current_direction = ROBOTARM_WORK_DIR_Y_POS;
    ctrl->current_alpha_rad = RobotArmKinematics_ModelZeroAlpha();
    ctrl->current_theta_rad[0] = 0.0f;
    ctrl->current_theta_rad[1] = 0.0f;
    ctrl->current_theta_rad[2] = 0.0f;

    ctrl->request.tool = R2_ARM_DEFAULT_TOOL;
    ctrl->request.state = R2_ARM_DEFAULT_STATE;
    ctrl->request.target_z_mm =
        R2_Arm_DefaultTargetZ(R2_ARM_DEFAULT_TOOL, R2_ARM_DEFAULT_STATE);
    ctrl->request.approach_yaw_rad = 0.0f;
    R2_Arm_FillLegacyTargetXY(&ctrl->request, ctrl->request.approach_yaw_rad);
    R2_Arm_SetRequestCurrent(ctrl, &ctrl->request);
    (void)RobotArmKinematics_SolveIK(&ctrl->request,
                                     &ctrl->kinematics_config,
                                     &ctrl->result);
    R2_Arm_CommitResultIfOk(ctrl, ctrl->result.status);
    R2_Arm_UpdateFlags(ctrl);
}

void R2_Arm_Enable(R2_Arm_Ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->enabled = 1U;
    ctrl->state = (ctrl->has_target != 0U) ?
        (uint8_t)R2_ARM_STATE_TARGET_VALID : (uint8_t)R2_ARM_STATE_READY;
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
    R2_Arm_UpdateFlags(ctrl);
}

RobotArmIKStatus_t R2_Arm_SetToolTarget(R2_Arm_Ctrl_t *ctrl,
                                        RobotArmTool_t tool,
                                        RobotArmToolState_t state,
                                        float target_z_mm,
                                        float approach_yaw_rad,
                                        uint32_t now_ms)
{
    RobotArmIKRequest_t request;
    RobotArmIKStatus_t ik_status;

    if (ctrl == NULL)
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    request.tool = tool;
    request.state = state;
    request.target_z_mm = target_z_mm;
    request.approach_yaw_rad = approach_yaw_rad;
    R2_Arm_FillLegacyTargetXY(&request, approach_yaw_rad);
    R2_Arm_SetRequestCurrent(ctrl, &request);

    ctrl->request = request;
    ctrl->has_target = 1U;
    ctrl->last_command_ms = now_ms;

    ik_status = RobotArmKinematics_SolveIK(&ctrl->request,
                                           &ctrl->kinematics_config,
                                           &ctrl->result);
    ctrl->last_ik_status = (uint8_t)ik_status;
    ctrl->last_ik_reason = (uint8_t)ctrl->result.reason;
    R2_Arm_CommitResultIfOk(ctrl, ik_status);
    ctrl->state = (ik_status == ROBOTARM_IK_OK) ?
        (uint8_t)R2_ARM_STATE_TARGET_VALID : (uint8_t)R2_ARM_STATE_ERROR;
    ctrl->output_enabled = ((ctrl->enabled != 0U) &&
                            (ik_status == ROBOTARM_IK_OK)) ? 1U : 0U;

    R2_Arm_UpdateFlags(ctrl);
    return ik_status;
}

RobotArmIKStatus_t R2_Arm_SetToolTargetXYZ(R2_Arm_Ctrl_t *ctrl,
                                           RobotArmTool_t tool,
                                           RobotArmToolState_t state,
                                           const RobotArmVec3_t *target_xyz_mm,
                                           uint32_t now_ms)
{
    RobotArmIKRequest_t request;
    RobotArmIKStatus_t ik_status;

    if ((ctrl == NULL) || (target_xyz_mm == NULL))
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    request.tool = tool;
    request.state = state;
    request.target_xyz_mm = *target_xyz_mm;
    request.target_z_mm = target_xyz_mm->z;
    request.approach_yaw_rad = 0.0f;
    R2_Arm_SetRequestCurrent(ctrl, &request);

    ctrl->request = request;
    ctrl->has_target = 1U;
    ctrl->last_command_ms = now_ms;

    ik_status = RobotArmKinematics_SolveIK(&ctrl->request,
                                           &ctrl->kinematics_config,
                                           &ctrl->result);
    ctrl->last_ik_status = (uint8_t)ik_status;
    ctrl->last_ik_reason = (uint8_t)ctrl->result.reason;
    R2_Arm_CommitResultIfOk(ctrl, ik_status);
    ctrl->state = (ik_status == ROBOTARM_IK_OK) ?
        (uint8_t)R2_ARM_STATE_TARGET_VALID : (uint8_t)R2_ARM_STATE_ERROR;
    ctrl->output_enabled = ((ctrl->enabled != 0U) &&
                            (ik_status == ROBOTARM_IK_OK)) ? 1U : 0U;

    R2_Arm_UpdateFlags(ctrl);
    return ik_status;
}

void R2_Arm_Update(R2_Arm_Ctrl_t *ctrl, uint32_t now_ms)
{
    RobotArmIKStatus_t ik_status;

    if (ctrl == NULL)
    {
        return;
    }

    ctrl->last_update_ms = now_ms;

    if ((ctrl->enabled == 0U) || (ctrl->has_target == 0U))
    {
        R2_Arm_UpdateFlags(ctrl);
        return;
    }

    R2_Arm_SetRequestCurrent(ctrl, &ctrl->request);
    ik_status = RobotArmKinematics_SolveIK(&ctrl->request,
                                           &ctrl->kinematics_config,
                                           &ctrl->result);
    ctrl->last_ik_status = (uint8_t)ik_status;
    ctrl->last_ik_reason = (uint8_t)ctrl->result.reason;

    if (ik_status == ROBOTARM_IK_OK)
    {
        R2_Arm_CommitResultIfOk(ctrl, ik_status);
        ctrl->state = (uint8_t)R2_ARM_STATE_TARGET_VALID;
        ctrl->output_enabled = 1U;
        R2_Arm_OutputApplyJointTargetsRad(&ctrl->result);
        ctrl->output_apply_count++;
    }
    else
    {
        ctrl->state = (uint8_t)R2_ARM_STATE_ERROR;
        ctrl->output_enabled = 0U;
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
    status->request = ctrl->request;
    status->result = ctrl->result;
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

__weak void R2_Arm_OutputApplyJointTargetsRad(const RobotArmIKResult_t *target)
{
    (void)target;
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
    if ((ctrl->last_ik_status == (uint8_t)ROBOTARM_IK_ERR_YAW_SWITCH_UNSAFE) ||
        (ctrl->last_ik_status == (uint8_t)ROBOTARM_IK_ERR_LONG_LINK_LIMIT))
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
    RobotArmIKRequest_t request;
    RobotArmIKResult_t result;
    RobotArmKinematicsConfig_t config;

    request.tool = tool;
    request.state = state;
    request.target_z_mm = 0.0f;
    request.approach_yaw_rad = 0.0f;
    RobotArmKinematics_DefaultConfig(&config);

    if (tool == ROBOTARM_TOOL_S1)
    {
        request.target_z_mm = 527.609445f;
    }
    else if (tool == ROBOTARM_TOOL_S2)
    {
        request.target_z_mm = 482.109445f;
    }
    else
    {
        request.target_z_mm = 492.409445f;
    }

    if (RobotArmKinematics_SolveToolHeight(&request, &config, &result) !=
        ROBOTARM_IK_OK)
    {
        return request.target_z_mm;
    }

    return result.tool_world_mm.z;
}
