#ifndef __R2_ARM_H
#define __R2_ARM_H

#include <stdint.h>
#include "robotarm_kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    R2_ARM_STATE_IDLE = 0,
    R2_ARM_STATE_READY = 1,
    R2_ARM_STATE_TARGET_VALID = 2,
    R2_ARM_STATE_STOPPED = 3,
    R2_ARM_STATE_ERROR = 4
} R2_ArmState_t;

#define R2_ARM_IK_TEST_DEFAULT_STEP_MS 8000U

typedef struct
{
    uint8_t enabled;
    uint8_t has_target;
    uint8_t output_enabled;
    uint8_t state;
    uint8_t status_flags;
    uint8_t error_flags;
    uint8_t last_ik_status;
    uint8_t last_ik_reason;
    uint32_t last_command_ms;
    uint32_t last_update_ms;
    uint32_t output_apply_count;
    uint8_t ik_test_active;
    uint8_t ik_test_step;
    uint8_t ik_test_done;
    uint8_t ik_test_error;
    uint32_t ik_test_step_start_ms;
    uint32_t ik_test_step_period_ms;
    uint8_t joint_jog_active;
    uint8_t joint_jog_index;
    uint8_t enable_startup_active;
    uint8_t enable_startup_phase;
    uint8_t workspace_switch_active;
    uint8_t workspace_switch_phase;
    RobotArmWorkDirection_t workspace_switch_target_direction;
    float workspace_switch_saved_theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    RobotArmWorkDirection_t current_direction;
    float current_alpha_rad;
    float current_theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    RobotArmToolState_t next_gripper_motion_state;
    RobotArmToolState_t shared_s2_state;
    RobotArmIKRequest_t request;
    RobotArmIKResult_t result;
    RobotArmKinematicsConfig_t kinematics_config;
} R2_Arm_Ctrl_t;

typedef struct
{
    uint8_t enabled;
    uint8_t has_target;
    uint8_t output_enabled;
    uint8_t state;
    uint8_t status_flags;
    uint8_t error_flags;
    uint8_t last_ik_status;
    uint8_t last_ik_reason;
    uint32_t last_command_ms;
    uint32_t last_update_ms;
    uint32_t output_apply_count;
    uint8_t ik_test_active;
    uint8_t ik_test_step;
    uint8_t ik_test_done;
    uint8_t ik_test_error;
    uint32_t ik_test_step_start_ms;
    uint32_t ik_test_step_period_ms;
    RobotArmIKRequest_t request;
    RobotArmIKResult_t result;
    float motor_feedback_rad[ROBOTARM_KIN_JOINT_COUNT];
    uint8_t motor_feedback_ok_mask;
} R2_ArmStatus_t;

extern R2_Arm_Ctrl_t g_r2_arm_usb;

void R2_Arm_Init(R2_Arm_Ctrl_t *ctrl);
void R2_Arm_Enable(R2_Arm_Ctrl_t *ctrl);
void R2_Arm_Disable(R2_Arm_Ctrl_t *ctrl);
void R2_Arm_Stop(R2_Arm_Ctrl_t *ctrl);
RobotArmIKStatus_t R2_Arm_SetToolTarget(R2_Arm_Ctrl_t *ctrl,
                                        RobotArmTool_t tool,
                                        RobotArmToolState_t state,
                                        float target_z_mm,
                                        float approach_yaw_rad,
                                        uint32_t now_ms);
RobotArmIKStatus_t R2_Arm_SetToolTargetXYZ(R2_Arm_Ctrl_t *ctrl,
                                           RobotArmTool_t tool,
                                           RobotArmToolState_t state,
                                           const RobotArmVec3_t *target_xyz_mm,
                                           uint32_t now_ms);
RobotArmIKStatus_t R2_Arm_SetWorkDirection(R2_Arm_Ctrl_t *ctrl,
                                           RobotArmWorkDirection_t direction,
                                           uint32_t now_ms);
RobotArmIKStatus_t R2_Arm_JogToolHeight(R2_Arm_Ctrl_t *ctrl,
                                        float delta_z_mm,
                                        uint32_t now_ms);
RobotArmIKStatus_t R2_Arm_MoveToolHeightLimit(R2_Arm_Ctrl_t *ctrl,
                                              uint8_t use_max,
                                              uint32_t now_ms);
RobotArmIKStatus_t R2_Arm_SetToolPosture(R2_Arm_Ctrl_t *ctrl,
                                         RobotArmTool_t tool,
                                         RobotArmToolState_t state,
                                         uint32_t now_ms);
RobotArmIKStatus_t R2_Arm_JogJointActual(R2_Arm_Ctrl_t *ctrl,
                                         uint8_t joint,
                                         float delta_deg,
                                         uint32_t now_ms);
void R2_Arm_StartIKTestFlow(R2_Arm_Ctrl_t *ctrl,
                            uint32_t now_ms,
                            uint32_t step_period_ms);
void R2_Arm_StopIKTestFlow(R2_Arm_Ctrl_t *ctrl);
void R2_Arm_Update(R2_Arm_Ctrl_t *ctrl, uint32_t now_ms);
void R2_Arm_GetStatus(const R2_Arm_Ctrl_t *ctrl, R2_ArmStatus_t *status);
uint8_t R2_Arm_IsMotorActive(const R2_Arm_Ctrl_t *ctrl);
uint8_t R2_Arm_IsJointTargetReached(const R2_Arm_Ctrl_t *ctrl,
                                    uint8_t joint,
                                    float tolerance_deg);
void R2_Arm_SetToolActuator(RobotArmTool_t tool, uint8_t enabled);

void R2_Arm_OutputApplyJointTargetsRad(const RobotArmIKResult_t *target);

#ifdef __cplusplus
}
#endif

#endif
