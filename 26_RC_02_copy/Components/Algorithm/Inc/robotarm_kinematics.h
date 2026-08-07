#ifndef __ROBOTARM_KINEMATICS_H
#define __ROBOTARM_KINEMATICS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOTARM_KIN_PI              3.14159265358979323846f
#define ROBOTARM_KIN_JOINT_COUNT     3U
#define ROBOTARM_KIN_TOOL_COUNT      3U
#define ROBOTARM_KIN_TOOL_STATE_COUNT 5U

/* Corrected power-on zero model, all units are millimeters. */
#define ROBOTARM_KIN_D1_MM           98.18f
#define ROBOTARM_KIN_J2_Y_MM         42.64f
#define ROBOTARM_KIN_L23_MM          442.26981444f
#define ROBOTARM_KIN_LONG_LINK_MM    330.0f
#define ROBOTARM_KIN_SHORT_LINK_MM   124.55f
#define ROBOTARM_KIN_BEND_RAD        (ROBOTARM_KIN_PI / 6.0f)
#define ROBOTARM_KIN_FRAME4_POWERON_PHI_RAD ROBOTARM_KIN_PI
#define ROBOTARM_KIN_ALPHA_Y_POS_MIN_RAD (-0.8726646259971648f)
#define ROBOTARM_KIN_ALPHA_X_MIN_RAD     (-0.5235987755982989f)
#define ROBOTARM_KIN_ALPHA_MAX_RAD       (0.5f * ROBOTARM_KIN_PI)
#define ROBOTARM_KIN_ALPHA_MIN_RAD       ROBOTARM_KIN_ALPHA_Y_POS_MIN_RAD

#ifndef ROBOTARM_KIN_J1_MIN_RAD
#define ROBOTARM_KIN_J1_MIN_RAD     (-0.5f * ROBOTARM_KIN_PI)
#endif
#ifndef ROBOTARM_KIN_J1_MAX_RAD
#define ROBOTARM_KIN_J1_MAX_RAD      (0.5f * ROBOTARM_KIN_PI)
#endif
#ifndef ROBOTARM_KIN_J2_MIN_RAD
#define ROBOTARM_KIN_J2_MIN_RAD     (-ROBOTARM_KIN_PI)
#endif
#ifndef ROBOTARM_KIN_J2_MAX_RAD
#define ROBOTARM_KIN_J2_MAX_RAD      (ROBOTARM_KIN_PI)
#endif
#ifndef ROBOTARM_KIN_J3_MIN_RAD
#define ROBOTARM_KIN_J3_MIN_RAD     (-2.0f * ROBOTARM_KIN_PI)
#endif
#ifndef ROBOTARM_KIN_J3_MAX_RAD
#define ROBOTARM_KIN_J3_MAX_RAD      (2.0f * ROBOTARM_KIN_PI)
#endif
#ifndef ROBOTARM_KIN_PREFERRED_THETA2_SIGN
#define ROBOTARM_KIN_PREFERRED_THETA2_SIGN (-1.0f)
#endif

typedef enum
{
    ROBOTARM_TOOL_S1 = 0,
    ROBOTARM_TOOL_S2 = 1,
    ROBOTARM_TOOL_GRIPPER = 2
} RobotArmTool_t;

typedef enum
{
    ROBOTARM_TOOL_STATE_0 = 0,
    ROBOTARM_TOOL_STATE_1 = 1,
    ROBOTARM_TOOL_STATE_2 = 2,
    ROBOTARM_TOOL_STATE_3 = 3,
    ROBOTARM_TOOL_STATE_4 = 4
} RobotArmToolState_t;

#define ROBOTARM_TOOL_STATE_S1_PREPARE       ROBOTARM_TOOL_STATE_0
#define ROBOTARM_TOOL_STATE_S1_CARRY_BY_S2   ROBOTARM_TOOL_STATE_1
#define ROBOTARM_TOOL_STATE_S1_PLACE         ROBOTARM_TOOL_STATE_2

#define ROBOTARM_TOOL_STATE_S2_PREPARE       ROBOTARM_TOOL_STATE_0
#define ROBOTARM_TOOL_STATE_S2_SUCTION_DOWN  ROBOTARM_TOOL_STATE_1
#define ROBOTARM_TOOL_STATE_S2_SHORT_PARALLEL ROBOTARM_TOOL_STATE_2
#define ROBOTARM_TOOL_STATE_S2_PLACE_Y_POS   ROBOTARM_TOOL_STATE_3

#define ROBOTARM_TOOL_STATE_GRIPPER_UP       ROBOTARM_TOOL_STATE_0
#define ROBOTARM_TOOL_STATE_GRIPPER_DOWN     ROBOTARM_TOOL_STATE_1
#define ROBOTARM_TOOL_STATE_GRIPPER_FORWARD  ROBOTARM_TOOL_STATE_2

/* Legacy aliases kept for callers while state meaning is now tool-local. */
#define ROBOTARM_TOOL_STATE_STOW             ROBOTARM_TOOL_STATE_0
#define ROBOTARM_TOOL_STATE_USE              ROBOTARM_TOOL_STATE_1

typedef enum
{
    ROBOTARM_WORK_DIR_Y_POS = 0,
    ROBOTARM_WORK_DIR_X_POS = 1,
    ROBOTARM_WORK_DIR_X_NEG = 2
} RobotArmWorkDirection_t;

typedef enum
{
    ROBOTARM_IK_OK = 0,
    ROBOTARM_IK_ERR_NULL = 1,
    ROBOTARM_IK_ERR_BAD_PARAM = 2,
    ROBOTARM_IK_ERR_HEIGHT_UNREACHABLE = 3,
    ROBOTARM_IK_ERR_JOINT_LIMIT = 4,
    ROBOTARM_IK_ERR_LONG_LINK_LIMIT = 5,
    ROBOTARM_IK_ERR_UNSUPPORTED_STATE = 7,
    ROBOTARM_IK_ERR_BAD_DIRECTION = 8
} RobotArmIKStatus_t;

typedef enum
{
    ROBOTARM_IK_REASON_NONE = 0,
    ROBOTARM_IK_REASON_BAD_TOOL = 1,
    ROBOTARM_IK_REASON_BAD_STATE = 2,
    ROBOTARM_IK_REASON_BAD_FLOAT = 3,
    ROBOTARM_IK_REASON_HEIGHT_OUTSIDE_LINK = 4,
    ROBOTARM_IK_REASON_J1_LIMIT = 5,
    ROBOTARM_IK_REASON_J2_LIMIT = 6,
    ROBOTARM_IK_REASON_J3_LIMIT = 7,
    ROBOTARM_IK_REASON_BAD_DIRECTION = 8,
    ROBOTARM_IK_REASON_UNSUPPORTED_STATE = 9,
    ROBOTARM_IK_REASON_LONG_LINK_LIMIT = 10
} RobotArmIKReason_t;

typedef struct
{
    float x;
    float y;
    float z;
} RobotArmVec3_t;

typedef struct
{
    float min_mm;
    float max_mm;
} RobotArmZLimit_t;

typedef struct
{
    float joint_min_rad[ROBOTARM_KIN_JOINT_COUNT];
    float joint_max_rad[ROBOTARM_KIN_JOINT_COUNT];
    float preferred_theta2_sign;
    float alpha_min_rad;
    float alpha_max_rad;
    float xy_deadband_mm;
} RobotArmKinematicsConfig_t;

typedef struct
{
    RobotArmTool_t tool;
    RobotArmToolState_t state;
    RobotArmVec3_t target_xyz_mm;
    RobotArmWorkDirection_t current_direction;
    uint8_t target_direction_valid;
    RobotArmWorkDirection_t target_direction;
    float current_theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    float current_alpha_rad;
    uint8_t posture_source_valid;
    RobotArmTool_t posture_source_tool;
    RobotArmToolState_t posture_source_state;

    /* Kept for the existing 4-float USB command and status packets. */
    float target_z_mm;
    float approach_yaw_rad;
} RobotArmIKRequest_t;

typedef struct
{
    RobotArmIKStatus_t status;
    RobotArmIKReason_t reason;
    RobotArmTool_t tool;
    RobotArmToolState_t state;
    RobotArmWorkDirection_t target_direction;
    float alpha_rad;
    float target_z_mm;
    float approach_yaw_rad;
    float theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    RobotArmVec3_t j2_world_mm;
    RobotArmVec3_t j3_world_mm;
    RobotArmVec3_t j4_world_mm;
    RobotArmVec3_t tool_world_mm;
    RobotArmVec3_t tool_x_axis;
    RobotArmVec3_t tool_y_axis;
    RobotArmVec3_t tool_z_axis;
} RobotArmIKResult_t;

void RobotArmKinematics_DefaultConfig(RobotArmKinematicsConfig_t *config);
float RobotArmKinematics_ModelZeroAlpha(void);
float RobotArmKinematics_DirectionYaw(RobotArmWorkDirection_t direction);
float RobotArmKinematics_ToolTargetZFromAlpha(RobotArmTool_t tool,
                                              RobotArmToolState_t state,
                                              float alpha_rad);
float RobotArmKinematics_RequestToolTargetZFromAlpha(
    const RobotArmIKRequest_t *request,
    float alpha_rad);
uint8_t RobotArmKinematics_GetToolZLimit(
    RobotArmTool_t tool,
    RobotArmToolState_t state,
    const RobotArmKinematicsConfig_t *config,
    RobotArmZLimit_t *limit);
uint8_t RobotArmKinematics_GetRequestToolZLimit(
    const RobotArmIKRequest_t *request,
    const RobotArmKinematicsConfig_t *config,
    RobotArmZLimit_t *limit);
RobotArmWorkDirection_t RobotArmKinematics_SelectDirection(
    const RobotArmVec3_t *target_xyz_mm,
    RobotArmWorkDirection_t current_direction);
RobotArmIKStatus_t RobotArmKinematics_Forward(
    RobotArmTool_t tool,
    const float theta_rad[ROBOTARM_KIN_JOINT_COUNT],
    RobotArmIKResult_t *result);
RobotArmIKStatus_t RobotArmKinematics_ForwardState(
    RobotArmTool_t tool,
    RobotArmToolState_t state,
    const float theta_rad[ROBOTARM_KIN_JOINT_COUNT],
    RobotArmIKResult_t *result);
float RobotArmKinematics_GetToolPosturePhi(RobotArmTool_t tool,
                                           RobotArmToolState_t state);
const RobotArmVec3_t *RobotArmKinematics_GetToolOffset(RobotArmTool_t tool);

#ifdef __cplusplus
}
#endif

#endif
