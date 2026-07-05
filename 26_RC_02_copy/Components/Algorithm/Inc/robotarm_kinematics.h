#ifndef __ROBOTARM_KINEMATICS_H
#define __ROBOTARM_KINEMATICS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOTARM_KIN_PI              3.14159265358979323846f
#define ROBOTARM_KIN_JOINT_COUNT     3U
#define ROBOTARM_KIN_TOOL_COUNT      3U

/* Corrected power-on zero model, all units are millimeters. */
#define ROBOTARM_KIN_D1_MM           98.18f
#define ROBOTARM_KIN_J2_Y_MM         42.64f
#define ROBOTARM_KIN_L23_MM          383.929445f
#define ROBOTARM_KIN_LONG_LINK_MM    270.98076211f
#define ROBOTARM_KIN_SHORT_LINK_MM   124.55f
#define ROBOTARM_KIN_BEND_RAD        (ROBOTARM_KIN_PI / 6.0f)
#define ROBOTARM_KIN_ALPHA_MIN_RAD   (-0.7853981633974483f)
#define ROBOTARM_KIN_ALPHA_MAX_RAD   (1.5707963267948966f)
#define ROBOTARM_KIN_YAW_SAFE_RAD    (1.2217304763960306f)

#ifndef ROBOTARM_KIN_J1_MIN_RAD
#define ROBOTARM_KIN_J1_MIN_RAD     (-ROBOTARM_KIN_PI)
#endif
#ifndef ROBOTARM_KIN_J1_MAX_RAD
#define ROBOTARM_KIN_J1_MAX_RAD      (ROBOTARM_KIN_PI)
#endif
#ifndef ROBOTARM_KIN_J2_MIN_RAD
#define ROBOTARM_KIN_J2_MIN_RAD     (-1.57079632679f)
#endif
#ifndef ROBOTARM_KIN_J2_MAX_RAD
#define ROBOTARM_KIN_J2_MAX_RAD      (1.57079632679f)
#endif
#ifndef ROBOTARM_KIN_J3_MIN_RAD
#define ROBOTARM_KIN_J3_MIN_RAD     (-ROBOTARM_KIN_PI)
#endif
#ifndef ROBOTARM_KIN_J3_MAX_RAD
#define ROBOTARM_KIN_J3_MAX_RAD      (ROBOTARM_KIN_PI)
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
    ROBOTARM_TOOL_STATE_STOW = 0,
    ROBOTARM_TOOL_STATE_USE = 1
} RobotArmToolState_t;

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
    ROBOTARM_IK_ERR_YAW_SWITCH_UNSAFE = 6,
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
    ROBOTARM_IK_REASON_LONG_LINK_LIMIT = 10,
    ROBOTARM_IK_REASON_YAW_SWITCH_UNSAFE = 11
} RobotArmIKReason_t;

typedef struct
{
    float x;
    float y;
    float z;
} RobotArmVec3_t;

typedef struct
{
    float joint_min_rad[ROBOTARM_KIN_JOINT_COUNT];
    float joint_max_rad[ROBOTARM_KIN_JOINT_COUNT];
    float preferred_theta2_sign;
    float alpha_min_rad;
    float alpha_max_rad;
    float yaw_switch_safe_alpha_rad;
    float xy_deadband_mm;
} RobotArmKinematicsConfig_t;

typedef struct
{
    RobotArmTool_t tool;
    RobotArmToolState_t state;
    RobotArmVec3_t target_xyz_mm;
    RobotArmWorkDirection_t current_direction;
    float current_theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    float current_alpha_rad;

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

typedef struct
{
    RobotArmIKStatus_t status;
    RobotArmIKReason_t reason;
    uint8_t checked_count;
    uint8_t failed_index;
    RobotArmIKResult_t last_result;
} RobotArmPathCheckResult_t;

void RobotArmKinematics_DefaultConfig(RobotArmKinematicsConfig_t *config);
float RobotArmKinematics_ModelZeroAlpha(void);
float RobotArmKinematics_DirectionYaw(RobotArmWorkDirection_t direction);
RobotArmWorkDirection_t RobotArmKinematics_SelectDirection(
    const RobotArmVec3_t *target_xyz_mm,
    RobotArmWorkDirection_t current_direction);
RobotArmIKStatus_t RobotArmKinematics_SolveIK(
    const RobotArmIKRequest_t *request,
    const RobotArmKinematicsConfig_t *config,
    RobotArmIKResult_t *result);
RobotArmIKStatus_t RobotArmKinematics_SolveToolHeight(
    const RobotArmIKRequest_t *request,
    const RobotArmKinematicsConfig_t *config,
    RobotArmIKResult_t *result);
RobotArmIKStatus_t RobotArmKinematics_Forward(
    RobotArmTool_t tool,
    const float theta_rad[ROBOTARM_KIN_JOINT_COUNT],
    RobotArmIKResult_t *result);
float RobotArmKinematics_GetToolPosturePhi(RobotArmTool_t tool,
                                           RobotArmToolState_t state);
const RobotArmVec3_t *RobotArmKinematics_GetToolOffset(RobotArmTool_t tool);
RobotArmIKStatus_t RobotArmKinematics_CheckPath(
    const RobotArmIKRequest_t *points,
    uint8_t point_count,
    const RobotArmKinematicsConfig_t *config,
    RobotArmPathCheckResult_t *path_result);

#ifdef __cplusplus
}
#endif

#endif
