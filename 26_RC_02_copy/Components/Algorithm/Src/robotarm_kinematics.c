#include "robotarm_kinematics.h"

#include <math.h>
#include <stddef.h>

#define ROBOTARM_KIN_EPS                    1.0e-5f
#define ROBOTARM_KIN_HEIGHT_TOL             1.0e-4f
#define ROBOTARM_KIN_SCORE_BAD              1.0e30f
#define ROBOTARM_KIN_DEFAULT_XY_DEADBAND_MM 1.0f
#define ROBOTARM_KIN_PATH_FAILED_NONE       0xFFU

typedef struct
{
    RobotArmVec3_t offset_mm;
    float phi_rad;
    uint8_t ik_enabled;
} RobotArmToolGeometry_t;

static const RobotArmToolGeometry_t s_tool_geometry[ROBOTARM_KIN_TOOL_COUNT][2] =
{
    {
        {{0.0f, -46.79f, 45.5f}, 0.0f, 1U},
        {{0.0f, -46.79f, 45.5f}, ROBOTARM_KIN_PI, 1U}
    },
    {
        {{0.0f, 92.6f, 0.0f}, 0.0f, 1U},
        {{0.0f, 92.6f, 0.0f}, -0.5f * ROBOTARM_KIN_PI, 1U}
    },
    {
        {{103.0f, -92.0f, 10.3f}, 0.0f, 1U},
        {{103.0f, -92.0f, 10.3f}, ROBOTARM_KIN_PI, 1U}
    }
};

static uint8_t IsFiniteFloat(float x);
static uint8_t IsToolValid(RobotArmTool_t tool);
static uint8_t IsStateIndexValid(RobotArmToolState_t state);
static uint8_t IsDirectionValid(RobotArmWorkDirection_t direction);
static uint8_t IsToolStateSupported(RobotArmTool_t tool,
                                    RobotArmToolState_t state);
static float ClampF(float x, float lo, float hi);
static float NormalizePi(float x);
static uint8_t InLimit(float x, float lo, float hi);
static float AbsF(float x);
static float LongToChordOffsetRad(void);
static RobotArmWorkDirection_t SelectDirectionWithDeadband(
    const RobotArmVec3_t *target_xyz_mm,
    RobotArmWorkDirection_t current_direction,
    float xy_deadband_mm);
static RobotArmWorkDirection_t DirectionFromLegacyYaw(float yaw_rad);
static RobotArmVec3_t Vec3(float x, float y, float z);
static RobotArmVec3_t Add3(RobotArmVec3_t a, RobotArmVec3_t b);
static RobotArmVec3_t Scale3(RobotArmVec3_t a, float s);
static void InitResult(const RobotArmIKRequest_t *request,
                       RobotArmIKResult_t *result);
static void FillForward(RobotArmTool_t tool,
                        RobotArmToolState_t state,
                        float target_z_mm,
                        const float theta_rad[ROBOTARM_KIN_JOINT_COUNT],
                        RobotArmIKResult_t *result);

void RobotArmKinematics_DefaultConfig(RobotArmKinematicsConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->joint_min_rad[0] = -0.5f * ROBOTARM_KIN_PI;
    config->joint_max_rad[0] = 0.5f * ROBOTARM_KIN_PI;
    config->joint_min_rad[1] = ROBOTARM_KIN_J2_MIN_RAD;
    config->joint_max_rad[1] = ROBOTARM_KIN_J2_MAX_RAD;
    config->joint_min_rad[2] = ROBOTARM_KIN_J3_MIN_RAD;
    config->joint_max_rad[2] = ROBOTARM_KIN_J3_MAX_RAD;
    config->preferred_theta2_sign = ROBOTARM_KIN_PREFERRED_THETA2_SIGN;
    config->alpha_min_rad = ROBOTARM_KIN_ALPHA_MIN_RAD;
    config->alpha_max_rad = ROBOTARM_KIN_ALPHA_MAX_RAD;
    config->yaw_switch_safe_alpha_rad = ROBOTARM_KIN_YAW_SAFE_RAD;
    config->xy_deadband_mm = ROBOTARM_KIN_DEFAULT_XY_DEADBAND_MM;
}

float RobotArmKinematics_ModelZeroAlpha(void)
{
    return (0.5f * ROBOTARM_KIN_PI) - LongToChordOffsetRad();
}

float RobotArmKinematics_DirectionYaw(RobotArmWorkDirection_t direction)
{
    switch (direction)
    {
    case ROBOTARM_WORK_DIR_Y_POS:
        return 0.0f;
    case ROBOTARM_WORK_DIR_X_POS:
        return -0.5f * ROBOTARM_KIN_PI;
    case ROBOTARM_WORK_DIR_X_NEG:
        return 0.5f * ROBOTARM_KIN_PI;
    default:
        return 0.0f;
    }
}

RobotArmWorkDirection_t RobotArmKinematics_SelectDirection(
    const RobotArmVec3_t *target_xyz_mm,
    RobotArmWorkDirection_t current_direction)
{
    return SelectDirectionWithDeadband(target_xyz_mm,
                                       current_direction,
                                       ROBOTARM_KIN_DEFAULT_XY_DEADBAND_MM);
}

RobotArmIKStatus_t RobotArmKinematics_SolveIK(
    const RobotArmIKRequest_t *request,
    const RobotArmKinematicsConfig_t *config,
    RobotArmIKResult_t *result)
{
    RobotArmKinematicsConfig_t local_config;
    const RobotArmKinematicsConfig_t *cfg;
    const RobotArmToolGeometry_t *tool_geometry;
    RobotArmWorkDirection_t target_direction;
    float phi;
    float sin_phi;
    float cos_phi;
    float required_j4_z_mm;
    float sin_gamma;
    float gamma_base;
    float gamma_candidates[2];
    float theta_candidate[ROBOTARM_KIN_JOINT_COUNT];
    float best_theta[ROBOTARM_KIN_JOINT_COUNT] = {0.0f, 0.0f, 0.0f};
    float best_alpha = 0.0f;
    float best_score = ROBOTARM_KIN_SCORE_BAD;
    RobotArmIKStatus_t reject_status = ROBOTARM_IK_ERR_LONG_LINK_LIMIT;
    RobotArmIKReason_t reject_reason = ROBOTARM_IK_REASON_LONG_LINK_LIMIT;
    float long_to_chord_rad;
    uint8_t i;

    if ((request == NULL) || (result == NULL))
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    InitResult(request, result);

    if (!IsToolValid(request->tool))
    {
        result->status = ROBOTARM_IK_ERR_BAD_PARAM;
        result->reason = ROBOTARM_IK_REASON_BAD_TOOL;
        return result->status;
    }

    if (!IsStateIndexValid(request->state) ||
        !IsToolStateSupported(request->tool, request->state))
    {
        result->status = ROBOTARM_IK_ERR_UNSUPPORTED_STATE;
        result->reason = ROBOTARM_IK_REASON_UNSUPPORTED_STATE;
        return result->status;
    }

    if (!IsDirectionValid(request->current_direction))
    {
        result->status = ROBOTARM_IK_ERR_BAD_DIRECTION;
        result->reason = ROBOTARM_IK_REASON_BAD_DIRECTION;
        return result->status;
    }

    if ((!IsFiniteFloat(request->target_xyz_mm.x)) ||
        (!IsFiniteFloat(request->target_xyz_mm.y)) ||
        (!IsFiniteFloat(request->target_xyz_mm.z)) ||
        (!IsFiniteFloat(request->current_alpha_rad)))
    {
        result->status = ROBOTARM_IK_ERR_BAD_PARAM;
        result->reason = ROBOTARM_IK_REASON_BAD_FLOAT;
        return result->status;
    }

    RobotArmKinematics_DefaultConfig(&local_config);
    cfg = (config != NULL) ? config : &local_config;

    target_direction = SelectDirectionWithDeadband(&request->target_xyz_mm,
                                                  request->current_direction,
                                                  cfg->xy_deadband_mm);
    if (!IsDirectionValid(target_direction))
    {
        result->status = ROBOTARM_IK_ERR_BAD_DIRECTION;
        result->reason = ROBOTARM_IK_REASON_BAD_DIRECTION;
        return result->status;
    }

    result->target_direction = target_direction;
    result->approach_yaw_rad = RobotArmKinematics_DirectionYaw(target_direction);
    theta_candidate[0] = result->approach_yaw_rad;

    if (!InLimit(theta_candidate[0], cfg->joint_min_rad[0], cfg->joint_max_rad[0]))
    {
        result->status = ROBOTARM_IK_ERR_JOINT_LIMIT;
        result->reason = ROBOTARM_IK_REASON_J1_LIMIT;
        return result->status;
    }

    if ((target_direction != request->current_direction) &&
        (request->current_alpha_rad <= (cfg->yaw_switch_safe_alpha_rad + ROBOTARM_KIN_EPS)))
    {
        result->status = ROBOTARM_IK_ERR_YAW_SWITCH_UNSAFE;
        result->reason = ROBOTARM_IK_REASON_YAW_SWITCH_UNSAFE;
        return result->status;
    }

    tool_geometry = &s_tool_geometry[(uint8_t)request->tool][(uint8_t)request->state];
    phi = tool_geometry->phi_rad;
    sin_phi = sinf(phi);
    cos_phi = cosf(phi);

    required_j4_z_mm = request->target_xyz_mm.z -
                       ROBOTARM_KIN_D1_MM -
                       tool_geometry->offset_mm.y * sin_phi -
                       tool_geometry->offset_mm.z * cos_phi;
    sin_gamma = required_j4_z_mm / ROBOTARM_KIN_L23_MM;

    if ((sin_gamma < (-1.0f - ROBOTARM_KIN_HEIGHT_TOL)) ||
        (sin_gamma > (1.0f + ROBOTARM_KIN_HEIGHT_TOL)))
    {
        result->status = ROBOTARM_IK_ERR_HEIGHT_UNREACHABLE;
        result->reason = ROBOTARM_IK_REASON_HEIGHT_OUTSIDE_LINK;
        return result->status;
    }

    sin_gamma = ClampF(sin_gamma, -1.0f, 1.0f);
    gamma_base = asinf(sin_gamma);
    gamma_candidates[0] = gamma_base;
    gamma_candidates[1] = ROBOTARM_KIN_PI - gamma_base;
    long_to_chord_rad = LongToChordOffsetRad();

    for (i = 0U; i < 2U; i++)
    {
        float gamma = gamma_candidates[i];
        float alpha = gamma - long_to_chord_rad;
        float theta2;
        float theta3;
        float score;
        float preferred_penalty = 0.0f;

        theta2 = NormalizePi(gamma - (0.5f * ROBOTARM_KIN_PI));
        theta3 = NormalizePi(phi - theta2);

        if (!InLimit(alpha, cfg->alpha_min_rad, cfg->alpha_max_rad))
        {
            reject_status = ROBOTARM_IK_ERR_LONG_LINK_LIMIT;
            reject_reason = ROBOTARM_IK_REASON_LONG_LINK_LIMIT;
            continue;
        }
        if (!InLimit(theta2, cfg->joint_min_rad[1], cfg->joint_max_rad[1]))
        {
            reject_status = ROBOTARM_IK_ERR_JOINT_LIMIT;
            reject_reason = ROBOTARM_IK_REASON_J2_LIMIT;
            continue;
        }
        if (!InLimit(theta3, cfg->joint_min_rad[2], cfg->joint_max_rad[2]))
        {
            reject_status = ROBOTARM_IK_ERR_JOINT_LIMIT;
            reject_reason = ROBOTARM_IK_REASON_J3_LIMIT;
            continue;
        }

        if ((cfg->preferred_theta2_sign < 0.0f) && (theta2 > ROBOTARM_KIN_EPS))
        {
            preferred_penalty = 0.01f;
        }
        else if ((cfg->preferred_theta2_sign > 0.0f) && (theta2 < -ROBOTARM_KIN_EPS))
        {
            preferred_penalty = 0.01f;
        }

        score = AbsF(alpha - request->current_alpha_rad) +
                0.25f * AbsF(theta3) +
                preferred_penalty;

        if (score < best_score)
        {
            best_score = score;
            best_alpha = alpha;
            best_theta[0] = theta_candidate[0];
            best_theta[1] = theta2;
            best_theta[2] = theta3;
        }
    }

    if (best_score >= (ROBOTARM_KIN_SCORE_BAD * 0.5f))
    {
        result->status = reject_status;
        result->reason = reject_reason;
        return result->status;
    }

    FillForward(request->tool, request->state, request->target_xyz_mm.z,
                best_theta, result);
    result->status = ROBOTARM_IK_OK;
    result->reason = ROBOTARM_IK_REASON_NONE;
    result->target_direction = target_direction;
    result->alpha_rad = best_alpha;
    result->target_z_mm = request->target_xyz_mm.z;
    result->approach_yaw_rad = RobotArmKinematics_DirectionYaw(target_direction);
    return result->status;
}

RobotArmIKStatus_t RobotArmKinematics_SolveToolHeight(
    const RobotArmIKRequest_t *request,
    const RobotArmKinematicsConfig_t *config,
    RobotArmIKResult_t *result)
{
    RobotArmIKRequest_t ik_request;
    RobotArmWorkDirection_t direction;
    float yaw;

    if ((request == NULL) || (result == NULL))
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    InitResult(request, result);
    if ((!IsFiniteFloat(request->target_z_mm)) ||
        (!IsFiniteFloat(request->approach_yaw_rad)))
    {
        result->status = ROBOTARM_IK_ERR_BAD_PARAM;
        result->reason = ROBOTARM_IK_REASON_BAD_FLOAT;
        return result->status;
    }

    direction = DirectionFromLegacyYaw(request->approach_yaw_rad);
    yaw = RobotArmKinematics_DirectionYaw(direction);

    ik_request = *request;
    ik_request.current_direction = direction;
    ik_request.current_alpha_rad = RobotArmKinematics_ModelZeroAlpha();
    ik_request.current_theta_rad[0] = yaw;
    ik_request.current_theta_rad[1] = 0.0f;
    ik_request.current_theta_rad[2] = 0.0f;
    ik_request.target_xyz_mm.x = 0.0f;
    ik_request.target_xyz_mm.y = 0.0f;
    ik_request.target_xyz_mm.z = request->target_z_mm;
    if (direction == ROBOTARM_WORK_DIR_Y_POS)
    {
        ik_request.target_xyz_mm.y = 1.0f;
    }
    else if (direction == ROBOTARM_WORK_DIR_X_POS)
    {
        ik_request.target_xyz_mm.x = 1.0f;
    }
    else
    {
        ik_request.target_xyz_mm.x = -1.0f;
    }
    ik_request.approach_yaw_rad = yaw;

    return RobotArmKinematics_SolveIK(&ik_request, config, result);
}

RobotArmIKStatus_t RobotArmKinematics_Forward(
    RobotArmTool_t tool,
    const float theta_rad[ROBOTARM_KIN_JOINT_COUNT],
    RobotArmIKResult_t *result)
{
    uint8_t i;

    if ((theta_rad == NULL) || (result == NULL))
    {
        return ROBOTARM_IK_ERR_NULL;
    }
    if (!IsToolValid(tool))
    {
        result->status = ROBOTARM_IK_ERR_BAD_PARAM;
        result->reason = ROBOTARM_IK_REASON_BAD_TOOL;
        return result->status;
    }
    for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
    {
        if (!IsFiniteFloat(theta_rad[i]))
        {
            result->status = ROBOTARM_IK_ERR_BAD_PARAM;
            result->reason = ROBOTARM_IK_REASON_BAD_FLOAT;
            return result->status;
        }
    }

    FillForward(tool, ROBOTARM_TOOL_STATE_STOW, 0.0f, theta_rad, result);
    result->target_z_mm = result->tool_world_mm.z;
    result->status = ROBOTARM_IK_OK;
    result->reason = ROBOTARM_IK_REASON_NONE;
    return result->status;
}

float RobotArmKinematics_GetToolPosturePhi(RobotArmTool_t tool,
                                           RobotArmToolState_t state)
{
    if (!IsToolStateSupported(tool, state))
    {
        return 0.0f;
    }
    return s_tool_geometry[(uint8_t)tool][(uint8_t)state].phi_rad;
}

const RobotArmVec3_t *RobotArmKinematics_GetToolOffset(RobotArmTool_t tool)
{
    if (!IsToolValid(tool))
    {
        return NULL;
    }
    return &s_tool_geometry[(uint8_t)tool][(uint8_t)ROBOTARM_TOOL_STATE_STOW].offset_mm;
}

RobotArmIKStatus_t RobotArmKinematics_CheckPath(
    const RobotArmIKRequest_t *points,
    uint8_t point_count,
    const RobotArmKinematicsConfig_t *config,
    RobotArmPathCheckResult_t *path_result)
{
    RobotArmIKRequest_t point_request;
    RobotArmIKResult_t point_result;
    RobotArmIKStatus_t status;
    RobotArmWorkDirection_t current_direction;
    float current_alpha_rad;
    float current_theta_rad[ROBOTARM_KIN_JOINT_COUNT];
    uint8_t i;
    uint8_t j;

    if ((points == NULL) || (path_result == NULL))
    {
        return ROBOTARM_IK_ERR_NULL;
    }

    path_result->status = ROBOTARM_IK_OK;
    path_result->reason = ROBOTARM_IK_REASON_NONE;
    path_result->checked_count = 0U;
    path_result->failed_index = ROBOTARM_KIN_PATH_FAILED_NONE;

    if (point_count == 0U)
    {
        path_result->status = ROBOTARM_IK_ERR_BAD_PARAM;
        path_result->reason = ROBOTARM_IK_REASON_BAD_FLOAT;
        return path_result->status;
    }

    current_direction = points[0].current_direction;
    current_alpha_rad = points[0].current_alpha_rad;
    for (j = 0U; j < ROBOTARM_KIN_JOINT_COUNT; j++)
    {
        current_theta_rad[j] = points[0].current_theta_rad[j];
    }

    for (i = 0U; i < point_count; i++)
    {
        point_request = points[i];
        point_request.current_direction = current_direction;
        point_request.current_alpha_rad = current_alpha_rad;
        for (j = 0U; j < ROBOTARM_KIN_JOINT_COUNT; j++)
        {
            point_request.current_theta_rad[j] = current_theta_rad[j];
        }

        status = RobotArmKinematics_SolveIK(&point_request, config, &point_result);
        path_result->checked_count = (uint8_t)(i + 1U);
        path_result->last_result = point_result;
        if (status != ROBOTARM_IK_OK)
        {
            path_result->status = status;
            path_result->reason = point_result.reason;
            path_result->failed_index = i;
            return status;
        }

        current_direction = point_result.target_direction;
        current_alpha_rad = point_result.alpha_rad;
        for (j = 0U; j < ROBOTARM_KIN_JOINT_COUNT; j++)
        {
            current_theta_rad[j] = point_result.theta_rad[j];
        }
    }

    return ROBOTARM_IK_OK;
}

static void InitResult(const RobotArmIKRequest_t *request,
                       RobotArmIKResult_t *result)
{
    uint8_t i;

    result->status = ROBOTARM_IK_ERR_BAD_PARAM;
    result->reason = ROBOTARM_IK_REASON_NONE;
    result->tool = request->tool;
    result->state = request->state;
    result->target_direction = request->current_direction;
    result->alpha_rad = request->current_alpha_rad;
    result->target_z_mm = request->target_z_mm;
    result->approach_yaw_rad = request->approach_yaw_rad;
    for (i = 0U; i < ROBOTARM_KIN_JOINT_COUNT; i++)
    {
        result->theta_rad[i] = request->current_theta_rad[i];
    }
    result->j2_world_mm = Vec3(0.0f, 0.0f, 0.0f);
    result->j3_world_mm = Vec3(0.0f, 0.0f, 0.0f);
    result->j4_world_mm = Vec3(0.0f, 0.0f, 0.0f);
    result->tool_world_mm = Vec3(0.0f, 0.0f, 0.0f);
    result->tool_x_axis = Vec3(1.0f, 0.0f, 0.0f);
    result->tool_y_axis = Vec3(0.0f, 1.0f, 0.0f);
    result->tool_z_axis = Vec3(0.0f, 0.0f, 1.0f);
}

static void FillForward(RobotArmTool_t tool,
                        RobotArmToolState_t state,
                        float target_z_mm,
                        const float theta_rad[ROBOTARM_KIN_JOINT_COUNT],
                        RobotArmIKResult_t *result)
{
    const RobotArmToolGeometry_t *tool_geometry =
        &s_tool_geometry[(uint8_t)tool][(uint8_t)state];
    float yaw = theta_rad[0];
    float theta2 = theta_rad[1];
    float theta3 = theta_rad[2];
    float phi = theta2 + theta3;
    float gamma = theta2 + (0.5f * ROBOTARM_KIN_PI);
    float alpha = gamma - LongToChordOffsetRad();
    float radial = ROBOTARM_KIN_L23_MM * cosf(gamma);
    float height = ROBOTARM_KIN_L23_MM * sinf(gamma);
    float cy = cosf(yaw);
    float sy = sinf(yaw);
    float cphi = cosf(phi);
    float sphi = sinf(phi);
    RobotArmVec3_t x_axis = Vec3(cy, sy, 0.0f);
    RobotArmVec3_t y_base = Vec3(-sy, cy, 0.0f);
    RobotArmVec3_t z_base = Vec3(0.0f, 0.0f, 1.0f);
    RobotArmVec3_t y_axis = Add3(Scale3(y_base, cphi), Scale3(z_base, sphi));
    RobotArmVec3_t z_axis = Add3(Scale3(y_base, -sphi), Scale3(z_base, cphi));
    RobotArmVec3_t j2 = Vec3(-ROBOTARM_KIN_J2_Y_MM * sy,
                              ROBOTARM_KIN_J2_Y_MM * cy,
                              ROBOTARM_KIN_D1_MM);
    RobotArmVec3_t j3 = Add3(j2,
                             Add3(Scale3(y_base, radial),
                                  Scale3(z_base, height)));
    RobotArmVec3_t tool_pos = j3;

    tool_pos = Add3(tool_pos, Scale3(x_axis, tool_geometry->offset_mm.x));
    tool_pos = Add3(tool_pos, Scale3(y_axis, tool_geometry->offset_mm.y));
    tool_pos = Add3(tool_pos, Scale3(z_axis, tool_geometry->offset_mm.z));

    result->tool = tool;
    result->state = state;
    result->target_direction = DirectionFromLegacyYaw(yaw);
    result->alpha_rad = alpha;
    result->target_z_mm = target_z_mm;
    result->approach_yaw_rad = yaw;
    result->theta_rad[0] = theta_rad[0];
    result->theta_rad[1] = theta_rad[1];
    result->theta_rad[2] = theta_rad[2];
    result->j2_world_mm = j2;
    result->j3_world_mm = j3;
    result->j4_world_mm = j3;
    result->tool_world_mm = tool_pos;
    result->tool_x_axis = x_axis;
    result->tool_y_axis = y_axis;
    result->tool_z_axis = z_axis;
}

static uint8_t IsFiniteFloat(float x)
{
    return ((x == x) && (x <= 3.4028234e38f) && (x >= -3.4028234e38f)) ? 1U : 0U;
}

static uint8_t IsToolValid(RobotArmTool_t tool)
{
    return ((uint8_t)tool < ROBOTARM_KIN_TOOL_COUNT) ? 1U : 0U;
}

static uint8_t IsStateIndexValid(RobotArmToolState_t state)
{
    return ((uint8_t)state < 2U) ? 1U : 0U;
}

static uint8_t IsDirectionValid(RobotArmWorkDirection_t direction)
{
    return ((direction == ROBOTARM_WORK_DIR_Y_POS) ||
            (direction == ROBOTARM_WORK_DIR_X_POS) ||
            (direction == ROBOTARM_WORK_DIR_X_NEG)) ? 1U : 0U;
}

static uint8_t IsToolStateSupported(RobotArmTool_t tool,
                                    RobotArmToolState_t state)
{
    if ((!IsToolValid(tool)) || (!IsStateIndexValid(state)))
    {
        return 0U;
    }
    return s_tool_geometry[(uint8_t)tool][(uint8_t)state].ik_enabled;
}

static float ClampF(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float NormalizePi(float x)
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

static uint8_t InLimit(float x, float lo, float hi)
{
    return ((x >= (lo - ROBOTARM_KIN_EPS)) &&
            (x <= (hi + ROBOTARM_KIN_EPS))) ? 1U : 0U;
}

static float AbsF(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float LongToChordOffsetRad(void)
{
    float adjacent = ROBOTARM_KIN_LONG_LINK_MM +
                     ROBOTARM_KIN_SHORT_LINK_MM * cosf(ROBOTARM_KIN_BEND_RAD);
    float opposite = ROBOTARM_KIN_SHORT_LINK_MM * sinf(ROBOTARM_KIN_BEND_RAD);
    return atan2f(opposite, adjacent);
}

static RobotArmWorkDirection_t SelectDirectionWithDeadband(
    const RobotArmVec3_t *target_xyz_mm,
    RobotArmWorkDirection_t current_direction,
    float xy_deadband_mm)
{
    float x;
    float y;
    float abs_x;

    if (target_xyz_mm == NULL)
    {
        return ROBOTARM_WORK_DIR_Y_POS;
    }

    x = target_xyz_mm->x;
    y = target_xyz_mm->y;
    abs_x = AbsF(x);
    if (xy_deadband_mm < 0.0f)
    {
        xy_deadband_mm = -xy_deadband_mm;
    }

    if (y < 0.0f)
    {
        if (abs_x <= xy_deadband_mm)
        {
            if (current_direction == ROBOTARM_WORK_DIR_X_POS)
            {
                return ROBOTARM_WORK_DIR_X_POS;
            }
            if (current_direction == ROBOTARM_WORK_DIR_X_NEG)
            {
                return ROBOTARM_WORK_DIR_X_NEG;
            }
            return ROBOTARM_WORK_DIR_X_POS;
        }
        return (x >= 0.0f) ? ROBOTARM_WORK_DIR_X_POS : ROBOTARM_WORK_DIR_X_NEG;
    }

    if ((y + xy_deadband_mm) >= abs_x)
    {
        return ROBOTARM_WORK_DIR_Y_POS;
    }
    return (x >= 0.0f) ? ROBOTARM_WORK_DIR_X_POS : ROBOTARM_WORK_DIR_X_NEG;
}

static RobotArmWorkDirection_t DirectionFromLegacyYaw(float yaw_rad)
{
    float yaw = NormalizePi(yaw_rad);

    if (yaw < (-0.25f * ROBOTARM_KIN_PI))
    {
        return ROBOTARM_WORK_DIR_X_POS;
    }
    if (yaw > (0.25f * ROBOTARM_KIN_PI))
    {
        return ROBOTARM_WORK_DIR_X_NEG;
    }
    return ROBOTARM_WORK_DIR_Y_POS;
}

static RobotArmVec3_t Vec3(float x, float y, float z)
{
    RobotArmVec3_t v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

static RobotArmVec3_t Add3(RobotArmVec3_t a, RobotArmVec3_t b)
{
    return Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

static RobotArmVec3_t Scale3(RobotArmVec3_t a, float s)
{
    return Vec3(a.x * s, a.y * s, a.z * s);
}
