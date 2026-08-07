#include "robotarm_kinematics.h"

#include <math.h>
#include <stddef.h>

#define ROBOTARM_KIN_EPS                    1.0e-5f
#define ROBOTARM_KIN_DEFAULT_XY_DEADBAND_MM 1.0f
#define ROBOTARM_KIN_S2_PREPARE_TILT_RAD    (ROBOTARM_KIN_PI / 12.0f)
#define ROBOTARM_KIN_Z_LIMIT_SAMPLE_COUNT   96U

typedef enum
{
    ROBOTARM_TOOL_POSTURE_FIXED = 0,
    ROBOTARM_TOOL_POSTURE_SHORT_PARALLEL = 1
} RobotArmToolPostureMode_t;

typedef struct
{
    RobotArmVec3_t offset_mm;
    float phi_offset_rad;
    RobotArmToolPostureMode_t posture_mode;
    uint8_t ik_enabled;
} RobotArmToolGeometry_t;

static const RobotArmToolGeometry_t s_tool_geometry[ROBOTARM_KIN_TOOL_COUNT][ROBOTARM_KIN_TOOL_STATE_COUNT] =
{
    {
        {{0.0f, -46.79f, 45.5f}, 0.0f, ROBOTARM_TOOL_POSTURE_FIXED, 1U},
        {{0.0f, -46.79f, 45.5f}, 0.0f, ROBOTARM_TOOL_POSTURE_FIXED, 1U},
        {{0.0f, -46.79f, 45.5f}, 0.0f, ROBOTARM_TOOL_POSTURE_FIXED, 1U},
        {{0.0f, -46.79f, 45.5f}, 0.0f, ROBOTARM_TOOL_POSTURE_FIXED, 0U},
        {{0.0f, -46.79f, 45.5f}, 0.0f, ROBOTARM_TOOL_POSTURE_FIXED, 0U}
    },
    {
        {{0.0f, 92.6f, 0.0f}, ROBOTARM_KIN_S2_PREPARE_TILT_RAD, ROBOTARM_TOOL_POSTURE_FIXED, 1U},
        {{0.0f, 92.6f, 0.0f}, 0.0f, ROBOTARM_TOOL_POSTURE_FIXED, 1U},
        {{0.0f, 92.6f, 0.0f}, 0.0f, ROBOTARM_TOOL_POSTURE_SHORT_PARALLEL, 1U},
        {{0.0f, 92.6f, 0.0f}, 0.5f * ROBOTARM_KIN_PI, ROBOTARM_TOOL_POSTURE_FIXED, 1U},
        {{0.0f, 92.6f, 0.0f}, 0.0f, ROBOTARM_TOOL_POSTURE_FIXED, 0U}
    },
    {
        {{103.0f, -92.0f, 10.3f}, 0.5f * ROBOTARM_KIN_PI, ROBOTARM_TOOL_POSTURE_FIXED, 1U},
        {{103.0f, -92.0f, 10.3f}, -0.5f * ROBOTARM_KIN_PI, ROBOTARM_TOOL_POSTURE_FIXED, 1U},
        {{103.0f, -92.0f, 10.3f}, 0.0f, ROBOTARM_TOOL_POSTURE_FIXED, 1U},
        {{103.0f, -92.0f, 10.3f}, 0.0f, ROBOTARM_TOOL_POSTURE_FIXED, 0U},
        {{103.0f, -92.0f, 10.3f}, 0.0f, ROBOTARM_TOOL_POSTURE_FIXED, 0U}
    }
};

static uint8_t IsFiniteFloat(float x);
static uint8_t IsToolValid(RobotArmTool_t tool);
static uint8_t IsStateIndexValid(RobotArmToolState_t state);
static uint8_t IsDirectionValid(RobotArmWorkDirection_t direction);
static uint8_t IsToolStateSupported(RobotArmTool_t tool,
                                    RobotArmToolState_t state);
static float NormalizePi(float x);
static uint8_t InLimit(float x, float lo, float hi);
static float AbsF(float x);
static float LongToChordOffsetRad(void);
static uint8_t ResolveToolGeometry(const RobotArmIKRequest_t *request,
                                   RobotArmToolGeometry_t *geometry);
static float ToolFixedPhiFromOffset(float phi_offset_rad);
static float ToolPhiFromAlpha(const RobotArmToolGeometry_t *geometry,
                              float alpha_rad);
static uint8_t GetAlphaLimitsForDirection(
    const RobotArmKinematicsConfig_t *config,
    RobotArmWorkDirection_t direction,
    float *alpha_min_rad,
    float *alpha_max_rad);
static float ToolTargetZFromAlphaGeometry(
    const RobotArmToolGeometry_t *geometry,
    float alpha_rad);
static uint8_t GetToolZLimitForGeometry(
    const RobotArmToolGeometry_t *geometry,
    const RobotArmKinematicsConfig_t *config,
    RobotArmWorkDirection_t direction,
    RobotArmZLimit_t *limit);
static void ExpandZLimit(RobotArmZLimit_t *limit,
                         float z_mm,
                         uint8_t *has_value);
static RobotArmWorkDirection_t SelectDirectionWithDeadband(
    const RobotArmVec3_t *target_xyz_mm,
    RobotArmWorkDirection_t current_direction,
    float xy_deadband_mm);
static RobotArmWorkDirection_t DirectionFromLegacyYaw(float yaw_rad);
static RobotArmVec3_t Vec3(float x, float y, float z);
static RobotArmVec3_t Add3(RobotArmVec3_t a, RobotArmVec3_t b);
static RobotArmVec3_t Scale3(RobotArmVec3_t a, float s);
static RobotArmVec3_t ModelToRobotXY(RobotArmVec3_t v);
static void FillForward(RobotArmTool_t tool,
                        RobotArmToolState_t state,
                        const RobotArmToolGeometry_t *geometry,
                        float target_z_mm,
                        const float theta_rad[ROBOTARM_KIN_JOINT_COUNT],
                        RobotArmIKResult_t *result);

void RobotArmKinematics_DefaultConfig(RobotArmKinematicsConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->joint_min_rad[0] = ROBOTARM_KIN_J1_MIN_RAD;
    config->joint_max_rad[0] = ROBOTARM_KIN_J1_MAX_RAD;
    config->joint_min_rad[1] = ROBOTARM_KIN_J2_MIN_RAD;
    config->joint_max_rad[1] = ROBOTARM_KIN_J2_MAX_RAD;
    config->joint_min_rad[2] = ROBOTARM_KIN_J3_MIN_RAD;
    config->joint_max_rad[2] = ROBOTARM_KIN_J3_MAX_RAD;
    config->preferred_theta2_sign = ROBOTARM_KIN_PREFERRED_THETA2_SIGN;
    config->alpha_min_rad = ROBOTARM_KIN_ALPHA_MIN_RAD;
    config->alpha_max_rad = ROBOTARM_KIN_ALPHA_MAX_RAD;
    config->xy_deadband_mm = ROBOTARM_KIN_DEFAULT_XY_DEADBAND_MM;
}

float RobotArmKinematics_ModelZeroAlpha(void)
{
    return (0.5f * ROBOTARM_KIN_PI) + LongToChordOffsetRad();
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

float RobotArmKinematics_ToolTargetZFromAlpha(RobotArmTool_t tool,
                                              RobotArmToolState_t state,
                                              float alpha_rad)
{
    RobotArmIKRequest_t request;
    RobotArmToolGeometry_t tool_geometry;

    if ((!IsToolStateSupported(tool, state)) ||
        (!IsFiniteFloat(alpha_rad)))
    {
        return 0.0f;
    }

    request.tool = tool;
    request.state = state;
    request.target_direction_valid = 0U;
    request.target_direction = ROBOTARM_WORK_DIR_Y_POS;
    request.posture_source_valid = 0U;
    request.posture_source_tool = tool;
    request.posture_source_state = state;

    if (!ResolveToolGeometry(&request, &tool_geometry))
    {
        return 0.0f;
    }

    return ToolTargetZFromAlphaGeometry(&tool_geometry, alpha_rad);
}

float RobotArmKinematics_RequestToolTargetZFromAlpha(
    const RobotArmIKRequest_t *request,
    float alpha_rad)
{
    RobotArmToolGeometry_t tool_geometry;

    if ((request == NULL) || (!IsFiniteFloat(alpha_rad)) ||
        (!ResolveToolGeometry(request, &tool_geometry)))
    {
        return 0.0f;
    }

    return ToolTargetZFromAlphaGeometry(&tool_geometry, alpha_rad);
}

uint8_t RobotArmKinematics_GetToolZLimit(
    RobotArmTool_t tool,
    RobotArmToolState_t state,
    const RobotArmKinematicsConfig_t *config,
    RobotArmZLimit_t *limit)
{
    RobotArmKinematicsConfig_t local_config;
    const RobotArmKinematicsConfig_t *cfg;
    RobotArmIKRequest_t request;
    RobotArmToolGeometry_t tool_geometry;

    if (limit == NULL)
    {
        return 0U;
    }

    limit->min_mm = 0.0f;
    limit->max_mm = 0.0f;

    if (!IsToolStateSupported(tool, state))
    {
        return 0U;
    }

    RobotArmKinematics_DefaultConfig(&local_config);
    cfg = (config != NULL) ? config : &local_config;

    request.tool = tool;
    request.state = state;
    request.target_direction_valid = 0U;
    request.target_direction = ROBOTARM_WORK_DIR_Y_POS;
    request.posture_source_valid = 0U;
    request.posture_source_tool = tool;
    request.posture_source_state = state;
    if (!ResolveToolGeometry(&request, &tool_geometry))
    {
        return 0U;
    }

    return GetToolZLimitForGeometry(&tool_geometry,
                                    cfg,
                                    ROBOTARM_WORK_DIR_Y_POS,
                                    limit);
}

uint8_t RobotArmKinematics_GetRequestToolZLimit(
    const RobotArmIKRequest_t *request,
    const RobotArmKinematicsConfig_t *config,
    RobotArmZLimit_t *limit)
{
    RobotArmKinematicsConfig_t local_config;
    const RobotArmKinematicsConfig_t *cfg;
    RobotArmToolGeometry_t tool_geometry;
    RobotArmWorkDirection_t direction;

    if (limit == NULL)
    {
        return 0U;
    }

    limit->min_mm = 0.0f;
    limit->max_mm = 0.0f;

    if (request == NULL)
    {
        return 0U;
    }

    RobotArmKinematics_DefaultConfig(&local_config);
    cfg = (config != NULL) ? config : &local_config;

    if (!ResolveToolGeometry(request, &tool_geometry))
    {
        return 0U;
    }

    if (request->target_direction_valid != 0U)
    {
        direction = request->target_direction;
    }
    else
    {
        direction = SelectDirectionWithDeadband(&request->target_xyz_mm,
                                                request->current_direction,
                                                cfg->xy_deadband_mm);
    }
    if (!IsDirectionValid(direction))
    {
        return 0U;
    }

    return GetToolZLimitForGeometry(&tool_geometry, cfg, direction, limit);
}

static uint8_t ResolveToolGeometry(const RobotArmIKRequest_t *request,
                                   RobotArmToolGeometry_t *geometry)
{
    const RobotArmToolGeometry_t *base;
    const RobotArmToolGeometry_t *source;

    if ((request == NULL) || (geometry == NULL) ||
        (!IsToolStateSupported(request->tool, request->state)))
    {
        return 0U;
    }

    base = &s_tool_geometry[(uint8_t)request->tool][(uint8_t)request->state];
    *geometry = *base;

    if (request->posture_source_valid == 1U)
    {
        if (!IsToolStateSupported(request->posture_source_tool,
                                  request->posture_source_state))
        {
            return 0U;
        }

        source =
            &s_tool_geometry[(uint8_t)request->posture_source_tool]
                            [(uint8_t)request->posture_source_state];
        geometry->phi_offset_rad = source->phi_offset_rad;
        geometry->posture_mode = source->posture_mode;
    }

    return 1U;
}

static float ToolFixedPhiFromOffset(float phi_offset_rad)
{
    return NormalizePi(ROBOTARM_KIN_FRAME4_POWERON_PHI_RAD + phi_offset_rad);
}

static float ToolPhiFromAlpha(const RobotArmToolGeometry_t *geometry,
                              float alpha_rad)
{
    if (geometry == NULL)
    {
        return 0.0f;
    }

    if (geometry->posture_mode == ROBOTARM_TOOL_POSTURE_SHORT_PARALLEL)
    {
        return NormalizePi(alpha_rad - ROBOTARM_KIN_BEND_RAD -
                           (0.5f * ROBOTARM_KIN_PI));
    }

    return ToolFixedPhiFromOffset(geometry->phi_offset_rad);
}

static uint8_t GetAlphaLimitsForDirection(
    const RobotArmKinematicsConfig_t *config,
    RobotArmWorkDirection_t direction,
    float *alpha_min_rad,
    float *alpha_max_rad)
{
    float safe_min;
    float safe_max;
    float alpha_min;
    float alpha_max;

    if ((config == NULL) || (alpha_min_rad == NULL) ||
        (alpha_max_rad == NULL) || (!IsDirectionValid(direction)))
    {
        return 0U;
    }

    safe_min = (direction == ROBOTARM_WORK_DIR_Y_POS) ?
        ROBOTARM_KIN_ALPHA_Y_POS_MIN_RAD : ROBOTARM_KIN_ALPHA_X_MIN_RAD;
    safe_max = ROBOTARM_KIN_ALPHA_MAX_RAD;
    alpha_min = config->alpha_min_rad;
    alpha_max = config->alpha_max_rad;

    if ((!IsFiniteFloat(alpha_min)) ||
        (!IsFiniteFloat(alpha_max)) ||
        (alpha_min > alpha_max))
    {
        return 0U;
    }

    if (alpha_min < safe_min)
    {
        alpha_min = safe_min;
    }
    if (alpha_max > safe_max)
    {
        alpha_max = safe_max;
    }
    if (alpha_min > alpha_max)
    {
        return 0U;
    }

    *alpha_min_rad = alpha_min;
    *alpha_max_rad = alpha_max;
    return 1U;
}

static float ToolTargetZFromAlphaGeometry(
    const RobotArmToolGeometry_t *geometry,
    float alpha_rad)
{
    float phi_rad;
    float gamma_rad;

    if ((geometry == NULL) || (!IsFiniteFloat(alpha_rad)))
    {
        return 0.0f;
    }

    phi_rad = ToolPhiFromAlpha(geometry, alpha_rad);
    gamma_rad = alpha_rad - LongToChordOffsetRad();

    return ROBOTARM_KIN_D1_MM +
           ROBOTARM_KIN_L23_MM * sinf(gamma_rad) +
           geometry->offset_mm.y * sinf(phi_rad) +
           geometry->offset_mm.z * cosf(phi_rad);
}

static uint8_t GetToolZLimitForGeometry(
    const RobotArmToolGeometry_t *geometry,
    const RobotArmKinematicsConfig_t *config,
    RobotArmWorkDirection_t direction,
    RobotArmZLimit_t *limit)
{
    float alpha_min;
    float alpha_max;
    float long_to_chord_rad;
    float alpha_at_min_z;
    float alpha_at_max_z;
    uint8_t has_value = 0U;
    uint8_t i;

    if ((geometry == NULL) || (config == NULL) || (limit == NULL))
    {
        return 0U;
    }

    limit->min_mm = 0.0f;
    limit->max_mm = 0.0f;

    if (!GetAlphaLimitsForDirection(config,
                                    direction,
                                    &alpha_min,
                                    &alpha_max))
    {
        return 0U;
    }

    if (geometry->posture_mode == ROBOTARM_TOOL_POSTURE_FIXED)
    {
        ExpandZLimit(limit,
                     ToolTargetZFromAlphaGeometry(geometry, alpha_min),
                     &has_value);
        ExpandZLimit(limit,
                     ToolTargetZFromAlphaGeometry(geometry, alpha_max),
                     &has_value);

        long_to_chord_rad = LongToChordOffsetRad();
        alpha_at_min_z = (-0.5f * ROBOTARM_KIN_PI) + long_to_chord_rad;
        alpha_at_max_z = (0.5f * ROBOTARM_KIN_PI) + long_to_chord_rad;

        if (InLimit(alpha_at_min_z, alpha_min, alpha_max))
        {
            ExpandZLimit(limit,
                         ToolTargetZFromAlphaGeometry(geometry,
                                                       alpha_at_min_z),
                         &has_value);
        }
        if (InLimit(alpha_at_max_z, alpha_min, alpha_max))
        {
            ExpandZLimit(limit,
                         ToolTargetZFromAlphaGeometry(geometry,
                                                       alpha_at_max_z),
                         &has_value);
        }
        return has_value;
    }

    for (i = 0U; i <= ROBOTARM_KIN_Z_LIMIT_SAMPLE_COUNT; i++)
    {
        float t = (float)i / (float)ROBOTARM_KIN_Z_LIMIT_SAMPLE_COUNT;
        float alpha = alpha_min + (alpha_max - alpha_min) * t;
        ExpandZLimit(limit,
                     ToolTargetZFromAlphaGeometry(geometry, alpha),
                     &has_value);
    }

    return has_value;
}

RobotArmWorkDirection_t RobotArmKinematics_SelectDirection(
    const RobotArmVec3_t *target_xyz_mm,
    RobotArmWorkDirection_t current_direction)
{
    return SelectDirectionWithDeadband(target_xyz_mm,
                                       current_direction,
                                       ROBOTARM_KIN_DEFAULT_XY_DEADBAND_MM);
}

RobotArmIKStatus_t RobotArmKinematics_Forward(
    RobotArmTool_t tool,
    const float theta_rad[ROBOTARM_KIN_JOINT_COUNT],
    RobotArmIKResult_t *result)
{
    return RobotArmKinematics_ForwardState(tool,
                                           ROBOTARM_TOOL_STATE_STOW,
                                           theta_rad,
                                           result);
}

RobotArmIKStatus_t RobotArmKinematics_ForwardState(
    RobotArmTool_t tool,
    RobotArmToolState_t state,
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
    if (!IsToolStateSupported(tool, state))
    {
        result->status = ROBOTARM_IK_ERR_UNSUPPORTED_STATE;
        result->reason = ROBOTARM_IK_REASON_UNSUPPORTED_STATE;
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

    FillForward(tool,
                state,
                &s_tool_geometry[(uint8_t)tool][(uint8_t)state],
                0.0f,
                theta_rad,
                result);
    result->target_z_mm = result->tool_world_mm.z;
    result->status = ROBOTARM_IK_OK;
    result->reason = ROBOTARM_IK_REASON_NONE;
    return result->status;
}

float RobotArmKinematics_GetToolPosturePhi(RobotArmTool_t tool,
                                           RobotArmToolState_t state)
{
    const RobotArmToolGeometry_t *geometry;

    if (!IsToolStateSupported(tool, state))
    {
        return 0.0f;
    }

    geometry = &s_tool_geometry[(uint8_t)tool][(uint8_t)state];
    if (geometry->posture_mode != ROBOTARM_TOOL_POSTURE_FIXED)
    {
        return geometry->phi_offset_rad;
    }

    return ToolPhiFromAlpha(geometry, 0.0f);
}

const RobotArmVec3_t *RobotArmKinematics_GetToolOffset(RobotArmTool_t tool)
{
    if (!IsToolValid(tool))
    {
        return NULL;
    }
    return &s_tool_geometry[(uint8_t)tool][(uint8_t)ROBOTARM_TOOL_STATE_STOW].offset_mm;
}

static void FillForward(RobotArmTool_t tool,
                        RobotArmToolState_t state,
                        const RobotArmToolGeometry_t *geometry,
                        float target_z_mm,
                        const float theta_rad[ROBOTARM_KIN_JOINT_COUNT],
                        RobotArmIKResult_t *result)
{
    const RobotArmToolGeometry_t *tool_geometry = geometry;
    float yaw = theta_rad[0];
    float theta2 = theta_rad[1];
    float theta3 = theta_rad[2];
    float phi = theta2 + theta3;
    float gamma = theta2 + (0.5f * ROBOTARM_KIN_PI);
    float alpha = gamma + LongToChordOffsetRad();
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

    if (tool_geometry == NULL)
    {
        tool_geometry = &s_tool_geometry[(uint8_t)tool][(uint8_t)state];
    }

    tool_pos = Add3(tool_pos, Scale3(x_axis, tool_geometry->offset_mm.x));
    tool_pos = Add3(tool_pos, Scale3(y_axis, tool_geometry->offset_mm.y));
    tool_pos = Add3(tool_pos, Scale3(z_axis, tool_geometry->offset_mm.z));

    j2 = ModelToRobotXY(j2);
    j3 = ModelToRobotXY(j3);
    tool_pos = ModelToRobotXY(tool_pos);
    x_axis = ModelToRobotXY(x_axis);
    y_axis = ModelToRobotXY(y_axis);
    z_axis = ModelToRobotXY(z_axis);

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
    return ((uint8_t)state < ROBOTARM_KIN_TOOL_STATE_COUNT) ? 1U : 0U;
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

static void ExpandZLimit(RobotArmZLimit_t *limit,
                         float z_mm,
                         uint8_t *has_value)
{
    if ((limit == NULL) || (has_value == NULL) || (!IsFiniteFloat(z_mm)))
    {
        return;
    }

    if (*has_value == 0U)
    {
        limit->min_mm = z_mm;
        limit->max_mm = z_mm;
        *has_value = 1U;
        return;
    }

    if (z_mm < limit->min_mm)
    {
        limit->min_mm = z_mm;
    }
    if (z_mm > limit->max_mm)
    {
        limit->max_mm = z_mm;
    }
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

    if ((yaw > (0.25f * ROBOTARM_KIN_PI)) &&
        (yaw <= (0.75f * ROBOTARM_KIN_PI)))
    {
        return ROBOTARM_WORK_DIR_X_NEG;
    }
    if ((yaw < (-0.25f * ROBOTARM_KIN_PI)) &&
        (yaw >= (-0.75f * ROBOTARM_KIN_PI)))
    {
        return ROBOTARM_WORK_DIR_X_POS;
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

static RobotArmVec3_t ModelToRobotXY(RobotArmVec3_t v)
{
    return Vec3(-v.x, -v.y, v.z);
}
