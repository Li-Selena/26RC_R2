#ifndef __R2_CLIMB_H
#define __R2_CLIMB_H

#include <stdint.h>

/*
 * FDCAN2 motor map:
 *   Leg 1: front right, start at right-top corner, clockwise
 *   Leg 2: rear  right
 *   Leg 3: rear  left
 *   Leg 4: front left
 *   Drive wheel 5: rear left  M2006
 *   Drive wheel 6: rear right M2006
 */

/*
 * Scale factors from the actual climb mechanism.
 *
 * Leg rack:   count/mm = encoder_count_per_motor_rev * total_reduction /
 *                        pinion_pitch_circumference_mm
 * Drive wheel count/mm = encoder_count_per_motor_rev * total_reduction /
 *                        wheel_circumference_mm
 */
#ifndef R2_CLIMB_PARAM_CONFIGURED
#define R2_CLIMB_PARAM_CONFIGURED        1U
#endif

#ifndef R2_CLIMB_ENCODER_COUNT_PER_REV
#define R2_CLIMB_ENCODER_COUNT_PER_REV   8192.0f
#endif

#ifndef R2_CLIMB_LEG_REDUCTION
#define R2_CLIMB_LEG_REDUCTION           (3591.0f / 187.0f)
#endif

#ifndef R2_CLIMB_LEG_PINION_TOOTH_COUNT
#define R2_CLIMB_LEG_PINION_TOOTH_COUNT  18.0f
#endif

#ifndef R2_CLIMB_LEG_RACK_PITCH_MM
#define R2_CLIMB_LEG_RACK_PITCH_MM       4.71f
#endif

#ifndef R2_CLIMB_LEG_PINION_TRAVEL_PER_REV_MM
#define R2_CLIMB_LEG_PINION_TRAVEL_PER_REV_MM \
    (R2_CLIMB_LEG_PINION_TOOTH_COUNT * R2_CLIMB_LEG_RACK_PITCH_MM)
#endif

#ifndef R2_CLIMB_DRIVE_REDUCTION
#define R2_CLIMB_DRIVE_REDUCTION         36.0f
#endif

#ifndef R2_CLIMB_DRIVE_WHEEL_DIAMETER_MM
#define R2_CLIMB_DRIVE_WHEEL_DIAMETER_MM 64.0f
#endif

#ifndef R2_CLIMB_PI
#define R2_CLIMB_PI                      3.14159265358979323846f
#endif

#ifndef R2_CLIMB_DRIVE_WHEEL_CIRCUMFERENCE_MM
#define R2_CLIMB_DRIVE_WHEEL_CIRCUMFERENCE_MM \
    (R2_CLIMB_PI * R2_CLIMB_DRIVE_WHEEL_DIAMETER_MM)
#endif

#ifndef R2_CLIMB_LEG_COUNT_PER_MM
#define R2_CLIMB_LEG_COUNT_PER_MM \
    ((R2_CLIMB_ENCODER_COUNT_PER_REV * R2_CLIMB_LEG_REDUCTION) / \
     R2_CLIMB_LEG_PINION_TRAVEL_PER_REV_MM)
#endif

#ifndef R2_CLIMB_DRIVE_COUNT_PER_MM
#define R2_CLIMB_DRIVE_COUNT_PER_MM \
    ((R2_CLIMB_ENCODER_COUNT_PER_REV * R2_CLIMB_DRIVE_REDUCTION) / \
     R2_CLIMB_DRIVE_WHEEL_CIRCUMFERENCE_MM)
#endif

#ifndef R2_CLIMB_LEG1_DIR
#define R2_CLIMB_LEG1_DIR                1.0f
#endif
#ifndef R2_CLIMB_LEG2_DIR
#define R2_CLIMB_LEG2_DIR                1.0f
#endif
#ifndef R2_CLIMB_LEG3_DIR
#define R2_CLIMB_LEG3_DIR                1.0f
#endif
#ifndef R2_CLIMB_LEG4_DIR
#define R2_CLIMB_LEG4_DIR                1.0f
#endif
#ifndef R2_CLIMB_DRIVE_LEFT_DIR
#define R2_CLIMB_DRIVE_LEFT_DIR          1.0f
#endif
#ifndef R2_CLIMB_DRIVE_RIGHT_DIR
#define R2_CLIMB_DRIVE_RIGHT_DIR         1.0f
#endif

#define R2_CLIMB_LIFT_SAFE_MM          220.0f
#define R2_CLIMB_LIFT_STEP_MM          200.0f
#define R2_CLIMB_FIRST_PUSH_MM         180.0f
#define R2_CLIMB_SECOND_PUSH_MM        250.0f

#define R2_CLIMB_LEG_TOL_MM              3.0f
#define R2_CLIMB_DRIVE_TOL_MM            5.0f

#define R2_CLIMB_RAISE_TIMEOUT_MS     6000U
#define R2_CLIMB_FIRST_PUSH_TIMEOUT_MS 6000U
#define R2_CLIMB_LOWER_TIMEOUT_MS     4000U
#define R2_CLIMB_FRONT_RETRACT_TIMEOUT_MS 4000U
#define R2_CLIMB_SECOND_PUSH_TIMEOUT_MS 8000U
#define R2_CLIMB_REAR_RETRACT_TIMEOUT_MS 5000U

#define R2_CLIMB_ERR_TIMEOUT             0x01U
#define R2_CLIMB_ERR_PARAM_NOT_CONFIGURED 0x02U

#define R2_CLIMB_DEBUG_SOURCE_USART   0U
#define R2_CLIMB_DEBUG_SOURCE_USB     1U
#define R2_CLIMB_DEBUG_SOURCE_NONE    2U

typedef enum
{
    R2_CLIMB_STATE_IDLE = 0,
    R2_CLIMB_STATE_RAISE_ALL,
    R2_CLIMB_STATE_FIRST_PUSH,
    R2_CLIMB_STATE_LOWER_ALL_TO_STEP,
    R2_CLIMB_STATE_RETRACT_FRONT_LEGS,
    R2_CLIMB_STATE_SECOND_PUSH,
    R2_CLIMB_STATE_RETRACT_REAR_LEGS,
    R2_CLIMB_STATE_DONE,
    R2_CLIMB_STATE_ERROR,
} R2_ClimbState_t;

typedef struct
{
    int16_t leg[4];    /* FDCAN2 ID 1..4 */
    int16_t drive[4];  /* FDCAN2 ID 5..8, only 5..6 are used */
} R2_ClimbMotorCmd_t;

typedef struct
{
    R2_ClimbState_t state;
    uint8_t enabled;
    uint8_t auto_run;
    uint8_t state_done;
    uint8_t error_flags;
    uint8_t pending_step;
    uint8_t pending_auto;
    uint8_t last_step_level;
    uint8_t last_auto_level;

    uint32_t state_start_ms;
    uint32_t last_update_ms;

    int32_t leg_zero[4];
    int32_t drive_segment_start[2];

    float leg_target_mm[4];
    float drive_target_mm[2];
    float leg_pos_mm[4];
    float drive_pos_mm[2];
} R2_Climb_Ctrl_t;

typedef struct
{
    uint8_t source;
    uint8_t state;
    uint8_t enabled;
    uint8_t auto_run;
    uint8_t state_done;
    uint8_t error_flags;
    uint8_t is_motor_active;
    uint8_t pending_step;
    uint8_t pending_auto;
    uint8_t param_ready;

    uint32_t state_start_ms;
    uint32_t last_update_ms;
    uint32_t elapsed_ms;
    const char *state_name;

    float leg_pos_mm[4];
    float leg_target_mm[4];
    float drive_pos_mm[2];
    float drive_target_mm[2];
    float leg_count_per_mm;
    float drive_count_per_mm;
    float leg_dir[4];
    float drive_dir[2];

    int32_t leg_zero[4];
    int32_t drive_segment_start[2];

    int16_t leg_current[4];
    int16_t drive_current[2];
} R2_ClimbDebug_t;

extern volatile R2_ClimbDebug_t g_r2_climb_debug_usart;
extern volatile R2_ClimbDebug_t g_r2_climb_debug_usb;
extern volatile R2_ClimbDebug_t g_r2_climb_debug_active;

void R2_Climb_Init(R2_Climb_Ctrl_t *ctrl);
void R2_Climb_Stop(R2_Climb_Ctrl_t *ctrl);
void R2_Climb_SetInput(R2_Climb_Ctrl_t *ctrl,
                       uint8_t enable_level,
                       uint8_t step_level,
                       uint8_t auto_level);
void R2_Climb_RequestStep(R2_Climb_Ctrl_t *ctrl);
void R2_Climb_RequestAuto(R2_Climb_Ctrl_t *ctrl);
void R2_Climb_Update(R2_Climb_Ctrl_t *ctrl, uint32_t now_ms);
uint8_t R2_Climb_IsMotorActive(const R2_Climb_Ctrl_t *ctrl);
void R2_Climb_GetMotorCurrent(const R2_Climb_Ctrl_t *ctrl,
                              R2_ClimbMotorCmd_t *cmd);
void R2_Climb_UpdateDebugViews(const R2_Climb_Ctrl_t *usart_ctrl,
                               const R2_Climb_Ctrl_t *usb_ctrl,
                               uint8_t active_source);
void R2_Climb_SetDebugMotorCurrent(uint8_t source,
                                   const R2_ClimbMotorCmd_t *cmd);

#endif
