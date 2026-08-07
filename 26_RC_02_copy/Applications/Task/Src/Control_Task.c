#include "cmsis_os.h"
#include "Control_Task.h"
#include "Data_Analysis.h"
#include "R2_arm.h"
#include "R2_yaw_autotune.h"
#include "CRC.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

extern volatile uint8_t USB_Task_flag;
extern volatile uint8_t USART_Task_flag;
extern ChassisVel_t total_vel;
extern WheelSpeed_t total_speed;
extern MecanumParam_t mecParam;

R2_Move_Ctrl_t g_r2_ctrl_usart;
R2_Move_Ctrl_t g_r2_ctrl_usb;
R2_Climb_Ctrl_t g_r2_climb_usart;
R2_Climb_Ctrl_t g_r2_climb_usb;
R2_TaskFlow_Ctrl_t g_r2_task_flow_usb;
R2_DebugOdom_t g_r2_debug_odom;

uint32_t g_r2_tick_ms = 0U;
int32_t g_r2_last_enc[CHASSIS_MOTOR_COUNT];
uint8_t g_r2_enc_inited = 0U;

static void R2_Control_1msStep(void);
static void R2_TaskFlow_Update(R2_TaskFlow_Ctrl_t *ctrl,
                               R2_Climb_Ctrl_t *climb,
                               R2_Arm_Ctrl_t *arm,
                               uint32_t now_ms);

static TaskHandle_t s_control_task_handle = NULL;

typedef enum
{
    R2_TASK_FLOW_OP_NONE = 0,
    R2_TASK_FLOW_OP_CLIMB_ACTION = 1,
    R2_TASK_FLOW_OP_ARM_JOG = 2,
    R2_TASK_FLOW_OP_TOOL_SET = 3,
    R2_TASK_FLOW_OP_POSTURE_CHECK = 4,
    R2_TASK_FLOW_OP_CLIMB_GATE = 5,
    R2_TASK_FLOW_OP_ARM_ENABLE_HOME = 6,
    R2_TASK_FLOW_OP_CLIMB_AUTO_PAUSE = 7,
    R2_TASK_FLOW_OP_CLIMB_AUTO_RESUME = 8,
    R2_TASK_FLOW_OP_ARM_WORKSPACE_SWITCH = 9,
    R2_TASK_FLOW_OP_TOOL_OFF_HELD = 10,
    R2_TASK_FLOW_OP_HOST_CHECKPOINT = 11,
} R2_TaskFlowOp_t;

typedef enum
{
    R2_TASK_FLOW_POSTURE_S1_UP_END = 1,
    R2_TASK_FLOW_POSTURE_S1_DOWN_END = 2,
} R2_TaskFlowPosture_t;

typedef struct
{
    uint8_t op;
    uint8_t arg;
    int16_t value;
    uint16_t repeat;
} R2_TaskFlowEntry_t;

typedef struct
{
    uint8_t id;
    const R2_TaskFlowEntry_t *entries;
    uint16_t count;
} R2_TaskFlowDesc_t;

#define R2_TASK_FLOW_ARRAY_LEN(a) ((uint16_t)(sizeof(a) / sizeof((a)[0])))
#define R2_TASK_CLIMB(action, repeat_count) \
    {R2_TASK_FLOW_OP_CLIMB_ACTION, (uint8_t)(action), 0, (uint16_t)(repeat_count)}
#define R2_TASK_ARM(joint, delta_deg, repeat_count) \
    {R2_TASK_FLOW_OP_ARM_JOG, (uint8_t)(joint), (int16_t)(delta_deg), (uint16_t)(repeat_count)}
#define R2_TASK_TOOL_ON(tool, repeat_count) \
    {R2_TASK_FLOW_OP_TOOL_SET, (uint8_t)(tool), 1, (uint16_t)(repeat_count)}
#define R2_TASK_TOOL_OFF(tool, repeat_count) \
    {R2_TASK_FLOW_OP_TOOL_SET, (uint8_t)(tool), 0, (uint16_t)(repeat_count)}
#define R2_TASK_POSTURE(which) \
    {R2_TASK_FLOW_OP_POSTURE_CHECK, (uint8_t)(which), 0, 1U}
#define R2_TASK_GATE(flow) \
    {R2_TASK_FLOW_OP_CLIMB_GATE, (uint8_t)(flow), 0, 1U}
#define R2_TASK_ARM_ENABLE_HOME() \
    {R2_TASK_FLOW_OP_ARM_ENABLE_HOME, 0U, 0, 1U}
#define R2_TASK_CLIMB_AUTO_PAUSE(flow) \
    {R2_TASK_FLOW_OP_CLIMB_AUTO_PAUSE, (uint8_t)(flow), 0, 1U}
#define R2_TASK_CLIMB_AUTO_RESUME() \
    {R2_TASK_FLOW_OP_CLIMB_AUTO_RESUME, 0U, 0, 1U}
#define R2_TASK_ARM_WORKSPACE_SWITCH() \
    {R2_TASK_FLOW_OP_ARM_WORKSPACE_SWITCH, 0xFFU, 0, 1U}
#define R2_TASK_ARM_WORKSPACE(direction) \
    {R2_TASK_FLOW_OP_ARM_WORKSPACE_SWITCH, (uint8_t)(direction), 0, 1U}
#define R2_TASK_TOOL_OFF_HELD() \
    {R2_TASK_FLOW_OP_TOOL_OFF_HELD, 0U, 0, 1U}
#define R2_TASK_HOST_CHECKPOINT(checkpoint) \
    {R2_TASK_FLOW_OP_HOST_CHECKPOINT, (uint8_t)(checkpoint), 0, 1U}

#define R2_TASK_FLOW_ARM_ENABLE_SETTLE_MS 500U
#define R2_TASK_FLOW_ARM_MIN_WAIT_MS      100U
#define R2_TASK_FLOW_ARM_REACHED_MS       100U
#define R2_TASK_FLOW_ARM_TIMEOUT_MS       40000U
#define R2_TASK_FLOW_ARM_TOLERANCE_DEG    1.0f
#define R2_TASK_FLOW_TOOL_SETTLE_MS       300U
#define R2_TASK_FLOW_POSTURE_SETTLE_MS    200U

static const R2_TaskFlowEntry_t s_task_flow_s1_up_v1[] = {
    R2_TASK_ARM_ENABLE_HOME(),
    R2_TASK_CLIMB_AUTO_PAUSE(R2_CLIMB_FLOW_UPSTAIRS),
    R2_TASK_ARM(3U, -60, 1U),
    R2_TASK_ARM(3U, -10, 1U),
    R2_TASK_ARM(3U, -5, 1U),
    R2_TASK_ARM(2U, -20, 1U),
    R2_TASK_ARM(2U, -1, 4U),
    R2_TASK_ARM(2U, -60, 1U),
    R2_TASK_TOOL_ON(ROBOTARM_TOOL_S2, 1U),
    R2_TASK_ARM(2U, 60, 1U),
    R2_TASK_ARM(3U, -30, 1U),
    R2_TASK_CLIMB_AUTO_RESUME(),
};

static const R2_TaskFlowEntry_t s_task_flow_s1_down_v1[] = {
    R2_TASK_ARM_ENABLE_HOME(),
    R2_TASK_CLIMB_AUTO_PAUSE(R2_CLIMB_FLOW_DOWNSTAIRS),
    R2_TASK_ARM(3U, -90, 1U),
    R2_TASK_ARM(3U, -10, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_DOWN_10, 1U),
    R2_TASK_ARM(2U, -90, 1U),
    R2_TASK_ARM(3U, -10, 1U),
    R2_TASK_ARM(3U, -5, 1U),
    R2_TASK_ARM(2U, -5, 1U),
    R2_TASK_TOOL_ON(ROBOTARM_TOOL_S2, 1U),
    R2_TASK_ARM(2U, 90, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_UP_10, 1U),
    R2_TASK_CLIMB_AUTO_RESUME(),
};

static const R2_TaskFlowEntry_t s_task_flow_s1_up_s2_down_v1[] = {
    R2_TASK_POSTURE(R2_TASK_FLOW_POSTURE_S1_UP_END),
    R2_TASK_GATE(R2_CLIMB_FLOW_DOWNSTAIRS),
    R2_TASK_CLIMB(R2_CLIMB_TEST_FRONT_UP_10, 22U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_REAR_UP_10, 4U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_UP_10, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30, 11U),
    R2_TASK_ARM(3U, -30, 2U),
    R2_TASK_ARM(2U, -60, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30, 5U),
    R2_TASK_ARM(2U, -10, 1U),
    R2_TASK_ARM(3U, -5, 1U),
    R2_TASK_ARM(2U, -10, 1U),
    R2_TASK_TOOL_ON(ROBOTARM_TOOL_S2, 1U),
    R2_TASK_ARM(2U, 60, 1U),
    R2_TASK_ARM(3U, -60, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30, 14U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_10, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_REAR_DOWN_10, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_DOWN_10, 5U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_10, 12U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_FRONT_UP_10, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_REAR_UP_10, 20U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30, 3U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_ZERO, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_DOWN_10, 3U),
};

static const R2_TaskFlowEntry_t s_task_flow_s1_down_s2_up_v1[] = {
    R2_TASK_POSTURE(R2_TASK_FLOW_POSTURE_S1_DOWN_END),
    R2_TASK_GATE(R2_CLIMB_FLOW_UPSTAIRS),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_220, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30, 2U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_10, 2U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_DOWN_10, 3U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_FRONT_MINUS_30, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30, 2U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_FRONT_UP_10, 4U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30, 2U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_UP_10, 5U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30, 9U),
    R2_TASK_ARM(3U, -30, 1U),
    R2_TASK_ARM(2U, -30, 2U),
    R2_TASK_ARM(2U, -5, 2U),
    R2_TASK_ARM(2U, 30, 3U),
    R2_TASK_ARM(3U, -30, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30, 19U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_ZERO, 1U),
};

static const R2_TaskFlowEntry_t s_task_flow_s1_down_s2_down_v1[] = {
    R2_TASK_POSTURE(R2_TASK_FLOW_POSTURE_S1_DOWN_END),
    R2_TASK_GATE(R2_CLIMB_FLOW_DOWNSTAIRS),
    R2_TASK_CLIMB(R2_CLIMB_TEST_FRONT_UP_10, 23U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_REAR_UP_10, 4U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_FRONT_UP_10, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_UP_10, 3U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_DOWN_10, 2U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_FRONT_UP_10, 2U),
    R2_TASK_ARM(3U, -30, 1U),
    R2_TASK_ARM(2U, -30, 2U),
    R2_TASK_ARM(3U, -30, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30, 13U),
    R2_TASK_ARM(3U, -20, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_DOWN_10, 2U),
    R2_TASK_ARM(2U, -10, 1U),
    R2_TASK_ARM(2U, -5, 1U),
    R2_TASK_ARM(3U, 5, 1U),
    R2_TASK_ARM(2U, -5, 2U),
    R2_TASK_ARM(3U, 5, 3U),
    R2_TASK_ARM(2U, 30, 3U),
    R2_TASK_ARM(3U, -5, 2U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_UP_10, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30, 17U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_DOWN_10, 4U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_30, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_10, 5U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_REAR_UP_10, 20U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_DRIVE_FORWARD_500, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_ZERO, 1U),
    R2_TASK_CLIMB(R2_CLIMB_TEST_ALL_LEGS_DOWN_10, 3U),
};

static const R2_TaskFlowEntry_t s_task_flow_weapon_grab_v1[] = {
    R2_TASK_ARM(3U, 30, 1U),
    R2_TASK_ARM(2U, -5, 1U),
    R2_TASK_ARM(2U, -30, 1U),
    R2_TASK_ARM(2U, -1, 3U),
    R2_TASK_ARM(3U, 5, 1U),
    R2_TASK_ARM(2U, -1, 1U),
    R2_TASK_ARM(2U, -5, 1U),
    R2_TASK_ARM(2U, -1, 1U),
    R2_TASK_ARM(3U, 1, 1U),
    R2_TASK_ARM(3U, 1, 1U),
    R2_TASK_ARM(2U, 5, 1U),
    R2_TASK_ARM(3U, 1, 1U),
    R2_TASK_ARM(3U, 1, 1U),
};

static const R2_TaskFlowEntry_t s_task_flow_throw_block_v1[] = {
    /* Workspace switching saves the current feedback pose and restores J2/J3. */
    R2_TASK_ARM_WORKSPACE_SWITCH(),
    R2_TASK_TOOL_OFF_HELD(),
};

static const R2_TaskFlowEntry_t s_task_flow_weapon_dock_test_v1[] = {
    R2_TASK_ARM_ENABLE_HOME(),
    R2_TASK_TOOL_ON(ROBOTARM_TOOL_GRIPPER, 1U),
    R2_TASK_ARM_WORKSPACE(ROBOTARM_WORK_DIR_X_POS),

    /* Full WEAPON_GRAB_V1 arm trajectory. */
    R2_TASK_ARM(3U, 30, 1U),
    R2_TASK_ARM(2U, -5, 1U),
    R2_TASK_ARM(2U, -30, 1U),
    R2_TASK_ARM(2U, -1, 3U),
    R2_TASK_ARM(3U, 5, 1U),
    R2_TASK_ARM(2U, -1, 1U),
    R2_TASK_ARM(2U, -5, 1U),
    R2_TASK_ARM(2U, -1, 1U),
    R2_TASK_ARM(3U, 1, 1U),
    R2_TASK_ARM(3U, 1, 1U),
    R2_TASK_ARM(2U, 5, 1U),
    R2_TASK_ARM(3U, 1, 1U),
    R2_TASK_ARM(3U, 1, 1U),

    /* Host stages pause until the matching host confirmation command arrives. */
    R2_TASK_ARM(2U, -1, 1U),
    R2_TASK_ARM(3U, 1, 2U),
    R2_TASK_HOST_CHECKPOINT(1U), /* chassis move */
    R2_TASK_TOOL_OFF(ROBOTARM_TOOL_GRIPPER, 1U),
    R2_TASK_ARM_WORKSPACE(ROBOTARM_WORK_DIR_X_NEG),
    R2_TASK_ARM(3U, -90, 1U),
    R2_TASK_ARM(3U, -5, 1U),
    R2_TASK_ARM(3U, 1, 4U),
    R2_TASK_HOST_CHECKPOINT(2U), /* dock-complete decision */
    R2_TASK_TOOL_ON(ROBOTARM_TOOL_GRIPPER, 1U),
    R2_TASK_ARM_ENABLE_HOME(),
};

static const R2_TaskFlowDesc_t s_task_flow_descs[] = {
    {R2_TASK_FLOW_S1_UP_V1,
     s_task_flow_s1_up_v1,
     R2_TASK_FLOW_ARRAY_LEN(s_task_flow_s1_up_v1)},
    {R2_TASK_FLOW_S1_DOWN_V1,
     s_task_flow_s1_down_v1,
     R2_TASK_FLOW_ARRAY_LEN(s_task_flow_s1_down_v1)},
    {R2_TASK_FLOW_S1_UP_S2_DOWN_V1,
     s_task_flow_s1_up_s2_down_v1,
     R2_TASK_FLOW_ARRAY_LEN(s_task_flow_s1_up_s2_down_v1)},
    {R2_TASK_FLOW_S1_DOWN_S2_UP_V1,
     s_task_flow_s1_down_s2_up_v1,
     R2_TASK_FLOW_ARRAY_LEN(s_task_flow_s1_down_s2_up_v1)},
    {R2_TASK_FLOW_S1_DOWN_S2_DOWN_V1,
     s_task_flow_s1_down_s2_down_v1,
     R2_TASK_FLOW_ARRAY_LEN(s_task_flow_s1_down_s2_down_v1)},
    {R2_TASK_FLOW_WEAPON_GRAB_V1,
     s_task_flow_weapon_grab_v1,
     R2_TASK_FLOW_ARRAY_LEN(s_task_flow_weapon_grab_v1)},
    {R2_TASK_FLOW_THROW_BLOCK_V1,
     s_task_flow_throw_block_v1,
     R2_TASK_FLOW_ARRAY_LEN(s_task_flow_throw_block_v1)},
    {R2_TASK_FLOW_WEAPON_DOCK_TEST_V1,
     s_task_flow_weapon_dock_test_v1,
     R2_TASK_FLOW_ARRAY_LEN(s_task_flow_weapon_dock_test_v1)},
};

static const R2_TaskFlowDesc_t *R2_TaskFlow_FindDesc(uint8_t flow_id)
{
    uint16_t i;

    for (i = 0U; i < R2_TASK_FLOW_ARRAY_LEN(s_task_flow_descs); i++)
    {
        if (s_task_flow_descs[i].id == flow_id)
        {
            return &s_task_flow_descs[i];
        }
    }

    return NULL;
}

static uint8_t R2_TaskFlow_RequiredS1End(uint8_t flow_id)
{
    switch ((R2_TaskFlowId_t)flow_id)
    {
    case R2_TASK_FLOW_S1_UP_S2_DOWN_V1:
        return (uint8_t)R2_TASK_FLOW_S1_END_UP;

    case R2_TASK_FLOW_S1_DOWN_S2_UP_V1:
    case R2_TASK_FLOW_S1_DOWN_S2_DOWN_V1:
        return (uint8_t)R2_TASK_FLOW_S1_END_DOWN;

    default:
        break;
    }

    return (uint8_t)R2_TASK_FLOW_S1_END_NONE;
}

static void R2_TaskFlow_MarkComplete(R2_TaskFlow_Ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    switch ((R2_TaskFlowId_t)ctrl->flow_id)
    {
    case R2_TASK_FLOW_S1_UP_V1:
        ctrl->completed_s1_end = (uint8_t)R2_TASK_FLOW_S1_END_UP;
        break;

    case R2_TASK_FLOW_S1_DOWN_V1:
        ctrl->completed_s1_end = (uint8_t)R2_TASK_FLOW_S1_END_DOWN;
        break;

    case R2_TASK_FLOW_THROW_BLOCK_V1:
        ctrl->completed_s1_end = (uint8_t)R2_TASK_FLOW_S1_END_NONE;
        break;

    default:
        break;
    }
}

static const R2_TaskFlowEntry_t *R2_TaskFlow_CurrentEntry(
    const R2_TaskFlow_Ctrl_t *ctrl)
{
    const R2_TaskFlowDesc_t *desc;

    if (ctrl == NULL)
    {
        return NULL;
    }

    desc = R2_TaskFlow_FindDesc(ctrl->flow_id);
    if ((desc == NULL) || (ctrl->entry_index >= desc->count))
    {
        return NULL;
    }

    return &desc->entries[ctrl->entry_index];
}

void R2_TaskFlow_Init(R2_TaskFlow_Ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->state = (uint8_t)R2_TASK_FLOW_STATE_IDLE;
    ctrl->completed_s1_end = (uint8_t)R2_TASK_FLOW_S1_END_NONE;
}

void R2_TaskFlow_Stop(R2_TaskFlow_Ctrl_t *ctrl)
{
    uint8_t completed_s1_end;

    if (ctrl == NULL)
    {
        return;
    }

    completed_s1_end = ctrl->completed_s1_end;
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->state = (uint8_t)R2_TASK_FLOW_STATE_IDLE;
    ctrl->completed_s1_end = completed_s1_end;
}

void R2_TaskFlow_Request(R2_TaskFlow_Ctrl_t *ctrl,
                         uint8_t flow_id,
                         uint32_t now_ms)
{
    R2_TaskFlow_RequestWithArg(ctrl, flow_id, 0U, now_ms);
}

void R2_TaskFlow_RequestWithArg(R2_TaskFlow_Ctrl_t *ctrl,
                                uint8_t flow_id,
                                uint8_t flow_arg,
                                uint32_t now_ms)
{
    const R2_TaskFlowDesc_t *desc;
    uint8_t required_s1_end;

    if (ctrl == NULL)
    {
        return;
    }

    desc = R2_TaskFlow_FindDesc(flow_id);
    if (desc == NULL)
    {
        R2_TaskFlow_Stop(ctrl);
        ctrl->flow_id = flow_id;
        ctrl->state = (uint8_t)R2_TASK_FLOW_STATE_ERROR;
        ctrl->error = (uint8_t)R2_TASK_FLOW_ERR_BAD_FLOW;
        ctrl->step_start_ms = now_ms;
        ctrl->last_update_ms = now_ms;
        return;
    }

    if ((flow_id == (uint8_t)R2_TASK_FLOW_THROW_BLOCK_V1) &&
        (flow_arg != (uint8_t)ROBOTARM_WORK_DIR_X_POS) &&
        (flow_arg != (uint8_t)ROBOTARM_WORK_DIR_X_NEG))
    {
        R2_TaskFlow_Stop(ctrl);
        ctrl->flow_id = flow_id;
        ctrl->flow_arg = flow_arg;
        ctrl->entry_count = desc->count;
        ctrl->state = (uint8_t)R2_TASK_FLOW_STATE_ERROR;
        ctrl->error = (uint8_t)R2_TASK_FLOW_ERR_BAD_FLOW;
        ctrl->step_start_ms = now_ms;
        ctrl->last_update_ms = now_ms;
        return;
    }

    if ((flow_id == (uint8_t)R2_TASK_FLOW_THROW_BLOCK_V1) &&
        (ctrl->completed_s1_end == (uint8_t)R2_TASK_FLOW_S1_END_NONE))
    {
        R2_TaskFlow_Stop(ctrl);
        ctrl->flow_id = flow_id;
        ctrl->flow_arg = flow_arg;
        ctrl->entry_count = desc->count;
        ctrl->state = (uint8_t)R2_TASK_FLOW_STATE_ERROR;
        ctrl->error = (uint8_t)R2_TASK_FLOW_ERR_PRECONDITION;
        ctrl->step_start_ms = now_ms;
        ctrl->last_update_ms = now_ms;
        return;
    }

    required_s1_end = R2_TaskFlow_RequiredS1End(flow_id);
    if ((required_s1_end != (uint8_t)R2_TASK_FLOW_S1_END_NONE) &&
        (ctrl->completed_s1_end != required_s1_end))
    {
        R2_TaskFlow_Stop(ctrl);
        ctrl->flow_id = flow_id;
        ctrl->entry_count = desc->count;
        ctrl->state = (uint8_t)R2_TASK_FLOW_STATE_ERROR;
        ctrl->error = (uint8_t)R2_TASK_FLOW_ERR_PRECONDITION;
        ctrl->step_start_ms = now_ms;
        ctrl->last_update_ms = now_ms;
        return;
    }

    if ((flow_id == (uint8_t)R2_TASK_FLOW_S1_UP_V1) ||
        (flow_id == (uint8_t)R2_TASK_FLOW_S1_DOWN_V1) ||
        (required_s1_end != (uint8_t)R2_TASK_FLOW_S1_END_NONE))
    {
        ctrl->completed_s1_end = (uint8_t)R2_TASK_FLOW_S1_END_NONE;
    }

    ctrl->flow_id = flow_id;
    ctrl->state = (uint8_t)R2_TASK_FLOW_STATE_ISSUE;
    ctrl->error = (uint8_t)R2_TASK_FLOW_ERR_NONE;
    ctrl->current_op = (uint8_t)R2_TASK_FLOW_OP_NONE;
    ctrl->entry_index = 0U;
    ctrl->entry_count = desc->count;
    ctrl->repeat_index = 0U;
    ctrl->repeat_count = 0U;
    ctrl->flow_arg = flow_arg;
    ctrl->arm_reached_ms = 0U;
    ctrl->step_start_ms = now_ms;
    ctrl->last_update_ms = now_ms;
    ctrl->completed_steps = 0U;
    ctrl->host_checkpoint_pending = 0U;
    ctrl->host_checkpoint_ack = 0U;
}

void R2_TaskFlow_ConfirmHostCheckpoint(R2_TaskFlow_Ctrl_t *ctrl,
                                       uint8_t checkpoint)
{
    const R2_TaskFlowEntry_t *entry;

    if ((ctrl == NULL) || (checkpoint == 0U))
    {
        return;
    }

    if ((ctrl->state != (uint8_t)R2_TASK_FLOW_STATE_WAIT) ||
        (ctrl->current_op != (uint8_t)R2_TASK_FLOW_OP_HOST_CHECKPOINT))
    {
        return;
    }

    entry = R2_TaskFlow_CurrentEntry(ctrl);
    if ((entry == NULL) ||
        (entry->op != (uint8_t)R2_TASK_FLOW_OP_HOST_CHECKPOINT) ||
        (entry->arg != checkpoint))
    {
        return;
    }

    ctrl->host_checkpoint_ack = checkpoint;
}

uint8_t R2_TaskFlow_IsActive(const R2_TaskFlow_Ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
        return 0U;
    }

    return ((ctrl->state == (uint8_t)R2_TASK_FLOW_STATE_ISSUE) ||
             (ctrl->state == (uint8_t)R2_TASK_FLOW_STATE_WAIT)) ? 1U : 0U;
}

static RobotArmWorkDirection_t R2_TaskFlow_WorkspaceDirection(
    const R2_TaskFlowEntry_t *entry,
    const R2_TaskFlow_Ctrl_t *ctrl)
{
    if ((entry != NULL) &&
        (entry->arg <= (uint8_t)ROBOTARM_WORK_DIR_X_NEG))
    {
        return (RobotArmWorkDirection_t)entry->arg;
    }

    return (ctrl != NULL) ?
        (RobotArmWorkDirection_t)ctrl->flow_arg :
        ROBOTARM_WORK_DIR_Y_POS;
}

static uint8_t R2_TaskFlow_IssueEntry(const R2_TaskFlowEntry_t *entry,
                                      R2_TaskFlow_Ctrl_t *ctrl,
                                      R2_Climb_Ctrl_t *climb,
                                      R2_Arm_Ctrl_t *arm,
                                      uint32_t now_ms)
{
    if ((entry == NULL) || (ctrl == NULL) ||
        (climb == NULL) || (arm == NULL))
    {
        return (uint8_t)R2_TASK_FLOW_ERR_BAD_FLOW;
    }

    switch ((R2_TaskFlowOp_t)entry->op)
    {
    case R2_TASK_FLOW_OP_ARM_ENABLE_HOME:
        R2_Arm_Enable(arm);
        return (uint8_t)R2_TASK_FLOW_ERR_NONE;

    case R2_TASK_FLOW_OP_CLIMB_GATE:
        R2_Climb_RequestFlowGate(climb, entry->arg);
        return (uint8_t)R2_TASK_FLOW_ERR_NONE;

    case R2_TASK_FLOW_OP_CLIMB_AUTO_PAUSE:
        R2_Climb_RequestFlowAutoPause(climb, entry->arg);
        return (uint8_t)R2_TASK_FLOW_ERR_NONE;

    case R2_TASK_FLOW_OP_CLIMB_AUTO_RESUME:
        R2_Climb_RequestAutoResume(climb);
        return (uint8_t)R2_TASK_FLOW_ERR_NONE;

    case R2_TASK_FLOW_OP_ARM_WORKSPACE_SWITCH:
        if (R2_Arm_SetWorkDirection(
                arm,
                R2_TaskFlow_WorkspaceDirection(entry, ctrl),
                now_ms) != ROBOTARM_IK_OK)
        {
            return (uint8_t)R2_TASK_FLOW_ERR_ARM;
        }
        return (uint8_t)R2_TASK_FLOW_ERR_NONE;

    case R2_TASK_FLOW_OP_TOOL_OFF_HELD:
        if ((ctrl->completed_s1_end == (uint8_t)R2_TASK_FLOW_S1_END_UP) ||
            (ctrl->completed_s1_end == (uint8_t)R2_TASK_FLOW_S1_END_DOWN))
        {
            R2_Arm_SetToolActuator(ROBOTARM_TOOL_S2, 0U);
        }
        else
        {
            return (uint8_t)R2_TASK_FLOW_ERR_PRECONDITION;
        }
        ctrl->completed_s1_end = (uint8_t)R2_TASK_FLOW_S1_END_NONE;
        return (uint8_t)R2_TASK_FLOW_ERR_NONE;

    case R2_TASK_FLOW_OP_CLIMB_ACTION:
        R2_Climb_RequestTestAction(climb, entry->arg);
        return (uint8_t)R2_TASK_FLOW_ERR_NONE;

    case R2_TASK_FLOW_OP_ARM_JOG:
        if (arm->enabled == 0U)
        {
            R2_Arm_Enable(arm);
        }
        if (R2_Arm_JogJointActual(arm,
                                  entry->arg,
                                  (float)entry->value,
                                  now_ms) != ROBOTARM_IK_OK)
        {
            return (uint8_t)R2_TASK_FLOW_ERR_ARM;
        }
        return (uint8_t)R2_TASK_FLOW_ERR_NONE;

    case R2_TASK_FLOW_OP_TOOL_SET:
        R2_Arm_SetToolActuator((RobotArmTool_t)entry->arg,
                              (entry->value != 0) ? 1U : 0U);
        return (uint8_t)R2_TASK_FLOW_ERR_NONE;

    case R2_TASK_FLOW_OP_HOST_CHECKPOINT:
        ctrl->host_checkpoint_pending = entry->arg;
        ctrl->host_checkpoint_ack = 0U;
        return (uint8_t)R2_TASK_FLOW_ERR_NONE;

    case R2_TASK_FLOW_OP_POSTURE_CHECK:
        if (arm->state == (uint8_t)R2_ARM_STATE_ERROR)
        {
            return (uint8_t)R2_TASK_FLOW_ERR_POSTURE;
        }
        return (uint8_t)R2_TASK_FLOW_ERR_NONE;

    default:
        break;
    }

    return (uint8_t)R2_TASK_FLOW_ERR_BAD_FLOW;
}

static uint8_t R2_TaskFlow_WaitEntryDone(const R2_TaskFlowEntry_t *entry,
                                         R2_TaskFlow_Ctrl_t *ctrl,
                                         const R2_Climb_Ctrl_t *climb,
                                         const R2_Arm_Ctrl_t *arm,
                                         uint32_t elapsed_ms,
                                         uint8_t *error)
{
    if (error != NULL)
    {
        *error = (uint8_t)R2_TASK_FLOW_ERR_NONE;
    }

    if ((entry == NULL) || (ctrl == NULL) ||
        (climb == NULL) || (arm == NULL))
    {
        if (error != NULL)
        {
            *error = (uint8_t)R2_TASK_FLOW_ERR_BAD_FLOW;
        }
        return 0U;
    }

    switch ((R2_TaskFlowOp_t)entry->op)
    {
    case R2_TASK_FLOW_OP_ARM_ENABLE_HOME:
        if (arm->state == (uint8_t)R2_ARM_STATE_ERROR)
        {
            if (error != NULL)
            {
                *error = (uint8_t)R2_TASK_FLOW_ERR_ARM;
            }
            return 0U;
        }
        return ((arm->enable_startup_active == 0U) &&
                (elapsed_ms >= R2_TASK_FLOW_ARM_ENABLE_SETTLE_MS)) ? 1U : 0U;

    case R2_TASK_FLOW_OP_CLIMB_GATE:
        if (climb->state == R2_CLIMB_STATE_ERROR)
        {
            if (error != NULL)
            {
                *error = (uint8_t)R2_TASK_FLOW_ERR_CLIMB;
            }
            return 0U;
        }
        return ((climb->gate_active == 0U) &&
                (climb->state_done != 0U)) ? 1U : 0U;

    case R2_TASK_FLOW_OP_CLIMB_AUTO_PAUSE:
        if (climb->state == R2_CLIMB_STATE_ERROR)
        {
            if (error != NULL)
            {
                *error = (uint8_t)R2_TASK_FLOW_ERR_CLIMB;
            }
            return 0U;
        }
        return ((climb->auto_pause_active != 0U) &&
                (climb->state_done != 0U)) ? 1U : 0U;

    case R2_TASK_FLOW_OP_CLIMB_AUTO_RESUME:
        if (climb->state == R2_CLIMB_STATE_ERROR)
        {
            if (error != NULL)
            {
                *error = (uint8_t)R2_TASK_FLOW_ERR_CLIMB;
            }
            return 0U;
        }
        return ((climb->auto_run == 0U) &&
                (climb->state == R2_CLIMB_STATE_DONE) &&
                (climb->state_done != 0U)) ? 1U : 0U;

    case R2_TASK_FLOW_OP_ARM_WORKSPACE_SWITCH:
        if (arm->state == (uint8_t)R2_ARM_STATE_ERROR)
        {
            if (error != NULL)
            {
                *error = (uint8_t)R2_TASK_FLOW_ERR_ARM;
            }
            return 0U;
        }
        return ((arm->workspace_switch_active == 0U) &&
                (arm->current_direction ==
                 R2_TaskFlow_WorkspaceDirection(entry, ctrl)) &&
                (elapsed_ms >= R2_TASK_FLOW_POSTURE_SETTLE_MS)) ? 1U : 0U;

    case R2_TASK_FLOW_OP_TOOL_OFF_HELD:
        return (elapsed_ms >= R2_TASK_FLOW_TOOL_SETTLE_MS) ? 1U : 0U;

    case R2_TASK_FLOW_OP_CLIMB_ACTION:
        if (climb->state == R2_CLIMB_STATE_ERROR)
        {
            if (error != NULL)
            {
                *error = (uint8_t)R2_TASK_FLOW_ERR_CLIMB;
            }
            return 0U;
        }
        return ((climb->test_active == 0U) &&
                (climb->state_done != 0U)) ? 1U : 0U;

    case R2_TASK_FLOW_OP_ARM_JOG:
        if (arm->state == (uint8_t)R2_ARM_STATE_ERROR)
        {
            if (error != NULL)
            {
                *error = (uint8_t)R2_TASK_FLOW_ERR_ARM;
            }
            return 0U;
        }
        if (elapsed_ms >= R2_TASK_FLOW_ARM_TIMEOUT_MS)
        {
            if (error != NULL)
            {
                *error = (uint8_t)R2_TASK_FLOW_ERR_ARM;
            }
            return 0U;
        }
        if ((elapsed_ms >= R2_TASK_FLOW_ARM_MIN_WAIT_MS) &&
            (R2_Arm_IsJointTargetReached(
                arm,
                entry->arg,
                R2_TASK_FLOW_ARM_TOLERANCE_DEG) != 0U))
        {
            if (ctrl->arm_reached_ms < R2_TASK_FLOW_ARM_REACHED_MS)
            {
                ctrl->arm_reached_ms++;
            }
        }
        else
        {
            ctrl->arm_reached_ms = 0U;
        }
        return (ctrl->arm_reached_ms >=
                R2_TASK_FLOW_ARM_REACHED_MS) ? 1U : 0U;

    case R2_TASK_FLOW_OP_TOOL_SET:
        return (elapsed_ms >= R2_TASK_FLOW_TOOL_SETTLE_MS) ? 1U : 0U;

    case R2_TASK_FLOW_OP_HOST_CHECKPOINT:
        (void)elapsed_ms;
        return ((ctrl->host_checkpoint_pending == entry->arg) &&
                (ctrl->host_checkpoint_ack == entry->arg)) ? 1U : 0U;

    case R2_TASK_FLOW_OP_POSTURE_CHECK:
        if (arm->state == (uint8_t)R2_ARM_STATE_ERROR)
        {
            if (error != NULL)
            {
                *error = (uint8_t)R2_TASK_FLOW_ERR_POSTURE;
            }
            return 0U;
        }
        return (elapsed_ms >= R2_TASK_FLOW_POSTURE_SETTLE_MS) ? 1U : 0U;

    default:
        break;
    }

    if (error != NULL)
    {
        *error = (uint8_t)R2_TASK_FLOW_ERR_BAD_FLOW;
    }
    return 0U;
}

static void R2_TaskFlow_Update(R2_TaskFlow_Ctrl_t *ctrl,
                               R2_Climb_Ctrl_t *climb,
                               R2_Arm_Ctrl_t *arm,
                               uint32_t now_ms)
{
    const R2_TaskFlowEntry_t *entry;
    uint32_t elapsed_ms;
    uint8_t error;

    if (ctrl == NULL)
    {
        return;
    }

    ctrl->last_update_ms = now_ms;

    if (ctrl->state == (uint8_t)R2_TASK_FLOW_STATE_ISSUE)
    {
        entry = R2_TaskFlow_CurrentEntry(ctrl);
        if (entry == NULL)
        {
            ctrl->state = (uint8_t)R2_TASK_FLOW_STATE_DONE;
            ctrl->current_op = (uint8_t)R2_TASK_FLOW_OP_NONE;
            R2_TaskFlow_MarkComplete(ctrl);
            return;
        }

        ctrl->repeat_count = (entry->repeat != 0U) ? entry->repeat : 1U;
        ctrl->current_op = entry->op;
        ctrl->step_start_ms = now_ms;
        ctrl->arm_reached_ms = 0U;
        error = R2_TaskFlow_IssueEntry(entry, ctrl, climb, arm, now_ms);
        if (error != (uint8_t)R2_TASK_FLOW_ERR_NONE)
        {
            ctrl->state = (uint8_t)R2_TASK_FLOW_STATE_ERROR;
            ctrl->error = error;
            return;
        }

        ctrl->state = (uint8_t)R2_TASK_FLOW_STATE_WAIT;
        return;
    }

    if (ctrl->state != (uint8_t)R2_TASK_FLOW_STATE_WAIT)
    {
        return;
    }

    entry = R2_TaskFlow_CurrentEntry(ctrl);
    if (entry == NULL)
    {
        ctrl->state = (uint8_t)R2_TASK_FLOW_STATE_DONE;
        ctrl->current_op = (uint8_t)R2_TASK_FLOW_OP_NONE;
        R2_TaskFlow_MarkComplete(ctrl);
        return;
    }

    elapsed_ms = now_ms - ctrl->step_start_ms;
    if (R2_TaskFlow_WaitEntryDone(entry,
                                  ctrl,
                                  climb,
                                  arm,
                                  elapsed_ms,
                                  &error) == 0U)
    {
        if (error != (uint8_t)R2_TASK_FLOW_ERR_NONE)
        {
            ctrl->state = (uint8_t)R2_TASK_FLOW_STATE_ERROR;
            ctrl->error = error;
        }
        return;
    }

    ctrl->completed_steps++;
    if (entry->op == (uint8_t)R2_TASK_FLOW_OP_HOST_CHECKPOINT)
    {
        ctrl->host_checkpoint_pending = 0U;
        ctrl->host_checkpoint_ack = 0U;
    }
    ctrl->repeat_index++;
    if (ctrl->repeat_index >= ctrl->repeat_count)
    {
        ctrl->entry_index++;
        ctrl->repeat_index = 0U;
        ctrl->repeat_count = 0U;
    }

    if (ctrl->entry_index >= ctrl->entry_count)
    {
        ctrl->state = (uint8_t)R2_TASK_FLOW_STATE_DONE;
        ctrl->current_op = (uint8_t)R2_TASK_FLOW_OP_NONE;
        R2_TaskFlow_MarkComplete(ctrl);
    }
    else
    {
        ctrl->state = (uint8_t)R2_TASK_FLOW_STATE_ISSUE;
    }
}

void Mecanum_task_USB(ChassisVel_t *chassis_user,
                      MecanumParam_t *param_user,
                      WheelSpeed_t *speed_user)
{
    Mecanum_Calc(chassis_user, param_user, speed_user);
}

void Control_Task(void const *argument)
{
    (void)argument;

    osDelay(3000);

    MX_USB_DEVICE_Init();
    HAL_UART_Receive_IT(&huart10, &btReceiveData, 1U);

    MCU_Init();

    R2_Move_Init(&g_r2_ctrl_usart, &mecParam, 0.001f);
    R2_Move_Init(&g_r2_ctrl_usb, &mecParam, 0.001f);
    R2_Climb_Init(&g_r2_climb_usart);
    R2_Climb_Init(&g_r2_climb_usb);
    R2_Arm_Init(&g_r2_arm_usb);
    R2_TaskFlow_Init(&g_r2_task_flow_usb);
    R2_YawAutoTune_Init();
    

    s_control_task_handle = xTaskGetCurrentTaskHandle();
//    HAL_TIM_Base_Start_IT(&htim3);
	  HAL_TIM_Base_Start_IT(&htim5);
	
#if defined(R2_ARM_ENABLE_POWERON_FDCAN_TEST) && (R2_ARM_ENABLE_POWERON_FDCAN_TEST != 0)
    RobotArm_FDCAN3_TestMain(0.0f, 0.0f, 0.0f);
#endif


    for (;;)
    {
        uint32_t pending_ticks = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));

        if (pending_ticks == 0U)
        {
            pending_ticks = 1U;
        }

        while (pending_ticks != 0U)
        {
            R2_Control_1msStep();
            pending_ticks--;
        }

        BT_Data_MAC_Process(&total_vel.vx, &total_vel.vy, &total_vel.vw, NULL);

		}
}

static void R2_Control_1msStep(void)
{
    float now_sec;
    float w_delta[CHASSIS_MOTOR_COUNT];
    float robot_dx;
    float robot_dy;
    float robot_dyaw;
    int32_t cur_enc[CHASSIS_MOTOR_COUNT];
    int32_t delta;
    uint8_t i;
    float imu_yaw_rad;
    float robot_vx_mps = 0.0f;
    float robot_vy_mps = 0.0f;
    float odom_vx_mps = 0.0f;
    float odom_vy_mps = 0.0f;
    float odom_wz_radps = 0.0f;
    R2_Move_Ctrl_t *active_ctrl;
    INS_NavState_t ins_state;
    uint8_t imu_yaw_valid;
    uint8_t climb_debug_source = R2_CLIMB_DEBUG_SOURCE_NONE;

    now_sec = (float)g_r2_tick_ms * 0.001f;
    g_r2_debug_odom.tick_ms = g_r2_tick_ms;
    g_r2_tick_ms++;

    INS_GetState(&ins_state);
    imu_yaw_rad = ins_state.yaw_total_rad;
    imu_yaw_valid = ins_state.imu_online;
    g_r2_debug_odom.imu_yaw_rad = imu_yaw_rad;
    g_r2_debug_odom.imu_yaw_valid = imu_yaw_valid;
    if (imu_yaw_valid != 0U)
    {
        R2_Move_UpdateYaw(&g_r2_ctrl_usart, imu_yaw_rad);
        R2_Move_UpdateYaw(&g_r2_ctrl_usb, imu_yaw_rad);
    }

    for (i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
    {
        cur_enc[i] = motor_fdcan1[i].total_angle;
        g_r2_debug_odom.current_enc[i] = cur_enc[i];
        g_r2_debug_odom.last_enc[i] = g_r2_last_enc[i];
    }

    if (g_r2_enc_inited != 0U)
    {
        for (i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
        {
            delta = cur_enc[i] - g_r2_last_enc[i];
            w_delta[i] = EncoderDeltaToWheelMeter(delta);
            g_r2_debug_odom.enc_delta[i] = delta;
            g_r2_debug_odom.wheel_delta_m[i] = w_delta[i];
        }

        {
            float fr_delta = w_delta[CHASSIS_MOTOR_FR];
            float br_delta = w_delta[CHASSIS_MOTOR_BR];
            float bl_delta = w_delta[CHASSIS_MOTOR_BL];
            float fl_delta = w_delta[CHASSIS_MOTOR_FL];

            robot_dx = MEC_RIGHT_SIGN *
                       (+fr_delta - br_delta - bl_delta + fl_delta) * 0.25f;
            robot_dy = MEC_FORWARD_SIGN *
                       (-fr_delta - br_delta + bl_delta + fl_delta) * 0.25f;
            robot_dyaw = -(fr_delta + br_delta + bl_delta + fl_delta)
                         * 0.25f / (mecParam.L + mecParam.W);
        }

        robot_vx_mps = robot_dx * 1000.0f;
        robot_vy_mps = robot_dy * 1000.0f;
        odom_wz_radps = robot_dyaw * 1000.0f;
        g_r2_debug_odom.robot_dx_m = robot_dx;
        g_r2_debug_odom.robot_dy_m = robot_dy;
        g_r2_debug_odom.robot_dyaw_rad = robot_dyaw;
        g_r2_debug_odom.odom_wz_radps = odom_wz_radps;

        R2_Move_UpdateOdom(&g_r2_ctrl_usart, robot_dx, robot_dy, robot_dyaw);
        R2_Move_UpdateOdom(&g_r2_ctrl_usb, robot_dx, robot_dy, robot_dyaw);

        if (imu_yaw_valid != 0U)
        {
            R2_Move_UpdateYaw(&g_r2_ctrl_usart, imu_yaw_rad);
            R2_Move_UpdateYaw(&g_r2_ctrl_usb, imu_yaw_rad);
        }
    }
    else
    {
        g_r2_enc_inited = 1U;
        for (i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
        {
            g_r2_debug_odom.enc_delta[i] = 0;
            g_r2_debug_odom.wheel_delta_m[i] = 0.0f;
        }
        g_r2_debug_odom.robot_dx_m = 0.0f;
        g_r2_debug_odom.robot_dy_m = 0.0f;
        g_r2_debug_odom.robot_dyaw_rad = 0.0f;
        g_r2_debug_odom.odom_vx_mps = 0.0f;
        g_r2_debug_odom.odom_vy_mps = 0.0f;
        g_r2_debug_odom.odom_wz_radps = 0.0f;
    }
    g_r2_debug_odom.enc_inited = g_r2_enc_inited;

    for (i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
    {
        g_r2_last_enc[i] = cur_enc[i];
    }

    USB_ControlWatchdog_Check();
    USART_ControlWatchdog_Check();

    R2_YawAutoTune_Step(&g_r2_ctrl_usb, g_r2_tick_ms);

    R2_Move_Update(&g_r2_ctrl_usart, now_sec);
    R2_Move_Update(&g_r2_ctrl_usb, now_sec);
    R2_Climb_Update(&g_r2_climb_usart, &g_r2_ctrl_usart, g_r2_tick_ms);
    R2_Climb_Update(&g_r2_climb_usb, &g_r2_ctrl_usb, g_r2_tick_ms);
    R2_Arm_Update(&g_r2_arm_usb, g_r2_tick_ms);
    R2_TaskFlow_Update(&g_r2_task_flow_usb,
                       &g_r2_climb_usb,
                       &g_r2_arm_usb,
                       g_r2_tick_ms);

    if (USART_Task_flag == 1U)
    {
        total_speed = g_r2_ctrl_usart.wheel_speed;
        total_vel = g_r2_ctrl_usart.robot_vel;
        active_ctrl = &g_r2_ctrl_usart;
        g_r2_debug_odom.active_source = CONTROL_SOURCE_USART;
        climb_debug_source = R2_CLIMB_DEBUG_SOURCE_USART;
    }
    else if (USB_Task_flag == 1U)
    {
        total_speed = g_r2_ctrl_usb.wheel_speed;
        total_vel = g_r2_ctrl_usb.robot_vel;
        active_ctrl = &g_r2_ctrl_usb;
        g_r2_debug_odom.active_source = CONTROL_SOURCE_USB;
        climb_debug_source = R2_CLIMB_DEBUG_SOURCE_USB;
    }
    else
    {
        active_ctrl = &g_r2_ctrl_usart;
        g_r2_debug_odom.active_source = CONTROL_SOURCE_NONE;
        climb_debug_source = R2_CLIMB_DEBUG_SOURCE_NONE;
    }

    R2_Climb_UpdateDebugViews(&g_r2_climb_usart,
                              &g_r2_climb_usb,
                              climb_debug_source);

    g_r2_debug_odom.active_odom_x = active_ctrl->odom_x;
    g_r2_debug_odom.active_odom_y = active_ctrl->odom_y;
    g_r2_debug_odom.active_odom_yaw = active_ctrl->odom_yaw;
    {
        float c = cosf(active_ctrl->odom_yaw);
        float s = sinf(active_ctrl->odom_yaw);
        odom_vx_mps = robot_vx_mps * c - robot_vy_mps * s;
        odom_vy_mps = robot_vx_mps * s + robot_vy_mps * c;
    }
    g_r2_debug_odom.odom_vx_mps = odom_vx_mps;
    g_r2_debug_odom.odom_vy_mps = odom_vy_mps;

    INS_SetOdometry(active_ctrl->odom_x,
                    active_ctrl->odom_y,
                    odom_vx_mps,
                    odom_vy_mps,
                    odom_wz_radps);
}

void control_tim1mscallback(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (s_control_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(s_control_task_handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
