#include "Data_Analysis.h"

#include "Control_Task.h"
#include "CRC.h"
#include "PC_TX_Task.h"
#include "R2_arm.h"
#include "R2_move.h"
#include "R2_yaw_autotune.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>

extern R2_Move_Ctrl_t g_r2_ctrl_usb;
extern R2_Climb_Ctrl_t g_r2_climb_usb;
extern volatile uint8_t USB_Task_flag;
extern volatile uint8_t USART_Task_flag;

static void USB_Read4Floats(const uint8_t *d, float *f);
static uint8_t USB_Read4FloatsChecked(const uint8_t *d, uint8_t len, float *f);
static uint8_t USB_AllowEmptyOrFloatPayload(uint8_t len);
static void USB_ChassisWatchdog_Feed(void);
static void USB_ChassisWatchdog_Disarm(void);
static void USB_CommandRx_Record(uint8_t cmd, const uint8_t *d, uint8_t len);
static float Clamp(float x, float lo, float hi);

uint8_t Mecanum_control_flag = 0U;

ChassisVel_t total_vel_USB = {0};
WheelSpeed_t total_speed_USB = {0};

static volatile uint32_t s_usb_chassis_last_tick = 0U;
static volatile uint8_t s_usb_chassis_watchdog_armed = 0U;
static volatile uint8_t s_usb_chassis_timeout = 0U;
static volatile uint32_t s_usb_cmd_last_tick = 0U;
static volatile uint32_t s_usb_cmd_count = 0U;
static volatile uint8_t s_usb_cmd_last_cmd = 0U;
static volatile uint8_t s_usb_cmd_last_len = 0U;
static volatile uint8_t s_usb_cmd_last_payload_valid = 0U;
static volatile uint8_t s_usb_cmd_last_data[16] = {0U};
static volatile float s_usb_cmd_last_f[4] = {0.0f};

void Data_Analysis(uint8_t cmd, const uint8_t *d, uint8_t len)
{
    float f[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t climb_auto_level = 0U;

    USB_CommandRx_Record(cmd, d, len);

    switch (cmd)
    {
    case USB_CMD_SYS_DISABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        USB_ChassisWatchdog_Disarm();
        taskENTER_CRITICAL();
        R2_YawAutoTune_Stop();
        R2_Move_Stop(&g_r2_ctrl_usb);
        R2_Climb_Stop(&g_r2_climb_usb);
        R2_Arm_Disable(&g_r2_arm_usb);
        taskEXIT_CRITICAL();
        Mecanum_control_flag = 0U;
        break;

    case USB_CMD_SYS_ENABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        taskENTER_CRITICAL();
        R2_Arm_Enable(&g_r2_arm_usb);
        taskEXIT_CRITICAL();
        Mecanum_control_flag = 1U;
        break;

    case USB_CMD_SYS_SWITCH_SOURCE:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        Control_SetSource(((uint8_t)f[0]) ? CONTROL_SOURCE_USB : CONTROL_SOURCE_USART);
        if (((uint8_t)f[0]) == 0U)
        {
            USB_ChassisWatchdog_Disarm();
            taskENTER_CRITICAL();
            R2_YawAutoTune_Stop();
            R2_Move_Stop(&g_r2_ctrl_usb);
            R2_Climb_Stop(&g_r2_climb_usb);
            R2_Arm_Disable(&g_r2_arm_usb);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_SYS_STOP:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        USB_ChassisWatchdog_Disarm();
        taskENTER_CRITICAL();
        R2_YawAutoTune_Stop();
        R2_Move_Stop(&g_r2_ctrl_usb);
        R2_Climb_Stop(&g_r2_climb_usb);
        R2_Arm_Stop(&g_r2_arm_usb);
        taskEXIT_CRITICAL();
        break;

    case USB_CMD_SYS_GET_STATUS:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        PC_TX_ReqSysStatus();
        break;

    case USB_CMD_CHS_DISABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        USB_ChassisWatchdog_Disarm();
        taskENTER_CRITICAL();
        R2_YawAutoTune_Stop();
        R2_Move_Stop(&g_r2_ctrl_usb);
        taskEXIT_CRITICAL();
        Mecanum_control_flag = 0U;
        break;

    case USB_CMD_CHS_ENABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        Mecanum_control_flag = 1U;
        break;

    case USB_CMD_CHS_SET_MODE:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        if (((uint8_t)f[0] <= 7U) && (USB_Task_flag != 0U) && (Mecanum_control_flag != 0U))
        {
            taskENTER_CRITICAL();
            R2_YawAutoTune_Stop();
            R2_Move_SetMode(&g_r2_ctrl_usb, (R2_MoveMode_t)((uint8_t)f[0]));
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CHS_SET_VEL:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        {
            uint8_t no_yaw_mode = R2_Move_IsNoYawMode(g_r2_ctrl_usb.mode);
            float yaw_data = f[2];

            f[0] = Clamp(f[0], MEC_REMOTE_VX_MIN_MPS, MEC_REMOTE_VX_MAX_MPS);
            f[1] = Clamp(f[1], MEC_REMOTE_VY_MIN_MPS, MEC_REMOTE_VY_MAX_MPS);
            if (no_yaw_mode == 0U)
            {
                f[2] = Clamp(f[2], MEC_REMOTE_VW_MIN_RAD_S, MEC_REMOTE_VW_MAX_RAD_S);
            }

            total_vel_USB.vx = f[0];
            total_vel_USB.vy = f[1];
            total_vel_USB.vw = (no_yaw_mode != 0U) ? 0.0f : f[2];

            if ((USB_Task_flag != 0U) && (Mecanum_control_flag != 0U))
            {
                taskENTER_CRITICAL();
                R2_YawAutoTune_Stop();
                if (no_yaw_mode != 0U)
                {
                    if (R2_Move_IsWorldMode(g_r2_ctrl_usb.mode))
                    {
                        R2_Move_SetWorldLockYaw(&g_r2_ctrl_usb, yaw_data * 0.0174533f);
                    }
                    else
                    {
                        R2_Move_SetRobotLockYaw(&g_r2_ctrl_usb, yaw_data * 0.0174533f);
                    }
                    R2_Move_SetVel(&g_r2_ctrl_usb, f[0], f[1], 0.0f);
                }
                else
                {
                    R2_Move_SetVel(&g_r2_ctrl_usb, f[0], f[1], f[2]);
                }
                taskEXIT_CRITICAL();
                USB_ChassisWatchdog_Feed();
            }
        }
        break;

    case USB_CMD_CHS_SET_POS:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        {
            uint8_t no_yaw_mode = R2_Move_IsNoYawMode(g_r2_ctrl_usb.mode);
            float yaw_data = f[2];
            int8_t set_result;

            total_vel_USB.vx = f[0];
            total_vel_USB.vy = f[1];
            total_vel_USB.vw = (no_yaw_mode != 0U) ? 0.0f : f[2];

            if ((USB_Task_flag != 0U) && (Mecanum_control_flag != 0U))
            {
                taskENTER_CRITICAL();
                R2_YawAutoTune_Stop();
                if (no_yaw_mode != 0U)
                {
                    if (R2_Move_IsWorldMode(g_r2_ctrl_usb.mode))
                    {
                        R2_Move_SetWorldLockYaw(&g_r2_ctrl_usb, yaw_data * 0.0174533f);
                    }
                    else
                    {
                        R2_Move_SetRobotLockYaw(&g_r2_ctrl_usb, yaw_data * 0.0174533f);
                    }
                    set_result = R2_Move_SetDist(&g_r2_ctrl_usb, f[0], f[1], 0.0f);
                }
                else
                {
                    set_result = R2_Move_SetDist(&g_r2_ctrl_usb, f[0], f[1], f[2]);
                }
                taskEXIT_CRITICAL();

                if (set_result == 0)
                {
                    USB_ChassisWatchdog_Disarm();
                }
            }
        }
        break;

    case USB_CMD_CHS_STOP:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        USB_ChassisWatchdog_Disarm();
        taskENTER_CRITICAL();
        R2_YawAutoTune_Stop();
        R2_Move_Stop(&g_r2_ctrl_usb);
        taskEXIT_CRITICAL();
        break;

    case USB_CMD_CHS_GET_STATUS:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        PC_TX_ReqChsStatus();
        break;

    case USB_CMD_ARM_DISABLE:
    case USB_CMD_TOOL_DISABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        taskENTER_CRITICAL();
        R2_Arm_Disable(&g_r2_arm_usb);
        taskEXIT_CRITICAL();
        break;

    case USB_CMD_ARM_ENABLE:
    case USB_CMD_TOOL_ENABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (USB_Task_flag != 0U)
        {
            taskENTER_CRITICAL();
            R2_Arm_Enable(&g_r2_arm_usb);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_ARM_SET_TARGET:
    case USB_CMD_TOOL_SET_MODE:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        if (USB_Task_flag != 0U)
        {
            taskENTER_CRITICAL();
            (void)R2_Arm_SetToolTarget(&g_r2_arm_usb,
                                       (RobotArmTool_t)((uint8_t)f[0]),
                                       (RobotArmToolState_t)((uint8_t)f[1]),
                                       f[2],
                                       f[3],
                                       HAL_GetTick());
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_ARM_STOP:
    case USB_CMD_TOOL_STOP:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        taskENTER_CRITICAL();
        R2_Arm_Stop(&g_r2_arm_usb);
        taskEXIT_CRITICAL();
        break;

    case USB_CMD_ARM_GET_STATUS:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        PC_TX_ReqArmStatus();
        break;

    case USB_CMD_TOOL_GET_STATUS:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        PC_TX_ReqToolStatus();
        break;

    case USB_CMD_ROBOT_GET_STATUS:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        PC_TX_ReqRobotStatus();
        break;

    case USB_CMD_YAW_TUNE_START:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if ((len == 16U) && (!USB_Read4FloatsChecked(d, len, f))) break;
        if ((USB_Task_flag != 0U) && (Mecanum_control_flag != 0U))
        {
            uint8_t pass_count = (uint8_t)f[0];
            uint8_t started;
            taskENTER_CRITICAL();
            started = R2_YawAutoTune_Start(&g_r2_ctrl_usb, pass_count);
            taskEXIT_CRITICAL();
            if (started != 0U)
            {
                USB_ChassisWatchdog_Disarm();
            }
        }
        break;

    case USB_CMD_YAW_TUNE_STOP:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        USB_ChassisWatchdog_Disarm();
        taskENTER_CRITICAL();
        R2_YawAutoTune_Stop();
        R2_Move_Stop(&g_r2_ctrl_usb);
        taskEXIT_CRITICAL();
        break;

    case USB_CMD_YAW_TUNE_GET_STATUS:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        PC_TX_ReqYawTuneStatus();
        break;

    case USB_CMD_CLIMB_DISABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        taskENTER_CRITICAL();
        if (g_r2_climb_usb.test_chassis_active != 0U)
        {
            R2_Move_Stop(&g_r2_ctrl_usb);
        }
        R2_Climb_Stop(&g_r2_climb_usb);
        taskEXIT_CRITICAL();
        break;

    case USB_CMD_CLIMB_ENABLE:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (USB_Task_flag != 0U)
        {
            taskENTER_CRITICAL();
            R2_Climb_SetInput(&g_r2_climb_usb, 1U, 0U, 0U);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_SET_CTRL:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        if (USB_Task_flag != 0U)
        {
            climb_auto_level = ((uint8_t)f[2]) ? 1U : 0U;
            taskENTER_CRITICAL();
            if ((((uint8_t)f[0]) == 0U) && (g_r2_climb_usb.test_chassis_active != 0U))
            {
                R2_Move_Stop(&g_r2_ctrl_usb);
            }
            R2_Climb_SetInput(&g_r2_climb_usb,
                              ((uint8_t)f[0]) ? 1U : 0U,
                              ((uint8_t)f[1]) ? 1U : 0U,
                              climb_auto_level);
            if ((((uint8_t)f[0]) != 0U) && (climb_auto_level != 0U))
            {
                USB_ChassisWatchdog_Disarm();
            }
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_STEP:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (USB_Task_flag != 0U)
        {
            taskENTER_CRITICAL();
            R2_Climb_RequestFlowStep(&g_r2_climb_usb, R2_CLIMB_FLOW_UPSTAIRS);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_UP_AUTO:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (USB_Task_flag != 0U)
        {
            taskENTER_CRITICAL();
            USB_ChassisWatchdog_Disarm();
            R2_Climb_RequestFlowAuto(&g_r2_climb_usb, R2_CLIMB_FLOW_UPSTAIRS);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_DOWN_STEP:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (USB_Task_flag != 0U)
        {
            taskENTER_CRITICAL();
            R2_Climb_RequestFlowStep(&g_r2_climb_usb, R2_CLIMB_FLOW_DOWNSTAIRS);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_DOWN_AUTO:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        if (USB_Task_flag != 0U)
        {
            taskENTER_CRITICAL();
            USB_ChassisWatchdog_Disarm();
            R2_Climb_RequestFlowAuto(&g_r2_climb_usb, R2_CLIMB_FLOW_DOWNSTAIRS);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_STOP:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        taskENTER_CRITICAL();
        if (g_r2_climb_usb.test_chassis_active != 0U)
        {
            R2_Move_Stop(&g_r2_ctrl_usb);
        }
        R2_Climb_Stop(&g_r2_climb_usb);
        taskEXIT_CRITICAL();
        break;

    case USB_CMD_CLIMB_TEST_ACTION:
        if (!USB_Read4FloatsChecked(d, len, f)) break;
        if (USB_Task_flag != 0U)
        {
            taskENTER_CRITICAL();
            if (g_r2_climb_usb.test_chassis_active != 0U)
            {
                R2_Move_Stop(&g_r2_ctrl_usb);
            }
            R2_Climb_RequestTestAction(&g_r2_climb_usb, (uint8_t)f[0]);
            taskEXIT_CRITICAL();
        }
        break;

    case USB_CMD_CLIMB_GET_STATUS:
        if (!USB_AllowEmptyOrFloatPayload(len)) break;
        PC_TX_ReqClimbStatus();
        break;

    default:
        break;
    }
}

static void USB_CommandRx_Record(uint8_t cmd, const uint8_t *d, uint8_t len)
{
    uint8_t i;
    uint8_t payload_valid;
    uint8_t copy_len = 0U;
    float f[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    payload_valid = ((d != NULL) && (len == 16U)) ? 1U : 0U;
    if (d != NULL)
    {
        copy_len = (len <= 16U) ? len : 16U;
    }
    if (payload_valid != 0U)
    {
        USB_Read4Floats(d, f);
    }

    taskENTER_CRITICAL();
    s_usb_cmd_last_tick = HAL_GetTick();
    s_usb_cmd_count++;
    s_usb_cmd_last_cmd = cmd;
    s_usb_cmd_last_len = len;
    s_usb_cmd_last_payload_valid = payload_valid;
    for (i = 0U; i < 16U; i++)
    {
        s_usb_cmd_last_data[i] = (i < copy_len) ? d[i] : 0U;
    }
    for (i = 0U; i < 4U; i++)
    {
        s_usb_cmd_last_f[i] = (payload_valid != 0U) ? f[i] : 0.0f;
    }
    taskEXIT_CRITICAL();
}

void USB_GetCommandRxState(USB_CommandRxState_t *out)
{
    uint8_t i;

    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    out->last_tick = s_usb_cmd_last_tick;
    out->count = s_usb_cmd_count;
    out->last_cmd = s_usb_cmd_last_cmd;
    out->last_len = s_usb_cmd_last_len;
    out->last_payload_valid = s_usb_cmd_last_payload_valid;
    out->reserved[0] = 0U;
    out->reserved[1] = 0U;
    out->reserved[2] = 0U;
    for (i = 0U; i < 16U; i++)
    {
        out->last_data[i] = s_usb_cmd_last_data[i];
    }
    for (i = 0U; i < 4U; i++)
    {
        out->last_f[i] = s_usb_cmd_last_f[i];
    }
    taskEXIT_CRITICAL();
}

static void USB_ChassisWatchdog_Feed(void)
{
    taskENTER_CRITICAL();
    s_usb_chassis_last_tick = HAL_GetTick();
    s_usb_chassis_watchdog_armed = 1U;
    s_usb_chassis_timeout = 0U;
    taskEXIT_CRITICAL();
}

static void USB_ChassisWatchdog_Disarm(void)
{
    taskENTER_CRITICAL();
    s_usb_chassis_watchdog_armed = 0U;
    s_usb_chassis_timeout = 0U;
    s_usb_chassis_last_tick = 0U;
    taskEXIT_CRITICAL();
}

void USB_ChassisWatchdog_Check(void)
{
    uint32_t now_tick;

    now_tick = HAL_GetTick();

    taskENTER_CRITICAL();
    if ((USB_Task_flag != 0U) &&
        (Mecanum_control_flag != 0U) &&
        (s_usb_chassis_watchdog_armed != 0U) &&
        (g_r2_ctrl_usb.emergency_stop == 0U) &&
        (R2_Move_IsVelMode(g_r2_ctrl_usb.mode) != 0U) &&
        ((now_tick - s_usb_chassis_last_tick) > USB_CHASSIS_TIMEOUT_MS))
    {
        R2_Move_Stop(&g_r2_ctrl_usb);
        total_vel_USB.vx = 0.0f;
        total_vel_USB.vy = 0.0f;
        total_vel_USB.vw = 0.0f;
        total_speed_USB.fl = 0.0f;
        total_speed_USB.fr = 0.0f;
        total_speed_USB.bl = 0.0f;
        total_speed_USB.br = 0.0f;
        s_usb_chassis_watchdog_armed = 0U;
        s_usb_chassis_timeout = 1U;
    }
    taskEXIT_CRITICAL();
}

void USB_ControlWatchdog_Check(void)
{
    USB_ChassisWatchdog_Check();
}

uint8_t USB_ChassisWatchdog_IsTimeout(void)
{
    uint8_t timeout;

    taskENTER_CRITICAL();
    timeout = s_usb_chassis_timeout;
    taskEXIT_CRITICAL();

    return timeout;
}

uint32_t USB_ChassisWatchdog_LastTick(void)
{
    uint32_t tick;

    taskENTER_CRITICAL();
    tick = s_usb_chassis_last_tick;
    taskEXIT_CRITICAL();

    return tick;
}

uint8_t USB_ControlWatchdog_TimeoutFlags(void)
{
    uint8_t flags = 0U;

    taskENTER_CRITICAL();
    if (s_usb_chassis_timeout != 0U) flags |= 0x01U;
    taskEXIT_CRITICAL();

    return flags;
}

static void USB_Read4Floats(const uint8_t *d, float *f)
{
    union { uint8_t b[4]; float v; } u;
    u.b[0] = d[0];  u.b[1] = d[1];  u.b[2] = d[2];  u.b[3] = d[3];  f[0] = u.v;
    u.b[0] = d[4];  u.b[1] = d[5];  u.b[2] = d[6];  u.b[3] = d[7];  f[1] = u.v;
    u.b[0] = d[8];  u.b[1] = d[9];  u.b[2] = d[10]; u.b[3] = d[11]; f[2] = u.v;
    u.b[0] = d[12]; u.b[1] = d[13]; u.b[2] = d[14]; u.b[3] = d[15]; f[3] = u.v;
}

static uint8_t USB_Read4FloatsChecked(const uint8_t *d, uint8_t len, float *f)
{
    if ((d == NULL) || (f == NULL) || (len != 16U))
    {
        return 0U;
    }

    USB_Read4Floats(d, f);
    return 1U;
}

static uint8_t USB_AllowEmptyOrFloatPayload(uint8_t len)
{
    return ((len == 0U) || (len == 16U)) ? 1U : 0U;
}

static float Clamp(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}
