#include "cmsis_os.h"
#include "Control_Task.h"
#include "Data_Analysis.h"
#include "R2_arm.h"
#include "R2_yaw_autotune.h"
#include "CRC.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

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
R2_DebugOdom_t g_r2_debug_odom;

uint32_t g_r2_tick_ms = 0U;
int32_t g_r2_last_enc[CHASSIS_MOTOR_COUNT];
uint8_t g_r2_enc_inited = 0U;

static void R2_Control_1msStep(void);

static TaskHandle_t s_control_task_handle = NULL;

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
