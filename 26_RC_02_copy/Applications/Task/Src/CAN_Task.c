#include "CAN_Task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid_user.h"

void CAN_Task(void const *argument)
{
    (void)argument;

    osDelay(5000);

    for (;;)
    {
        WheelSpeed_t wheel_cmd = {0};
        R2_Climb_Ctrl_t climb_snapshot;
        R2_ClimbMotorCmd_t climb_cmd;
        uint8_t has_source = 0U;
        uint8_t has_climb_source = 0U;
        uint8_t climb_debug_source = R2_CLIMB_DEBUG_SOURCE_NONE;
        float mecNum = MOTOR_IN2OUT * RPM_TO_MS;

        taskENTER_CRITICAL();
        if (USART_Task_flag == 1U)
        {
            wheel_cmd = g_r2_ctrl_usart.wheel_speed;
            climb_snapshot = g_r2_climb_usart;
            has_source = 1U;
            has_climb_source = 1U;
            climb_debug_source = R2_CLIMB_DEBUG_SOURCE_USART;
        }
        else if (USB_Task_flag == 1U)
        {
            wheel_cmd = g_r2_ctrl_usb.wheel_speed;
            climb_snapshot = g_r2_climb_usb;
            has_source = 1U;
            has_climb_source = 1U;
            climb_debug_source = R2_CLIMB_DEBUG_SOURCE_USB;
        }
        taskEXIT_CRITICAL();

        if (has_source != 0U)
        {
            FDCAN1_CMD_1(
                PID_velocity_realize_1(-wheel_cmd.fr / mecNum, 1),
                PID_velocity_realize_1(-wheel_cmd.br / mecNum, 2),
                PID_velocity_realize_1( wheel_cmd.bl / mecNum, 3),
                PID_velocity_realize_1( wheel_cmd.fl / mecNum, 4)
            );
        }
        else
        {
            FDCAN1_CMD_1(0, 0, 0, 0);
        }

        if (has_climb_source != 0U)
        {
            R2_Climb_GetMotorCurrent(&climb_snapshot, &climb_cmd);
        }
        else
        {
            climb_cmd.leg[0] = (int16_t)PID_velocity_realize_2(0.0f, 1);
            climb_cmd.leg[1] = (int16_t)PID_velocity_realize_2(0.0f, 2);
            climb_cmd.leg[2] = (int16_t)PID_velocity_realize_2(0.0f, 3);
            climb_cmd.leg[3] = (int16_t)PID_velocity_realize_2(0.0f, 4);
            climb_cmd.front_drive[0] = (int16_t)PID_velocity_realize_1(0.0f, 5);
            climb_cmd.front_drive[1] = (int16_t)PID_velocity_realize_1(0.0f, 6);
            climb_cmd.front_drive[2] = 0;
            climb_cmd.front_drive[3] = 0;
            climb_cmd.drive[0] = (int16_t)PID_velocity_realize_2(0.0f, 5);
            climb_cmd.drive[1] = (int16_t)PID_velocity_realize_2(0.0f, 6);
            climb_cmd.drive[2] = 0;
            climb_cmd.drive[3] = 0;
        }

        R2_Climb_SetDebugMotorCurrent(climb_debug_source, &climb_cmd);

        FDCAN1_CMD_2(
            climb_cmd.front_drive[0],
            climb_cmd.front_drive[1],
            0,
            0
        );

        FDCAN2_CMD_1(
            climb_cmd.leg[0],
            climb_cmd.leg[1],
            climb_cmd.leg[2],
            climb_cmd.leg[3]
        );

        FDCAN2_CMD_2(
            climb_cmd.drive[0],
            climb_cmd.drive[1],
            0,
            0
        );

        osDelay(1);
    }
}
