#include "robot_def.h"
#include "include.h"

void Task_Mech_Entry(void *argument)
{
    Mech_Cmd_t cmd={MODE_LOCK_ALL,0,0,0};
   
    const TickType_t xFrequency = pdMS_TO_TICKS(3);
    TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {    
       
       if (osMessageQueueGet(Mech_Cmd_QHandle, &cmd, NULL, 0) == osOK)
        {
        
        switch (cmd.mode)
            {
            case MODE_SERVO_2D:/*Mech_Servo_Logic(cmd.ch2_raw,cmd.lift_speed); */Mech_PneuGripper3_Logic(cmd.ch2_raw,cmd.lift_speed);break;

            case MODE_RELAY_2D:Mech_Suction_Logic(cmd.ch3_raw,cmd.ch2_raw,cmd.lift_speed);break;

            case MODE_CHASSIS_FULL:Mech_Lift_Logic(cmd.ch3_raw,cmd.lift_speed);break;

            case MODE_LOCK_ALL:
            default:
                // 锁死/安全模式
                Pneumatic_Gripper_Ctrl(VALVE_OFF);
                Pneumatic_Suction_Ctrl(VALVE_OFF);
                can1_tx_table.target_current[4]=0;
                can1_tx_table.update_flag = 0;
                break;
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
