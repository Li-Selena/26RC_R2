#include "robot_def.h"
#include "include.h"
#include "tool_calc.h"
#include "FreeRTOS.h"
#include "queue.h"

void Task_Remote_Entry(void *argument)
{
       Chassis_Cmd_t chassis_cmd={0};
       Mech_Cmd_t mech_cmd={MODE_LOCK_ALL,0,0,0};

    Robot_Mode_e current_mode = MODE_LOCK_ALL;

    const TickType_t xFrequency = pdMS_TO_TICKS(10);
   TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
       /*×´Ì¬»ú*/
        if (rc.sw1 == RC_SW_DOWN) {
            if (rc.sw2 == RC_SW_DOWN)     current_mode = MODE_LOCK_ALL;
        }
        else if (rc.sw1 == RC_SW_MID) {
            if (rc.sw2 == RC_SW_MID)      current_mode = MODE_CHASSIS_FULL;
            else if (rc.sw2 == RC_SW_UP)  current_mode = MODE_RELAY_2D; 
        }
        else if (rc.sw1 == RC_SW_UP) {
            if (rc.sw2 == RC_SW_MID)       current_mode = MODE_SERVO_2D;
            else if (rc.sw2 == RC_SW_UP)   current_mode = MODE_SHANGWEIJI;
;
        }
        chassis_cmd.mode=current_mode;
        mech_cmd.mode = current_mode;
        
        switch(current_mode)
        {
            case MODE_CHASSIS_FULL:
                    chassis_cmd.vy = rc_process_channel(rc.ch0, MAX_SPEED_VY, RC_EXPO_RATIO, LPF_RC_ALPHA_VY, RC_CH_VY_IDX);
                    chassis_cmd.vx = rc_process_channel(rc.ch1, MAX_SPEED_VX, RC_EXPO_RATIO, LPF_RC_ALPHA_VX, RC_CH_VX_IDX);
                    chassis_cmd.omega = rc_process_channel(rc.ch2, MAX_SPEED_OMEGA, RC_EXPO_RATIO, LPF_RC_ALPHA_OMEGA, RC_CH_OMEGA_IDX); 
                    mech_cmd.ch3_raw=rc.ch3;mech_cmd.lift_speed=rc.roll; break;
            
            case MODE_SERVO_2D:
                  chassis_cmd.vy = rc_process_channel(rc.ch0, MAX_SPEED_VY, RC_EXPO_RATIO, LPF_RC_ALPHA_VY, RC_CH_VY_IDX);
                  chassis_cmd.vx = rc_process_channel(rc.ch1, MAX_SPEED_VX, RC_EXPO_RATIO, LPF_RC_ALPHA_VX, RC_CH_VX_IDX);
                  chassis_cmd.omega = 0;
                  mech_cmd.ch2_raw=rc.ch2; mech_cmd.ch3_raw=rc.ch3;mech_cmd.lift_speed=rc.roll; break;
            case MODE_RELAY_2D:
                 mech_cmd.ch2_raw=rc.ch2; mech_cmd.ch3_raw=rc.ch3;mech_cmd.lift_speed=rc.roll; break;
            case MODE_SHANGWEIJI:  break;                  
            case MODE_LOCK_ALL:
             default:
                chassis_cmd.vx = chassis_cmd.vy = chassis_cmd.omega = 0;mech_cmd.ch3_raw=0; mech_cmd.ch2_raw=0;break;
        }       
    xQueueOverwrite(Chassis_Cmd_QHandle, &chassis_cmd);
    xQueueOverwrite(Mech_Cmd_QHandle, &mech_cmd);
        
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

}

