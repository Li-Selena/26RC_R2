#include "robot_def.h"
#include "include.h"
#include "recv_send.h"
#include "Imu_fusion.h"

void Task_Chassis_Entry(void *argument)
{
    Chassis_Cmd_t cmd = {0};
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    TickType_t xLastWakeTime = xTaskGetTickCount();
 
  for(;;)
  {
      Chassis_Cmd_t new_cmd;
      
     IMU_Update(0.001f);
        // 如果 Remote 任务还没发新数据，这里会返回 osErrorResource
        if (osMessageQueueGet(Chassis_Cmd_QHandle, &new_cmd, NULL, 0) == osOK)
        {
            // 只有拿到了才更新
            cmd = new_cmd;
        }
     if (cmd.mode == 0)
        {
            Swerve_Stop();
            
           for(int i=0; i<4; i++) can2_tx_table.target_current[i] = 0;
           can2_tx_table.update_flag = 0;
        }
     else if(cmd.mode == 4)
     {
         for (int i = 0; i < 4; i++) {
         swerve_modules[i].current_angle_deg = motor_6020[i].current_angle_deg;
        }
        if (g_data_ready){
            x_recv = g_data_buf_rx.float32[0];
            y_recv = g_data_buf_rx.float32[1];
            w_recv = g_data_buf_rx.float32[2];
            g_data_ready = 0;
          }
//        Chassis_Control(-x_recv,-y_recv,w_recv,0.001f);
           Swerve_Update(-x_recv,-y_recv,w_recv,imu_data.yaw_rad);
            // PID控制(dt保持0.001)
        Swerve_Execute(0.001f);
     }     
    else
       {
         for (int i = 0; i < 4; i++) {
         swerve_modules[i].current_angle_deg = motor_6020[i].current_angle_deg;
        }
        Swerve_Update(-cmd.vx,-cmd.vy,-cmd.omega,imu_data.yaw_rad);
//        Chassis_Control(-cmd.vx,-cmd.vy,-cmd.omega,0.001f);

            // PID控制(dt保持0.001)
        Swerve_Execute(0.001f);
       }
 
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
