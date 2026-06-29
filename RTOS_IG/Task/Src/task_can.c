#include "robot_def.h"
#include "include.h"

 CAN_Command_Table_t can1_tx_table = { .target_current = {0,0,0,0,0,0,0,0},
                                      .update_flag=0,
                                      .std_id = {0x1FF,0x2FF,0} };

 CAN_Command_Table_t can2_tx_table = { .target_current = {0,0,0,0,0,0,0,0},
                                      .update_flag = 0,
                                      .std_id = {0x200,0x1FF,0} };
// CAN_Command_Table_t can1_tx_table_2006 = { .target_current = {0,0,0,0,0,0,0,0},
//                                      .update_flag=0,
//                                      .std_id = {0x200,0x1FF,0} };

                                     
 //CAN_TX                                     
void Task_CAN_Tx_Entry(void *argument)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(2);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
  for(;;)
  {
    Send_Motor_Commands(&hcan1, &can1_tx_table);
    Send_Motor_Commands(&hcan2, &can2_tx_table);
//    Send_Motor_Commands(&hcan1, &can1_tx_table_2006);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
