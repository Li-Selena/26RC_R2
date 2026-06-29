#include "can_receive.h"
#include "task.h"


motor_measure_t motor_can1[8];
motor_measure_t motor_can2[8];
motor_measure_t motor_6020[8];
//motor_measure_t motor_6020_2[4];


// 定义电机注册表 
motor_instance_t motor_map[] = {

    // 舵向 6020 电机 (挂在 CAN1)
    {&hcan1, CAN_6020_M1_ID, &motor_6020[0],OFFSET_FIXED_VALUE,1355},
    {&hcan1, CAN_6020_M2_ID, &motor_6020[1],OFFSET_FIXED_VALUE,2057},
    {&hcan1, CAN_6020_M3_ID, &motor_6020[2],OFFSET_FIXED_VALUE,1373}, 
    {&hcan1, CAN_6020_M4_ID, &motor_6020[3],OFFSET_FIXED_VALUE,2057},
//    {&hcan1, CAN_6020_M5_ID, &motor_6020[4],OFFSET_FIXED_VALUE,0},
    {&hcan1, CAN_2006_M1_ID, &motor_can1[0], OFFSET_AUTO_ON_STARTUP, 0},
    
    // 底盘 3508 电机 (挂在 CAN2)
    {&hcan2, CAN_3508_M1_ID, &motor_can2[0],OFFSET_AUTO_ON_STARTUP, 0},
    {&hcan2, CAN_3508_M2_ID, &motor_can2[1],OFFSET_AUTO_ON_STARTUP, 0},
    {&hcan2, CAN_3508_M3_ID, &motor_can2[2],OFFSET_AUTO_ON_STARTUP, 0},
    {&hcan2, CAN_3508_M4_ID, &motor_can2[3],OFFSET_AUTO_ON_STARTUP, 0},
    {&hcan2, CAN_3508_M5_ID, &motor_can2[4],OFFSET_AUTO_ON_STARTUP, 0},
    {&hcan2, CAN_3508_M6_ID, &motor_can2[5],OFFSET_AUTO_ON_STARTUP, 0},
//    {&hcan2, CAN_3508_M7_ID, &motor_can2[6],OFFSET_AUTO_ON_STARTUP, 0}
    
    // 未来扩展：只需加一行
    // {&hcan1, 0x209, &extra_motor}, 
};
#define MOTOR_MAP_SIZE (sizeof(motor_map) / sizeof(motor_instance_t))

void get_motor_measure(motor_measure_t *ptr,uint8_t data[])                                                     
    {   
        (ptr)->last_angle = (ptr)->angle;                                                          
        (ptr)->angle = data[0] << 8 | data[1];           
        (ptr)->speed_rpm = (int16_t)(data[2] << 8 | data[3]);     
        (ptr)->given_current = data[4] << 8 | data[5]; 
        (ptr)->temperature = data[6];                                              
//				((ptr)->angle) = (int32_t)(((ptr)->ecd) - ((ptr)->last_ecd));

					if(ptr->angle - ptr->last_angle > 4096)
						ptr->round_cnt --;
					else if (ptr->angle - ptr->last_angle < -4096)
						ptr->round_cnt ++;
					ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
                  
        ptr->current_angle_deg = (fp32)ptr->total_angle * 360.0f / ENCODER_RESOLUTION;
        // ptr->current_angle_deg_wrap = normalize_deg(ptr->current_angle_deg);
    }
		
		
void get_motor_offset(motor_measure_t *ptr, uint8_t data[])
{
	ptr->angle = data[0]<<8 |data[1] ;
	ptr->offset_angle = ptr->angle;
}


//// 极速解码器
//void DJI_Motor_Decode_Fast(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t *rx_data) 
//{
//    motor_measure_t *motor = NULL;
//    uint8_t offset_style = OFFSET_FIXED_VALUE; // 默认值
//    int16_t fixed_offset = 0;

//    if (hcan == &hcan2 && std_id >= CAN_3508_M1_ID && std_id <= CAN_3508_M7_ID) {
//        uint8_t idx = std_id - CAN_3508_M1_ID;
//        uint8_t map_idx = 5 + idx; 
//  
//        if (map_idx < MOTOR_MAP_SIZE) {
//        motor = &motor_can2[idx];
//        offset_style = motor_map[map_idx].offset_style; 
//        fixed_offset = motor_map[map_idx].fixed_offset;
//        }            
//    }
//    // 6020 (挂在 CAN1)
//    else if (hcan == &hcan1 && std_id >= CAN_6020_M1_ID && std_id <= CAN_6020_M6_ID) {
//        uint8_t idx = std_id - CAN_6020_M1_ID;
//        uint8_t map_idx = 0 + idx; 
//        
//        if (map_idx < MOTOR_MAP_SIZE) {
//        motor = &motor_6020[idx];
//        offset_style = motor_map[map_idx].offset_style;
//        fixed_offset = motor_map[map_idx].fixed_offset;
//        }
//    }

//    if (motor != NULL) {
//        // 首次接收到数据，进行初始化
//        if (motor->msg_cnt == 0) { 
//            uint16_t tmp_angle = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
//            motor->angle = tmp_angle;       
//            motor->last_angle = tmp_angle;
//            motor->round_cnt = 0;
//            
//            if (offset_style == OFFSET_FIXED_VALUE) {
//                motor->offset_angle = fixed_offset;
//            } else {
//                motor->offset_angle = tmp_angle; // 上电自动读取当前值为零点
//            }
//        }
//        
//        if (offset_style == OFFSET_AUTO_ON_STARTUP && motor->msg_cnt < 50) {
//            get_motor_offset(motor, rx_data);
//        } else {
//            get_motor_measure(motor, rx_data);
//        }
//        
//        motor->msg_cnt++; 
//    }
//}

void DJI_Motor_Decode_Fast(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t *rx_data) 
{
    motor_measure_t *motor = NULL;
    uint8_t offset_style = OFFSET_FIXED_VALUE;
    int16_t fixed_offset = 0;

    if (hcan == &hcan1) {
        // 6020 
        if (std_id >= CAN_6020_M1_ID && std_id <= CAN_6020_M5_ID) {  
            uint8_t idx = std_id - CAN_6020_M1_ID;
            uint8_t map_idx = idx;                     // 索引 0~4
            if (map_idx < MOTOR_MAP_SIZE) {
                motor = &motor_6020[idx];
                offset_style = motor_map[map_idx].offset_style;
                fixed_offset = motor_map[map_idx].fixed_offset;
            }
        }
        //2006
        else if (std_id >= CAN_2006_M1_ID && std_id <= CAN_2006_M4_ID) {
            uint8_t idx = std_id - CAN_2006_M1_ID;
            uint8_t map_idx = 4 + idx;             
            if (map_idx < MOTOR_MAP_SIZE) {
                motor = &motor_can1[idx];
                offset_style = motor_map[map_idx].offset_style;
                fixed_offset = motor_map[map_idx].fixed_offset;
            }
        }
    }
    else if (hcan == &hcan2) {
        //3508
        if (std_id >= CAN_3508_M1_ID && std_id <= CAN_3508_M6_ID) {
            uint8_t idx = std_id - CAN_3508_M1_ID;
            uint8_t map_idx =5+ idx;                 // 索引 9~14（前面 5 个 6020 + 4 个 2006 = 9）
            if (map_idx < MOTOR_MAP_SIZE) {
                motor = &motor_can2[idx];
                offset_style = motor_map[map_idx].offset_style;
                fixed_offset = motor_map[map_idx].fixed_offset;
            }
        }
    }

    if (motor != NULL) {
        if (motor->msg_cnt == 0) { 
            uint16_t tmp_angle = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
            motor->angle = tmp_angle;       
            motor->last_angle = tmp_angle;
            motor->round_cnt = 0;
            
            if (offset_style == OFFSET_FIXED_VALUE) {
                motor->offset_angle = fixed_offset;
            } else {
                motor->offset_angle = tmp_angle;
            }
        }
        
        if (offset_style == OFFSET_AUTO_ON_STARTUP && motor->msg_cnt < 50) {
            get_motor_offset(motor, rx_data);
        } else {
            get_motor_measure(motor, rx_data);
        }
        motor->msg_cnt++; 
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];

    // 直接在中断中极速读取并解码
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) 
    {
        DJI_Motor_Decode_Fast(hcan, rx_header.StdId, rx_data);
    }    

}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK)
    {
        DJI_Motor_Decode_Fast(hcan, rx_header.StdId, rx_data);
    }
}

void CAN_Send(CAN_HandleTypeDef* hcan, uint32_t std_id, int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    
    CAN_TxHeaderTypeDef can_tx_message;
    uint8_t can_send_data[8];
    uint32_t send_mail_box;

    can_tx_message.StdId = std_id;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;

    can_send_data[0] = (m1 >> 8); can_send_data[1] = m1;
    can_send_data[2] = (m2 >> 8); can_send_data[3] = m2;
    can_send_data[4] = (m3 >> 8); can_send_data[5] = m3;
    can_send_data[6] = (m4 >> 8); can_send_data[7] = m4;

   
        HAL_CAN_AddTxMessage(hcan, &can_tx_message, can_send_data, &send_mail_box);
}

void Send_Motor_Commands(CAN_HandleTypeDef* hcan, CAN_Command_Table_t* table)
{
    // 没有新数据,直接返回
    if (table->update_flag == 0) return;

     CAN_Send(hcan,table->std_id[0], table->target_current[0], table->target_current[1],
        table->target_current[2], table->target_current[3]);
    
     CAN_Send(hcan,table->std_id[1], table->target_current[4], table->target_current[5],
        table->target_current[6], table->target_current[7]);
    
    table->update_flag = 0; 
}

