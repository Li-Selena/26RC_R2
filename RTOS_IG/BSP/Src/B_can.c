//#include "robot_def.h"

//static void CAN_Push_Queue_ISR(CAN_HandleTypeDef *hcan)
//{
//    CAN_RxHeaderTypeDef rx_header;
//    CAN_Rx_Frame_t rx_frame;

//    // 循环读空硬件 FIFO，防止突发数据丢失
//    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
//    {
//        // 读取数据
//        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_frame.Data);
//        
//        // 填充自定义的数据包
//        rx_frame.StdId = rx_header.StdId;
//        rx_frame.hcan  = hcan;

//        // 入队，不等待 (timeout = 0)
//        osMessageQueuePut(CAN_Rx_QHandle, &rx_frame, 0, 0);
//    }
//}

///* CAN中断回调 */
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
//{
//    if (hcan == &hcan1) {
//        CAN_Push_Queue_ISR(&hcan1);
//    }
//    else if (hcan == &hcan2) {
//        CAN_Push_Queue_ISR(&hcan2);
//    }
//}

