#include "bsp_can.h"
#include "can.h"

void CAN_Start(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_Start(hcan);

	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
	}
  
    
	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
	}
}



void CAN1_Filter_Init(void)
{
    CAN_FilterTypeDef CAN_Filter_st;
    CAN_Filter_st.FilterActivation = ENABLE;
    CAN_Filter_st.FilterMode = CAN_FILTERMODE_IDLIST;     // 标识符列表
    CAN_Filter_st.FilterScale = CAN_FILTERSCALE_16BIT;    // 16位 ID
    CAN_Filter_st.FilterBank = 0;
    CAN_Filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    // 四个舵轮 6020 的 ID
    CAN_Filter_st.FilterIdHigh = CAN_6020_M2_ID << 5;    // 第二个 ID
    CAN_Filter_st.FilterIdLow  = CAN_6020_M1_ID << 5;
    CAN_Filter_st.FilterMaskIdHigh = CAN_6020_M4_ID << 5; // 第四个 ID
    CAN_Filter_st.FilterMaskIdLow  = CAN_6020_M3_ID << 5;
    HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_st);
//    CAN_Filter_st.FilterIdHigh = CAN_6020_M4_ID << 5;    // 第二个 ID
//    CAN_Filter_st.FilterIdLow  = CAN_6020_M3_ID << 5;
//    CAN_Filter_st.FilterMaskIdHigh = CAN_6020_M6_ID << 5; // 第四个 ID
//    CAN_Filter_st.FilterMaskIdLow  = CAN_6020_M5_ID << 5;
//    HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_st);

    
//    CAN_Filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
//    CAN_Filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_Filter_st.FilterBank = 1;
    CAN_Filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
    CAN_Filter_st.FilterIdHigh = CAN_2006_M1_ID << 5;
    CAN_Filter_st.FilterIdLow  = CAN_2006_M1_ID << 5;
    CAN_Filter_st.FilterMaskIdHigh =  CAN_2006_M1_ID << 5;
    CAN_Filter_st.FilterMaskIdLow  = CAN_2006_M1_ID << 5; 
    HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_st);
//    CAN_Filter_st.FilterBank = 1;
//    CAN_Filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
//    CAN_Filter_st.FilterIdHigh = CAN_6020_M2_ID << 5;
//    CAN_Filter_st.FilterIdLow  = CAN_6020_M2_ID << 5;
//    CAN_Filter_st.FilterMaskIdHigh =  CAN_6020_M2_ID << 5;
//    CAN_Filter_st.FilterMaskIdLow  =  CAN_6020_M2_ID << 5; 
//    HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_st);

    
//    CAN_Filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
//    CAN_Filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
//    CAN_Filter_st.FilterIdHigh = 0x0000;
//    CAN_Filter_st.FilterIdLow = 0x0000;
//    CAN_Filter_st.FilterMaskIdHigh = 0x0000;
//    CAN_Filter_st.FilterMaskIdLow = 0x0000;
//    CAN_Filter_st.FilterBank = 0;
//    CAN_Filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
//	  HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_st);        //滤波器初始化
//    
//    CAN_Filter_st.FilterBank = 1;                     // 第1组滤波器
//    CAN_Filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;// 数据进 FIFO1
//    HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_st);

}


void CAN2_Filter_Init(void)
{
	  CAN_FilterTypeDef CAN_Filter_st;
    CAN_Filter_st.FilterActivation = ENABLE;
    CAN_Filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
    CAN_Filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
    CAN_Filter_st.FilterBank = 14;
    CAN_Filter_st.SlaveStartFilterBank = 14;      
    CAN_Filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    CAN_Filter_st.FilterIdHigh = CAN_3508_M2_ID << 5;
    CAN_Filter_st.FilterIdLow  = CAN_3508_M1_ID << 5;
    CAN_Filter_st.FilterMaskIdHigh = CAN_3508_M4_ID << 5;
    CAN_Filter_st.FilterMaskIdLow  = CAN_3508_M3_ID << 5;
    HAL_CAN_ConfigFilter(&hcan2, &CAN_Filter_st);

//    CAN_Filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
//    CAN_Filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_Filter_st.FilterBank = 15;
    CAN_Filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
    CAN_Filter_st.FilterIdHigh = CAN_3508_M6_ID << 5;
    CAN_Filter_st.FilterIdLow  = CAN_3508_M5_ID << 5;
    CAN_Filter_st.FilterMaskIdHigh =  CAN_3508_M6_ID << 5;
    CAN_Filter_st.FilterMaskIdLow  =  CAN_3508_M5_ID << 5;
    HAL_CAN_ConfigFilter(&hcan2, &CAN_Filter_st);
    
//    CAN_Filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
//    CAN_Filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
//    CAN_Filter_st.FilterIdHigh = 0x0000;
//    CAN_Filter_st.FilterIdLow = 0x0000;
//    CAN_Filter_st.FilterMaskIdHigh = 0x0000;
//    CAN_Filter_st.FilterMaskIdLow = 0x0000;
//    CAN_Filter_st.FilterBank = 14;
//    CAN_Filter_st.SlaveStartFilterBank = 14;
//    CAN_Filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
//	  HAL_CAN_ConfigFilter(&hcan2, &CAN_Filter_st);        //滤波器初始化
//    
//    CAN_Filter_st.FilterBank = 15;               // 换下一个滤波器
//    CAN_Filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
//    HAL_CAN_ConfigFilter(&hcan2, &CAN_Filter_st);
}





