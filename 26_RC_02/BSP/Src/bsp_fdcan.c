#include "bsp_fdcan.h"
#include "fdcan_receive.h"
#include "stm32h7xx_hal_fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

extern motor_measure_t motor_fdcan1[8];
extern motor_measure_t motor_fdcan2[8];
extern motor_measure_t motor_fdcan3[8];



//dji
void FDCAN_Start(FDCAN_HandleTypeDef *hfdcan)
{
    /* 启动 FDCAN */
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        Error_Handler();
    }

	if(hfdcan == &hfdcan1){
        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        {
            Error_Handler();
        }
	}
		if(hfdcan == &hfdcan2){
        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
        {
            Error_Handler();
        }
	}
		if(hfdcan == &hfdcan3){
        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
        {
            Error_Handler();
        }
	}

    /* 激活 FIFO0 新消息中断 */
}
void FDCAN1_Filter_Init(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;                        // 选择索引 0，便于与原来 bank 对照
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;                      // mask 0 -> 接收所有
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
	FDCAN_Start(&hfdcan1);
}


void FDCAN2_Filter_Init(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 7;                        // 选择索引 7，便于与原来 bank 对照
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;                      // mask 0 -> 接收所有
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
	FDCAN_Start(&hfdcan2);
}

void FDCAN3_Filter_Init(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 14;                        // 选择索引 14，便于与原来 bank 对照
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;                      // mask 0 -> 接收所有
    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
	FDCAN_Start(&hfdcan3);
}

//fdcan_callback
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
    if (hfdcan == &hfdcan1)
    {
		if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
        {
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
            {
                /* 可选：错误处理 */
                return;
            }

            /* 处理电机数据 ID */
				// DJI 3508 / M2006 电机反馈 ID 范围：0x201 ~ 0x207
            if (rx_header.Identifier >= CAN_3508_M1_ID && rx_header.Identifier <= CAN_3508_M7_ID)
            {
                uint8_t i = (uint8_t)(rx_header.Identifier - CAN_3508_M1_ID);
                motor_measure_t *m = NULL;

            
                m = &motor_fdcan1[i];

                if (m)
                {
                    m->msg_cnt++;
                    if (m->msg_cnt <= 50)
                        get_motor_offset(m, rx_data);
                    else
                        get_motor_measure(m, rx_data);
                }
            }
            else
            {
                /* 其它 ID 可按需扩展处理 */
            }
      }

    }
}
//fdcan_callback
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

		if (hfdcan == &hfdcan2)
    {
		
      if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
      {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK)
        {
            /* 可选：错误处理 */
            return;
        }

        /* 处理电机数据 ID */
				// DJI 3508 / M2006 电机反馈 ID 范围：0x201 ~ 0x207
        if (rx_header.Identifier >= CAN_3508_M1_ID && rx_header.Identifier <= CAN_3508_M7_ID)
        {
            uint8_t i = (uint8_t)(rx_header.Identifier - CAN_3508_M1_ID);
            motor_measure_t *m = NULL;

            
            m = &motor_fdcan2[i];

            if (m)
            {
                m->msg_cnt++;
                if (m->msg_cnt <= 50)
                    get_motor_offset(m, rx_data);
                else
                    get_motor_measure(m, rx_data);
            }
        }
        else
        {
            /* 其它 ID 可按需扩展处理 */
        }
      }
    }
		if (hfdcan == &hfdcan3)
    {
		/* 如果是 FIFO0 新消息中断 */
      if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
      {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK)
        {
            /* 可选：错误处理 */
            return;
        }

        /* 处理电机数据 ID */
				// DJI 3508 / M2006 电机反馈 ID 范围：0x201 ~ 0x207
        if (rx_header.Identifier >= CAN_3508_M1_ID && rx_header.Identifier <= CAN_3508_M7_ID)
        {
            uint8_t i = (uint8_t)(rx_header.Identifier - CAN_3508_M1_ID);
            motor_measure_t *m = NULL;

            
            m = &motor_fdcan3[i];

            if (m)
            {
                m->msg_cnt++;
                if (m->msg_cnt <= 50)
                    get_motor_offset(m, rx_data);
                else
                    get_motor_measure(m, rx_data);
            }
        }
        else
        {
            /* 其它 ID 可按需扩展处理 */
        }
      }
    }

}
