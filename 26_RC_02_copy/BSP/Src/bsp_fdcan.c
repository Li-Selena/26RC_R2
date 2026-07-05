#include "bsp_fdcan.h"
#include "fdcan_receive.h"
#include "stm32h7xx_hal_fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

extern motor_measure_t motor_fdcan1[8];
extern motor_measure_t motor_fdcan2[8];

can_diag_t g_can_diag = {0};
can_diag_t g_fdcan_diag[BSP_FDCAN_BUS_COUNT] = {0};

const bsp_fdcan_bus_audit_t g_fdcan_bus_audit[BSP_FDCAN_BUS_COUNT] = {
    {0U,    200U,  200U,  4U, 4U, 10U, 10U, 2U, 10U, 10U, 10U, 85U, 1U},
    {850U,  200U, 1050U,  4U, 4U, 10U, 10U, 2U, 10U, 10U, 10U, 85U, 1U},
    {1600U, 101U, 1701U,  5U, 0U,  0U,  4U, 0U,  0U,  0U,  8U, 35U, 0U},
};

static can_diag_t *BSP_FDCAN_SelectDiag(FDCAN_HandleTypeDef *hcan)
{
    if (hcan == &hfdcan1)
    {
        return &g_fdcan_diag[BSP_FDCAN_BUS1];
    }
    if (hcan == &hfdcan2)
    {
        return &g_fdcan_diag[BSP_FDCAN_BUS2];
    }
    if (hcan == &hfdcan3)
    {
        return &g_fdcan_diag[BSP_FDCAN_BUS3];
    }
    return NULL;
}

static void BSP_FDCAN_CopyDiagToLegacyIfArmBus(FDCAN_HandleTypeDef *hcan, const can_diag_t *diag)
{
    if ((hcan == &hfdcan3) && (diag != NULL))
    {
        g_can_diag.tx_ok = diag->tx_ok;
        g_can_diag.tx_fail = diag->tx_fail;
        g_can_diag.tx_fifo_full = diag->tx_fifo_full;
        g_can_diag.rx_cnt = diag->rx_cnt;
        g_can_diag.tx_per_sec = diag->tx_per_sec;
        g_can_diag.tx_fail_per_sec = diag->tx_fail_per_sec;
        g_can_diag.tx_fifo_full_per_sec = diag->tx_fifo_full_per_sec;
        g_can_diag.rx_per_sec = diag->rx_per_sec;
        g_can_diag.last_sample_ms = diag->last_sample_ms;
        g_can_diag.last_sample_tx_ok = diag->last_sample_tx_ok;
        g_can_diag.last_sample_tx_fail = diag->last_sample_tx_fail;
        g_can_diag.last_sample_tx_fifo_full = diag->last_sample_tx_fifo_full;
        g_can_diag.last_sample_rx_cnt = diag->last_sample_rx_cnt;
        g_can_diag.last_hal_err = diag->last_hal_err;
        g_can_diag.last_lec = diag->last_lec;
        g_can_diag.last_act = diag->last_act;
    }
}

static void BSP_FDCAN_UpdateRate(can_diag_t *diag)
{
    uint32_t now;
    uint32_t elapsed;

    if (diag == NULL)
    {
        return;
    }

    now = HAL_GetTick();
    if (diag->last_sample_ms == 0U)
    {
        diag->last_sample_ms = now;
        diag->last_sample_tx_ok = diag->tx_ok;
        diag->last_sample_tx_fail = diag->tx_fail;
        diag->last_sample_tx_fifo_full = diag->tx_fifo_full;
        diag->last_sample_rx_cnt = diag->rx_cnt;
        return;
    }

    elapsed = now - diag->last_sample_ms;
    if (elapsed < 1000U)
    {
        return;
    }

    diag->tx_per_sec = ((diag->tx_ok - diag->last_sample_tx_ok) * 1000U) / elapsed;
    diag->tx_fail_per_sec = ((diag->tx_fail - diag->last_sample_tx_fail) * 1000U) / elapsed;
    diag->tx_fifo_full_per_sec = ((diag->tx_fifo_full - diag->last_sample_tx_fifo_full) * 1000U) / elapsed;
    diag->rx_per_sec = ((diag->rx_cnt - diag->last_sample_rx_cnt) * 1000U) / elapsed;

    diag->last_sample_ms = now;
    diag->last_sample_tx_ok = diag->tx_ok;
    diag->last_sample_tx_fail = diag->tx_fail;
    diag->last_sample_tx_fifo_full = diag->tx_fifo_full;
    diag->last_sample_rx_cnt = diag->rx_cnt;
}

static void BSP_FDCAN_UpdateProtocolStatus(FDCAN_HandleTypeDef *hcan, can_diag_t *diag)
{
    FDCAN_ProtocolStatusTypeDef psr;

    if ((hcan == NULL) || (diag == NULL))
    {
        return;
    }

    diag->last_hal_err = HAL_FDCAN_GetError(hcan);
    if (HAL_FDCAN_GetProtocolStatus(hcan, &psr) == HAL_OK)
    {
        diag->last_lec = psr.LastErrorCode;
        diag->last_act = psr.Activity;
    }
}

void BSP_FDCAN_RecordRx(FDCAN_HandleTypeDef *hcan)
{
    can_diag_t *diag = BSP_FDCAN_SelectDiag(hcan);

    if (diag == NULL)
    {
        return;
    }

    diag->rx_cnt++;
    BSP_FDCAN_UpdateRate(diag);
    BSP_FDCAN_UpdateProtocolStatus(hcan, diag);
    BSP_FDCAN_CopyDiagToLegacyIfArmBus(hcan, diag);
}

void BSP_FDCAN_RecordTxResult(FDCAN_HandleTypeDef *hcan, uint8_t ok, uint8_t fifo_full)
{
    can_diag_t *diag = BSP_FDCAN_SelectDiag(hcan);

    if (diag == NULL)
    {
        return;
    }

    if (ok != 0U)
    {
        diag->tx_ok++;
    }
    else
    {
        diag->tx_fail++;
        if (fifo_full != 0U)
        {
            diag->tx_fifo_full++;
        }
    }

    BSP_FDCAN_UpdateRate(diag);
    BSP_FDCAN_UpdateProtocolStatus(hcan, diag);
    BSP_FDCAN_CopyDiagToLegacyIfArmBus(hcan, diag);
}

static uint32_t bsp_fdcan_len_to_dlc(uint32_t len)
{
    switch (len)
    {
    case 0: return FDCAN_DLC_BYTES_0;
    case 1: return FDCAN_DLC_BYTES_1;
    case 2: return FDCAN_DLC_BYTES_2;
    case 3: return FDCAN_DLC_BYTES_3;
    case 4: return FDCAN_DLC_BYTES_4;
    case 5: return FDCAN_DLC_BYTES_5;
    case 6: return FDCAN_DLC_BYTES_6;
    case 7: return FDCAN_DLC_BYTES_7;
    case 8: return FDCAN_DLC_BYTES_8;
    default: return FDCAN_DLC_BYTES_8;
    }
}

uint8_t bsp_fdcan_dlc_to_len(uint32_t dlc)
{
    switch (dlc)
    {
    case FDCAN_DLC_BYTES_0: return 0U;
    case FDCAN_DLC_BYTES_1: return 1U;
    case FDCAN_DLC_BYTES_2: return 2U;
    case FDCAN_DLC_BYTES_3: return 3U;
    case FDCAN_DLC_BYTES_4: return 4U;
    case FDCAN_DLC_BYTES_5: return 5U;
    case FDCAN_DLC_BYTES_6: return 6U;
    case FDCAN_DLC_BYTES_7: return 7U;
    case FDCAN_DLC_BYTES_8: return 8U;
    default: return 8U;
    }
}

static void FDCAN_Motor_Filter_Init(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0U;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = fifo;
    sFilterConfig.FilterID1 = CAN_CHASSIS_ALL_ID;
    sFilterConfig.FilterID2 = 0x7F0U;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

static void FDCAN_Process_Motor_Rx(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo, motor_measure_t *motor)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint32_t motor_index;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, fifo) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        BSP_FDCAN_RecordRx(hfdcan);

        if ((rx_header.IdType != FDCAN_STANDARD_ID) ||
            (rx_header.Identifier < CAN_3508_M1_ID) ||
            (rx_header.Identifier > CAN_3508_M8_ID))
        {
            continue;
        }

        motor_index = rx_header.Identifier - CAN_3508_M1_ID;
        motor[motor_index].msg_cnt++;

        if (motor[motor_index].msg_cnt <= 50U)
        {
            get_motor_offset(&motor[motor_index], rx_data);
        }
        else
        {
            get_motor_measure(&motor[motor_index], rx_data);
        }
    }
}

void FDCAN_Config(FDCAN_HandleTypeDef *hcan)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};

    if (hcan == NULL)
    {
        return;
    }

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0U;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = 0x000U;
    sFilterConfig.FilterID2 = 0x000U;

    if (HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(hcan,
                                     FDCAN_ACCEPT_IN_RX_FIFO1,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(hcan) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0U) != HAL_OK)
    {
        Error_Handler();
    }
}

uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};

    if ((hcan == NULL) || (data == NULL))
    {
        return 1U;
    }

    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = bsp_fdcan_len_to_dlc(len);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hcan) == 0U)
    {
        BSP_FDCAN_RecordTxResult(hcan, 0U, 1U);
        return 1U;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, data) != HAL_OK)
    {
        BSP_FDCAN_RecordTxResult(hcan, 0U, 0U);
        return 1U;
    }

    BSP_FDCAN_RecordTxResult(hcan, 1U, 0U);
    return 0U;
}

static void FDCAN3_Process_Arm_Rx(void)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[24];

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan3, FDCAN_RX_FIFO1) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        BSP_FDCAN_RecordRx(&hfdcan3);
        FDCAN3_RxMessageCallback(&rx_header, rx_data);
    }
}

void FDCAN_Start(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t notification = 0U;

    if (hfdcan == NULL)
    {
        return;
    }

    if ((hfdcan == &hfdcan1) || (hfdcan == &hfdcan2))
    {
        notification = FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
    }
    else
    {
        return;
    }

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan, notification, 0U) != HAL_OK)
    {
        Error_Handler();
    }
}

void FDCAN1_Filter_Init(void)
{
    FDCAN_Motor_Filter_Init(&hfdcan1, FDCAN_FILTER_TO_RXFIFO0);
}

void FDCAN2_Filter_Init(void)
{
    FDCAN_Motor_Filter_Init(&hfdcan2, FDCAN_FILTER_TO_RXFIFO0);
}

void FDCAN_Motor_Start_All(void)
{
    FDCAN_Start(&hfdcan1);
    FDCAN_Start(&hfdcan2);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((hfdcan == &hfdcan1) && ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET))
    {
        FDCAN_Process_Motor_Rx(hfdcan, FDCAN_RX_FIFO0, motor_fdcan1);
    }
    else if ((hfdcan == &hfdcan2) && ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET))
    {
        FDCAN_Process_Motor_Rx(hfdcan, FDCAN_RX_FIFO0, motor_fdcan2);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == RESET)
    {
        return;
    }

    if (hfdcan == &hfdcan3)
    {
        FDCAN3_Process_Arm_Rx();
    }
}

__weak void FDCAN3_RxMessageCallback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    (void)rx_header;
    (void)rx_data;
}
