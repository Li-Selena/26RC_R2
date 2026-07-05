#ifndef ROBOTARM_CAN_BSP_H
#define ROBOTARM_CAN_BSP_H

#include "fdcan.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    volatile uint32_t tx_ok;
    volatile uint32_t tx_fail;
    volatile uint32_t rx_cnt;
    volatile uint32_t last_hal_err;
    volatile uint8_t last_lec;
    volatile uint8_t last_act;
} can_diag_t;

void FDCAN_Config(FDCAN_HandleTypeDef *hcan);
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t bsp_fdcan_dlc_to_len(uint32_t dlc);
void FDCAN3_RxMessageCallback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);

extern can_diag_t g_can_diag;

#ifdef __cplusplus
}
#endif

#endif
