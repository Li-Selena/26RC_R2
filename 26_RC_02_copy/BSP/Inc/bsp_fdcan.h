#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"
#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    volatile uint32_t tx_ok;
    volatile uint32_t tx_fail;
    volatile uint32_t tx_fifo_full;
    volatile uint32_t rx_cnt;
    volatile uint32_t tx_per_sec;
    volatile uint32_t tx_fail_per_sec;
    volatile uint32_t tx_fifo_full_per_sec;
    volatile uint32_t rx_per_sec;
    volatile uint32_t last_sample_ms;
    volatile uint32_t last_sample_tx_ok;
    volatile uint32_t last_sample_tx_fail;
    volatile uint32_t last_sample_tx_fifo_full;
    volatile uint32_t last_sample_rx_cnt;
    volatile uint32_t last_hal_err;
    volatile uint8_t last_lec;
    volatile uint8_t last_act;
} can_diag_t;

typedef enum
{
    BSP_FDCAN_BUS1 = 0,
    BSP_FDCAN_BUS2,
    BSP_FDCAN_BUS3,
    BSP_FDCAN_BUS_COUNT
} bsp_fdcan_bus_t;

typedef struct
{
    uint16_t message_ram_offset_words;
    uint16_t message_ram_used_words;
    uint16_t message_ram_end_words;
    uint8_t std_filters;
    uint8_t ext_filters;
    uint8_t rx_fifo0_elts;
    uint8_t rx_fifo1_elts;
    uint8_t rx_buffer_elts;
    uint8_t tx_event_elts;
    uint8_t tx_buffer_elts;
    uint8_t tx_fifo_elts;
    uint8_t estimated_load_percent;
    uint8_t high_load;
} bsp_fdcan_bus_audit_t;

//dji
void FDCAN_Start(FDCAN_HandleTypeDef *hfdcan);

void FDCAN1_Filter_Init(void);
void FDCAN2_Filter_Init(void);
void FDCAN_Motor_Start_All(void);

void FDCAN_Config(FDCAN_HandleTypeDef *hcan);
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t bsp_fdcan_dlc_to_len(uint32_t dlc);
void BSP_FDCAN_RecordRx(FDCAN_HandleTypeDef *hcan);
void BSP_FDCAN_RecordTxResult(FDCAN_HandleTypeDef *hcan, uint8_t ok, uint8_t fifo_full);
void FDCAN3_RxMessageCallback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);

extern can_diag_t g_can_diag;
extern can_diag_t g_fdcan_diag[BSP_FDCAN_BUS_COUNT];
extern const bsp_fdcan_bus_audit_t g_fdcan_bus_audit[BSP_FDCAN_BUS_COUNT];

#ifdef __cplusplus
}
#endif

#endif
