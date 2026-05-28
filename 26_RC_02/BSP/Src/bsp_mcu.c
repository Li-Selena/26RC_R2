#include "bsp_mcu.h"
#include "main.h"
#include "fdcan.h"
#include "memorymap.h"
#include "usart.h"
#include "gpio.h"
#include "include.h"
#include "bsp_tick.h"

static void DJI_Moter_Init(void);
static void H7_power(void); 


void MCU_Init(void)
{
	H7_power();
	DJI_Moter_Init();
	PID_devices_Init();
}



static void DJI_Moter_Init(void)
{
	FDCAN1_Filter_Init();
	HAL_FDCAN_Start(&hfdcan1);
	FDCAN2_Filter_Init();
	HAL_FDCAN_Start(&hfdcan2);
	FDCAN3_Filter_Init();
	HAL_FDCAN_Start(&hfdcan3);

    PID_devices_Init();
}

static void H7_power(void)
{
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
}

