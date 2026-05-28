#include "bsp_mcu.h"
#include "main.h"
#include "fdcan.h"
#include "memorymap.h"
#include "usart.h"
#include "gpio.h"
#include "include.h"
#include "dm_motor_ctrl.h"
#include "bsp_tick.h"

static void DM_Moter_Init(void);
static void DJI_Moter_Init(void);
static void H7_power(void); 
static void DM_Moter1_Init(int Mode);
static void DM_Moter2_Init(int Mode);
static void DM_Moter3_Init(int Mode);
static void DM_Moter4_Init(int Mode);
static void DM_Moter5_Init(int Mode);
static void DM_Moter6_Init(int Mode);
static void DM_save_pos_zero(void);

void MCU_Init(void)
{
	H7_power();
	DM_Moter_Init();
	DJI_Moter_Init();
	PID_devices_Init();
	PID_Yaw_Init();

}

static void DM_Moter_Init(void)
{
	bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M);
	bsp_can_init();
	dm_motor_init();
	DM_Moter1_Init(pos_mode);
	DM_Moter2_Init(pos_mode);
	DM_Moter3_Init(pos_mode);
	DM_Moter4_Init(pos_mode);
	DM_Moter5_Init(pos_mode);
	DM_Moter6_Init(pos_mode);
	// DM_save_pos_zero();
	
}

static void DJI_Moter_Init(void)
{
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

static void DM_Moter1_Init(int Mode)
{
	motor[Motor1].ctrl.mode 	= Mode;//  ???????
	Delay_ms(100);
	
	write_motor_data(motor[Motor1].id , 10, Mode, 0, 0, 0);
	Delay_ms(100);

	read_motor_data(motor[Motor1].id, RID_CAN_BR); 
	dm_motor_disable(&hfdcan1, &motor[Motor1]);
	Delay_ms(100);
	
	save_motor_data(motor[Motor1].id, 10);
	Delay_ms(100);
	
	dm_motor_enable(&hfdcan1, &motor[Motor1]);	
	
}
static void DM_Moter2_Init(int Mode)
{
	motor[Motor2].ctrl.mode 	= Mode;//  ???????
	Delay_ms(100);
	
	write_motor_data(motor[Motor2].id , 10, Mode, 0, 0, 0);
	Delay_ms(100);

	read_motor_data(motor[Motor2].id, RID_CAN_BR); 
	dm_motor_disable(&hfdcan1, &motor[Motor2]);
	Delay_ms(100);
	
	save_motor_data(motor[Motor2].id, 10);
	Delay_ms(100);
	
	dm_motor_enable(&hfdcan1, &motor[Motor2]);	
}
static void DM_Moter3_Init(int Mode)
{
	motor[Motor3].ctrl.mode 	= Mode;//  ???????
	Delay_ms(100);
	
	write_motor_data(motor[Motor3].id , 10, Mode, 0, 0, 0);
	Delay_ms(100);

	read_motor_data(motor[Motor3].id, RID_CAN_BR); 
	dm_motor_disable(&hfdcan1, &motor[Motor3]);
	Delay_ms(100);
	
	save_motor_data(motor[Motor3].id, 10);
	Delay_ms(100);
	
	dm_motor_enable(&hfdcan1, &motor[Motor3]);	
	
}
static void DM_Moter4_Init(int Mode)
{
	motor[Motor4].ctrl.mode 	= Mode;//  ???????
	Delay_ms(100);
	
	write_motor_data(motor[Motor4].id , 10, Mode, 0, 0, 0);
	Delay_ms(100);

	read_motor_data(motor[Motor4].id, RID_CAN_BR); 
	dm_motor_disable(&hfdcan1, &motor[Motor4]);
	Delay_ms(100);
	
	save_motor_data(motor[Motor4].id, 10);
	Delay_ms(100);
	
	dm_motor_enable(&hfdcan1, &motor[Motor4]);	
	
}
static void DM_Moter5_Init(int Mode)
{
	motor[Motor5].ctrl.mode 	= Mode;//  ???????
	Delay_ms(100);
	
	write_motor_data(motor[Motor5].id , 10, Mode, 0, 0, 0);
	Delay_ms(100);

	read_motor_data(motor[Motor5].id, RID_CAN_BR); 
	dm_motor_disable(&hfdcan1, &motor[Motor5]);
	Delay_ms(100);
	
	save_motor_data(motor[Motor5].id, 10);
	Delay_ms(100);
	
	dm_motor_enable(&hfdcan1, &motor[Motor5]);	
	
}
static void DM_Moter6_Init(int Mode)
{
	motor[Motor6].ctrl.mode 	= Mode;//  ???????
	Delay_ms(100);
	
	write_motor_data(motor[Motor6].id , 10, Mode, 0, 0, 0);
	Delay_ms(100);

	read_motor_data(motor[Motor6].id, RID_CAN_BR); 
	dm_motor_disable(&hfdcan1, &motor[Motor6]);
	Delay_ms(100);
	
	save_motor_data(motor[Motor6].id, 10);
	Delay_ms(100);
	
	dm_motor_enable(&hfdcan1, &motor[Motor6]);	
	
}

static void DM_save_pos_zero(void)
{
	save_pos_zero(&hfdcan1, motor[Motor1].id,POS_MODE);
	save_pos_zero(&hfdcan1, motor[Motor2].id,POS_MODE);
	save_pos_zero(&hfdcan1, motor[Motor3].id,POS_MODE);
	save_pos_zero(&hfdcan1, motor[Motor4].id,POS_MODE);
	save_pos_zero(&hfdcan1, motor[Motor5].id,POS_MODE);
	save_pos_zero(&hfdcan1, motor[Motor6].id,POS_MODE);
}
