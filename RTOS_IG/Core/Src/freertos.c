/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "robot_def.h"
#include "queue.h" 

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 全局姿态数据 (使用 volatile 防止编译器过度优化，因为是跨任务读写)
//float real_gyro_z=0;
//Imu_Data_t imu_msg = {0};
       

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Task_Chassis */
osThreadId_t Task_ChassisHandle;
const osThreadAttr_t Task_Chassis_attributes = {
  .name = "Task_Chassis",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task_Remote */
osThreadId_t Task_RemoteHandle;
const osThreadAttr_t Task_Remote_attributes = {
  .name = "Task_Remote",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Mech */
osThreadId_t Task_MechHandle;
const osThreadAttr_t Task_Mech_attributes = {
  .name = "Task_Mech",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task_CAN_Tx */
osThreadId_t Task_CAN_TxHandle;
const osThreadAttr_t Task_CAN_Tx_attributes = {
  .name = "Task_CAN_Tx",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Chassis_Cmd_Q */
osMessageQueueId_t Chassis_Cmd_QHandle;
const osMessageQueueAttr_t Chassis_Cmd_Q_attributes = {
  .name = "Chassis_Cmd_Q"
};
/* Definitions for Mech_Cmd_Q */
osMessageQueueId_t Mech_Cmd_QHandle;
const osMessageQueueAttr_t Mech_Cmd_Q_attributes = {
  .name = "Mech_Cmd_Q"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Task_Chassis_Entry(void *argument);
void Task_Remote_Entry(void *argument);
void Task_Mech_Entry(void *argument);
void Task_CAN_Tx_Entry(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Chassis_Cmd_Q */
  Chassis_Cmd_QHandle = osMessageQueueNew (1, sizeof(Chassis_Cmd_t), &Chassis_Cmd_Q_attributes);

  /* creation of Mech_Cmd_Q */
  Mech_Cmd_QHandle = osMessageQueueNew (1, sizeof(Mech_Cmd_t), &Mech_Cmd_Q_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Task_Chassis */
  Task_ChassisHandle = osThreadNew(Task_Chassis_Entry, NULL, &Task_Chassis_attributes);

  /* creation of Task_Remote */
  Task_RemoteHandle = osThreadNew(Task_Remote_Entry, NULL, &Task_Remote_attributes);

  /* creation of Task_Mech */
  Task_MechHandle = osThreadNew(Task_Mech_Entry, NULL, &Task_Mech_attributes);

  /* creation of Task_CAN_Tx */
  Task_CAN_TxHandle = osThreadNew(Task_CAN_Tx_Entry, NULL, &Task_CAN_Tx_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
//      HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12);  

//    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Task_Chassis_Entry */
/**
* @brief Function implementing the Task_Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Chassis_Entry */
__weak void Task_Chassis_Entry(void *argument)
{
  /* USER CODE BEGIN Task_Chassis_Entry */
 
  /* Infinite loop */
  for(;;)
  {
     osDelay(1);
  }
  /* USER CODE END Task_Chassis_Entry */
}

/* USER CODE BEGIN Header_Task_Remote_Entry */
/**
* @brief Function implementing the Task_Remote thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Remote_Entry */
__weak void Task_Remote_Entry(void *argument)
{
  /* USER CODE BEGIN Task_Remote_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }

  /* USER CODE END Task_Remote_Entry */
}

/* USER CODE BEGIN Header_Task_Mech_Entry */
/**
* @brief Function implementing the Task_Mech thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Mech_Entry */
__weak void Task_Mech_Entry(void *argument)
{
  /* USER CODE BEGIN Task_Mech_Entry */
  /* Infinite loop */
  for(;;)
  {    
      osDelay(1);
  }
  /* USER CODE END Task_Mech_Entry */
}

/* USER CODE BEGIN Header_Task_CAN_Tx_Entry */
/**
* @brief Function implementing the Task_CAN_Tx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_CAN_Tx_Entry */
__weak void Task_CAN_Tx_Entry(void *argument)
{
  /* USER CODE BEGIN Task_CAN_Tx_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_CAN_Tx_Entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

