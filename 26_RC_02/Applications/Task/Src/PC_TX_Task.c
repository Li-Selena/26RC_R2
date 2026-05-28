#include "PC_TX_Task.h"
#include "cmsis_os.h"


//执行器任务标志位
extern uint8_t Mecanum_control_flag ;
extern uint8_t Arm_control_flag ;

//麦克纳姆底盘参数
extern MecanumParam_t mecParam;
extern ChassisVel_t total_vel_USB;
extern WheelSpeed_t total_speed_USB;

//机械臂位置参数
extern float ARM_setX_USB ;
extern float ARM_setY_USB ;
extern float ARM_setZ_USB ;

extern float ctrl_J_USB[4];
extern float model_J_USB[4];
extern float ctrl_J_USART[4];
extern float model_J_USART[4];

//工具句柄
extern clamp_Handle_t clamp;
extern chuck_Handle_t chuck;


//电机测量数据
extern motor_measure_t motor_fdcan1[8];
extern motor_measure_t motor_fdcan2[8];
extern motor_measure_t motor_fdcan3[8];



void PC_TX_Task(void const * argument)
{
  /* USER CODE BEGIN PC_TX_Task */
  /* Infinite loop */
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END PC_TX_Task */
}