#include "meth_driver.h"

Servo_t servo_gimbal; //ÔÆÌ¨¶æ»ú
Servo_t servo_claw;  //¼Ð×¦¶æ»ú

void Servo_Init_All(void) {
     Servo_Init(&servo_gimbal, &htim1, TIM_CHANNEL_1);
     Servo_Init(&servo_claw,   &htim1, TIM_CHANNEL_2);
    
      HAL_TIM_Base_Start(&htim8);
   HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
  
}

void Servo_Set_Angle_Dual(float angle1, float angle2) {

     Servo_SetAngle(&servo_gimbal, angle1);
     Servo_SetAngle(&servo_claw,   angle2);
}

void Pneumatic_Gripper_Ctrl(Valve_State_e state) {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, (state == VALVE_ON) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Pneumatic_Suction_Ctrl(Valve_State_e state) {

    if (state == VALVE_ON) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
    }
    else {
        HAL_GPIO_WritePin(GPIOB,  GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
    }
}
