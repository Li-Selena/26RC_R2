#include "Control_Task.h"
#include "cmsis_os.h"
#include "pid_user.h"
#include "CAN_Task.h"

extern float ctrl_j1,ctrl_j2,ctrl_j3;
extern float model_theta1,model_theta2,model_theta3;
extern float ctrl_J_USB[4];
extern float model_J_USB[4];
extern float ctrl_J_USART[4];
extern float model_J_USART[4];

extern WheelSpeed_t total_speed ;
//腿电机角度 rad
extern leg lf_leg ;//前腿
extern leg rf_leg ;

extern float lb_leg ;//后腿
extern float rb_leg ;

extern int8_t pre_step_flag;
extern float pre_step_wheel_speed ;

static float AngleClamp60(float deg);


void CAN_Task(void const * argument){
	Delay_ms(5000);
	float Tnum23 = 360.0f / 8192.0f / 3591.0f *187.0f / 5.0f;
	float Tnum1  = 360.0f / 8192.0f / 36.0f / 94.0f * 19.0f;
	for(;;)
    {

        if(USART_Task_flag == 1U)
		{
			if(pre_step_flag == 0)
        	{
		    	FDCAN2_CMD_1(PID_velocity_realize_2(total_speed.fl,1),
					         PID_velocity_realize_2(-total_speed.fr,2),
					         PID_velocity_realize_2(total_speed.bl,3),
					         PID_velocity_realize_2(-total_speed.br,4)
		    	);

			}
			if(pre_step_flag == 1)
			{
				FDCAN2_CMD_1(PID_velocity_realize_2(pre_step_wheel_speed,1),
					         PID_velocity_realize_2(-pre_step_wheel_speed,2),
				 	         PID_velocity_realize_2(total_speed.bl,3),
				 	         PID_velocity_realize_2(-total_speed.br,4)
		    	);
			}

			FDCAN3_CMD_1(pid_call_3(-AngleClamp60(ctrl_J_USART[0]) / Tnum1, 1),
		    	         pid_call_3( ctrl_J_USART[1] / Tnum23, 2), 
		    	         pid_call_3( ctrl_J_USART[2] / Tnum23 ,3), 
		    	         0
			);
			// FDCAN3_CMD_1(0,
		    // 	         30.0f / Tnum, 
		    // 	         30.0f / Tnum , 
		    // 	         0
			// );


		}
		if(USB_Task_flag == 1U)
		{
            if(pre_step_flag == 0)
            {
		        FDCAN2_CMD_1(PID_velocity_realize_2(total_speed_USB.fl,1),
				             PID_velocity_realize_2(-total_speed_USB.fr,2),
				             PID_velocity_realize_2(total_speed_USB.bl,3),
				             PID_velocity_realize_2(-total_speed_USB.br,4)
		        );

		    }
		    if(pre_step_flag == 1)
		    {
			    FDCAN2_CMD_1(PID_velocity_realize_2(pre_step_wheel_speed,1),
				             PID_velocity_realize_2(-pre_step_wheel_speed,2),
				             PID_velocity_realize_2(total_speed_USB.bl,3),
				             PID_velocity_realize_2(-total_speed_USB.br,4)
		        );
		    }
		    FDCAN3_CMD_1(pid_call_3(-AngleClamp60(ctrl_J_USB[0]) / Tnum1,1),
		                 pid_call_3(ctrl_J_USB[1] / Tnum23, 2), 
		                 pid_call_3( ctrl_J_USB[2] / Tnum23 ,3), 
		                 0
		    );


		    Delay_ms(1);
		}
            pos_ctrl(&hfdcan1,motor[Motor1].id,lf_leg.theta1,0.2);
		    pos_ctrl(&hfdcan1,motor[Motor2].id,lf_leg.theta2,0.2);
		    pos_ctrl(&hfdcan1,motor[Motor3].id,rf_leg.theta1,0.2);
		    pos_ctrl(&hfdcan1,motor[Motor4].id,rf_leg.theta2,0.2);
		    pos_ctrl(&hfdcan1,motor[Motor5].id,lb_leg,1);
		    pos_ctrl(&hfdcan1,motor[Motor6].id,rb_leg,1);

		    // pos_ctrl(&hfdcan1,motor[Motor1].id,0,0.5);
		    // pos_ctrl(&hfdcan1,motor[Motor2].id,0,0.5);
		    // pos_ctrl(&hfdcan1,motor[Motor3].id,0,0.5);
		    // pos_ctrl(&hfdcan1,motor[Motor4].id,0,0.5);
		    // pos_ctrl(&hfdcan1,motor[Motor5].id,0,2);
		    // pos_ctrl(&hfdcan1,motor[Motor6].id,0,2);
        osDelay(1);
    }

	
}


float AngleClamp60(float deg)
{
    if (deg > 60.0f)
        return 60.0f;
    else if (deg < -60.0f)
        return -60.0f;
    else
        return deg;
}
//测试用例
	// FDCAN3_CMD_1(PID_velocity_realize_3(1500,1), 
	// 	         PID_velocity_realize_3(1500,2), 
	// 	         PID_velocity_realize_3(1500,3), 
	// 	         PID_velocity_realize_3(1500,4)
	// );
//		FDCAN3_CMD_1(0, 
//		             0, 
//		             0, 
//		             PID_velocity_realize_3(1500,4)
//		);
//		FDCAN3_CMD_1(0,
//		             pid_call_3(-ctrl_j2 / Tnum, 2), 
//		             pid_call_3( ctrl_j3 / Tnum ,3), 
//		             0
//		);

    // osDelay(1);


	// FDCAN2_CMD_1(PID_velocity_realize_2(1500,1), 
	// 	         PID_velocity_realize_2(1500,2), 
	// 	         PID_velocity_realize_2(1500,3), 
	// 	         PID_velocity_realize_2(1500,4)
	// );

    // osDelay(1);
		// FDCAN2_CMD_1(PID_velocity_realize_2(1500,1), 
		// 	         PID_velocity_realize_2(1500,2), 
		// 	         PID_velocity_realize_2(1500,3), 
		// 	         PID_velocity_realize_2(1500,4)
		// 			);	
		// FDCAN3_CMD_1(PID_velocity_realize_3(1500,1), 
		// 	         PID_velocity_realize_3(1500,2), 
		// 	         PID_velocity_realize_3(1500,3), 
		// 	         PID_velocity_realize_3(1500,4)
		// 			);		
		// FDCAN2_CMD_1(0, 
		// 	         0, 
		// 	         0, 
		// 	         0
		// 			);	
		// FDCAN3_CMD_1(0, 
		// 	         0, 
		// 	         0, 
		// 	         0
		// 			);	
