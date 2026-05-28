#include "Data_Analysis.h"
#include "dm_motor_drv.h"
#include "fdcan.h"
#include "mecanum_classic.h"
#include <string.h>
#include <stddef.h>
#include "pid_user.h"

static float USB_BytesToFloatLE(const uint8_t *buf);
static uint8_t USB_Decode4Float(const uint8_t *datas, uint8_t len,
                                float *d1, float *d2, float *d3, float *d4);
static float Remote_Clamp(float num, float min_val, float max_val);

extern void Mecanum_task_USB(ChassisVel_t *chassis_user, TrapezoidMecanumParam_t *param_user, WheelSpeed_t *speed_user);    //麦克纳姆轮底盘控制处理，专门给USB数据解析调用的接口
extern void LEG_task_USB(float legx,float legy,float h);
extern void Arm_task_USB(float x,float y,float z);

//模式选择
extern uint8_t USB_Task_flag;
extern uint8_t USART_Task_flag ;

//使能标志位
uint8_t Mecanum_control_flag = 0U;
uint8_t LEG_control_flag = 0U;
uint8_t Arm_control_flag = 0U;

//麦克纳姆底盘参数
extern TrapezoidMecanumParam_t mecParam;
ChassisVel_t total_vel_USB = {0};
WheelSpeed_t total_speed_USB = {0};

//腿部位置参数
float legx_USB = 0.0f;//前腿目标点坐标
float legy_USB = 0.0f ;
float leghtheta_USB = 0.0f;//后腿高度对应角度

//机械臂位置参数
float ARM_setX_USB = 0.0f;
float ARM_setY_USB = 0.0f;
float ARM_setZ_USB = 0.0f;

uint8_t state_arm_flag_num = 0U;
uint32_t state_arm_flag_count = 0U;

static void USB_ALL_ENABLE(const uint8_t *datas, uint8_t len);
static void USB_ALL_DISABLE(const uint8_t *datas, uint8_t len);
static void USB_ALL_MODE_SWITCH(const uint8_t *datas, uint8_t len);
static void USB_ALL_STOP(const uint8_t *datas, uint8_t len);
static void USB_ALL_GET_STATUS(const uint8_t *datas, uint8_t len);

static void USB_MEC_ENABLE(const uint8_t *datas, uint8_t len);
static void USB_MEC_DISABLE(const uint8_t *datas, uint8_t len);
static void USB_MEC_SET_TARGET1(const uint8_t *datas, uint8_t len);
static void USB_MEC_SET_TARGET2(const uint8_t *datas, uint8_t len);
static void USB_MEC_STOP(const uint8_t *datas, uint8_t len);
static void USB_MEC_GET_STATUS(const uint8_t *datas, uint8_t len);

static void USB_LEG_ENABLE(const uint8_t *datas, uint8_t len);
static void USB_LEG_DISABLE(const uint8_t *datas, uint8_t len);
static void USB_LEG_SET_TARGET(const uint8_t *datas, uint8_t len);
static void USB_LEG_STOP(const uint8_t *datas, uint8_t len);
static void USB_LEG_GET_STATUS(const uint8_t *datas, uint8_t len);

static void USB_ARM_ENABLE(const uint8_t *datas, uint8_t len);
static void USB_ARM_DISABLE(const uint8_t *datas, uint8_t len);
static void USB_ARM_SET_TARGET(const uint8_t *datas, uint8_t len);
static void USB_ARM_STOP(const uint8_t *datas, uint8_t len);
static void USB_ARM_GET_STATUS(const uint8_t *datas, uint8_t len);

void Data_Analysis(uint8_t cmd, const uint8_t* datas, uint8_t len)
{

    switch (cmd)
    {
        case USB_CMD_ALL_ENABLE:
            USB_ALL_ENABLE(datas, len);
        break;

        case USB_CMD_ALL_DISABLE:
            USB_ALL_DISABLE(datas, len);
        break;

        case USB_CMD_ALL_MODE_SWITCH:
            USB_ALL_MODE_SWITCH(datas, len);
        break;

        case USB_CMD_ALL_STOP:
            USB_ALL_STOP(datas, len);
        break;

        case USB_CMD_ALL_GET_STATUS:
            USB_ALL_GET_STATUS(datas, len);
        break;

        case USB_CMD_MEC_ENABLE:
            USB_MEC_ENABLE(datas, len); 
        break;  

        case USB_CMD_MEC_DISABLE:
            USB_MEC_DISABLE(datas, len);
        break;

        case USB_CMD_MEC_SET_TARGET1:
            USB_MEC_SET_TARGET1(datas, len);
        break;

        case USB_CMD_MEC_SET_TARGET2:
            USB_MEC_SET_TARGET2(datas, len);
        break;

        case USB_CMD_MEC_STOP:
            USB_MEC_STOP(datas, len);
        break;

        case USB_CMD_MEC_GET_STATUS:
            USB_MEC_GET_STATUS(datas, len); 
        break;

        case USB_CMD_LEG_ENABLE:
            USB_LEG_ENABLE(datas, len);
        break;

        case USB_CMD_LEG_DISABLE:
            USB_LEG_DISABLE(datas, len);    
        break;

        case USB_CMD_LEG_SET_TARGET:
            USB_LEG_SET_TARGET(datas, len);
        break;

        case USB_CMD_LEG_STOP:
            USB_LEG_STOP(datas, len);   
        break;

        case USB_CMD_LEG_GET_STATUS:
            USB_LEG_GET_STATUS(datas, len);
        break;

        case USB_CMD_ARM_ENABLE:
            USB_ARM_ENABLE(datas, len); 
        break;

        case USB_CMD_ARM_DISABLE:
            USB_ARM_DISABLE(datas, len);    
        break;  

        case USB_CMD_ARM_SET_TARGET:
            USB_ARM_SET_TARGET(datas, len);
        break;

        case USB_CMD_ARM_STOP:
            USB_ARM_STOP(datas, len);
        break;

        case USB_CMD_ARM_GET_STATUS:
            USB_ARM_GET_STATUS(datas, len); 
        break;
        
        default:
            /* δ֪������� */
        break;
    }
}


static void USB_ALL_ENABLE(const uint8_t *datas, uint8_t len)
{
    Mecanum_control_flag = 1U;
    LEG_control_flag = 1U;  
    Arm_control_flag = 1U;
    USB_MEC_ENABLE(datas, len);
    USB_LEG_ENABLE(datas, len); 
    USB_ARM_ENABLE(datas, len);
}
static void USB_ALL_DISABLE(const uint8_t *datas, uint8_t len)
{
    Mecanum_control_flag = 0U;
    LEG_control_flag = 0U;
    Arm_control_flag = 0U;
    USB_MEC_DISABLE(datas, len);
    USB_LEG_DISABLE(datas, len);
    USB_ARM_DISABLE(datas, len);
}
static void USB_ALL_MODE_SWITCH(const uint8_t *datas, uint8_t len)
{
    USB_Task_flag = 1U;
    USART_Task_flag  = 0U;
}
static void USB_ALL_STOP(const uint8_t *datas, uint8_t len)
{
    USB_MEC_STOP(datas, len);
    USB_LEG_STOP(datas, len);
    USB_ARM_STOP(datas, len);
}
static void USB_ALL_GET_STATUS(const uint8_t *datas, uint8_t len)
{

}
static void USB_MEC_ENABLE(const uint8_t *datas, uint8_t len)
{
    Mecanum_control_flag = 1U;
}
static void USB_MEC_DISABLE(const uint8_t *datas, uint8_t len)
{
    Mecanum_control_flag = 0U;
}
static void USB_MEC_SET_TARGET1(const uint8_t *datas, uint8_t len)
{
    Chassis_Yaw_PID_Clear();
    float vx, vy, vw, angle;

    if (USB_Decode4Float(datas, len, &vx, &vy, &vw, &angle) == 0U)
    {
        /* 这里可以换成底盘自己的错误回传命令 */
        return;
    }

    /* 根据你的底盘约束修改范围 */
    vx = Remote_Clamp(vx, MEC_REMOTE_VX_MIN_RPM, MEC_REMOTE_VX_MAX_RPM);
    vy = Remote_Clamp(vy, MEC_REMOTE_VY_MIN_RPM, MEC_REMOTE_VY_MAX_RPM);
    vw = Remote_Clamp(vw + Chassis_Yaw_Robot_Frame_Ctrl(angle), MEC_REMOTE_VW_MIN_RAD_S, MEC_REMOTE_VW_MAX_RAD_S);

    /* 保存最近一次USB底盘目标 */
    total_vel_USB.vx = vx;
    total_vel_USB.vy = vy;
    total_vel_USB.vw = vw;

    if (USB_Task_flag == 1U)
    {
        if (Mecanum_control_flag == 1U)
        {
            Mecanum_task_USB(&total_vel_USB, &mecParam, &total_speed_USB);
        }
        else 
        {
            total_speed_USB.fl = 0.0f;
            total_speed_USB.fr = 0.0f;
            total_speed_USB.bl = 0.0f;
            total_speed_USB.br = 0.0f;
        }
    }
}
static void USB_MEC_SET_TARGET2(const uint8_t *datas, uint8_t len)
{
    Chassis_Yaw_PID_Clear();
    float vx, vy, vw, angle;

    if (USB_Decode4Float(datas, len, &vx, &vy, &vw, &angle) == 0U)
    {
        /* 这里可以换成底盘自己的错误回传命令 */
        return;
    }

    /* 根据你的底盘约束修改范围 */
    vx = Remote_Clamp(vx, MEC_REMOTE_VX_MIN_RPM, MEC_REMOTE_VX_MAX_RPM);
    vy = Remote_Clamp(vy, MEC_REMOTE_VY_MIN_RPM, MEC_REMOTE_VY_MAX_RPM);
    vw = Remote_Clamp(vw + Chassis_Yaw_World_Frame_Ctrl(angle), MEC_REMOTE_VW_MIN_RAD_S, MEC_REMOTE_VW_MAX_RAD_S);

    /* 保存最近一次USB底盘目标 */
    total_vel_USB.vx = vx;
    total_vel_USB.vy = vy;
    total_vel_USB.vw = vw;

    if (USB_Task_flag == 1U)
    {
        if (Mecanum_control_flag == 1U)
        {
            Mecanum_task_USB(&total_vel_USB, &mecParam, &total_speed_USB);
        }
        else 
        {
            total_speed_USB.fl = 0.0f;
            total_speed_USB.fr = 0.0f;
            total_speed_USB.bl = 0.0f;
            total_speed_USB.br = 0.0f;
        }
    }}
static void USB_MEC_STOP(const uint8_t *datas, uint8_t len)
{
    total_speed_USB.fl = 0.0f;
    total_speed_USB.fr = 0.0f;
    total_speed_USB.bl = 0.0f;
    total_speed_USB.br = 0.0f;
}
static void USB_MEC_GET_STATUS(const uint8_t *datas, uint8_t len)
{

}
static void USB_LEG_ENABLE(const uint8_t *datas, uint8_t len)
{
    LEG_control_flag = 1U;
    dm_motor_enable(&hfdcan1,&motor[Motor1]);
    dm_motor_enable(&hfdcan1,&motor[Motor2]);
    dm_motor_enable(&hfdcan1,&motor[Motor3]);
    dm_motor_enable(&hfdcan1,&motor[Motor4]);
    dm_motor_enable(&hfdcan1,&motor[Motor5]);
    dm_motor_enable(&hfdcan1,&motor[Motor6]);
}
static void USB_LEG_DISABLE(const uint8_t *datas, uint8_t len)
{
    LEG_control_flag = 0U;
    dm_motor_disable(&hfdcan1,&motor[Motor1]);
    dm_motor_disable(&hfdcan1,&motor[Motor2]);
    dm_motor_disable(&hfdcan1,&motor[Motor3]);
    dm_motor_disable(&hfdcan1,&motor[Motor4]);
    dm_motor_disable(&hfdcan1,&motor[Motor5]);
    dm_motor_disable(&hfdcan1,&motor[Motor6]);
}
static void USB_LEG_SET_TARGET(const uint8_t *datas, uint8_t len)
{
    float legx, legy, h;

    if (USB_Decode4Float(datas, len, &legx, &legy, &h, NULL) == 0U)
    {
        /* 这里你可以换成腿部自己的错误回传命令 */
        return;
    }

    /* 这里按你的机构限制改 */
    legx = Remote_Clamp(legx, LEG_REMOTE_X_MIN_MM, LEG_REMOTE_X_MAX_MM);
    legy = Remote_Clamp(legy, LEG_REMOTE_Y_MIN_MM, LEG_REMOTE_Y_MAX_MM);
    h    = Remote_Clamp(h,    LEG_REMOTE_H_MIN_MM, LEG_REMOTE_H_MAX_MM);

    legx_USB = legx;
    legy_USB = legy;
    leghtheta_USB = h;

    if (USB_Task_flag == 1U)
    {
        if(LEG_control_flag == 1U)
        {
            LEG_task_USB(legx, legy, h);
        }
        else 
        {
		    lf_leg.theta1 = 0.0f;
		    lf_leg.theta2 = 0.0f;
		    rf_leg.theta1 = 0.0f;
		    rf_leg.theta2 = 0.0f;
				
		    lf_last_theta1 = 0.0f;
            lf_last_theta2 = 0.0f;
		    rf_last_theta1 = 0.0f;
            rf_last_theta2 = 0.0f;
			
		    lb_leg = 0.0f;
		    rb_leg = 0.0f;
        }        
    }
}
static void USB_LEG_STOP(const uint8_t *datas, uint8_t len)
{
    lf_leg.theta1 = 0.0f;
	lf_leg.theta2 = 0.0f;
	rf_leg.theta1 = 0.0f;
	rf_leg.theta2 = 0.0f;
				
	lf_last_theta1 = 0.0f;
    lf_last_theta2 = 0.0f;
	rf_last_theta1 = 0.0f;
    rf_last_theta2 = 0.0f;
			
	lb_leg = 0.0f;
	rb_leg = 0.0f;
}

static void USB_LEG_GET_STATUS(const uint8_t *datas, uint8_t len)
{

}
static void USB_ARM_ENABLE(const uint8_t *datas, uint8_t len)
{
    Arm_control_flag = 1U;
}
static void USB_ARM_DISABLE(const uint8_t *datas, uint8_t len)
{
    Arm_control_flag = 0U;
}
static void USB_ARM_SET_TARGET(const uint8_t *datas, uint8_t len)
{
    float x, y, z;

    if (USB_Decode4Float(datas, len, &x, &y, &z, NULL) == 0U)
    {
        uint8_t tx_data[2];
        tx_data[0] = ARM_IK_RESULT_PARAM_ERR;
        tx_data[1] = 0U;
        (void)Send_Cmd_Data(USB_CMD_ARM_IK_RESULT, tx_data, 2U);
        return;
    }

    x = Remote_Clamp(x, ARM_REMOTE_X_MIN_MM, ARM_REMOTE_X_MAX_MM);
    y = Remote_Clamp(y, ARM_REMOTE_Y_MIN_MM, ARM_REMOTE_Y_MAX_MM);
    z = Remote_Clamp(z, ARM_REMOTE_Z_MIN_MM, ARM_REMOTE_Z_MAX_MM);

    ARM_setX_USB = x;
    ARM_setY_USB = y;
    ARM_setZ_USB = z;

    if (USB_Task_flag == 1U)
    {
        if(Arm_control_flag == 1U)
        {
            Arm_task_USB(x, y, z);
        }
        else 
        {
            ctrl_J_USB[0] = 0.0f;
	        ctrl_J_USB[1] = 0.0f;
	        ctrl_J_USB[2] = 0.0f;
        }
        // if(state_arm_flag_num != Arm_control_flag)
        // {
        //     state_arm_flag_num = Arm_control_flag;
        //     state_arm_flag_count ++;
        // }

    }
}
static void USB_ARM_STOP(const uint8_t *datas, uint8_t len)
{
    ctrl_J_USB[0] = 0.0f;
	ctrl_J_USB[1] = 0.0f;
	ctrl_J_USB[2] = 0.0f;
}
static void USB_ARM_GET_STATUS(const uint8_t *datas, uint8_t len)
{

}



/* С���ֽ���ת float */
static float USB_BytesToFloatLE(const uint8_t *buf)
{
    union
    {
        uint8_t b[4];
        float   f;
    } u;

    u.b[0] = buf[0];
    u.b[1] = buf[1];
    u.b[2] = buf[2];
    u.b[3] = buf[3];

    return u.f;
}

static uint8_t USB_Decode4Float(const uint8_t *datas, uint8_t len,
                                float *d1, float *d2, float *d3, float *d4)
{
    /* 前3个参数是刚需，必须非空 */
    if ((datas == NULL) || (d1 == NULL) || (d2 == NULL) || (d3 == NULL))
    {
        return 0U;
    }

    /* 兼容 16 字节 (4个float) */
    if (len == 16U && d4 != NULL)
    {
        *d1 = USB_BytesToFloatLE(&datas[0]);
        *d2 = USB_BytesToFloatLE(&datas[4]);
        *d3 = USB_BytesToFloatLE(&datas[8]);
        *d4 = USB_BytesToFloatLE(&datas[12]);
        return 1U;
    }
    /* 兼容 12 字节 (3个float) */
    else if (len == 12U)
    {
        *d1 = USB_BytesToFloatLE(&datas[0]);
        *d2 = USB_BytesToFloatLE(&datas[4]);
        *d3 = USB_BytesToFloatLE(&datas[8]);
        /* 如果外部依然传入了 d4 的地址，保险起见清零 */
        if (d4 != NULL) *d4 = 0.0f; 
        return 1U;
    }

    return 0U; // 长度不符合预期
}
static float Remote_Clamp(float num, float min_val, float max_val)
{
    if (num < min_val)
    {
        return min_val;
    }

    if (num  > max_val)
    {
        return max_val;
    }

    return num;
}

