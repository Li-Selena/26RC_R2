#include "mecanum_classic.h"
//#include "pid_user.h"

ChassisVel_t total_vel = {0,0,0};
WheelSpeed_t total_speed = {0,0,0,0};
MecanumParam_t mecParam = {0.150f,0.5f,0.405f,1500.0f};


static float abs_f(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float max_f(float a, float b)
{
    return (a > b) ? a : b;
}

/**
 * @brief �����ķ�����˶�ѧ
 */


void Mecanum_Calc(
    const ChassisVel_t *chassis,
    const MecanumParam_t *param,
    WheelSpeed_t *wheel)
{
    float r = param->wheel_radius;       // ���Ӱ뾶
    float L = param->wheel_base * 0.5f;  // ����һ�루ǰ�������ĵ��������ĵľ��룩
    float W = param->wheel_track * 0.5f; // �־��һ�루���������ĵ��������ĵľ��룩

    // ��ȡ�����ٶȣ�chassis->vy=ǰ��/���ˣ�chassis->vx=��/��ƽ�ƣ�chassis->vw=��ת��
    float chassis_vx = chassis->vx;  // ��X�᣺����Ϊ��
    float chassis_vy = chassis->vy;  // ��Y�᣺���ϣ�ǰ����Ϊ��
    float chassis_vw = -chassis->vw;  // ��ת����ʱ�루��ת��Ϊ��//chassis->vwΪĿ����ٶ�pid��pid(�����Ƕ�ȡֵ��Ŀ��ֵ)

    // �������Ĺ�ʽ������������ϵ��
    float k = (L + W) * chassis_vw;  // ��ת��ϵ��

    // ����ת�ټ��㣨rad/s����fl=��ǰ��fr=��ǰ��bl=���br=�Һ�
    wheel->fl = (chassis_vy + chassis_vx - k) / r;  // ��ǰ��
    wheel->fr = (chassis_vy - chassis_vx + k) / r;  // ��ǰ��
    wheel->bl = (chassis_vy - chassis_vx - k) / r;  // �����
    wheel->br = (chassis_vy + chassis_vx + k) / r;  // �Һ���

    // �ٶȹ�һ������ֹ��������ת�ٳ����������
    float max_val = 0.0f;
    max_val = max_f(max_val, abs_f(wheel->fl));
    max_val = max_f(max_val, abs_f(wheel->fr));
    max_val = max_f(max_val, abs_f(wheel->bl));
    max_val = max_f(max_val, abs_f(wheel->br));

    if (max_val > param->max_wheel_speed)
    {
        float scale = param->max_wheel_speed / max_val;
        wheel->fl *= scale;
        wheel->fr *= scale;
        wheel->bl *= scale;
        wheel->br *= scale;
    }
}