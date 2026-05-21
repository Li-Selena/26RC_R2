#include "CRC.h"
#include "usart.h"
#include "struct_typedef.h"
#include "math.h"
#include <string.h>



/* ���ھ�� */
extern UART_HandleTypeDef huart10;

/* ȫ�ֽ���״̬ */
ParseState BT_Uart10 = STATE_WAIT_HEADER;
uint8_t bt_data[BT_FRAME_DATA_LEN];
uint8_t data_index = 0;
uint8_t checksum = 0;
volatile uint8_t bt_parse_ok = 0;
uint8_t btReceiveData = 0;

uint8_t USB_Task_flag = 0;
uint8_t USART_Task_flag = 0;
uint8_t UU_flag = 0;


/* �Ȳ������� */
int8_t leg_flag = 0;
float legx = 0.0f;
float legy = 0.0f;
float leghtheta = 0.0f;

/* ��е�ۿ����� */
int8_t arm_flag = 0;
float arm_X = 0.0f;
float arm_Y = 0.0f;
float arm_Z = 0.0f;

float mid_X = 0.0f;
float mid_Y = 0.0f; 
float mid_Z = 0.0f;


//上台阶前轮转速
int8_t pre_step_flag = 0;
float pre_step_wheel_speed = 0.0f;


static int8_t RemoteArm_ClampRawInt8(int8_t raw, int8_t min_val, int8_t max_val);
static float RemoteArm_MapToRange(float raw, float in_min, float in_max, float out_min, float out_max);
static float RemoteArm_Clamp(float x, float min_val, float max_val);

/*
 * UART10 ���ֽڽ���״̬��
 *
 * ֡��ʽ��
 *   0xA5 + 11�ֽ����� + 1�ֽ�У��� + 0x5A
 *
 * У�����
 *   checksum = data[0] + data[1] + ... + data[10]
 *   ȡ��8λ
 */
void UART10_Receive(uint8_t receiveData)
{
    switch (BT_Uart10)
    {
        case STATE_WAIT_HEADER:
        {
            if (receiveData == 0xA5U)
            {
                BT_Uart10 = STATE_RECV_DATA;
                data_index = 0;
                memset(bt_data, 0, sizeof(bt_data));
            }
        }
        break;

        case STATE_RECV_DATA:
        {
            bt_data[data_index++] = receiveData;

            if (data_index >= BT_FRAME_DATA_LEN)
            {
                BT_Uart10 = STATE_RECV_CHECKSUM;
            }
        }
        break;

        case STATE_RECV_CHECKSUM:
        {
            checksum = receiveData;
            BT_Uart10 = STATE_RECV_TAIL;
        }
        break;

        case STATE_RECV_TAIL:
        {
            if (receiveData == 0x5AU)
            {
                uint8_t calc_checksum = 0U;
                uint8_t i;

                for (i = 0; i < BT_FRAME_DATA_LEN; i++)
                {
                    calc_checksum += bt_data[i];
                }

                if (calc_checksum == checksum)
                {
                    bt_parse_ok = 1U;
                }
            }

            /* ���۳ɹ�ʧ�ܣ��ص��ȴ���ͷ */
            BT_Uart10 = STATE_WAIT_HEADER;
            data_index = 0;
        }
        break;

        default:
        {
            BT_Uart10 = STATE_WAIT_HEADER;
            data_index = 0;
        }
        break;
    }
}

/*
 * ��������һ֡ң��������
 *
 * ע�⣺
 * ���ﲻ�ٴ����洫 byte[] ������
 * ֱ�Ӷ�ȡȫ�� bt_data����������һ�ݱ��ؿ��գ�
 * �����ж���д����һ֡ʱ�ѵ�ǰ�������̴�ϡ�
 */
void BT_Data_MAC_Process(float *V_x, float *V_y, float *V_w, int8_t *cmd)
{
    uint8_t frame[BT_FRAME_DATA_LEN];

    (void)cmd; /* �㵱ǰ������û�����õ� cmd�������ȱ����ӿ� */

    if ((V_x == 0) || (V_y == 0) || (V_w == 0))
    {
        return;
    }

    if (bt_parse_ok == 0U)
    {
        return;
    }

    __disable_irq();
    memcpy(frame, bt_data, sizeof(frame));
    bt_parse_ok = 0U;
    __enable_irq();

    /* ���̿��� */
    *V_x = (int8_t)frame[0] / 128.0f * 256.0f;
    *V_y = (int8_t)frame[1] / 128.0f * 256.0f;
    *V_w = (int8_t)frame[2] / 128.0f * 256.0f;

    /* �Ȳ����� */
    legx = (float)(int8_t)frame[3] / 128.0f * 180.0f;
    legy = (float)(int8_t)frame[4] / 128.0f * 180.0f;
    leghtheta = (float)(int8_t)frame[5] / 128.0f * 55.0f;

    if (fabsf(legx) >= 180.0f)      legx = (legx > 0.0f) ? 180.0f : -180.0f;
    if (fabsf(legy) >= 180.0f)      legy = (legy > 0.0f) ? 180.0f : -180.0f;
    if (fabsf(leghtheta) >= 55.0f)  leghtheta = (leghtheta > 0.0f) ? 55.0f : -55.0f;

    leg_flag = (int8_t)frame[6];

    /* ��е�ۿ��� */
    arm_flag = (int8_t)frame[7];
    // 步骤1：提取原始值（int8_t）
    int8_t raw_x = (int8_t)frame[8];
    int8_t raw_y = (int8_t)frame[9];
    int8_t raw_z = (int8_t)frame[10];

    // 步骤2：原始值截断（确保不超出通信约定的安全范围）
    raw_x = RemoteArm_ClampRawInt8(raw_x, ARM_REMOTE_RAW_XY_MIN, ARM_REMOTE_RAW_XY_MAX);
    raw_y = RemoteArm_ClampRawInt8(raw_y, ARM_REMOTE_RAW_XY_MIN, ARM_REMOTE_RAW_XY_MAX);
    raw_z = RemoteArm_ClampRawInt8(raw_z, ARM_REMOTE_RAW_Z_MIN, ARM_REMOTE_RAW_Z_MAX);

    // 步骤3：映射到机械臂物理范围（mm）
    mid_X = RemoteArm_MapToRange((float)raw_x, 
                                ARM_REMOTE_RAW_XY_MIN, ARM_REMOTE_RAW_XY_MAX,
                                ARM_REMOTE_X_MIN_MM, ARM_REMOTE_X_MAX_MM);
    mid_Y = RemoteArm_MapToRange((float)raw_y, 
                                ARM_REMOTE_RAW_XY_MIN, ARM_REMOTE_RAW_XY_MAX,
                                ARM_REMOTE_Y_MIN_MM, ARM_REMOTE_Y_MAX_MM);
    mid_Z = RemoteArm_MapToRange((float)raw_z, 
                                ARM_REMOTE_RAW_Z_MIN, ARM_REMOTE_RAW_Z_MAX,
                                ARM_REMOTE_Z_MIN_MM, ARM_REMOTE_Z_MAX_MM);

    // 步骤4：最终物理范围校验（双重保险，防止映射计算误差）
    mid_X = RemoteArm_Clamp(mid_X, ARM_REMOTE_X_MIN_MM, ARM_REMOTE_X_MAX_MM);
    mid_Y = RemoteArm_Clamp(mid_Y, ARM_REMOTE_Y_MIN_MM, ARM_REMOTE_Y_MAX_MM);
    mid_Z = RemoteArm_Clamp(mid_Z, ARM_REMOTE_Z_MIN_MM, ARM_REMOTE_Z_MAX_MM);    

    if (arm_flag == 1)
    {
        arm_X = mid_X;
        arm_Y = mid_Y;
        arm_Z = mid_Z;
    }
    else
    {
        arm_X = 0.0f;
        arm_Y = 0.0f;
        arm_Z = 0.0f;
    }
    UU_flag = frame[11];
    if(UU_flag == 0U)
    {
        USART_Task_flag = 1U;
        USB_Task_flag = 0U;
    }
    if(UU_flag == 1U)
    {
        USB_Task_flag = 1U;
        USART_Task_flag = 0U;
    }

    pre_step_flag = frame[12];
    pre_step_wheel_speed = (int8_t)frame[13] / 128.0f * 2048.0f;
   

}


// 新增：原始值截断函数（针对int8_t类型）
static int8_t RemoteArm_ClampRawInt8(int8_t raw, int8_t min_val, int8_t max_val)
{
    if (raw < min_val)
    {
        return min_val;
    }
    if (raw > max_val)
    {
        return max_val;
    }
    return raw;
}

// 原有：浮点数截断函数（保留）
static float RemoteArm_Clamp(float x, float min_val, float max_val)
{
    if (x < min_val)
    {
        return min_val;
    }
    if (x > max_val)
    {
        return max_val;
    }
    return x;
}

// 改造：通用映射函数（支持任意输入范围→输出范围）
static float RemoteArm_MapToRange(float raw, float in_min, float in_max, float out_min, float out_max)
{
    // 避免除零（输入范围无效时返回输出最小值）
    if (in_max - in_min < 1e-6f)
    {
        return out_min;
    }
    // 归一化→映射到目标范围
    return out_min + (raw - in_min) * (out_max - out_min) / (in_max - in_min);
}