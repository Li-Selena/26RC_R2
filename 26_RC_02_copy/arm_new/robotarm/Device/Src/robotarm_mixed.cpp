#include "robotarm_mixed.h"

#include "Robstride.h"

#include <string.h>

/* EL05 使用 RobStride 类库发送 MIT 标准帧。
 * 第二个参数 true 表示强制使用 MIT 模式，不走灵足私有扩展帧协议。
 */
RobStride_Motor robotarm_el05(ROBOTARM_EL05_ID, true);
chassis_t chassis_move;

volatile uint8_t el05_enable_sent = 0U;
volatile uint8_t el05_feedback_ok = 0U;
volatile uint32_t el05_rx_cnt = 0U;
volatile float el05_angle = 0.0f;
volatile float el05_speed = 0.0f;
volatile float el05_torque = 0.0f;
volatile uint32_t el05_tx_id = 0U;
volatile uint8_t el05_tx_data[8] = {0U};
volatile uint8_t el05_tx_status = 0U;

/* 软件使能标志：
 * 只有 RobotArm_Mixed_Enable() 执行后，RobotArm_Mixed_Control() 才会真正发控制帧。
 */
static uint8_t robotarm_mixed_enabled = 0U;

static uint16_t robotarm_el05_float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x - x_min;
    uint32_t max_int = (1UL << bits) - 1UL;

    if (x <= x_min)
    {
        return 0U;
    }
    if (x >= x_max)
    {
        return (uint16_t)max_int;
    }

    return (uint16_t)((offset * (float)max_int) / span);
}

static float robotarm_el05_uint_to_float(uint16_t x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    uint32_t max_int = (1UL << bits) - 1UL;

    return ((float)x) * span / ((float)max_int) + x_min;
}

static uint8_t robotarm_el05_send_raw(uint8_t data[8])
{
    el05_tx_id = ROBOTARM_EL05_ID;
    for (uint8_t i = 0U; i < 8U; i++)
    {
        el05_tx_data[i] = data[i];
    }

    el05_tx_status = canx_send_data(&hfdcan3, ROBOTARM_EL05_ID, data, 8U);
    return el05_tx_status;
}

static uint8_t robotarm_el05_send_enable(void)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU};
    return robotarm_el05_send_raw(data);
}

static uint8_t robotarm_el05_send_disable(void)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFDU};
    return robotarm_el05_send_raw(data);
}

static uint8_t robotarm_el05_send_zero(void)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFEU};
    return robotarm_el05_send_raw(data);
}

static uint8_t robotarm_el05_send_mit_control(float pos, float vel, float kp, float kd, float tor)
{
    uint8_t data[8] = {0U};
    uint16_t p = robotarm_el05_float_to_uint(pos, -12.57f, 12.57f, 16U);
    uint16_t v = robotarm_el05_float_to_uint(vel, -50.0f, 50.0f, 12U);
    uint16_t k_p = robotarm_el05_float_to_uint(kp, 0.0f, 500.0f, 12U);
    uint16_t k_d = robotarm_el05_float_to_uint(kd, 0.0f, 5.0f, 12U);
    uint16_t t = robotarm_el05_float_to_uint(tor, -6.0f, 6.0f, 12U);

    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)((v << 4) | (k_p >> 8));
    data[4] = (uint8_t)k_p;
    data[5] = (uint8_t)(k_d >> 4);
    data[6] = (uint8_t)((k_d << 4) | (t >> 8));
    data[7] = (uint8_t)t;

    return robotarm_el05_send_raw(data);
}

static void robotarm_el05_parse_mit_feedback(uint8_t *data)
{
    uint16_t p;
    uint16_t v;
    uint16_t t;

    if (data[0] != ROBOTARM_EL05_ID)
    {
        return;
    }

    p = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
    v = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
    t = (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8) | data[5]);

    el05_feedback_ok = 1U;
    el05_rx_cnt++;
    el05_angle = robotarm_el05_uint_to_float(p, -12.57f, 12.57f, 16U);
    el05_speed = robotarm_el05_uint_to_float(v, -50.0f, 50.0f, 12U);
    el05_torque = robotarm_el05_uint_to_float(t, -6.0f, 6.0f, 12U);
}

void RobotArm_Mixed_Init(void)
{
    /* 清空达妙反馈、CAN 诊断等全局状态，避免上一次运行留下旧数据。 */
    memset(&chassis_move, 0, sizeof(chassis_move));
    el05_enable_sent = 0U;
    el05_feedback_ok = 0U;
    el05_rx_cnt = 0U;
    el05_angle = 0.0f;
    el05_speed = 0.0f;
    el05_torque = 0.0f;
    el05_tx_id = 0U;
    el05_tx_status = 0U;
    for (uint8_t i = 0U; i < 8U; i++)
    {
        el05_tx_data[i] = 0U;
    }

    /* 这两个 mode 只是给达妙反馈结构做标记，真正的 MIT 使能在 Enable 函数里发。 */
    chassis_move.joint_motor[0].mode = MIT_MODE;
    chassis_move.joint_motor[1].mode = MIT_MODE;
}

void RobotArm_Mixed_Enable(void)
{
    /* 上电后连续发 3 轮使能帧，提高刚上电或 CAN 总线刚稳定时的成功率。
     * 达妙和 EL05 都是 MIT 模式，所以都是标准帧：FF FF FF FF FF FF FF FC。
     */
    for (uint8_t retry = 0U; retry < 3U; retry++)
    {
        (void)dm8006_enter_motor_mode(&hfdcan3, ROBOTARM_DM8006_1_ID);
        HAL_Delay(2);
        (void)dm8006_enter_motor_mode(&hfdcan3, ROBOTARM_DM8006_2_ID);
        HAL_Delay(2);
        (void)robotarm_el05_send_enable();
        el05_enable_sent = 1U;
        HAL_Delay(5);
    }

    robotarm_mixed_enabled = 1U;
}

void RobotArm_Mixed_Disable(void)
{
    /* 先关软件标志，防止失能过程中主循环继续发控制帧。 */
    robotarm_mixed_enabled = 0U;
    el05_enable_sent = 0U;

    /* MIT 失能帧一般是 FF FF FF FF FF FF FF FD。 */
    (void)robotarm_el05_send_disable();
    HAL_Delay(2);
    (void)dm8006_exit_motor_mode(&hfdcan3, ROBOTARM_DM8006_1_ID);
    HAL_Delay(2);
    (void)dm8006_exit_motor_mode(&hfdcan3, ROBOTARM_DM8006_2_ID);
}

void RobotArm_EL05_MIT_Enable(void)
{
    (void)robotarm_el05_send_enable();
    el05_enable_sent = 1U;
}

void RobotArm_EL05_MIT_Disable(void)
{
    (void)robotarm_el05_send_disable();
    el05_enable_sent = 0U;
}

void RobotArm_EL05_MIT_Control(float pos, float vel, float kp, float kd, float tor)
{
    (void)robotarm_el05_send_mit_control(pos, vel, kp, kd, tor);
}

void RobotArm_EL05_MIT_SetZero(void)
{
    (void)robotarm_el05_send_zero();
}

void RobotArm_EL05_SetProtocol(uint8_t protocol)
{
    robotarm_el05.RobStride_Motor_MotorModeSet(protocol);
}

void RobotArm_EL05_SetProtocolMIT(void)
{
    RobotArm_EL05_SetProtocol(ROBOTARM_EL05_PROTOCOL_MIT);
}

void RobotArm_Mixed_Control(const RobotArm_MixedCommand_t *cmd)
{
    /* 防呆：cmd 为空或还没使能，就不允许发控制帧。 */
    if ((cmd == NULL) || (robotarm_mixed_enabled == 0U))
    {
        return;
    }

    /* 发送达妙 1 的 MIT 控制帧。
     * MIT 输出关系可以理解为：
     * torque = kp * (pos_target - pos_now)
     *        + kd * (vel_target - vel_now)
     *        + tor_ff
     */
    (void)dm8006_send_mit_command(&hfdcan3,
                                  ROBOTARM_DM8006_1_ID,
                                  cmd->dm8006_1.pos,
                                  cmd->dm8006_1.vel,
                                  cmd->dm8006_1.kp,
                                  cmd->dm8006_1.kd,
                                  cmd->dm8006_1.tor);

    /* 发送达妙 2 的 MIT 控制帧。 */
    (void)dm8006_send_mit_command(&hfdcan3,
                                  ROBOTARM_DM8006_2_ID,
                                  cmd->dm8006_2.pos,
                                  cmd->dm8006_2.vel,
                                  cmd->dm8006_2.kp,
                                  cmd->dm8006_2.kd,
                                  cmd->dm8006_2.tor);

    /* 发送 EL05 的 MIT 控制帧。参数顺序和达妙封装后的接口保持一致：
     * pos, vel, kp, kd, tor。
     */
    (void)robotarm_el05_send_mit_control(cmd->el05.pos,
                                         cmd->el05.vel,
                                         cmd->el05.kp,
                                         cmd->el05.kd,
                                         cmd->el05.tor);
}

void RobotArm_Mixed_HandleRx(const FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    /* FDCAN 中断里会调用这个函数，先检查指针，避免异常数据导致 HardFault。 */
    if ((rx_header == NULL) || (rx_data == NULL))
    {
        return;
    }

    /* 诊断计数：调试时可以看 g_can_diag.rx_cnt 是否增长，判断有没有收到反馈。 */
    g_can_diag.rx_cnt++;

    /* 当前混合控制统一使用 MIT 标准帧。
     * 如果收到扩展帧，这里直接忽略，避免灵足旧私有协议影响当前逻辑。
     */
    if (rx_header->IdType == FDCAN_STANDARD_ID)
    {
        /* 根据标准帧 ID 区分是哪台电机的反馈。 */
        switch (rx_header->Identifier)
        {
        case ROBOTARM_DM8006_1_ID:
            /* 达妙 1 反馈解析后存到 chassis_move.joint_motor[0]。 */
            dm8006_fbdata(&chassis_move.joint_motor[0], rx_data, rx_header->DataLength);
            break;

        case ROBOTARM_DM8006_2_ID:
            /* 达妙 2 反馈解析后存到 chassis_move.joint_motor[1]。 */
            dm8006_fbdata(&chassis_move.joint_motor[1], rx_data, rx_header->DataLength);
            break;

        case ROBOTARM_EL05_ID:
        case ROBOTARM_EL05_MIT_RX_ID:
            /* EL05 MIT 反馈可能用电机 ID 0x03，也可能用主机 ID 0xFD。
             * 只有 8 字节反馈才交给 RobStride_Motor_Analysis 解析。
             */
            if (bsp_fdcan_dlc_to_len(rx_header->DataLength) >= 8U)
            {
                robotarm_el05_parse_mit_feedback(rx_data);
            }
            break;

        default:
            break;
        }
    }
}

extern "C" void FDCAN3_RxMessageCallback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    RobotArm_Mixed_HandleRx(rx_header, rx_data);
}

void leg_angle (int a)
{
       switch(a)
			{
				 case 0 :
				 {(void)dm8006_send_mit_command(&hfdcan3,
        ROBOTARM_DM8006_1_ID,
        -0.0f,//负号往前转
        0.5f,
        2.8f,
        0.3f,
        0.0f);
					 break; }

				case 1 :
				 {(void)dm8006_send_mit_command(&hfdcan3,
        ROBOTARM_DM8006_1_ID,
        1.04f,//负号往前转
        0.5f,
        2.8f,
        0.3f,
        0.0f);
					 break; }
				 
				 case 2 :
				 {(void)dm8006_send_mit_command(&hfdcan3,
        ROBOTARM_DM8006_1_ID,
        -2.08f,//负号往前转
        0.1f,
        3.8f,
        0.3f,
        0.0f);
					 break; }
				 	case 3 :
				 {(void)dm8006_send_mit_command(&hfdcan3,
        ROBOTARM_DM8006_1_ID,
        -5.695f,//负号往前转
        0.1f,
        1.8f,
        0.3f,
        0.0f);
					 break; }
				}
}
