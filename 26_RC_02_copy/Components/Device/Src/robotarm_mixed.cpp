#include "robotarm_mixed.h"

#include "cmsis_os.h"
#include "Robstride.h"

#include <string.h>

/* EL05 使用 RobStride 类库发送 MIT 标准帧。
 * 第二个参数 true 表示强制使用 MIT 模式，不走灵足私有扩展帧协议。
 */
RobStride_Motor robotarm_el05(ROBOTARM_EL05_ID, true);
chassis_t chassis_move;
volatile RobotArm_FDCAN3Feedback_t g_robotarm_fdcan3_feedback = {0};
volatile RobotArm_FDCAN3RxDebug_t g_robotarm_fdcan3_rx_debug = {0};
volatile RobotArm_FDCAN3TxDebug_t g_robotarm_fdcan3_tx_debug = {0};

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
 * RobotArm_Mixed_Control()/SetControlCommand() 只更新缓存目标；
 * 只有 RobotArm_Mixed_Enable() 执行后，TIM5 调用的 Transmit1ms 才会发控制帧。
 */
static uint8_t robotarm_mixed_enabled = 0U;
static volatile uint8_t robotarm_control_cmd_valid = 0U;
static RobotArm_MixedCommand_t robotarm_control_cmd;
static uint32_t robotarm_dm8006_1_rx_cnt = 0U;
static uint32_t robotarm_dm8006_2_rx_cnt = 0U;

#define ROBOTARM_FDCAN3_TEST_SPEED_RAD_S     0.0f
#define ROBOTARM_FDCAN3_TEST_KP              0.0f

static const RobotArm_MixedConfig_t robotarm_default_config = {
    ROBOTARM_COMM_POS_SPEED,
    ROBOTARM_COMM_POS_SPEED,
    ROBOTARM_COMM_POS_SPEED,
    0U
};

static RobotArm_MixedConfig_t robotarm_config = {
    ROBOTARM_COMM_POS_SPEED,
    ROBOTARM_COMM_POS_SPEED,
    ROBOTARM_COMM_POS_SPEED,
    0U
};

static RobotArm_MotorCommMode_t robotarm_sanitize_mode(RobotArm_MotorCommMode_t mode)
{
    if (mode == ROBOTARM_COMM_MIT)
    {
        return ROBOTARM_COMM_MIT;
    }
    return ROBOTARM_COMM_POS_SPEED;
}

static uint16_t robotarm_comm_to_dm_mode(RobotArm_MotorCommMode_t mode)
{
    return (mode == ROBOTARM_COMM_MIT) ? MIT_MODE : POS_MODE;
}

static void robotarm_feedback_reset(void)
{
    memset((void *)&g_robotarm_fdcan3_feedback, 0, sizeof(g_robotarm_fdcan3_feedback));

    g_robotarm_fdcan3_feedback.dm8006_1.id = ROBOTARM_DM8006_1_ID;
    g_robotarm_fdcan3_feedback.dm8006_1.mode = robotarm_comm_to_dm_mode(robotarm_config.dm8006_1_mode);
    g_robotarm_fdcan3_feedback.dm8006_2.id = ROBOTARM_DM8006_2_ID;
    g_robotarm_fdcan3_feedback.dm8006_2.mode = robotarm_comm_to_dm_mode(robotarm_config.dm8006_2_mode);
    g_robotarm_fdcan3_feedback.el05_3.id = ROBOTARM_EL05_ID;
    g_robotarm_fdcan3_feedback.el05_3.mode = robotarm_comm_to_dm_mode(robotarm_config.el05_mode);

    robotarm_dm8006_1_rx_cnt = 0U;
    robotarm_dm8006_2_rx_cnt = 0U;
}

static void robotarm_apply_modes_from_config(void)
{
    chassis_move.joint_motor[0].mode = robotarm_comm_to_dm_mode(robotarm_config.dm8006_1_mode);
    chassis_move.joint_motor[1].mode = robotarm_comm_to_dm_mode(robotarm_config.dm8006_2_mode);

    g_robotarm_fdcan3_feedback.dm8006_1.mode =
        robotarm_comm_to_dm_mode(robotarm_config.dm8006_1_mode);
    g_robotarm_fdcan3_feedback.dm8006_2.mode =
        robotarm_comm_to_dm_mode(robotarm_config.dm8006_2_mode);
    g_robotarm_fdcan3_feedback.el05_3.mode =
        robotarm_comm_to_dm_mode(robotarm_config.el05_mode);
}

static void robotarm_control_cmd_reset_from_config(void)
{
    memset(&robotarm_control_cmd, 0, sizeof(robotarm_control_cmd));
    robotarm_control_cmd.dm8006_1.mode = robotarm_config.dm8006_1_mode;
    robotarm_control_cmd.dm8006_2.mode = robotarm_config.dm8006_2_mode;
    robotarm_control_cmd.el05.mode = robotarm_config.el05_mode;
}

static void robotarm_apply_modes_from_command(RobotArm_MixedCommand_t *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    cmd->dm8006_1.mode = robotarm_sanitize_mode(cmd->dm8006_1.mode);
    cmd->dm8006_2.mode = robotarm_sanitize_mode(cmd->dm8006_2.mode);
    cmd->el05.mode = robotarm_sanitize_mode(cmd->el05.mode);

    robotarm_config.dm8006_1_mode = cmd->dm8006_1.mode;
    robotarm_config.dm8006_2_mode = cmd->dm8006_2.mode;
    robotarm_config.el05_mode = cmd->el05.mode;
    robotarm_apply_modes_from_config();
}

static void robotarm_mixed_send_now(const RobotArm_MixedCommand_t *cmd);
static void robotarm_tx_debug_record(const RobotArm_MixedCommand_t *cmd);
static uint8_t robotarm_handle_dm_feedback(uint8_t payload_id,
                                           const FDCAN_RxHeaderTypeDef *rx_header,
                                           uint8_t *rx_data);

static void robotarm_feedback_update_dm(volatile RobotArm_MotorFeedback_t *out,
                                        const Joint_Motor_t *motor,
                                        uint16_t id,
                                        uint32_t rx_cnt)
{
    if ((out == NULL) || (motor == NULL))
    {
        return;
    }

    out->id = id;
    out->mode = motor->mode;
    out->state = motor->para.state;
    out->rx_cnt = rx_cnt;
    out->feedback_ok = 1U;
    out->p_int = motor->para.p_int;
    out->v_int = motor->para.v_int;
    out->t_int = motor->para.t_int;
    out->pos = motor->para.pos;
    out->vel = motor->para.vel;
    out->tor = motor->para.tor;
    out->tmos = motor->para.Tmos;
    out->tcoil = motor->para.Tcoil;
}

static void robotarm_rx_debug_record(const FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    uint8_t len;
    uint8_t i;

    if ((rx_header == NULL) || (rx_data == NULL))
    {
        return;
    }

    len = bsp_fdcan_dlc_to_len(rx_header->DataLength);
    if (len > 8U)
    {
        len = 8U;
    }

    g_robotarm_fdcan3_rx_debug.total_rx_cnt++;
    g_robotarm_fdcan3_rx_debug.last_identifier = (uint16_t)rx_header->Identifier;
    g_robotarm_fdcan3_rx_debug.last_dlc_len = len;
    g_robotarm_fdcan3_rx_debug.last_payload_id = rx_data[0] & 0x0FU;

    for (i = 0U; i < len; i++)
    {
        g_robotarm_fdcan3_rx_debug.last_data[i] = rx_data[i];
    }
    for (; i < 8U; i++)
    {
        g_robotarm_fdcan3_rx_debug.last_data[i] = 0U;
    }
}

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

static void robotarm_tx_debug_pack_mit(uint8_t data[8],
                                       float pos,
                                       float vel,
                                       float kp,
                                       float kd,
                                       float tor,
                                       float p_min,
                                       float p_max,
                                       float v_min,
                                       float v_max,
                                       float kp_max,
                                       float kd_max,
                                       float t_min,
                                       float t_max)
{
    uint16_t p = robotarm_el05_float_to_uint(pos, p_min, p_max, 16U);
    uint16_t v = robotarm_el05_float_to_uint(vel, v_min, v_max, 12U);
    uint16_t k_p = robotarm_el05_float_to_uint(kp, 0.0f, kp_max, 12U);
    uint16_t k_d = robotarm_el05_float_to_uint(kd, 0.0f, kd_max, 12U);
    uint16_t t = robotarm_el05_float_to_uint(tor, t_min, t_max, 12U);

    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)(((v & 0x0FU) << 4) | (k_p >> 8));
    data[4] = (uint8_t)k_p;
    data[5] = (uint8_t)(k_d >> 4);
    data[6] = (uint8_t)(((k_d & 0x0FU) << 4) | (t >> 8));
    data[7] = (uint8_t)t;
}

static void robotarm_tx_debug_pack_pos_speed(uint8_t data[8], float pos, float vel)
{
    memcpy(&data[0], &pos, sizeof(pos));
    memcpy(&data[4], &vel, sizeof(vel));
}

static uint8_t robotarm_el05_send_raw(uint16_t id, uint8_t data[8])
{
    el05_tx_id = id;
    for (uint8_t i = 0U; i < 8U; i++)
    {
        el05_tx_data[i] = data[i];
    }

    el05_tx_status = canx_send_data(&hfdcan3, id, data, 8U);
    return el05_tx_status;
}

static uint8_t robotarm_el05_send_run_mode(RobotArm_MotorCommMode_t mode)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x00U, 0xFCU};

    data[6] = (mode == ROBOTARM_COMM_MIT) ? 0U : 1U;
    return robotarm_el05_send_raw(ROBOTARM_EL05_ID, data);
}

static uint16_t robotarm_el05_command_id(RobotArm_MotorCommMode_t mode)
{
    if (mode == ROBOTARM_COMM_MIT)
    {
        return ROBOTARM_EL05_ID;
    }
    return (uint16_t)(ROBOTARM_EL05_ID | POS_MODE);
}

static void robotarm_tx_debug_record(const RobotArm_MixedCommand_t *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    g_robotarm_fdcan3_tx_debug.tx_cnt++;
    memcpy((void *)&g_robotarm_fdcan3_tx_debug.last_cmd, cmd, sizeof(*cmd));

    g_robotarm_fdcan3_tx_debug.dm8006_1_tx_id =
        (uint16_t)(ROBOTARM_DM8006_1_ID |
                   ((robotarm_config.dm8006_1_mode == ROBOTARM_COMM_MIT) ? MIT_MODE : POS_MODE));
    g_robotarm_fdcan3_tx_debug.dm8006_2_tx_id =
        (uint16_t)(ROBOTARM_DM8006_2_ID |
                   ((robotarm_config.dm8006_2_mode == ROBOTARM_COMM_MIT) ? MIT_MODE : POS_MODE));
    g_robotarm_fdcan3_tx_debug.el05_tx_id = robotarm_el05_command_id(robotarm_config.el05_mode);

    if (robotarm_config.dm8006_1_mode == ROBOTARM_COMM_MIT)
    {
        robotarm_tx_debug_pack_mit((uint8_t *)g_robotarm_fdcan3_tx_debug.dm8006_1_data,
                                   cmd->dm8006_1.pos,
                                   cmd->dm8006_1.vel,
                                   cmd->dm8006_1.kp,
                                   cmd->dm8006_1.kd,
                                   cmd->dm8006_1.tor,
                                   DM8006_P_MIN,
                                   DM8006_P_MAX,
                                   DM8006_V_MIN,
                                   DM8006_V_MAX,
                                   DM8006_KP_MAX,
                                   DM8006_KD_MAX,
                                   DM8006_T_MIN,
                                   DM8006_T_MAX);
    }
    else
    {
        robotarm_tx_debug_pack_pos_speed((uint8_t *)g_robotarm_fdcan3_tx_debug.dm8006_1_data,
                                         cmd->dm8006_1.pos,
                                         cmd->dm8006_1.vel);
    }

    if (robotarm_config.dm8006_2_mode == ROBOTARM_COMM_MIT)
    {
        robotarm_tx_debug_pack_mit((uint8_t *)g_robotarm_fdcan3_tx_debug.dm8006_2_data,
                                   cmd->dm8006_2.pos,
                                   cmd->dm8006_2.vel,
                                   cmd->dm8006_2.kp,
                                   cmd->dm8006_2.kd,
                                   cmd->dm8006_2.tor,
                                   DM8006_P_MIN,
                                   DM8006_P_MAX,
                                   DM8006_V_MIN,
                                   DM8006_V_MAX,
                                   DM8006_KP_MAX,
                                   DM8006_KD_MAX,
                                   DM8006_T_MIN,
                                   DM8006_T_MAX);
    }
    else
    {
        robotarm_tx_debug_pack_pos_speed((uint8_t *)g_robotarm_fdcan3_tx_debug.dm8006_2_data,
                                         cmd->dm8006_2.pos,
                                         cmd->dm8006_2.vel);
    }

    if (robotarm_config.el05_mode == ROBOTARM_COMM_MIT)
    {
        robotarm_tx_debug_pack_mit((uint8_t *)g_robotarm_fdcan3_tx_debug.el05_data,
                                   cmd->el05.pos,
                                   cmd->el05.vel,
                                   cmd->el05.kp,
                                   cmd->el05.kd,
                                   cmd->el05.tor,
                                   -12.57f,
                                   12.57f,
                                   -50.0f,
                                   50.0f,
                                   500.0f,
                                   5.0f,
                                   -6.0f,
                                   6.0f);
    }
    else
    {
        robotarm_tx_debug_pack_pos_speed((uint8_t *)g_robotarm_fdcan3_tx_debug.el05_data,
                                         cmd->el05.pos,
                                         cmd->el05.vel);
    }
}

static uint8_t robotarm_el05_send_enable(RobotArm_MotorCommMode_t mode)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU};
    return robotarm_el05_send_raw(robotarm_el05_command_id(mode), data);
}

static uint8_t robotarm_el05_send_disable(RobotArm_MotorCommMode_t mode)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFDU};
    return robotarm_el05_send_raw(robotarm_el05_command_id(mode), data);
}

static uint8_t robotarm_el05_send_zero(RobotArm_MotorCommMode_t mode)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFEU};
    return robotarm_el05_send_raw(robotarm_el05_command_id(mode), data);
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

    return robotarm_el05_send_raw(robotarm_el05_command_id(ROBOTARM_COMM_MIT), data);
}

static uint8_t robotarm_el05_send_pos_speed_control(float pos, float vel)
{
    uint8_t data[8] = {0U};

    memcpy(&data[0], &pos, sizeof(pos));
    memcpy(&data[4], &vel, sizeof(vel));

    return robotarm_el05_send_raw(robotarm_el05_command_id(ROBOTARM_COMM_POS_SPEED), data);
}

static void robotarm_el05_parse_mit_feedback(uint8_t *data)
{
    uint16_t p;
    uint16_t v;
    uint16_t t;
    uint16_t temp_decic;
    uint8_t status;

    if (data[0] != ROBOTARM_EL05_ID)
    {
        return;
    }

    p = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
    v = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
    t = (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8) | data[5]);
    status = data[6];
    temp_decic = (uint16_t)((((uint16_t)status & 0x0FU) << 8) | data[7]);

    el05_feedback_ok = 1U;
    el05_rx_cnt++;
    el05_angle = robotarm_el05_uint_to_float(p, -12.57f, 12.57f, 16U);
    el05_speed = robotarm_el05_uint_to_float(v, -50.0f, 50.0f, 12U);
    el05_torque = robotarm_el05_uint_to_float(t, -6.0f, 6.0f, 12U);

    g_robotarm_fdcan3_feedback.el05_3.id = ROBOTARM_EL05_ID;
    g_robotarm_fdcan3_feedback.el05_3.mode = robotarm_comm_to_dm_mode(robotarm_config.el05_mode);
    g_robotarm_fdcan3_feedback.el05_3.state = (uint16_t)((status >> 6) & 0x03U);
    g_robotarm_fdcan3_feedback.el05_3.rx_cnt = el05_rx_cnt;
    g_robotarm_fdcan3_feedback.el05_3.feedback_ok = 1U;
    g_robotarm_fdcan3_feedback.el05_3.fault = (uint8_t)((status >> 5) & 0x01U);
    g_robotarm_fdcan3_feedback.el05_3.warning = (uint8_t)((status >> 4) & 0x01U);
    g_robotarm_fdcan3_feedback.el05_3.p_int = (int)p;
    g_robotarm_fdcan3_feedback.el05_3.v_int = (int)v;
    g_robotarm_fdcan3_feedback.el05_3.t_int = (int)t;
    g_robotarm_fdcan3_feedback.el05_3.pos = el05_angle;
    g_robotarm_fdcan3_feedback.el05_3.vel = el05_speed;
    g_robotarm_fdcan3_feedback.el05_3.tor = el05_torque;
    g_robotarm_fdcan3_feedback.el05_3.tmos = 0.0f;
    g_robotarm_fdcan3_feedback.el05_3.tcoil = (float)temp_decic * 0.1f;
}

static uint8_t robotarm_handle_dm_feedback(uint8_t payload_id,
                                           const FDCAN_RxHeaderTypeDef *rx_header,
                                           uint8_t *rx_data)
{
    uint8_t len;
    uint8_t route_id;

    if ((rx_header == NULL) || (rx_data == NULL))
    {
        return 0U;
    }

    len = bsp_fdcan_dlc_to_len(rx_header->DataLength);
    if (len < 8U)
    {
        return 0U;
    }

    route_id = payload_id;
    if ((route_id != ROBOTARM_DM8006_1_ID) && (route_id != ROBOTARM_DM8006_2_ID))
    {
        if ((rx_header->Identifier == ROBOTARM_DM8006_1_ID) ||
            (rx_header->Identifier == ROBOTARM_DM8006_2_ID))
        {
            route_id = (uint8_t)rx_header->Identifier;
        }
        else
        {
            return 0U;
        }
    }

    if (route_id == ROBOTARM_DM8006_1_ID)
    {
        dm8006_fbdata(&chassis_move.joint_motor[0], rx_data, rx_header->DataLength);
        robotarm_dm8006_1_rx_cnt++;
        g_robotarm_fdcan3_rx_debug.dm8006_1_rx_cnt++;
        if (rx_header->Identifier != ROBOTARM_DM8006_1_ID)
        {
            g_robotarm_fdcan3_rx_debug.dm_id_mismatch_cnt++;
        }
        robotarm_feedback_update_dm(&g_robotarm_fdcan3_feedback.dm8006_1,
                                    &chassis_move.joint_motor[0],
                                    ROBOTARM_DM8006_1_ID,
                                    robotarm_dm8006_1_rx_cnt);
        return 1U;
    }

    dm8006_fbdata(&chassis_move.joint_motor[1], rx_data, rx_header->DataLength);
    robotarm_dm8006_2_rx_cnt++;
    g_robotarm_fdcan3_rx_debug.dm8006_2_rx_cnt++;
    if (rx_header->Identifier != ROBOTARM_DM8006_2_ID)
    {
        g_robotarm_fdcan3_rx_debug.dm_id_mismatch_cnt++;
    }
    robotarm_feedback_update_dm(&g_robotarm_fdcan3_feedback.dm8006_2,
                                &chassis_move.joint_motor[1],
                                ROBOTARM_DM8006_2_ID,
                                robotarm_dm8006_2_rx_cnt);
    return 1U;
}

void RobotArm_Mixed_Init(void)
{
    RobotArm_Mixed_InitWithConfig(&robotarm_default_config);
}

void RobotArm_Mixed_InitWithConfig(const RobotArm_MixedConfig_t *config)
{
    /* 清空达妙反馈、CAN 诊断等全局状态，避免上一次运行留下旧数据。 */
    if (config == NULL)
    {
        robotarm_config = robotarm_default_config;
    }
    else
    {
        robotarm_config = *config;
        robotarm_config.dm8006_1_mode = robotarm_sanitize_mode(robotarm_config.dm8006_1_mode);
        robotarm_config.dm8006_2_mode = robotarm_sanitize_mode(robotarm_config.dm8006_2_mode);
        robotarm_config.el05_mode = robotarm_sanitize_mode(robotarm_config.el05_mode);
    }

    memset(&chassis_move, 0, sizeof(chassis_move));
    memset((void *)&g_robotarm_fdcan3_rx_debug, 0, sizeof(g_robotarm_fdcan3_rx_debug));
    memset((void *)&g_robotarm_fdcan3_tx_debug, 0, sizeof(g_robotarm_fdcan3_tx_debug));
    robotarm_control_cmd_reset_from_config();
    robotarm_mixed_enabled = 0U;
    robotarm_control_cmd_valid = 0U;
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
    robotarm_feedback_reset();
    robotarm_apply_modes_from_config();

}

void RobotArm_Mixed_Enable(void)
{
    if ((robotarm_config.el05_set_mit_protocol_once != 0U) &&
        (robotarm_config.el05_mode == ROBOTARM_COMM_MIT))
    {
        RobotArm_EL05_SetProtocolMIT();
        robotarm_config.el05_set_mit_protocol_once = 0U;
        HAL_Delay(10);
    }

    for (uint8_t retry = 0U; retry < 3U; retry++)
    {
        if (robotarm_config.dm8006_1_mode == ROBOTARM_COMM_MIT)
        {
            (void)dm8006_enter_motor_mode(&hfdcan3, ROBOTARM_DM8006_1_ID);
        }
        else
        {
            (void)dm8006_enter_pos_speed_mode(&hfdcan3, ROBOTARM_DM8006_1_ID);
        }
        HAL_Delay(2);

        if (robotarm_config.dm8006_2_mode == ROBOTARM_COMM_MIT)
        {
            (void)dm8006_enter_motor_mode(&hfdcan3, ROBOTARM_DM8006_2_ID);
        }
        else
        {
            (void)dm8006_enter_pos_speed_mode(&hfdcan3, ROBOTARM_DM8006_2_ID);
        }
        HAL_Delay(2);

        (void)robotarm_el05_send_run_mode(robotarm_config.el05_mode);
        HAL_Delay(2);
        (void)robotarm_el05_send_enable(robotarm_config.el05_mode);
        el05_enable_sent = 1U;
        HAL_Delay(5);
    }

    robotarm_mixed_enabled = 1U;
}

void RobotArm_Mixed_Disable(void)
{
    /* 先关软件标志，防止失能过程中主循环继续发控制帧。 */
    robotarm_mixed_enabled = 0U;
    robotarm_control_cmd_valid = 0U;
    el05_enable_sent = 0U;

    (void)robotarm_el05_send_run_mode(robotarm_config.el05_mode);
    HAL_Delay(2);
    (void)robotarm_el05_send_disable(robotarm_config.el05_mode);
    HAL_Delay(2);

    if (robotarm_config.dm8006_1_mode == ROBOTARM_COMM_MIT)
    {
        (void)dm8006_exit_motor_mode(&hfdcan3, ROBOTARM_DM8006_1_ID);
    }
    else
    {
        (void)dm8006_exit_pos_speed_mode(&hfdcan3, ROBOTARM_DM8006_1_ID);
    }
    HAL_Delay(2);

    if (robotarm_config.dm8006_2_mode == ROBOTARM_COMM_MIT)
    {
        (void)dm8006_exit_motor_mode(&hfdcan3, ROBOTARM_DM8006_2_ID);
    }
    else
    {
        (void)dm8006_exit_pos_speed_mode(&hfdcan3, ROBOTARM_DM8006_2_ID);
    }
}

void RobotArm_Mixed_SetZero(void)
{
    if (robotarm_config.dm8006_1_mode == ROBOTARM_COMM_MIT)
    {
        (void)dm8006_set_zero_position(&hfdcan3, ROBOTARM_DM8006_1_ID);
    }
    else
    {
        (void)dm8006_set_zero_position_pos_speed(&hfdcan3, ROBOTARM_DM8006_1_ID);
    }
    HAL_Delay(20);

    if (robotarm_config.dm8006_2_mode == ROBOTARM_COMM_MIT)
    {
        (void)dm8006_set_zero_position(&hfdcan3, ROBOTARM_DM8006_2_ID);
    }
    else
    {
        (void)dm8006_set_zero_position_pos_speed(&hfdcan3, ROBOTARM_DM8006_2_ID);
    }
    HAL_Delay(20);

    (void)robotarm_el05_send_run_mode(robotarm_config.el05_mode);
    HAL_Delay(2);
    (void)robotarm_el05_send_zero(robotarm_config.el05_mode);
}

void RobotArm_EL05_MIT_Enable(void)
{
    (void)robotarm_el05_send_run_mode(ROBOTARM_COMM_MIT);
    HAL_Delay(2);
    (void)robotarm_el05_send_enable(ROBOTARM_COMM_MIT);
    el05_enable_sent = 1U;
}

void RobotArm_EL05_MIT_Disable(void)
{
    (void)robotarm_el05_send_run_mode(ROBOTARM_COMM_MIT);
    HAL_Delay(2);
    (void)robotarm_el05_send_disable(ROBOTARM_COMM_MIT);
    el05_enable_sent = 0U;
}

void RobotArm_EL05_MIT_Control(float pos, float vel, float kp, float kd, float tor)
{
    (void)robotarm_el05_send_mit_control(pos, vel, kp, kd, tor);
}

void RobotArm_EL05_PosSpeed_Control(float pos, float vel)
{
    (void)robotarm_el05_send_run_mode(ROBOTARM_COMM_POS_SPEED);
    (void)robotarm_el05_send_pos_speed_control(pos, vel);
}

void RobotArm_EL05_MIT_SetZero(void)
{
    (void)robotarm_el05_send_run_mode(ROBOTARM_COMM_MIT);
    HAL_Delay(2);
    (void)robotarm_el05_send_zero(ROBOTARM_COMM_MIT);
}

void RobotArm_EL05_SetProtocol(uint8_t protocol)
{
    robotarm_el05.RobStride_Motor_MotorModeSet(protocol);
}

void RobotArm_EL05_SetProtocolMIT(void)
{
    RobotArm_EL05_SetProtocol(ROBOTARM_EL05_PROTOCOL_MIT);
}

void RobotArm_Mixed_SetCommModes(RobotArm_MotorCommMode_t dm8006_1_mode,
                                 RobotArm_MotorCommMode_t dm8006_2_mode,
                                 RobotArm_MotorCommMode_t el05_mode,
                                 uint8_t el05_set_mit_protocol_once)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    robotarm_config.dm8006_1_mode = robotarm_sanitize_mode(dm8006_1_mode);
    robotarm_config.dm8006_2_mode = robotarm_sanitize_mode(dm8006_2_mode);
    robotarm_config.el05_mode = robotarm_sanitize_mode(el05_mode);
    robotarm_config.el05_set_mit_protocol_once = el05_set_mit_protocol_once;
    robotarm_control_cmd.dm8006_1.mode = robotarm_config.dm8006_1_mode;
    robotarm_control_cmd.dm8006_2.mode = robotarm_config.dm8006_2_mode;
    robotarm_control_cmd.el05.mode = robotarm_config.el05_mode;
    robotarm_apply_modes_from_config();
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void RobotArm_Mixed_SetMotorParam(RobotArm_MotorIndex_t motor,
                                  RobotArm_MotorCommMode_t mode,
                                  float pos,
                                  float vel,
                                  float kp,
                                  float kd,
                                  float tor)
{
    RobotArm_MixedCommand_t cmd;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    cmd = robotarm_control_cmd;
    if (primask == 0U)
    {
        __enable_irq();
    }

    switch (motor)
    {
    case ROBOTARM_MOTOR_DM8006_1:
        cmd.dm8006_1.mode = mode;
        cmd.dm8006_1.pos = pos;
        cmd.dm8006_1.vel = vel;
        cmd.dm8006_1.kp = kp;
        cmd.dm8006_1.kd = kd;
        cmd.dm8006_1.tor = tor;
        break;
    case ROBOTARM_MOTOR_DM8006_2:
        cmd.dm8006_2.mode = mode;
        cmd.dm8006_2.pos = pos;
        cmd.dm8006_2.vel = vel;
        cmd.dm8006_2.kp = kp;
        cmd.dm8006_2.kd = kd;
        cmd.dm8006_2.tor = tor;
        break;
    case ROBOTARM_MOTOR_EL05:
        cmd.el05.mode = mode;
        cmd.el05.pos = pos;
        cmd.el05.vel = vel;
        cmd.el05.kp = kp;
        cmd.el05.kd = kd;
        cmd.el05.tor = tor;
        break;
    default:
        return;
    }

    RobotArm_Mixed_SetControlCommand(&cmd);
}

void RobotArm_Mixed_SetTargetCommands(const RobotArm_MotorCommand_t *dm8006_1,
                                      const RobotArm_MotorCommand_t *dm8006_2,
                                      const RobotArm_MotorCommand_t *el05)
{
    RobotArm_MixedCommand_t cmd;

    if ((dm8006_1 == NULL) || (dm8006_2 == NULL) || (el05 == NULL))
    {
        return;
    }

    cmd.dm8006_1 = *dm8006_1;
    cmd.dm8006_2 = *dm8006_2;
    cmd.el05 = *el05;

    RobotArm_Mixed_SetControlCommand(&cmd);
}

void RobotArm_Mixed_SetControlCommand(const RobotArm_MixedCommand_t *cmd)
{
    uint32_t primask;
    RobotArm_MixedCommand_t sanitized_cmd;

    if (cmd == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    sanitized_cmd = *cmd;
    robotarm_apply_modes_from_command(&sanitized_cmd);
    robotarm_control_cmd = sanitized_cmd;
    robotarm_control_cmd_valid = 1U;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void RobotArm_Mixed_Control(const RobotArm_MixedCommand_t *cmd)
{
    RobotArm_Mixed_SetControlCommand(cmd);
}

void RobotArm_Mixed_Transmit1ms(void)
{
    RobotArm_MixedCommand_t cmd;
    uint32_t primask;

    if ((robotarm_mixed_enabled == 0U) || (robotarm_control_cmd_valid == 0U))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    cmd = robotarm_control_cmd;
    if (primask == 0U)
    {
        __enable_irq();
    }

    robotarm_mixed_send_now(&cmd);
}

void RobotArm_Mixed_GetFeedbackSnapshot(RobotArm_FDCAN3Feedback_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(out, (const void *)&g_robotarm_fdcan3_feedback, sizeof(*out));
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void robotarm_mixed_send_now(const RobotArm_MixedCommand_t *cmd)
{
    if ((cmd == NULL) || (robotarm_mixed_enabled == 0U))
    {
        return;
    }

    robotarm_tx_debug_record(cmd);

    if (robotarm_config.dm8006_1_mode == ROBOTARM_COMM_MIT)
    {
        (void)dm8006_send_mit_command(&hfdcan3,
                                      ROBOTARM_DM8006_1_ID,
                                      cmd->dm8006_1.pos,
                                      cmd->dm8006_1.vel,
                                      cmd->dm8006_1.kp,
                                      cmd->dm8006_1.kd,
                                      cmd->dm8006_1.tor);
    }
    else
    {
        (void)dm8006_send_pos_speed_command(&hfdcan3,
                                            ROBOTARM_DM8006_1_ID,
                                            cmd->dm8006_1.pos,
                                            cmd->dm8006_1.vel);
    }

    if (robotarm_config.dm8006_2_mode == ROBOTARM_COMM_MIT)
    {
        (void)dm8006_send_mit_command(&hfdcan3,
                                      ROBOTARM_DM8006_2_ID,
                                      cmd->dm8006_2.pos,
                                      cmd->dm8006_2.vel,
                                      cmd->dm8006_2.kp,
                                      cmd->dm8006_2.kd,
                                      cmd->dm8006_2.tor);
    }
    else
    {
        (void)dm8006_send_pos_speed_command(&hfdcan3,
                                            ROBOTARM_DM8006_2_ID,
                                            cmd->dm8006_2.pos,
                                            cmd->dm8006_2.vel);
    }

    if (robotarm_config.el05_mode == ROBOTARM_COMM_MIT)
    {
        (void)robotarm_el05_send_mit_control(cmd->el05.pos,
                                             cmd->el05.vel,
                                             cmd->el05.kp,
                                             cmd->el05.kd,
                                             cmd->el05.tor);
    }
    else
    {
        (void)robotarm_el05_send_pos_speed_control(cmd->el05.pos, cmd->el05.vel);
    }
}

void RobotArm_FDCAN3_TestMITKpZeroStart(float dm8006_1_rad,
                                        float dm8006_2_rad,
                                        float el05_rad)
{
    RobotArm_Mixed_Init();
    RobotArm_Mixed_SetCommModes(ROBOTARM_COMM_POS_SPEED,
                                ROBOTARM_COMM_POS_SPEED,
                                ROBOTARM_COMM_POS_SPEED,
                                0U);
    osDelay(100U);

    RobotArm_Mixed_Enable();
    osDelay(100U);

    RobotArm_Mixed_SetMotorParam(ROBOTARM_MOTOR_DM8006_1,
                                 ROBOTARM_COMM_POS_SPEED,
                                 0.0f,
                                 0.5f,
                                 0.0f,
                                 0.0f,
                                 0.0f);
    RobotArm_Mixed_SetMotorParam(ROBOTARM_MOTOR_DM8006_2,
                                 ROBOTARM_COMM_POS_SPEED,
                                 2.0f,
                                 2.0f,
                                 0.0f,
                                 0.0f,
                                 0.0f);
    RobotArm_Mixed_SetMotorParam(ROBOTARM_MOTOR_EL05,
                                 ROBOTARM_COMM_POS_SPEED,
                                 el05_rad,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 0.0f);
}

void RobotArm_FDCAN3_TestMain(float dm8006_1_rad,
                              float dm8006_2_rad,
                              float el05_rad)
{
    RobotArm_FDCAN3_TestMITKpZeroStart(dm8006_1_rad, dm8006_2_rad, el05_rad);

    for (;;)
    {
        osDelay(1000U);
    }
}

void RobotArm_Mixed_HandleRx(const FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    uint8_t payload_id;

    /* FDCAN 中断里会调用这个函数，先检查指针，避免异常数据导致 HardFault。 */
    if ((rx_header == NULL) || (rx_data == NULL))
    {
        return;
    }

    if (rx_header->IdType == FDCAN_STANDARD_ID)
    {
        robotarm_rx_debug_record(rx_header, rx_data);
        payload_id = rx_data[0] & 0x0FU;
        if (robotarm_handle_dm_feedback(payload_id, rx_header, rx_data) != 0U)
        {
            return;
        }

        /* 根据标准帧 ID 区分是哪台电机的反馈。 */
        switch (rx_header->Identifier)
        {
        case ROBOTARM_DM8006_1_ID:
            /* 达妙 1 反馈解析后存到 chassis_move.joint_motor[0]。 */
            dm8006_fbdata(&chassis_move.joint_motor[0], rx_data, rx_header->DataLength);
            if (bsp_fdcan_dlc_to_len(rx_header->DataLength) >= 8U)
            {
                robotarm_dm8006_1_rx_cnt++;
                robotarm_feedback_update_dm(&g_robotarm_fdcan3_feedback.dm8006_1,
                                            &chassis_move.joint_motor[0],
                                            ROBOTARM_DM8006_1_ID,
                                            robotarm_dm8006_1_rx_cnt);
            }
            break;

        case ROBOTARM_DM8006_2_ID:
            /* 达妙 2 反馈解析后存到 chassis_move.joint_motor[1]。 */
            dm8006_fbdata(&chassis_move.joint_motor[1], rx_data, rx_header->DataLength);
            if (bsp_fdcan_dlc_to_len(rx_header->DataLength) >= 8U)
            {
                robotarm_dm8006_2_rx_cnt++;
                robotarm_feedback_update_dm(&g_robotarm_fdcan3_feedback.dm8006_2,
                                            &chassis_move.joint_motor[1],
                                            ROBOTARM_DM8006_2_ID,
                                            robotarm_dm8006_2_rx_cnt);
            }
            break;

        case ROBOTARM_EL05_ID:
        case ROBOTARM_EL05_MIT_RX_ID:
            /* EL05 MIT 反馈可能用电机 ID 0x03，也可能用主机 ID 0xFD。
             * 只有 8 字节反馈才交给 RobStride_Motor_Analysis 解析。
             */
            if (bsp_fdcan_dlc_to_len(rx_header->DataLength) >= 8U)
            {
                robotarm_el05_parse_mit_feedback(rx_data);
                g_robotarm_fdcan3_rx_debug.el05_rx_cnt = el05_rx_cnt;
            }
            break;

        default:
            g_robotarm_fdcan3_rx_debug.unknown_rx_cnt++;
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
				 {(void)dm8006_send_pos_speed_command(&hfdcan3,
        ROBOTARM_DM8006_1_ID,
        -0.0f,//负号往前转
        0.5f);
					 break; }

				case 1 :
				 {(void)dm8006_send_pos_speed_command(&hfdcan3,
        ROBOTARM_DM8006_1_ID,
        1.04f,//负号往前转
        0.5f);
					 break; }
				 
				 case 2 :
				 {(void)dm8006_send_pos_speed_command(&hfdcan3,
        ROBOTARM_DM8006_1_ID,
        -2.08f,//负号往前转
        0.1f);
					 break; }
				 	case 3 :
				 {(void)dm8006_send_pos_speed_command(&hfdcan3,
        ROBOTARM_DM8006_1_ID,
        -5.695f,//负号往前转
        0.1f);
					 break; }
				}
}
