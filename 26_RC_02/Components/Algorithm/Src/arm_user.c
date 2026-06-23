#include <string.h>
#include <math.h>
#include "arm_user.h"
#include "bsp_usb.h"
#include "arm_ik_3r_safe_stm32h7.h"

/* ========================= ȫ�ֶ��� ========================= */
/*
 * ���������
 */
Arm3R_Handle_t g_arm_ik;

/*
 * Ӧ�ò�����״̬
 */
ArmIK_AppState_t g_arm_ik_app;
ArmIK_FullState_t g_arm_ik_full_state;

static ArmIK_MotorDeg_t s_arm_actual_motor_deg;




/* ========================= �ڲ��������� ========================= */
/*
 * ������Щ����ֻ�ڱ��ļ��ڲ�ʹ��
 */
static void ArmIK_ResetAppState(void);
static void ArmIK_SendResultToUSB(uint8_t cmd, const uint8_t *data, uint16_t len);
static void ArmIK_SendResultToUART10(uint8_t cmd, const uint8_t *data, uint16_t len);
static void ArmIK_SendResultToAll(uint8_t status_code,
                                  uint8_t unsafe_reason,
                                  uint8_t action_code,
                                  const Arm3R_Result_t *res);
static void ArmIK_MotorCtrlRadToDeg(const Arm3R_CtrlAngles_t *motor_ctrl_rad,
                                    ArmIK_MotorDeg_t *motor_ctrl_deg);
static void ArmIK_ModelRadToDeg(const Arm3R_ModelAngles_t *model_rad,
                                ArmIK_ModelDeg_t *model_deg);
static float ArmIK_ClampJ1Deg(float deg);
static void ArmIK_ApplyJ1CoordinateLimit(ArmIK_MotorDeg_t *motor_ctrl_deg);
static void ArmIK_BuildFullState(ArmIK_FullState_t *state);
static void ArmIK_UpdateFullState(void);
static void ArmIK_SaveAsLastValidTarget(float x,
                                        float y,
                                        float z,
                                        const Arm3R_Result_t *res,
                                        const Arm3R_CtrlAngles_t *motor_ctrl_rad,
                                        const ArmIK_MotorDeg_t *motor_ctrl_deg);
static uint8_t ArmIK_HandleInvalidTarget(void);

/* ========================= �ڲ�����ʵ�� ========================= */
/*
 * ���Ӧ�ò�״̬
 * ֻ�ڳ�ʼ��ʱ����
 */
static void ArmIK_ResetAppState(void)
{
    memset(&g_arm_ik_app, 0, sizeof(g_arm_ik_app));
    memset(&s_arm_actual_motor_deg, 0, sizeof(s_arm_actual_motor_deg));
    ArmIK_UpdateFullState();
}

/*
 * ���� USB ��λ��
 * ����������ԭ���� Send_Cmd_Data()
 */
static void ArmIK_SendResultToUSB(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    Send_Cmd_Data(cmd, (uint8_t *)data, len);
}

/*
 * ���� UART10 ����ң����
 *
 * �����Ҳ�����ǿ��д���ײ㺯������
 * ��Ϊ��û���ϴ� UART10 �� BSP �ļ���
 * �Ҳ�֪����ʵ�ʷ��ͽӿڽ�ʲô��
 *
 * ��ֻ��Ҫ���������ע���滻�����Լ��� UART10 ���ͺ������ɡ�
 *
 * ���磺
 * Uart10_Send_Cmd_Data(cmd, data, len);
 * ���ߣ�
 * HAL_UART_Transmit(&huart10, ...);
 */
static void ArmIK_SendResultToUART10(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    (void)cmd;
    (void)data;
    (void)len;

    /* TODO���滻�����Լ��� UART10 ���ͽӿ� */
    /* Uart10_Send_Cmd_Data(cmd, (uint8_t *)data, len); */
}

/*
 * ͬʱ���� USB �� UART10
 *
 * ״̬��һ�� 6 �ֽڣ���ʽ��ͷ�ļ�˵��
 */
static void ArmIK_SendResultToAll(uint8_t status_code,
                                  uint8_t unsafe_reason,
                                  uint8_t action_code,
                                  const Arm3R_Result_t *res)
{
    uint8_t tx_data[6];

    tx_data[0] = status_code;
    tx_data[1] = unsafe_reason;
    tx_data[2] = action_code;

    if (res != 0)
    {
        tx_data[3] = (uint8_t)res->reachable;
        tx_data[4] = (uint8_t)res->safe;
    }
    else
    {
        tx_data[3] = 0U;
        tx_data[4] = 0U;
    }

    tx_data[5] = g_arm_ik_app.has_last_valid;

    /*
     * �������������ԭ���������Ѿ����õ������֣�
     * USB_CMD_ARM_IK_RESULT
     */
    ArmIK_SendResultToUSB(USB_CMD_ARM_IK_RESULT, tx_data, sizeof(tx_data));
    ArmIK_SendResultToUART10(USB_CMD_ARM_IK_RESULT, tx_data, sizeof(tx_data));
}

/*
 * ���������ƽǣ�rad�� -> ���Ŀ��ǣ�deg��
 *
 * ע�⣺
 * ����ֻ��������ת�Ƕȡ���
 * dir ����ӳ���Ѿ��� Arm3R_GeomCtrlToMotorCtrl() �������ˡ�
 */
static void ArmIK_MotorCtrlRadToDeg(const Arm3R_CtrlAngles_t *motor_ctrl_rad,
                                    ArmIK_MotorDeg_t *motor_ctrl_deg)
{
    if ((motor_ctrl_rad == 0) || (motor_ctrl_deg == 0))
    {
        return;
    }

    motor_ctrl_deg->j1_deg = Arm3R_RadToDeg(motor_ctrl_rad->j1);
    motor_ctrl_deg->j2_deg = Arm3R_RadToDeg(motor_ctrl_rad->j2);
    motor_ctrl_deg->j3_deg = Arm3R_RadToDeg(motor_ctrl_rad->j3);
    motor_ctrl_deg->valid  = motor_ctrl_rad->valid;
}

static void ArmIK_ModelRadToDeg(const Arm3R_ModelAngles_t *model_rad,
                                ArmIK_ModelDeg_t *model_deg)
{
    if ((model_rad == 0) || (model_deg == 0))
    {
        return;
    }

    model_deg->theta1_deg = Arm3R_RadToDeg(model_rad->theta1);
    model_deg->theta2_deg = Arm3R_RadToDeg(model_rad->theta2);
    model_deg->theta3_deg = Arm3R_RadToDeg(model_rad->theta3);
    model_deg->valid = model_rad->valid;
}

static void ArmIK_BuildFullState(ArmIK_FullState_t *state)
{
    const Arm3R_Result_t *res;

    if (state == 0)
    {
        return;
    }

    memset(state, 0, sizeof(*state));

    res = &g_arm_ik.result;

    state->cfg = g_arm_ik.cfg;
    state->inited = g_arm_ik.inited;
    state->has_last_valid = g_arm_ik_app.has_last_valid;
    state->reachable = res->reachable;
    state->safe = res->safe;
    state->base_singular = res->base_singular;
    state->last_status_code = g_arm_ik_app.last_status_code;
    state->last_action_code = g_arm_ik_app.last_action_code;
    state->actual_motor_valid = s_arm_actual_motor_deg.valid;
    state->solve_status = res->status;
    state->unsafe_reason = res->unsafe_reason;

    state->requested_pt = g_arm_ik_app.last_req_pt;
    state->solved_req_pt = res->req_pt;
    state->last_valid_pt = g_arm_ik_app.last_valid_pt;

    state->solved_model_rad = res->model;
    ArmIK_ModelRadToDeg(&state->solved_model_rad, &state->solved_model_deg);

    state->solved_geom_rad = res->ctrl;
    ArmIK_MotorCtrlRadToDeg(&state->solved_geom_rad, &state->solved_geom_deg);

    state->active_model_rad = g_arm_ik_app.active_model;
    ArmIK_ModelRadToDeg(&state->active_model_rad, &state->active_model_deg);

    if (g_arm_ik_app.has_last_valid != 0U)
    {
        state->active_geom_rad = g_arm_ik_app.last_valid_geom;
        state->active_motor_rad = g_arm_ik_app.last_valid_motor;
    }

    ArmIK_MotorCtrlRadToDeg(&state->active_geom_rad, &state->active_geom_deg);

    state->active_motor_deg = g_arm_ik_app.active_motor_deg;
    state->last_valid_model_rad = g_arm_ik_app.last_valid_model;
    ArmIK_ModelRadToDeg(&state->last_valid_model_rad, &state->last_valid_model_deg);

    state->last_valid_geom_rad = g_arm_ik_app.last_valid_geom;
    ArmIK_MotorCtrlRadToDeg(&state->last_valid_geom_rad, &state->last_valid_geom_deg);

    state->last_valid_motor_rad = g_arm_ik_app.last_valid_motor;
    state->last_valid_motor_deg = g_arm_ik_app.last_valid_motor_deg;
    state->actual_motor_deg = s_arm_actual_motor_deg;
}

static void ArmIK_UpdateFullState(void)
{
    ArmIK_BuildFullState(&g_arm_ik_full_state);
}

static float ArmIK_ClampJ1Deg(float deg)
{
    if (deg > ARM_IK_J1_LIMIT_DEG)
    {
        return ARM_IK_J1_LIMIT_DEG;
    }

    if (deg < -ARM_IK_J1_LIMIT_DEG)
    {
        return -ARM_IK_J1_LIMIT_DEG;
    }

    return deg;
}

static void ArmIK_ApplyJ1CoordinateLimit(ArmIK_MotorDeg_t *motor_ctrl_deg)
{
    float ref_j1;

    if (motor_ctrl_deg == 0)
    {
        return;
    }

    ref_j1 = (g_arm_ik_app.has_last_valid != 0U) ?
             g_arm_ik_app.active_motor_deg.j1_deg :
             0.0f;

    /*
     * XY -> J1 uses atan2(y, x):
     * +Y is positive, -Y is negative. Only the exact rear axis is ambiguous.
     * Keep that +/-180 boundary on the previous side to avoid a sign flip.
     */
    if ((motor_ctrl_deg->j1_deg >= (ARM_IK_J1_LIMIT_DEG - 1.0e-3f)) &&
        (ref_j1 < 0.0f))
    {
        motor_ctrl_deg->j1_deg = -ARM_IK_J1_LIMIT_DEG;
    }
    else if ((motor_ctrl_deg->j1_deg <= (-ARM_IK_J1_LIMIT_DEG + 1.0e-3f)) &&
             (ref_j1 > 0.0f))
    {
        motor_ctrl_deg->j1_deg = ARM_IK_J1_LIMIT_DEG;
    }
    else
    {
        motor_ctrl_deg->j1_deg = ArmIK_ClampJ1Deg(motor_ctrl_deg->j1_deg);
    }
}

/*
 * ������Ŀ��㡰�ɴ��Ұ�ȫ��ʱ��
 * ���䱣��ɡ���һ�鰲ȫĿ�ꡱ
 *
 * ��������һ���յ��Ƿ�Ŀ��㣬
 * �Ϳ���ֱ�ӱ�����һ�鰲ȫ�Ƕ����
 */
static void ArmIK_SaveAsLastValidTarget(float x,
                                        float y,
                                        float z,
                                        const Arm3R_Result_t *res,
                                        const Arm3R_CtrlAngles_t *motor_ctrl_rad,
                                        const ArmIK_MotorDeg_t *motor_ctrl_deg)
{
    if ((res == 0) || (motor_ctrl_rad == 0) || (motor_ctrl_deg == 0))
    {
        return;
    }

    /* �������һ�ΰ�ȫĿ��� */
    g_arm_ik_app.last_valid_pt.x = x;
    g_arm_ik_app.last_valid_pt.y = y;
    g_arm_ik_app.last_valid_pt.z = z;

    g_arm_ik_app.last_valid_model = res->model;   /* ���� */

    /* �������һ�ΰ�ȫ���ƽ� */
    g_arm_ik_app.last_valid_geom = res->ctrl;
    g_arm_ik_app.last_valid_motor = *motor_ctrl_rad;
    g_arm_ik_app.last_valid_motor_deg = *motor_ctrl_deg;

    g_arm_ik_app.active_model = res->model;       /* ���� */

    /*
     * ��ǰʵ��ά�����Ҳͬ������Ϊ�����Ŀ��
     */
    g_arm_ik_app.active_motor_deg = *motor_ctrl_deg;

    g_arm_ik_app.has_last_valid = 1U;
}

/*
 * ������Ŀ���Ƿ�ʱ��ִ�б�������
 *
 * �������£�
 * 1. ����Ѿ�����ʷ��ȫĿ�꣬�򱣳���һ�鰲ȫ�Ƕ�Ŀ��
 * 2. �����û����ʷ��ȫĿ�꣬�򱣳ֵ�ǰ����
 *
 * ����ֵ��
 *   ARM_IK_ACTION_HOLD_LAST
 *   ARM_IK_ACTION_KEEP_CURRENT
 */
static uint8_t ArmIK_HandleInvalidTarget(void)
{
    if (g_arm_ik_app.has_last_valid != 0U)
    {
        /*
         * ά����һ�鰲ȫĿ��
         */
        g_arm_ik_app.active_model = g_arm_ik_app.last_valid_model;        //ģ�ͽ�
        g_arm_ik_app.active_motor_deg = g_arm_ik_app.last_valid_motor_deg;//���ƽ�
        return ARM_IK_ACTION_HOLD_LAST;
    }

    /*
     * ���ϵͳ����δ��ù�һ����Ч��ȫĿ�꣬
     * �ǾͲ�������������ֵ�ǰ״̬
     */
    return ARM_IK_ACTION_KEEP_CURRENT;
}

/* ========================= ����ӿ�ʵ�� ========================= */
/*
 * ��ʼ����е�����Ӧ�ò�
 *
 * ����������ԭ�� arm_user.c ������ã�
 * d1 = 0
 * a2 = 320
 * a3 = 320
 * j1_ref = 0 deg
 * j2_ref = 80 deg
 * j3_ref = -165 deg
 */
void ArmIK_ComponentInit(void)
{
    Arm3R_Config_t cfg;

    /* ���˲��� */
    cfg.link.d1 = 0.0f;
    cfg.link.a2 = 320.0f;
    cfg.link.a3 = 320.0f;

    /* �ϵ�ο�ģ�ͽ� */
    cfg.j1_ref.model_ref = Arm3R_DegToRad(0.0f);
    cfg.j2_ref.model_ref = Arm3R_DegToRad(80.0f);
    cfg.j3_ref.model_ref = Arm3R_DegToRad(-165.0f);

    /* �������ӳ�� */
    cfg.j1_ref.dir = +1;
    cfg.j2_ref.dir = -1;
    cfg.j3_ref.dir = +1;

    /* ��ʼ�������� */
    Arm3R_Init(&g_arm_ik, &cfg);

    /* ���Ӧ�ò�����״̬ */
    ArmIK_ResetAppState();
}

uint8_t ArmIK_TargetInputAllowed(float x,
                                 float y,
                                 float z,
                                 Arm3R_Status_t *status,
                                 Arm3R_UnsafeReason_t *unsafe_reason)
{
    Arm3R_Handle_t check_arm;
    Arm3R_Status_t check_status;

    if (status != 0)
    {
        *status = ARM3R_ERR_PARAM;
    }

    if (unsafe_reason != 0)
    {
        *unsafe_reason = ARM3R_UNSAFE_NONE;
    }

    if ((!isfinite(x)) || (!isfinite(y)) || (!isfinite(z)))
    {
        return 0U;
    }

    if (g_arm_ik.inited == 0U)
    {
        if (status != 0)
        {
            *status = ARM3R_ERR_NOT_INIT;
        }
        return 0U;
    }

    check_arm = g_arm_ik;
    check_status = Arm3R_Solve(&check_arm, x, y, z, 0.0f);

    if (status != 0)
    {
        *status = check_status;
    }

    if (unsafe_reason != 0)
    {
        *unsafe_reason = check_arm.result.unsafe_reason;
    }

    return (check_status == ARM3R_OK) ? 1U : 0U;
}

/*
 * ����һ��Ŀ��㣬��ɣ�
 * 1. ���
 * 2. ��ȫ�ж�
 * 3. ״̬˫·�ش�
 * 4. ���µ�ǰ�Ƕ���Ŀ��
 *
 * ����Ӧ�ò�������
 */
void ArmIK_ComponentStep(float x, float y, float z)
{
    Arm3R_Status_t ret;
    const Arm3R_Result_t *res;

    /*
     * motor_ctrl_rad��
     * �Ѿ��˹� dir �ĵ��������ƽǣ���λ��Ȼ�� rad
     */
    Arm3R_CtrlAngles_t motor_ctrl_rad;

    /*
     * motor_ctrl_deg��
     * ���ո�����õĽǶ���Ŀ�꣬��λ�� deg
     */
    ArmIK_MotorDeg_t motor_ctrl_deg;

    uint8_t action_code;

    /* �������һ���յ���Ŀ��� */
    g_arm_ik_app.last_req_pt.x = x;
    g_arm_ik_app.last_req_pt.y = y;
    g_arm_ik_app.last_req_pt.z = z;

    if ((!isfinite(x)) || (!isfinite(y)) || (!isfinite(z)))
    {
        action_code = ArmIK_HandleInvalidTarget();
        g_arm_ik_app.last_status_code = ARM_IK_RESULT_PARAM_ERR;
        g_arm_ik_app.last_action_code = action_code;
        ArmIK_UpdateFullState();
        ArmIK_SendResultToAll(ARM_IK_RESULT_PARAM_ERR, 0U, action_code, 0);
        return;
    }

    /*
     * ִ����� + ��ȫ�ж�
     * theta1_hint �������� 0.0f
     */
    ret = Arm3R_Solve(&g_arm_ik, x, y, z, 0.0f);

    /*
     * ��ȡ�ڲ�������ṹ��
     */
    res = Arm3R_GetResult(&g_arm_ik);

    if (ret == ARM3R_OK)
    {
        /*
         * ��� 1��Ŀ���ɴ��Ұ�ȫ
         *
         * �������̣�
         * 1. �Ѽ��ο��ƽ�ӳ��ɵ��������ƽǣ�rad��
         * 2. �ٰ� rad ת�� deg
         * 3. ����Ϊ��ǰ��ЧĿ��
         * 4. ״̬�ش��� USB �� UART10
         */
        Arm3R_GeomCtrlToMotorCtrl(&res->ctrl, &g_arm_ik.cfg, &motor_ctrl_rad);
        ArmIK_MotorCtrlRadToDeg(&motor_ctrl_rad, &motor_ctrl_deg);
        ArmIK_ApplyJ1CoordinateLimit(&motor_ctrl_deg);
        motor_ctrl_rad.j1 = Arm3R_DegToRad(motor_ctrl_deg.j1_deg);

        ArmIK_SaveAsLastValidTarget(x, y, z, res, &motor_ctrl_rad, &motor_ctrl_deg);

        action_code = ARM_IK_ACTION_APPLY_NEW;
        g_arm_ik_app.last_status_code = ARM_IK_RESULT_OK;
        g_arm_ik_app.last_action_code = action_code;
        ArmIK_UpdateFullState();
        ArmIK_SendResultToAll(ARM_IK_RESULT_OK, 0U, action_code, res);
    }
    else if (ret == ARM3R_ERR_UNREACHABLE)
    {
        /*
         * ��� 2�����β��ɴ�
         *
         * ���ܲ��ñ�����Ŀ�꣬
         * �߱������ԣ�������һ�鰲ȫĿ��򱣳ֵ�ǰ����
         */
        action_code = ArmIK_HandleInvalidTarget();
        g_arm_ik_app.last_status_code = ARM_IK_RESULT_UNREACHABLE;
        g_arm_ik_app.last_action_code = action_code;
        ArmIK_UpdateFullState();
        ArmIK_SendResultToAll(ARM_IK_RESULT_UNREACHABLE, 0U, action_code, res);
    }
    else if (ret == ARM3R_ERR_UNSAFE)
    {
        /*
         * ��� 3�����οɴ�������㰲ȫԼ��
         *
         * ͬ�����ܲ��ñ�����Ŀ�꣬
         * ����ά�ְ�ȫ���
         */
        uint8_t unsafe_reason = 0U;

        if (res != 0)
        {
            unsafe_reason = (uint8_t)res->unsafe_reason;
        }

        action_code = ArmIK_HandleInvalidTarget();
        g_arm_ik_app.last_status_code = ARM_IK_RESULT_UNSAFE;
        g_arm_ik_app.last_action_code = action_code;
        ArmIK_UpdateFullState();
        ArmIK_SendResultToAll(ARM_IK_RESULT_UNSAFE, unsafe_reason, action_code, res);
    }
    else
    {
        /*
         * ��� 4����������δ��ʼ���������쳣
         *
         * ��Ȼ�����ñ���Ŀ�꣬�߱�������
         */
        action_code = ArmIK_HandleInvalidTarget();
        g_arm_ik_app.last_status_code = ARM_IK_RESULT_PARAM_ERR;
        g_arm_ik_app.last_action_code = action_code;
        ArmIK_UpdateFullState();
        ArmIK_SendResultToAll(ARM_IK_RESULT_PARAM_ERR, 0U, action_code, res);
    }
}

/*
 * �����ⲿ�յ��� 3 άĿ��㸺��
 *
 * ���ظ�ʽ�̶�Ϊ��
 * payload[0..3]   -> float x
 * payload[4..7]   -> float y
 * payload[8..11]  -> float z
 *
 * ע�⣺
 * ����Ĭ����λ�� / ң���� �� STM32 ���� little-endian float ͨ��
 */
void ArmIK_ComponentHandleXYZPayload(const uint8_t *payload, uint16_t len)
{
    float x;
    float y;
    float z;
    uint8_t action_code;

    /*
     * ���Ȳ��ԣ����߿�ָ�룺
     * ��Ϊ��Ч����
     */
    if ((payload == 0) || (len != ARM_IK_XYZ_PAYLOAD_LEN))
    {
        action_code = ArmIK_HandleInvalidTarget();
        g_arm_ik_app.last_status_code = ARM_IK_RESULT_PARAM_ERR;
        g_arm_ik_app.last_action_code = action_code;
        ArmIK_UpdateFullState();
        ArmIK_SendResultToAll(ARM_IK_RESULT_PARAM_ERR, 0U, action_code, 0);
        return;
    }

    /*
     * �� memcpy ���� float��
     * ����ֱ��ǿתָ�������δ�����������
     */
    memcpy(&x, &payload[0], 4);
    memcpy(&y, &payload[4], 4);
    memcpy(&z, &payload[8], 4);

    /*
     * ��������������
     */
    ArmIK_ComponentStep(x, y, z);
}

/*
 * ��ȡ��ǰʵ��ά�ֵĽǶ���Ŀ��
 *
 * ���Լ��ĵ�����Ʋ����ֻ�������սǶȣ�
 * ֱ�Ӷ�ȡ�������ֵ����
 */
const ArmIK_MotorDeg_t *ArmIK_GetActiveMotorDeg(void)
{
    return &g_arm_ik_app.active_motor_deg;
}

void ArmIK_SetActualMotorDeg(float j1_deg,
                             float j2_deg,
                             float j3_deg,
                             uint8_t valid)
{
    s_arm_actual_motor_deg.j1_deg = j1_deg;
    s_arm_actual_motor_deg.j2_deg = j2_deg;
    s_arm_actual_motor_deg.j3_deg = j3_deg;
    s_arm_actual_motor_deg.valid = (valid != 0U) ? 1U : 0U;
    ArmIK_UpdateFullState();
}

const ArmIK_FullState_t *ArmIK_GetFullState(void)
{
    ArmIK_UpdateFullState();
    return &g_arm_ik_full_state;
}

/*
 * ��ȡ����Ӧ�ò�״̬
 *
 * ����ʱ�ɲ鿴��
 * 1. last_req_pt
 * 2. last_valid_pt
 * 3. active_motor_deg
 * 4. has_last_valid
 */
const ArmIK_AppState_t *ArmIK_GetAppState(void)
{
    return &g_arm_ik_app;
}
