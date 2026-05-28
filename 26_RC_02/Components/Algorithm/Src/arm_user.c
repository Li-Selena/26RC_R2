#include <string.h>
#include "arm_user.h"
#include "bsp_usb.h"
#include "arm_ik_3r_safe_stm32h7.h"

/* ========================= 全局对象 ========================= */
/*
 * 逆解组件句柄
 */
Arm3R_Handle_t g_arm_ik;

/*
 * 应用层运行状态
 */
ArmIK_AppState_t g_arm_ik_app;




/* ========================= 内部函数声明 ========================= */
/*
 * 下面这些函数只在本文件内部使用
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
static void ArmIK_SaveAsLastValidTarget(float x,
                                        float y,
                                        float z,
                                        const Arm3R_Result_t *res,
                                        const Arm3R_CtrlAngles_t *motor_ctrl_rad,
                                        const ArmIK_MotorDeg_t *motor_ctrl_deg);
static uint8_t ArmIK_HandleInvalidTarget(void);

/* ========================= 内部函数实现 ========================= */
/*
 * 清空应用层状态
 * 只在初始化时调用
 */
static void ArmIK_ResetAppState(void)
{
    memset(&g_arm_ik_app, 0, sizeof(g_arm_ik_app));
}

/*
 * 发给 USB 上位机
 * 这里沿用你原来的 Send_Cmd_Data()
 */
static void ArmIK_SendResultToUSB(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    Send_Cmd_Data(cmd, (uint8_t *)data, len);
}

/*
 * 发给 UART10 蓝牙遥控器
 *
 * 这里我不给你强行写死底层函数名，
 * 因为你没有上传 UART10 的 BSP 文件，
 * 我不知道你实际发送接口叫什么。
 *
 * 你只需要把下面这句注释替换成你自己的 UART10 发送函数即可。
 *
 * 例如：
 * Uart10_Send_Cmd_Data(cmd, data, len);
 * 或者：
 * HAL_UART_Transmit(&huart10, ...);
 */
static void ArmIK_SendResultToUART10(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    (void)cmd;
    (void)data;
    (void)len;

    /* TODO：替换成你自己的 UART10 发送接口 */
    /* Uart10_Send_Cmd_Data(cmd, (uint8_t *)data, len); */
}

/*
 * 同时发给 USB 和 UART10
 *
 * 状态包一共 6 字节，格式见头文件说明
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
     * 这里继续沿用你原来工程里已经在用的命令字：
     * USB_CMD_ARM_IK_RESULT
     */
    ArmIK_SendResultToUSB(USB_CMD_ARM_IK_RESULT, tx_data, sizeof(tx_data));
    ArmIK_SendResultToUART10(USB_CMD_ARM_IK_RESULT, tx_data, sizeof(tx_data));
}

/*
 * 电机方向控制角（rad） -> 电机目标角（deg）
 *
 * 注意：
 * 这里只做“弧度转角度”。
 * dir 方向映射已经在 Arm3R_GeomCtrlToMotorCtrl() 里做完了。
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

/*
 * 当本次目标点“可达且安全”时，
 * 将其保存成“上一组安全目标”
 *
 * 这样后面一旦收到非法目标点，
 * 就可以直接保持上一组安全角度输出
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

    /* 保存最近一次安全目标点 */
    g_arm_ik_app.last_valid_pt.x = x;
    g_arm_ik_app.last_valid_pt.y = y;
    g_arm_ik_app.last_valid_pt.z = z;

    g_arm_ik_app.last_valid_model = res->model;   /* 新增 */

    /* 保存最近一次安全控制角 */
    g_arm_ik_app.last_valid_geom = res->ctrl;
    g_arm_ik_app.last_valid_motor = *motor_ctrl_rad;
    g_arm_ik_app.last_valid_motor_deg = *motor_ctrl_deg;

    g_arm_ik_app.active_model = res->model;       /* 新增 */

    /*
     * 当前实际维持输出也同步更新为这次新目标
     */
    g_arm_ik_app.active_motor_deg = *motor_ctrl_deg;

    g_arm_ik_app.has_last_valid = 1U;
}

/*
 * 当输入目标点非法时，执行保护动作
 *
 * 策略如下：
 * 1. 如果已经有历史安全目标，则保持上一组安全角度目标
 * 2. 如果还没有历史安全目标，则保持当前不动
 *
 * 返回值：
 *   ARM_IK_ACTION_HOLD_LAST
 *   ARM_IK_ACTION_KEEP_CURRENT
 */
static uint8_t ArmIK_HandleInvalidTarget(void)
{
    if (g_arm_ik_app.has_last_valid != 0U)
    {
        /*
         * 维持上一组安全目标
         */
        g_arm_ik_app.active_model = g_arm_ik_app.last_valid_model;        //模型角
        g_arm_ik_app.active_motor_deg = g_arm_ik_app.last_valid_motor_deg;//控制角
        return ARM_IK_ACTION_HOLD_LAST;
    }

    /*
     * 如果系统还从未获得过一组有效安全目标，
     * 那就不更新输出，保持当前状态
     */
    return ARM_IK_ACTION_KEEP_CURRENT;
}

/* ========================= 对外接口实现 ========================= */
/*
 * 初始化机械臂逆解应用层
 *
 * 参数沿用你原来 arm_user.c 里的配置：
 * d1 = 0
 * a2 = 320
 * a3 = 320
 * j1_ref = 0 deg
 * j2_ref = 70 deg
 * j3_ref = -145 deg
 */
void ArmIK_ComponentInit(void)
{
    Arm3R_Config_t cfg;

    /* 连杆参数 */
    cfg.link.d1 = 0.0f;
    cfg.link.a2 = 320.0f;
    cfg.link.a3 = 320.0f;

    /* 上电参考模型角 */
    cfg.j1_ref.model_ref = Arm3R_DegToRad(0.0f);
    cfg.j2_ref.model_ref = Arm3R_DegToRad(70.0f);
    cfg.j3_ref.model_ref = Arm3R_DegToRad(-145.0f);

    /* 电机方向映射 */
    cfg.j1_ref.dir = +1;
    cfg.j2_ref.dir = -1;
    cfg.j3_ref.dir = +1;

    /* 初始化逆解组件 */
    Arm3R_Init(&g_arm_ik, &cfg);

    /* 清空应用层运行状态 */
    ArmIK_ResetAppState();
}

/*
 * 输入一个目标点，完成：
 * 1. 逆解
 * 2. 安全判断
 * 3. 状态双路回传
 * 4. 更新当前角度制目标
 *
 * 这是应用层的主入口
 */
void ArmIK_ComponentStep(float x, float y, float z)
{
    Arm3R_Status_t ret;
    const Arm3R_Result_t *res;

    /*
     * motor_ctrl_rad：
     * 已经乘过 dir 的电机方向控制角，单位仍然是 rad
     */
    Arm3R_CtrlAngles_t motor_ctrl_rad;

    /*
     * motor_ctrl_deg：
     * 最终给电机用的角度制目标，单位是 deg
     */
    ArmIK_MotorDeg_t motor_ctrl_deg;

    uint8_t action_code;

    /* 保存最近一次收到的目标点 */
    g_arm_ik_app.last_req_pt.x = x;
    g_arm_ik_app.last_req_pt.y = y;
    g_arm_ik_app.last_req_pt.z = z;

    /*
     * 执行逆解 + 安全判定
     * theta1_hint 这里沿用 0.0f
     */
    ret = Arm3R_Solve(&g_arm_ik, x, y, z, 0.0f);

    /*
     * 读取内部求解结果结构体
     */
    res = Arm3R_GetResult(&g_arm_ik);

    if (ret == ARM3R_OK)
    {
        /*
         * 情况 1：目标点可达且安全
         *
         * 处理流程：
         * 1. 把几何控制角映射成电机方向控制角（rad）
         * 2. 再把 rad 转成 deg
         * 3. 保存为当前有效目标
         * 4. 状态回传给 USB 和 UART10
         */
        Arm3R_GeomCtrlToMotorCtrl(&res->ctrl, &g_arm_ik.cfg, &motor_ctrl_rad);
        ArmIK_MotorCtrlRadToDeg(&motor_ctrl_rad, &motor_ctrl_deg);

        ArmIK_SaveAsLastValidTarget(x, y, z, res, &motor_ctrl_rad, &motor_ctrl_deg);

        action_code = ARM_IK_ACTION_APPLY_NEW;
        ArmIK_SendResultToAll(ARM_IK_RESULT_OK, 0U, action_code, res);
    }
    else if (ret == ARM3R_ERR_UNREACHABLE)
    {
        /*
         * 情况 2：几何不可达
         *
         * 不能采用本次新目标，
         * 走保护策略：保持上一组安全目标或保持当前不动
         */
        action_code = ArmIK_HandleInvalidTarget();
        ArmIK_SendResultToAll(ARM_IK_RESULT_UNREACHABLE, 0U, action_code, res);
    }
    else if (ret == ARM3R_ERR_UNSAFE)
    {
        /*
         * 情况 3：几何可达，但不满足安全约束
         *
         * 同样不能采用本次新目标，
         * 继续维持安全输出
         */
        uint8_t unsafe_reason = 0U;

        if (res != 0)
        {
            unsafe_reason = (uint8_t)res->unsafe_reason;
        }

        action_code = ArmIK_HandleInvalidTarget();
        ArmIK_SendResultToAll(ARM_IK_RESULT_UNSAFE, unsafe_reason, action_code, res);
    }
    else
    {
        /*
         * 情况 4：参数错误、未初始化、其他异常
         *
         * 依然不采用本次目标，走保护策略
         */
        action_code = ArmIK_HandleInvalidTarget();
        ArmIK_SendResultToAll(ARM_IK_RESULT_PARAM_ERR, 0U, action_code, res);
    }
}

/*
 * 处理外部收到的 3 维目标点负载
 *
 * 负载格式固定为：
 * payload[0..3]   -> float x
 * payload[4..7]   -> float y
 * payload[8..11]  -> float z
 *
 * 注意：
 * 这里默认上位机 / 遥控器 与 STM32 都按 little-endian float 通信
 */
void ArmIK_ComponentHandleXYZPayload(const uint8_t *payload, uint16_t len)
{
    float x;
    float y;
    float z;
    uint8_t action_code;

    /*
     * 长度不对，或者空指针：
     * 视为无效输入
     */
    if ((payload == 0) || (len != ARM_IK_XYZ_PAYLOAD_LEN))
    {
        action_code = ArmIK_HandleInvalidTarget();
        ArmIK_SendResultToAll(ARM_IK_RESULT_PARAM_ERR, 0U, action_code, 0);
        return;
    }

    /*
     * 用 memcpy 解析 float，
     * 避免直接强转指针带来的未对齐访问问题
     */
    memcpy(&x, &payload[0], 4);
    memcpy(&y, &payload[4], 4);
    memcpy(&z, &payload[8], 4);

    /*
     * 交给主处理函数
     */
    ArmIK_ComponentStep(x, y, z);
}

/*
 * 获取当前实际维持的角度制目标
 *
 * 你自己的电机控制层如果只想拿最终角度，
 * 直接读取这个返回值即可
 */
const ArmIK_MotorDeg_t *ArmIK_GetActiveMotorDeg(void)
{
    return &g_arm_ik_app.active_motor_deg;
}

/*
 * 获取完整应用层状态
 *
 * 调试时可查看：
 * 1. last_req_pt
 * 2. last_valid_pt
 * 3. active_motor_deg
 * 4. has_last_valid
 */
const ArmIK_AppState_t *ArmIK_GetAppState(void)
{
    return &g_arm_ik_app;
}