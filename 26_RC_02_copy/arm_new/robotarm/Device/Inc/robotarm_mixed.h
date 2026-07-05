#ifndef ROBOTARM_MIXED_H
#define ROBOTARM_MIXED_H

#include "fdcan.h"
#include "main.h"
#include "robotarm_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 三个电机在同一条 CAN 总线上，所以每个电机的标准帧 ID 必须不同。
 * 如果你用上位机改了电机 ID，这里也要同步改。
 */
#define ROBOTARM_DM8006_1_ID       0x01U  /* 第 1 个达妙 DM-J8006 的 CAN ID */
#define ROBOTARM_DM8006_2_ID       0x02U  /* 第 2 个达妙 DM-J8006 的 CAN ID */
#define ROBOTARM_EL05_ID           0x03U  /* 灵足 EL05 的 CAN ID */

/* EL05 在 MIT 模式下可能用 0xFD 作为主机/反馈 ID。
 * 接收函数会同时识别 0x03 和 0xFD，避免只收得到使能、收不到反馈。
 */
#define ROBOTARM_EL05_MIT_RX_ID    0xFDU

#define ROBOTARM_EL05_PROTOCOL_PRIVATE  0x00U
#define ROBOTARM_EL05_PROTOCOL_CANOPEN  0x01U
#define ROBOTARM_EL05_PROTOCOL_MIT      0x02U

typedef struct
{
    float pos;  /* 目标位置，单位 rad。MIT 模式里这是绝对位置，不是“转多少”。 */
    float vel;  /* 目标速度，单位 rad/s。 */
    float kp;   /* 位置刚度。Kp=0 时，pos 不会产生任何位置保持力矩。 */
    float kd;   /* 速度阻尼。常用于抑制振动，也能按速度误差产生力矩。 */
    float tor;  /* 前馈力矩，单位 Nm。可以用来测试电机是否真的会出力。 */
} RobotArm_MitCommand_t;

typedef struct
{
    RobotArm_MitCommand_t dm8006_1;  /* 发给达妙 1 的 MIT 命令 */
    RobotArm_MitCommand_t dm8006_2;  /* 发给达妙 2 的 MIT 命令 */
    RobotArm_MitCommand_t el05;      /* 发给 EL05 的 MIT 命令 */
} RobotArm_MixedCommand_t;

void RobotArm_Mixed_Init(void);       /* 清空反馈变量，并把两个达妙关节标记为 MIT 模式 */
void RobotArm_Mixed_Enable(void);     /* 依次给三台电机发送 MIT 使能帧 */
void RobotArm_Mixed_Disable(void);    /* 依次给三台电机发送 MIT 失能帧 */
void RobotArm_Mixed_Control(const RobotArm_MixedCommand_t *cmd);  /* 周期发送三台电机的 MIT 控制帧 */
void RobotArm_Mixed_HandleRx(const FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);  /* FDCAN3 收到反馈后的分流解析 */

void RobotArm_EL05_MIT_Enable(void);
void RobotArm_EL05_MIT_Disable(void);
void RobotArm_EL05_MIT_Control(float pos, float vel, float kp, float kd, float tor);
void RobotArm_EL05_MIT_SetZero(void);
void RobotArm_EL05_SetProtocol(uint8_t protocol);
void RobotArm_EL05_SetProtocolMIT(void);

extern volatile uint8_t el05_enable_sent;
extern volatile uint8_t el05_feedback_ok;
extern volatile uint32_t el05_rx_cnt;
extern volatile float el05_angle;
extern volatile float el05_speed;
extern volatile float el05_torque;
extern volatile uint32_t el05_tx_id;
extern volatile uint8_t el05_tx_data[8];
extern volatile uint8_t el05_tx_status;

void leg_angle (int a);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class RobStride_Motor;
extern RobStride_Motor robotarm_el05;
#endif

#endif
