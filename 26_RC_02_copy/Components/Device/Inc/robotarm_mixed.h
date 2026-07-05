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

typedef enum
{
    ROBOTARM_COMM_POS_SPEED = 0,
    ROBOTARM_COMM_MIT = 1
} RobotArm_MotorCommMode_t;

typedef enum
{
    ROBOTARM_MOTOR_DM8006_1 = 0,
    ROBOTARM_MOTOR_DM8006_2 = 1,
    ROBOTARM_MOTOR_EL05 = 2
} RobotArm_MotorIndex_t;

typedef struct
{
    RobotArm_MotorCommMode_t dm8006_1_mode;
    RobotArm_MotorCommMode_t dm8006_2_mode;
    RobotArm_MotorCommMode_t el05_mode;
    uint8_t el05_set_mit_protocol_once;
} RobotArm_MixedConfig_t;

typedef struct
{
    RobotArm_MotorCommMode_t mode;
    float pos;
    float vel;
    float kp;
    float kd;
    float tor;
} RobotArm_MotorCommand_t;

typedef RobotArm_MotorCommand_t RobotArm_MitCommand_t;

typedef struct
{
    RobotArm_MotorCommand_t dm8006_1;
    RobotArm_MotorCommand_t dm8006_2;
    RobotArm_MotorCommand_t el05;
} RobotArm_MixedCommand_t;

typedef struct
{
    uint16_t id;
    uint16_t mode;
    uint16_t state;
    uint32_t rx_cnt;
    uint8_t feedback_ok;
    uint8_t fault;
    uint8_t warning;
    int p_int;
    int v_int;
    int t_int;
    float pos;
    float vel;
    float tor;
    float tmos;
    float tcoil;
} RobotArm_MotorFeedback_t;

typedef struct
{
    RobotArm_MotorFeedback_t dm8006_1;
    RobotArm_MotorFeedback_t dm8006_2;
    RobotArm_MotorFeedback_t el05_3;
} RobotArm_FDCAN3Feedback_t;

typedef struct
{
    uint32_t total_rx_cnt;
    uint32_t dm8006_1_rx_cnt;
    uint32_t dm8006_2_rx_cnt;
    uint32_t el05_rx_cnt;
    uint32_t unknown_rx_cnt;
    uint32_t dm_id_mismatch_cnt;
    uint16_t last_identifier;
    uint8_t last_dlc_len;
    uint8_t last_payload_id;
    uint8_t last_data[8];
} RobotArm_FDCAN3RxDebug_t;

typedef struct
{
    uint32_t tx_cnt;
    RobotArm_MixedCommand_t last_cmd;
    uint16_t dm8006_1_tx_id;
    uint16_t dm8006_2_tx_id;
    uint16_t el05_tx_id;
    uint8_t dm8006_1_data[8];
    uint8_t dm8006_2_data[8];
    uint8_t el05_data[8];
} RobotArm_FDCAN3TxDebug_t;

void RobotArm_Mixed_Init(void);
void RobotArm_Mixed_InitWithConfig(const RobotArm_MixedConfig_t *config);
void RobotArm_Mixed_Enable(void);
void RobotArm_Mixed_Disable(void);
void RobotArm_Mixed_SetZero(void);
void RobotArm_Mixed_SetCommModes(RobotArm_MotorCommMode_t dm8006_1_mode,
                                 RobotArm_MotorCommMode_t dm8006_2_mode,
                                 RobotArm_MotorCommMode_t el05_mode,
                                 uint8_t el05_set_mit_protocol_once);
void RobotArm_Mixed_SetMotorParam(RobotArm_MotorIndex_t motor,
                                  RobotArm_MotorCommMode_t mode,
                                  float pos,
                                  float vel,
                                  float kp,
                                  float kd,
                                  float tor);
void RobotArm_Mixed_SetTargetCommands(const RobotArm_MotorCommand_t *dm8006_1,
                                      const RobotArm_MotorCommand_t *dm8006_2,
                                      const RobotArm_MotorCommand_t *el05);
void RobotArm_Mixed_SetControlCommand(const RobotArm_MixedCommand_t *cmd);
void RobotArm_Mixed_Control(const RobotArm_MixedCommand_t *cmd);
void RobotArm_Mixed_Transmit1ms(void);
void RobotArm_Mixed_GetFeedbackSnapshot(RobotArm_FDCAN3Feedback_t *out);
void RobotArm_Mixed_HandleRx(const FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);

void RobotArm_FDCAN3_TestMITKpZeroStart(float dm8006_1_rad,
                                        float dm8006_2_rad,
                                        float el05_rad);
void RobotArm_FDCAN3_TestMain(float dm8006_1_rad,
                              float dm8006_2_rad,
                              float el05_rad);

void RobotArm_EL05_MIT_Enable(void);
void RobotArm_EL05_MIT_Disable(void);
void RobotArm_EL05_MIT_Control(float pos, float vel, float kp, float kd, float tor);
void RobotArm_EL05_PosSpeed_Control(float pos, float vel);
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
extern volatile RobotArm_FDCAN3Feedback_t g_robotarm_fdcan3_feedback;
extern volatile RobotArm_FDCAN3RxDebug_t g_robotarm_fdcan3_rx_debug;
extern volatile RobotArm_FDCAN3TxDebug_t g_robotarm_fdcan3_tx_debug;

void leg_angle (int a);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class RobStride_Motor;
extern RobStride_Motor robotarm_el05;
#endif

#endif
