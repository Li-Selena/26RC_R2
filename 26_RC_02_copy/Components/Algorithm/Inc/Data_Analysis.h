#ifndef __DATA_ANALYSIS_H__
#define __DATA_ANALYSIS_H__

#include <stdint.h>
#include "bsp_usb.h"
#include "mecanum_classic.h"

/*
 * USB command map.
 *
 * Arm/tool commands use the new constrained arm interface:
 *   f0=tool enum, f1=tool state, f2=target_z_mm, f3=approach_yaw_rad.
 */

#define USB_CMD_SYS_DISABLE         0x00U
#define USB_CMD_SYS_ENABLE          0x01U
#define USB_CMD_SYS_SWITCH_SOURCE   0x02U  /* datas[0]: 0=USART 1=USB */
#define USB_CMD_SYS_STOP            0x05U
#define USB_CMD_SYS_GET_STATUS      0x06U

#define USB_CMD_CHS_DISABLE         0x10U
#define USB_CMD_CHS_ENABLE          0x11U
#define USB_CMD_CHS_SET_MODE        0x12U  /* datas[0]: mode 0-7 */
#define USB_CMD_CHS_SET_VEL         0x13U  /* vx, vy, yaw_data, reserved */
#define USB_CMD_CHS_SET_POS         0x14U  /* dx, dy, yaw_data, reserved */
#define USB_CMD_CHS_STOP            0x15U
#define USB_CMD_CHS_GET_STATUS      0x16U

#define USB_CMD_ARM_DISABLE         0x20U
#define USB_CMD_ARM_ENABLE          0x21U
#define USB_CMD_ARM_SET_WORKSPACE   0x22U  /* f0=direction 0=Y+ 1=X+ 2=X- */
#define USB_CMD_ARM_SET_TARGET      0x23U
#define USB_CMD_ARM_SET_TARGET_XYZ  0x24U  /* f0=x_mm f1=y_mm f2=z_mm */
#define USB_CMD_ARM_STOP            0x25U
#define USB_CMD_ARM_GET_STATUS      0x26U
#define USB_CMD_ARM_IK_TEST_FLOW    0x27U  /* empty=start, f0=1 start/f0=0 stop, f1=step_ms */
#define USB_CMD_ARM_HEIGHT_JOG      0x28U  /* f0=delta_z_mm */
#define USB_CMD_ARM_HEIGHT_LIMIT    0x29U  /* f0=0 min, f0=1 max */
#define USB_CMD_ARM_SET_POSTURE     0x2AU  /* f0=tool, f1=state, keep xyz */
#define USB_CMD_ARM_JOINT_JOG       0x2BU  /* f0=joint 2/3, f1=signed delta_deg */

#define USB_CMD_TOOL_DISABLE        0x30U
#define USB_CMD_TOOL_ENABLE         0x31U
#define USB_CMD_TOOL_SET_MODE       0x32U
#define USB_CMD_TOOL_STOP           0x35U
#define USB_CMD_TOOL_GET_STATUS     0x36U

#define USB_CMD_ROBOT_GET_STATUS    0x46U
#define USB_CMD_YAW_TUNE_START      0x47U  /* optional f0=pass_count, default 1 */
#define USB_CMD_YAW_TUNE_STOP       0x48U
#define USB_CMD_YAW_TUNE_GET_STATUS 0x49U

#define USB_CMD_CLIMB_DISABLE       0x50U
#define USB_CMD_CLIMB_ENABLE        0x51U
#define USB_CMD_CLIMB_SET_CTRL      0x52U  /* f0=enable, f1=step, f2=auto */
#define USB_CMD_CLIMB_STEP          0x53U
#define USB_CMD_CLIMB_UP_STEP       USB_CMD_CLIMB_STEP
#define USB_CMD_CLIMB_UP_AUTO       0x54U
#define USB_CMD_CLIMB_AUTO          USB_CMD_CLIMB_UP_AUTO
#define USB_CMD_CLIMB_RUN           USB_CMD_CLIMB_UP_AUTO
#define USB_CMD_CLIMB_UP_RUN        USB_CMD_CLIMB_UP_AUTO
#define USB_CMD_CLIMB_STOP          0x55U
#define USB_CMD_CLIMB_GET_STATUS    0x56U
#define USB_CMD_CLIMB_TEST_ACTION   0x57U  /* f0=R2_ClimbTestAction_t */
#define USB_CMD_CLIMB_DOWN_STEP     0x58U
#define USB_CMD_CLIMB_DOWN_AUTO     0x59U
#define USB_CMD_CLIMB_DOWN_RUN      USB_CMD_CLIMB_DOWN_AUTO
#define USB_CMD_CLIMB_UP_GATE       0x5AU
#define USB_CMD_CLIMB_DOWN_GATE     0x5BU
#define USB_CMD_CLIMB_UP_AUTO_PAUSE 0x5CU
#define USB_CMD_CLIMB_DOWN_AUTO_PAUSE 0x5DU
#define USB_CMD_CLIMB_AUTO_RESUME   0x5EU

#define USB_CMD_FLOW_S1_UP          0x60U
#define USB_CMD_FLOW_S1_DOWN        0x61U
#define USB_CMD_FLOW_S1_UP_S2_DOWN  0x63U
#define USB_CMD_FLOW_S1_DOWN_S2_UP  0x64U
#define USB_CMD_FLOW_S1_DOWN_S2_DOWN 0x65U
#define USB_CMD_FLOW_GET_STATUS     0x66U
#define USB_CMD_FLOW_WEAPON_GRAB    0x67U
#define USB_CMD_FLOW_THROW_BLOCK    0x68U  /* f0=1 X+, f0=2 X- */
#define USB_CMD_FLOW_WEAPON_DOCK_TEST 0x69U /* lower-computer arm/gripper test flow */
#define USB_CMD_FLOW_CHASSIS_MOVE_DONE 0x6AU /* host confirms checkpoint 1 */
#define USB_CMD_FLOW_DOCK_DONE      0x6BU /* host confirms checkpoint 2 */

#define USB_CHASSIS_TIMEOUT_MS      100U

typedef struct
{
    uint32_t last_tick;
    uint32_t count;
    uint8_t last_cmd;
    uint8_t last_len;
    uint8_t last_payload_valid;
    uint8_t reserved[3];
    uint8_t last_data[16];
    float last_f[4];
} USB_CommandRxState_t;

extern uint8_t Mecanum_control_flag;
extern ChassisVel_t total_vel_USB;
extern WheelSpeed_t total_speed_USB;

void Data_Analysis(uint8_t cmd, const uint8_t *datas, uint8_t len);
void USB_GetCommandRxState(USB_CommandRxState_t *out);
void USB_ControlWatchdog_Check(void);
void USB_ChassisWatchdog_Check(void);
uint8_t USB_ChassisWatchdog_IsTimeout(void);
uint32_t USB_ChassisWatchdog_LastTick(void);
uint8_t USB_ControlWatchdog_TimeoutFlags(void);

#endif
