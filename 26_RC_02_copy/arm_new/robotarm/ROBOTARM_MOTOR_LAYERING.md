# 机械臂电机逻辑与分层说明

本说明只对应新机械臂测试工程 `arm_new/robotarm`。

## 当前电机使用逻辑

- 总线：三台电机共用 `FDCAN3`，使用 Classic CAN 标准帧发送 MIT 控制帧。
- 电机 ID：
  - 达妙 DM-J8006 1：`0x01`
  - 达妙 DM-J8006 2：`0x02`
  - 灵足 EL05：`0x03`
  - EL05 MIT 反馈兼容接收 `0x03` 和 `0xFD`
- 上电初始化顺序：
  - `main.c` 调用 `MX_GPIO_Init()`、`MX_FDCAN3_Init()`、`MX_TIM6_Init()`
  - `RobotArm_Mixed_Init()` 清空反馈状态，并将两个达妙关节标记为 MIT 模式
  - 如果 `ARM_EL05_SET_MIT_PROTOCOL_ON_BOOT=1`，调用 `RobotArm_EL05_SetProtocolMIT()` 将 EL05 切到 MIT 协议
  - `RobotArm_Mixed_Enable()` 连续 3 轮发送使能帧，顺序为 DM1、DM2、EL05
- 使能/失能/设零帧：
  - 使能：`FF FF FF FF FF FF FF FC`
  - 失能：`FF FF FF FF FF FF FF FD`
  - 设零：`FF FF FF FF FF FF FF FE`
- 主循环控制：
  - 周期由 `ARM_MIT_TEST_CONTROL_PERIOD_MS` 设置，当前为 `100 ms`
  - 每周期先调用 `leg_angle(2)`，给 DM1 发送一帧测试角度命令
  - 随后构造 `RobotArm_MixedCommand_t` 并调用 `RobotArm_Mixed_Control()`，依次发送 DM1、DM2、EL05 的 MIT 控制帧
  - 注意：当前 `RobotArm_Mixed_Control()` 会再次给 DM1 发 `cmd.dm8006_1`，因此会覆盖同周期内 `leg_angle(2)` 刚发给 DM1 的那帧效果
- 接收反馈：
  - BSP 层的 `HAL_FDCAN_RxFifo0Callback()` 循环读空 FIFO0
  - Device 层实现强符号 `FDCAN3_RxMessageCallback()`，并转发给 `RobotArm_Mixed_HandleRx()`
  - DM1/DM2 反馈解析到 `chassis_move.joint_motor[0/1]`
  - EL05 反馈解析到 `el05_angle`、`el05_speed`、`el05_torque`

## 底层 FDCAN 配置

- 外设：`FDCAN3`
- 引脚：`PD12 = FDCAN3_RX`，`PD13 = FDCAN3_TX`，复用 `GPIO_AF5_FDCAN3`
- 帧格式：`FDCAN_FRAME_CLASSIC`
- 模式：`FDCAN_MODE_NORMAL`
- 自动重发：开启
- FDCAN 时钟：`RCC_FDCANCLKSOURCE_PLL2`，当前计算值 `100 MHz`
- 仲裁段参数：
  - `NominalPrescaler = 4`
  - `NominalTimeSeg1 = 19`
  - `NominalTimeSeg2 = 5`
  - `NominalSyncJumpWidth = 1`
  - 对应约 `1 Mbps`
- 数据段参数：
  - `DataPrescaler = 4`
  - `DataTimeSeg1 = 19`
  - `DataTimeSeg2 = 5`
  - 当前使用 Classic CAN，未启用 BRS
- FIFO/过滤：
  - 标准过滤器数量：`5`
  - 扩展过滤器数量：`0`
  - Rx FIFO0：`4` 个元素，元素大小 `24 bytes`
  - Tx FIFO Queue：`8` 个元素，元素大小 `24 bytes`
  - 标准帧进入 FIFO0，扩展帧拒收
- 中断：
  - `FDCAN3_IT0_IRQn`、`FDCAN3_IT1_IRQn`
  - 优先级均为 `0`
  - 启用 `FDCAN_IT_RX_FIFO0_NEW_MESSAGE`

## 代码分层

- BSP 外设配置层：
  - `BSP/Inc/can_bsp.h`
  - `BSP/Src/can_bsp.c`
  - 负责 FDCAN 滤波/启动、RX FIFO0 中断取帧、原始标准帧发送、DLC 转换、运行诊断计数，不包含具体电机协议。
- Device 设备驱动层：
  - `Device/Inc/dm_motor.h`
  - `Device/Src/dm_motor.c`
  - `Device/Inc/Robstride.h`
  - `Device/Src/Robstride01.cpp`
  - `Device/Inc/robotarm_state.h`
  - `Device/Inc/robotarm_mixed.h`
  - `Device/Src/robotarm_mixed.cpp`
  - 封装达妙 DM8006/DM6215 与灵足 RobStride/EL05 的协议、MIT 帧打包、反馈解析、三电机使能/失能、控制帧发送和反馈分流。
- Algorithm 算法层：
  - 当前置空。
  - 后续只放机械臂控制算法，例如运动学、逆解、轨迹规划、限位处理、插补、闭环控制策略等。
  - 不直接实现电机协议、CAN 发送、使能/失能或反馈解析。
