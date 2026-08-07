# USB 指令工具使用说明

这个工具用于上位机通过 USB 虚拟串口给下位机发送指令，并解析下位机返回的数据。

默认你现在用的是 `COM26`，所以后面的示例都写成 `COM26`。

## 1. 安装依赖

```powershell
cd F:\spareE\Vinci_Robocon_2026\26_RC_Projects\26_RC_02\PC_USB_Serial_Tool
python -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
```

## 2. 进入 USB 指令交互工具

```powershell
.\.venv\Scripts\python usb_cmd_tool.py shell --port COM26 --baud 115200
```

进入后可以输入：

```text
usb
enable
status
```

工具会自动解析返回的 USB 回包，并打印 JSON。

## Climb wait commands

在 shell 里可以让上位机先等待当前上/下台阶动作完成，再发送下一条 USB 命令：

```text
climb_wait 20
climb_step_wait 40
climb_up_auto_wait 180
climb_auto_wait 180
climb_run_wait 180
climb_up_step_wait 40
climb_upstairs_step_wait 40
climb_upstairs_auto_wait 180
climb_upstairs_run_wait 180
climb_down_step_wait 40
climb_down_auto_wait 180
climb_downstairs_step_wait 40
climb_downstairs_auto_wait 180
climb_downstairs_run_wait 180
climb_wait_then CLIMB_UP_STEP
climb_wait_then CLIMB_DOWN_STEP
climb_tests
climb_test CHASSIS_FORWARD_100
climb_test_wait FRONT_UP_10 10
climb_front_220 10
climb_front_minus_30 10
climb_rear_220 10
climb_rear_minus_30 10
climb_chassis_forward_100
climb_chassis_backward_100
climb_chassis_forward_300
climb_chassis_backward_300
climb_front_drive_forward_30
climb_front_drive_forward_10
climb_front_drive_backward_10
climb_front_drive_backward_30
climb_front_drive_forward_500
climb_front_drive_backward_500
climb_rear_drive_forward_30
climb_rear_drive_forward_10
climb_rear_drive_backward_10
climb_rear_drive_backward_30
climb_rear_drive_forward_500
climb_rear_drive_backward_500
climb_all_drive_forward_30
climb_all_drive_forward_10
climb_all_drive_backward_10
climb_all_drive_backward_30
climb_all_drive_forward_500
climb_all_drive_backward_500
climb_all_legs_220 5
flow_recover
```

等待逻辑会轮询 `CLIMB_GET_STATUS`，新版固件优先看 `summary_state`：`READY/DONE` 继续，`RUNNING` 继续等待，`ERROR` 报错；旧固件则回退到 `state_done=1`、`IDLE` 或 `DONE`。
新版 `CLIMB_GET_STATUS` 还会解析 `status_flags`、`leg_reached`、`drive_reached`、`summary_state`，自动化脚本可直接判断 `READY/RUNNING/DONE/ERROR/PAUSED` 或逐个执行机构是否到位；`state=29/30` 只表示自动下台阶正在执行内部修补。自动上台阶失败会直接进入 `ERROR`，不再修补或重跑；若 `ERROR` 且 `error_flags.recovery_failed=true`，说明下台阶修补或重跑一次后仍失败，需要上位机接管。

## Mixed arm/base/climb block flow

`blocks` now lists arm, base, tool, and climb block commands in one table.
Use `block_flow_start` to record a mixed task flow, type block IDs such as
`arm_down_20mm`, `base_forward_50mm`, and `climb_front_220`, then export or run it:

```text
blocks
block_flow_start mixed_task_v1
arm_down_20mm
climb_front_220
base_forward_50mm
block_flow_export mixed_task_v1.json
block_flow_run mixed_task_v1.json 40 1.0
```

The runner is a small state machine: `IDLE -> SEND_STEP -> WAIT_STEP -> DONE`.
Wait policy is inferred from each block: climb waits for `CLIMB_GET_STATUS`,
base waits for `CHS_GET_STATUS.pos_state`, and arm waits for a clean
`ARM_GET_STATUS` plus the configured settle time.

Non-interactive run:

```powershell
.\.venv\Scripts\python -m serial_tool block-flow-run --port COM26 --baud 115200 mixed_task_v1.json --timeout 40 --arm-settle 1.0
.\.venv\Scripts\python -m serial_tool send --port COM26 --baud 115200 FLOW_WEAPON_GRAB --wait 0.2
.\.venv\Scripts\python -m serial_tool send --port COM26 --baud 115200 FLOW_WEAPON_DOCK_TEST --wait 0.2
.\.venv\Scripts\python -m serial_tool send --port COM26 --baud 115200 FLOW_CHASSIS_MOVE_DONE --wait 0.2
.\.venv\Scripts\python -m serial_tool send --port COM26 --baud 115200 FLOW_DOCK_DONE --wait 0.2
.\.venv\Scripts\python -m serial_tool poll --port COM26 --baud 115200 FLOW_GET_STATUS --rate 10
.\.venv\Scripts\python -m serial_tool block-flow-run --port COM26 --baud 115200 weapon_grab_v1.json --timeout 20 --arm-settle 1.0
```

## 3. 底层 USB 命令速查

查看工具内置命令表：

```powershell
.\.venv\Scripts\python usb_cmd_tool.py commands
```

常用底层命令名：

```text
send YAW_TUNE_START 1       # 启动 yaw 自动调参，pass_count=1
send YAW_TUNE_GET_STATUS    # 查询 yaw 自动调参状态
send YAW_TUNE_STOP          # 停止 yaw 自动调参
send CLIMB_UP_STEP          # 上台阶手动推进一步
send CLIMB_UP_AUTO          # 上台阶自动执行/继续
send CLIMB_DOWN_STEP        # 下台阶手动推进一步
send CLIMB_DOWN_AUTO        # 下台阶自动执行/继续
send ARM_IK_TEST_FLOW       # 启动机械臂逆解测试流程，默认 8000ms/step
send ARM_IK_TEST_FLOW 1 8000 # 启动测试流程，指定 8000ms/step
send ARM_IK_TEST_FLOW 0 0   # 停止测试流程
send ARM_SET_TARGET_XYZ 120 0 450 0 # 发送三维目标；Z 逆解，X/Y 只选运动空间
send ARM_SET_WORKSPACE 1    # 切换到 X+ 运动空间，0=Y+, 1=X+, 2=X-
send ARM_HEIGHT_JOG 20      # 当前高度升高 20mm
send ARM_HEIGHT_JOG -50     # 当前高度下降 50mm
send ARM_HEIGHT_LIMIT 1     # 当前工具姿态移动到最大 z，0=最小，1=最大
send ARM_SET_POSTURE 1 2    # 切换工具姿态：tool=S2, state=2
```

交互 shell 也提供更顺手的机械臂快捷命令：

```text
arm_enable / arm_disable
arm_space Y+|X+|X-
arm_xyz X_MM Y_MM Z_MM
arm_up/down 5|10|20|50|100|200|500
arm_up20 / arm_down50
arm_zmin / arm_zmax
arm_posture S1|S2|gripper STATE
tool_on S2|gripper
tool_off S2|gripper
arm_status / tool_status
```

`arm_xyz` 的 `X/Y` 只决定 `Y+ / X+ / X-` 工作空间；因机械结构无法在平面内到达的坐标，需要底盘命令另行实现。

Auto laser gate:

- `CLIMB_UP_AUTO` starts a forward mecanum search immediately; firmware drives forward until valid X reaches `0 <= x < 35mm` before starting the climb flow. Transient X invalid readings (`-1`) are tolerated, but continuous X invalid for `1000ms` enters `ERROR` with the `timeout` flag.
- `CLIMB_DOWN_AUTO` starts a forward mecanum search immediately. During `DOWN_LASER_APPROACH_H_GT_65`, valid height `h > 65mm` detects the 50-53mm to 160mm+ drop jump; transient height invalid readings (`-1`) are tolerated, continuous invalid height for `1000ms` enters `ERROR`, and no trigger within `8000ms` also times out. After trigger, the chassis moves forward another `5mm`, then enters `PREPARE_ALL_LEGS_MINUS_30` before the normal downstairs flow.
- `CLIMB_UP_GATE` and `CLIMB_DOWN_GATE` run only the same auto laser gate stage. The block commands are `climb_up_laser_gate` and `climb_down_laser_gate`.
- `CLIMB_UP_AUTO_PAUSE` and `CLIMB_DOWN_AUTO_PAUSE` run the real firmware auto flow and pause at the v2 interrupt point. `CLIMB_AUTO_RESUME` continues that paused auto flow to final `DONE`. The block commands are `climb_up_auto_pause`, `climb_down_auto_pause`, and `climb_auto_resume`; the six v2 flows use these commands around their 100 interrupt slots.
- Old names `CLIMB_AUTO` and `CLIMB_DOWNSTAIRS_AUTO` are still accepted as aliases.

只想生成帧、不发送串口时可以用：

```powershell
.\.venv\Scripts\python usb_cmd_tool.py pack YAW_TUNE_GET_STATUS
.\.venv\Scripts\python usb_cmd_tool.py pack CLIMB_UP_AUTO
.\.venv\Scripts\python usb_cmd_tool.py pack CLIMB_DOWN_AUTO
.\.venv\Scripts\python usb_cmd_tool.py pack CLIMB_UP_GATE
.\.venv\Scripts\python usb_cmd_tool.py pack CLIMB_DOWN_GATE
.\.venv\Scripts\python usb_cmd_tool.py pack CLIMB_UP_AUTO_PAUSE
.\.venv\Scripts\python usb_cmd_tool.py pack CLIMB_DOWN_AUTO_PAUSE
.\.venv\Scripts\python usb_cmd_tool.py pack CLIMB_AUTO_RESUME
```

## 4. 自动调参指令

启动 yaw 自动调参，一般先用 1 轮：

```text
tune_start 1
```

查询调参状态：

```text
tune_status
```

停止调参：

```text
tune_stop
```

也可以直接用底层命令：

```text
send YAW_TUNE_START 1
query YAW_TUNE
send YAW_TUNE_STOP
```

自动调参状态回包是 `CMD=0x49, LEN=64`，工具会解析：

- `state_name`: `IDLE / RUNNING / DONE / FAILED / STOPPED`
- `fail_reason_name`: 失败原因
- `segment_index / segment_count`: 当前段进度
- `pass_index / pass_count`: 当前轮次
- `yaw_error_deg`
- `yaw_error_abs_max_deg`
- `yaw_rate_error_rms_dps`
- `score`
- `pid.angle_kp / pid.angle_kd / pid.rate_kp / pid.rate_ki / pid.rate_kd / pid.pos_kp_yaw`

## 5. 底盘指令示例

```text
usb
enable
send CHS_ENABLE
send CHS_SET_MODE 3
send CHS_SET_VEL 0.2 0 0 0
query CHS
send CHS_STOP
```

`CHS_SET_VEL` 的 4 个参数是：

```text
vx vy yaw_data 0
```

`yaw_data` 是复用字段：`ROBOT_NO_YAW` 下表示机器人系 `target_yaw_robot_deg`，`WORLD_NO_YAW` 下表示世界系 `target_yaw_world_deg`，其它速度模式下表示 `vw_rad_s`。速度控制有 100ms 看门狗，持续运动时需要周期发送速度命令。

## 6. 单条命令发送

不进入交互模式，也可以直接发一条命令并等待回包：

```powershell
.\.venv\Scripts\python usb_cmd_tool.py send --port COM26 YAW_TUNE_GET_STATUS --wait 0.5
```

周期查询自动调参状态：

```powershell
.\.venv\Scripts\python usb_cmd_tool.py poll --port COM26 YAW_TUNE --rate 10
```

监听所有回包：

```powershell
.\.venv\Scripts\python usb_cmd_tool.py monitor --port COM26 --baud 115200
```
