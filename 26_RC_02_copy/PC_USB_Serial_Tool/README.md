# Robot USB Serial Tool

这是一个独立的上位机串口工具工程，只新增在 `PC_USB_Serial_Tool` 文件夹内，不修改现有 STM32 工程。

功能：

- 读取 USB CDC / 串口数据，并按当前固件协议流式解帧。
- 校验并解析下位机状态回包：System、Chassis、Arm、Tool、Robot、Climb、Arm IK result。
- 按命令名或原始 hex 发送数据给下位机。
- 支持交互式 shell，边读边发，适合 VSCode 调试。
- 可生成 USART 遥控帧：`A5 + 40 DATA + checksum + 5A`。

## 快速开始

在 VSCode 中打开本文件夹：

```powershell
cd F:\spareE\Vinci_Robocon_2026\26_RC_Projects\26_RC_02\PC_USB_Serial_Tool
python -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
```

列出串口：

```powershell
.\.venv\Scripts\python -m serial_tool ports
```

进入交互 shell：

```powershell
.\.venv\Scripts\python -m serial_tool shell --port COM7 --baud 115200
```

USB CDC 虚拟串口通常不真正依赖波特率，但 Windows/pyserial 仍然需要传一个值，默认 `115200` 即可。

## 常用 shell 命令

进入 `shell` 后可直接输入：

```text
usb                         # 切换到 USB 控制源
usart                       # 切换到 USART 控制源
enable                      # 系统使能
disable                     # 系统失能
stop                        # 系统停止
status                      # 查询整车综合状态
tune_start 1                # 切到 USB、使能并开始 yaw 自动整定
tune_status                 # 查询 yaw 自动整定状态
tune_stop                   # 停止 yaw 自动整定并查询状态
climb_up_auto               # 切到 USB、使能上台阶并自动运行
climb_step                  # 切到 USB、使能上台阶并步进一步
climb_down_auto             # 切到 USB、使能下台阶并自动运行
climb_downstairs_step       # 切到 USB、使能下台阶并步进一步
climb_up_auto_pause         # 完整积木：跑上台阶 auto，到 v2 中断点暂停
climb_down_auto_pause       # 完整积木：跑下台阶 auto，到 v2 中断点暂停
climb_auto_resume           # 从 v2 中断点继续自动流程直到 DONE
climb_wait 20               # 轮询 CLIMB_GET_STATUS，直到就绪或完成
climb_step_wait 40          # 等当前上台阶动作完成，再步进并等待这一步完成
climb_up_auto_wait 180      # 发送自动上台阶，并等待最终 DONE
climb_downstairs_step_wait 40 # 等当前下台阶动作完成，再步进并等待这一步完成
climb_down_auto_wait 180    # 发送自动下台阶，并等待最终 DONE
climb_wait_then CLIMB_UP_STEP # 等当前上/下台阶动作完成，再发送一条 USB 命令
climb_tests                 # 列出上台阶独立调试动作 ID
climb_test CHASSIS_FORWARD_100      # 单独发送底盘麦轮前进 100mm，不等待完成
climb_test_wait FRONT_UP_10 10      # 发送前两根立杆上升 10mm，并等待完成，超时 10s
flow_start upstairs_v1      # 开始记录一套上台阶调试流程
flow_confirm lift front     # 询问是否保存刚才发送的 climb_test/climb_test_wait 动作
flow_save lift front        # 不询问，直接保存刚才发送的动作
flow_show                   # 显示已记录的流程步骤
flow_export upstairs_v1.json # 导出 JSON，后续用于生成状态机
flow_recover                # 从 .climb_flow_autosave.json 恢复误触 Ctrl+C 前的流程
test s1_up                  # 运行 v2 流程到中断区前并暂停
test_continue skip          # 跳过空白中断动作，继续执行后续流程
test_continue slots         # 执行已填写的中断动作，再继续后续流程
send CHS_SET_MODE 3
send CHS_SET_VEL 0.4 0 0 0
send YAW_TUNE_START 1
query YAW_TUNE
arm_xyz 120 0 450
arm_space X+
arm_up20
arm_down50
arm_zmax
arm_posture S2 1
tool_on S2
tool_off S2
send ARM_IK_TEST_FLOW
query CLIMB_GET_STATUS
raw A5 5A 00 56 C5 82 FF
commands
exit
```

`COM26` can run directly like this:

```powershell
.\.venv\Scripts\python usb_cmd_tool.py shell --port COM26 --baud 115200
```

More USB command examples are in `USB_COMMAND_TOOL.md`.
All mechanism shortcut commands are listed in `ALL_MECHANISM_COMMANDS.md`.

机械臂 XYZ 命令说明：

- `arm_xyz X Y Z`：发送 `ARM_SET_TARGET_XYZ`，`Z` 参与机械臂角度逆解，`X/Y` 只选择 `Y+ / X+ / X-` 运动空间；底盘平面移动需要另发底盘命令。
- `arm_space Y+|X+|X-`：保持当前工具/姿态/高度，只切换运动空间。
- `arm_up/down 5|10|20|50|100|200|500`，或 `arm_up20` / `arm_down50`：当前高度点动。
- `arm_zmin` / `arm_zmax`：移动到当前工具姿态的最小/最大 z 限位。
- `arm_posture TOOL STATE`：只切工具姿态，`TOOL` 可写 `S1`、`S2`、`gripper`。
- `tool_on S2` / `tool_off S2`：通过 PE13 高/低电平控制唯一的吸盘2；吸盘1执行器已删除。

## 机械臂/底盘积木流程保存

下面这些积木 ID 可直接用 `block BLOCK_ID` 执行；创建最终流程时先 `block_flow_start`，之后直接输入积木 ID 和备注，例如 `arm_j2_cw_10deg 调整J2`。每一步执行后都会询问是否保存，只有回答 `y` / `yes` 才会加入最终流程。

J2/J3 关节转动是实际单关节点动：只给目标关节新角度，其他关节保持当前反馈位置，不做整臂 IK 联动。当前点上升/下降会锁定 J3，只通过 J2 改变高度。

```text
# 机械臂实际关节 J2：从 YOZ 平面观察，X+ 指向屏幕外
arm_j2_cw_1deg        # J2 顺时针 1deg
arm_j2_ccw_1deg       # J2 逆时针 1deg
arm_j2_cw_5deg        # J2 顺时针 5deg
arm_j2_ccw_5deg       # J2 逆时针 5deg
arm_j2_cw_10deg       # J2 顺时针 10deg
arm_j2_ccw_10deg      # J2 逆时针 10deg
arm_j2_cw_20deg       # J2 顺时针 20deg
arm_j2_ccw_20deg      # J2 逆时针 20deg
arm_j2_cw_30deg       # J2 顺时针 30deg
arm_j2_ccw_30deg      # J2 逆时针 30deg
arm_j2_cw_60deg       # J2 顺时针 60deg
arm_j2_ccw_60deg      # J2 逆时针 60deg
arm_j2_cw_90deg       # J2 顺时针 90deg
arm_j2_ccw_90deg      # J2 逆时针 90deg

# 机械臂实际关节 J3：从 YOZ 平面观察，X+ 指向屏幕外
arm_j3_cw_1deg        # J3 顺时针 1deg
arm_j3_ccw_1deg       # J3 逆时针 1deg
arm_j3_cw_5deg        # J3 顺时针 5deg
arm_j3_ccw_5deg       # J3 逆时针 5deg
arm_j3_cw_10deg       # J3 顺时针 10deg
arm_j3_ccw_10deg      # J3 逆时针 10deg
arm_j3_cw_20deg       # J3 顺时针 20deg
arm_j3_ccw_20deg      # J3 逆时针 20deg
arm_j3_cw_30deg       # J3 顺时针 30deg
arm_j3_ccw_30deg      # J3 逆时针 30deg
arm_j3_cw_60deg       # J3 顺时针 60deg
arm_j3_ccw_60deg      # J3 逆时针 60deg
arm_j3_cw_90deg       # J3 顺时针 90deg
arm_j3_ccw_90deg      # J3 逆时针 90deg

# 当前点高度点动
arm_up_1mm            # 当前点上升 1mm
arm_down_1mm          # 当前点下降 1mm
arm_up_5mm            # 当前点上升 5mm
arm_down_5mm          # 当前点下降 5mm
arm_up_10mm           # 当前点上升 10mm
arm_down_10mm         # 当前点下降 10mm
arm_up_20mm           # 当前点上升 20mm
arm_down_20mm         # 当前点下降 20mm
arm_up_50mm           # 当前点上升 50mm
arm_down_50mm         # 当前点下降 50mm
arm_up_100mm          # 当前点上升 100mm
arm_down_100mm        # 当前点下降 100mm

# 工具执行器开关
tool_s2_on            # PE13 高电平，吸盘2开启
tool_s2_off           # PE13 低电平，吸盘2关闭
tool_gripper_on       # Gripper 开启
tool_gripper_off      # Gripper 关闭

# 保持当前位置和状态，切换机械臂运动空间
arm_space_y_pos       # 切到 Y+ 空间
arm_space_x_pos       # 切到 X+ 空间
arm_space_x_neg       # 切到 X- 空间

# 底盘机器人坐标系平移
base_forward_10mm     # 底盘前进 10mm
base_back_10mm        # 底盘后退 10mm
base_left_10mm        # 底盘左移 10mm
base_right_10mm       # 底盘右移 10mm
base_forward_50mm     # 底盘前进 50mm
base_back_50mm        # 底盘后退 50mm
base_left_50mm        # 底盘左移 50mm
base_right_50mm       # 底盘右移 50mm
base_forward_100mm    # 底盘前进 100mm
base_back_100mm       # 底盘后退 100mm
base_left_100mm       # 底盘左移 100mm
base_right_100mm      # 底盘右移 100mm

# 底盘机器人坐标系旋转
base_cw_90deg         # 底盘顺时针旋转 90deg
base_ccw_90deg        # 底盘逆时针旋转 90deg

# 自动激光判断门控
climb_up_laser_gate   # 上台阶自动判断阶段：四腿 -30，底盘 0.08m/s 前进，等待 0 <= x_pos < 35
climb_down_laser_gate # 下台阶自动判断阶段：等待高度 h > 65，再自动前进 5mm
climb_up_auto_pause   # 上台阶完整 auto，到中断点暂停
climb_down_auto_pause # 下台阶完整 auto，到中断点暂停
climb_auto_resume     # 中断动作结束后，继续当前 auto 流程直到 DONE

# 状态记录
arm_show_record_status # 记录当前点、关节模型角度、实际电机反馈角度
```

先进入交互 shell，然后按下面方式记录。`block_flow_start` 后直接输入积木 ID 即可执行并生成候选步骤，只有回答 `y` / `yes` 才会加入最终流程；直接回车或回答 `n` 会丢弃这一候选步骤。

```text
blocks
block_flow_start pick_place_v1
arm_j2_cw_10deg 调整J2
base_forward_50mm 底盘前进50
arm_show_record_status 保存当前位置和角度
block_flow_show
block_flow_export pick_place_v1.json
block_flow_export_c pick_place_v1.c g_pick_place_v1
```

底盘机器人坐标系平移积木为 `base_forward/back/left/right_10mm`、`base_forward/back/left/right_50mm`、`base_forward/back/left/right_100mm`。如果确认前中断，可重新进入 shell 后执行 `block_flow_recover`，再用 `block_flow_confirm` 或 `block_flow_discard` 处理未确认候选。

## 六个 v2 中断流程

五个 v2 JSON 流程放在本目录下；`s1_up_s2_up_v2` 已合并进新的 `s1_up_v2` 并删除：

```text
s1_up_v2.json
s1_down_v2.json
s1_up_s2_down_v2.json
s1_down_s2_up_v2.json
s1_down_s2_down_v2.json
```

运行关系：

- `s1_up_v2.json` 和 `s1_down_v2.json` 是两个起点流程。
- `s1_up_s2_down_v2.json` 必须在 `s1_up_v2.json` 已经跑完后再作为 S2 分支运行。
- `s1_down_s2_up_v2.json`、`s1_down_s2_down_v2.json` 必须在 `s1_down_v2.json` 已经跑完后再作为 S2 分支运行。
- 即使 S2 后缀流程里的 climb 判断条件已经满足，没有对应的 S1 前置流程，也不要直接运行 S2 后缀流程。
- 串口工具会在同一个交互会话里记录已完成的 `s1_up_v2` / `s1_down_v2`，未完成对应 S1 时会拒绝启动 S2 后缀流程。

每个 v2 流程第一步都会直接走固件里的 `up/down_auto`，但会在中断点自动暂停：

- 上台阶三份流程第一步是 `climb_up_auto_pause`，底层发送 `SYS_SWITCH_SOURCE USB -> CLIMB_ENABLE -> CLIMB_UP_AUTO_PAUSE`。固件从 `UP_LASER_APPROACH_X_LT_35` 开始，四腿目标设为 `-30mm`，底盘以 `0.08m/s` 前进，直到前方 X 激光满足 `0 <= x_pos < 35`；随后继续原上台阶自动状态机，并在 `STEP_09_ALL_DRIVE_FORWARD_200_PAUSE` 完成后暂停。
- 下台阶三份流程第一步是 `climb_down_auto_pause`，底层发送 `SYS_SWITCH_SOURCE USB -> CLIMB_ENABLE -> CLIMB_DOWN_AUTO_PAUSE`。固件从 `DOWN_LASER_APPROACH_H_GT_65` 开始，高度满足 `h > 65mm` 后自动前进 `5mm`；随后继续原下台阶自动状态机，并在 `DOWN_05_ALL_DRIVE_FORWARD_500` 完成后暂停。
- 中断动作执行完成后，最后一步 `climb_auto_resume` 会发送 `CLIMB_AUTO_RESUME`，从暂停点继续固件自动流程并等待最终 `DONE`。

中断槽位置：

- 六个 v2 文件现在都是 102 步：第 1 步是 `climb_*_auto_pause`，第 2-101 步是 100 个空白中断槽，第 102 步是 `climb_auto_resume`。
- 上台阶三份流程在固件自动上台阶的 200mm 暂停点进入中断槽。
- 下台阶三份流程在固件自动下台阶的 `DOWN_05_ALL_DRIVE_FORWARD_500` 完成后进入中断槽。
- 暂停期间中断槽可以使用 arm 积木，也可以使用原有 climb 测试积木；固件会保留 auto 暂停点，最后 `climb_auto_resume` 仍从原暂停点继续。

交互测试并填写中断动作：

```text
test s1_up 40 1.0
arm_j2_cw_10deg
# 根据提示选择是否保存到下一个空白中断槽
arm_j3_ccw_5deg
# 根据提示选择是否保存
test_status
test_show slots
test_continue slots
test_export s1_up_v2_filled.json
```

`test FLOW` 会运行到中断槽前一个动作并暂停。暂停后可直接输入任意已有积木命令，例如 `arm_j2_cw_10deg`、`arm_up_20mm`、`tool_s2_on`；执行后工具会询问是否写入下一个空白中断槽。`test_continue skip` 会跳过所有空白中断槽直接执行剩余动作；`test_continue slots` 会执行已经填写的中断槽，然后继续执行剩余动作。

也可以不连机器人，直接编辑中断槽：

```powershell
.\.venv\Scripts\python -m serial_tool block-flow-slots s1_up_v2.json
.\.venv\Scripts\python -m serial_tool block-flow-slot-set s1_up_v2.json 1 arm_j2_cw_10deg
.\.venv\Scripts\python -m serial_tool block-flow-slot-clear s1_up_v2.json 1
.\.venv\Scripts\python -m serial_tool block-flow-slot-clear-all s1_up_v2.json
```

完整运行某个 v2 流程：

```powershell
.\.venv\Scripts\python -m serial_tool block-flow-run --port COM30 --baud 115200 s1_up_v2.json --timeout 40 --arm-settle 1.0
.\.venv\Scripts\python -m serial_tool send --port COM30 --baud 115200 FLOW_WEAPON_GRAB --wait 0.2
.\.venv\Scripts\python -m serial_tool poll --port COM30 --baud 115200 FLOW_GET_STATUS --rate 10
.\.venv\Scripts\python -m serial_tool block-flow-run --port COM30 --baud 115200 weapon_grab_v1.json --timeout 20 --arm-settle 1.0
```

### 下位机夹取武器并对接测试状态机

下位机内置 `WEAPON_DOCK_TEST_V1` 状态机。上位机先发送 `FLOW_WEAPON_DOCK_TEST (0x69)` 启动，MCU 执行机械臂和夹爪动作；到达底盘移动与对接完成检查点时会暂停，分别等待上位机发送 `FLOW_CHASSIS_MOVE_DONE (0x6A)` 和 `FLOW_DOCK_DONE (0x6B)` 后继续。状态机只调用 `ARM_ENABLE` 语义，不发送 `SYS_ENABLE`，也不使能 climb 或自动启动底盘运动。

Shell 内运行：

```text
weapon_dock_test          # 只发送启动命令
weapon_dock_status        # 查询 FLOW_GET_STATUS
weapon_chassis_done       # 底盘移动完成，确认 checkpoint 1
weapon_dock_done          # 对接完成，确认 checkpoint 2
weapon_dock_test_wait 180 # 启动并等待 MCU 状态机 DONE，超时 180s
```

直接运行：

```powershell
.\.venv\Scripts\python -m serial_tool send --port COM30 --baud 115200 FLOW_WEAPON_DOCK_TEST --wait 0.2
.\.venv\Scripts\python -m serial_tool send --port COM30 --baud 115200 FLOW_CHASSIS_MOVE_DONE --wait 0.2
.\.venv\Scripts\python -m serial_tool send --port COM30 --baud 115200 FLOW_DOCK_DONE --wait 0.2
.\.venv\Scripts\python -m serial_tool poll --port COM30 --baud 115200 FLOW_GET_STATUS --rate 10
```

MCU 测试顺序为：机械臂使能、夹爪张开、切 X+、执行完整 `WEAPON_GRAB` 轨迹（含 J2 逆时针 5°、J3 逆时针 1°、J3 逆时针 1°优化尾段）、J2 顺时针 1°、J3 逆时针 1°×2、等待上位机确认底盘移动完成、夹爪闭合、切 X-、J3 顺时针 90°、J3 顺时针 5°、J3 逆时针 1°×4、等待上位机确认对接完成、夹爪松开、机械臂重新使能。任一步失败都会进入 `FLOW_GET_STATUS.state=ERROR` 并停止后续动作。

在 shell 内也可以直接运行：

```text
block_flow_run s1_up_v2.json 40 1.0
```

gate 命令也可以单独测试：

```text
climb_up_auto_pause
climb_down_auto_pause
climb_auto_resume
climb_up_laser_gate
climb_down_laser_gate
# 如果已经手动 usb + climb_enable，也可以只发底层命令：
send CLIMB_UP_AUTO_PAUSE
send CLIMB_DOWN_AUTO_PAUSE
send CLIMB_AUTO_RESUME
send CLIMB_UP_GATE
send CLIMB_DOWN_GATE
```

## 上台阶流程积木命令

这些命令不会改原来的完整上台阶状态机，只是单独执行一个 `CLIMB_TEST_ACTION`，方便你调出参数后用 `flow_confirm` 确认保存为流程步骤。
带 `timeout_s` 时会等待 `CLIMB_GET_STATUS.state_done == 1`；不带参数时只发送动作并查询一次状态。

```text
# 立杆：四根
climb_all_legs_220 [timeout_s]        # 四根立杆统一到 220mm
climb_all_legs_300 [timeout_s]        # 四根立杆统一到 300mm
climb_all_legs_zero [timeout_s]       # 四根立杆统一回到 0mm
climb_all_legs_up_10 [timeout_s]      # 四根立杆基于当前位置上升 10mm
climb_all_legs_down_10 [timeout_s]    # 四根立杆基于当前位置下降 10mm

# 立杆：前两根，ID1/ID4
climb_front_zero [timeout_s]          # 前两根立杆回到 0mm
climb_front_220 [timeout_s]           # 前两根立杆目标到 220mm
climb_front_minus_30 [timeout_s]      # 前两根立杆目标到 -30mm
climb_front_up_10 [timeout_s]         # 前两根立杆基于当前位置上升 10mm
climb_front_down_10 [timeout_s]       # 前两根立杆基于当前位置下降 10mm

# 立杆：后两根，ID2/ID3
climb_rear_zero [timeout_s]           # 后两根立杆回到 0mm
climb_rear_220 [timeout_s]            # 后两根立杆目标到 220mm
climb_rear_minus_30 [timeout_s]       # 后两根立杆目标到 -30mm
climb_rear_up_10 [timeout_s]          # 后两根立杆基于当前位置上升 10mm
climb_rear_down_10 [timeout_s]        # 后两根立杆基于当前位置下降 10mm

# 前小驱动轮：FDCAN1 ID5/ID6，前两根立杆下方
climb_front_drive_forward_30 [timeout_s]    # 前小驱动轮前进 30mm
climb_front_drive_forward_10 [timeout_s]    # 前小驱动轮前进 10mm
climb_front_drive_backward_10 [timeout_s]   # 前小驱动轮后退 10mm
climb_front_drive_backward_30 [timeout_s]   # 前小驱动轮后退 30mm
climb_front_drive_forward_500 [timeout_s]   # 前小驱动轮前进 500mm
climb_front_drive_backward_500 [timeout_s]  # 前小驱动轮后退 500mm

# 后小驱动轮：FDCAN2 ID5/ID6，后两根立杆下方
climb_rear_drive_forward_30 [timeout_s]     # 后小驱动轮前进 30mm
climb_rear_drive_forward_10 [timeout_s]     # 后小驱动轮前进 10mm
climb_rear_drive_backward_10 [timeout_s]    # 后小驱动轮后退 10mm
climb_rear_drive_backward_30 [timeout_s]    # 后小驱动轮后退 30mm
climb_rear_drive_forward_500 [timeout_s]    # 后小驱动轮前进 500mm
climb_rear_drive_backward_500 [timeout_s]   # 后小驱动轮后退 500mm

# 整体小驱动轮：FDCAN1/FDCAN2 ID5/ID6，前后四个一起动
climb_all_drive_forward_30 [timeout_s]      # 前后小驱动轮一起前进 30mm
climb_all_drive_forward_10 [timeout_s]      # 前后小驱动轮一起前进 10mm
climb_all_drive_backward_10 [timeout_s]     # 前后小驱动轮一起后退 10mm
climb_all_drive_backward_30 [timeout_s]     # 前后小驱动轮一起后退 30mm
climb_all_drive_forward_500 [timeout_s]     # 前后小驱动轮一起前进 500mm
climb_all_drive_backward_500 [timeout_s]    # 前后小驱动轮一起后退 500mm

# 底盘麦轮
climb_chassis_forward_100 [timeout_s]       # 底盘麦轮前进 100mm
climb_chassis_backward_100 [timeout_s]      # 底盘麦轮后退 100mm
climb_chassis_forward_50 [timeout_s]        # 底盘麦轮前进 50mm
climb_chassis_backward_50 [timeout_s]       # 底盘麦轮后退 50mm
climb_chassis_forward_300 [timeout_s]       # 底盘麦轮前进 300mm
climb_chassis_backward_300 [timeout_s]      # 底盘麦轮后退 300mm
```

也可以用动作名或编号：

```text
climb_test_wait ALL_LEGS_220 10
climb_test_wait 16 10
```

调试并记录流程的典型用法：

```text
flow_start upstairs_v1
climb_all_legs_220 10
flow_confirm 四根立柱到 220
climb_rear_drive_forward_30 10
flow_confirm 后小驱动轮前进 30
flow_show
flow_export upstairs_v1.json
```

每次 `climb_test*` 生成候选动作、`flow_confirm`/`flow_save` 保存动作、`flow_add` 手动补动作后，工具都会自动写入 `.climb_flow_autosave.json`。如果误触 `Ctrl+C` 退出，重新进入 shell 后执行：

```text
flow_recover
flow_show
flow_export downstairs_v1.json
```

收到下位机回包后，工具会打印 JSON 形式的解析结果和原始帧 hex。

## 命令行发送

只打包不发送：

```powershell
.\.venv\Scripts\python -m serial_tool pack SYS_ENABLE
.\.venv\Scripts\python -m serial_tool pack CHS_SET_VEL 0.4 0 0 0
```

发送单条命令并等待 0.5 秒回包：

```powershell
.\.venv\Scripts\python -m serial_tool send --port COM7 ROBOT_GET_STATUS --wait 0.5
```

持续监听：

```powershell
.\.venv\Scripts\python -m serial_tool monitor --port COM7
```

周期轮询状态：

```powershell
.\.venv\Scripts\python -m serial_tool poll --port COM7 ROBOT_GET_STATUS --rate 20
```

生成 USART 遥控帧：

```powershell
.\.venv\Scripts\python -m serial_tool remote-pack --mode 3 --source-usb --chassis 0.2 0 0
```

USART 遥控帧 `byte10..12` 当前保留；新机械臂工具目标通过 USB `ARM_SET_TARGET_XYZ`、`ARM_SET_TARGET` 或 `TOOL_SET_MODE` 设置。上台阶字段位于 `byte13..15`：`climb_enable / climb_step / climb_auto`。步进和自动都按 `0->1` 上升沿触发；自动启动后仍要持续发送 `--source-usart --climb-enable` 保活帧，否则固件 `300ms` USART 看门狗会停止 USART 侧控制器。

## 协议来源

本工具按仓库内当前实现对齐：

- USB 帧格式：`BSP/Src/bsp_usb.c`
- USB 命令路由：`Components/Algorithm/Src/Data_Analysis.c`
- 状态回包：`Applications/Task/Src/PC_TX_Task.c`
- USART 遥控帧：`Components/Algorithm/Src/CRC.c`

特别注意：当前固件的底盘状态回包 `CHS_GET_STATUS` 实际为 `LEN=88`，工具按源码中的真实长度解析。
