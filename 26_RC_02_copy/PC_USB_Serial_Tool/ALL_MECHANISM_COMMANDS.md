# 全机构 USB 指令清单

运行交互工具：

```powershell
cd F:\spareE\Vinci_Robocon_2026\26_RC_Projects\26_RC_02\PC_USB_Serial_Tool # 进入 USB 串口工具目录
.\.venv\Scripts\python usb_cmd_tool.py shell --port COM26 --baud 115200     # 打开 COM26 交互式串口命令行
```

进入后建议先执行：

```text
usb             # 切换到 USB 控制源
enable          # 使能 USB 控制的各机构
status          # 查询整车综合状态
```

## 系统

```text
usb             # 切换到 USB 控制源
usart           # 切回 USART 控制源
enable          # 系统使能，等价 SYS_ENABLE
disable         # 系统失能，等价 SYS_DISABLE
stop            # 系统停止，等价 SYS_STOP
status          # 查询整车综合状态，等价 ROBOT_GET_STATUS
query SYS       # 查询系统状态，等价 SYS_GET_STATUS
```

## 底盘

```text
chs_enable      # 底盘使能
chs_disable     # 底盘失能
chs_mode 3      # 设置底盘模式，3=WORLD_VEL
vel 0.2 0 0 0   # 发送速度指令 vx=0.2, vy=0, vw=0, 锁 yaw=0
pos 0.5 0 0     # 发送位置指令 dx=0.5, dy=0, dyaw=0
chs_stop        # 底盘停止
chs_status      # 查询底盘状态
```

底盘模式：

```text
0 ROBOT_NO_YAW_VEL   # 机器人坐标系速度模式，不给 yaw 速度
1 ROBOT_VEL          # 机器人坐标系速度模式，带 yaw 速度
2 WORLD_NO_YAW_VEL   # 世界坐标系速度模式，不给 yaw 速度
3 WORLD_VEL          # 世界坐标系速度模式，带 yaw 速度
4 ROBOT_NO_YAW_POS   # 机器人坐标系位置模式，不给 yaw 目标
5 ROBOT_POS          # 机器人坐标系位置模式，带 yaw 目标
6 WORLD_NO_YAW_POS   # 世界坐标系位置模式，不给 yaw 目标
7 WORLD_POS          # 世界坐标系位置模式，带 yaw 目标
```

底盘速度命令：

```text
vel VX VY YAW_DATA           # ROBOT_NO_YAW: robot yaw deg；WORLD_NO_YAW: world yaw deg；其它速度模式: vw_rad_s
```

底盘位置命令：

```text
pos DX DY YAW_DATA          # ROBOT_NO_YAW: robot yaw deg；WORLD_NO_YAW: world yaw deg；其它位置模式: dyaw_rad
```

## 机械臂

```text
arm_enable          # 机械臂使能
arm_disable         # 机械臂失能
arm_space Y+        # 切到 Y+ 工作空间，保持当前工具/姿态/高度
arm_space X+        # 切到 X+ 工作空间
arm_space X-        # 切到 X- 工作空间
arm_xyz 200 0 180   # 发送三维目标；Z 逆解，X/Y 只选择工作空间
arm_up 20           # 当前高度升高 20mm，步长支持 5/10/20/50/100/200/500
arm_down 50         # 当前高度下降 50mm
arm_up20            # 等价 arm_up 20
arm_down50          # 等价 arm_down 50
arm_zmin            # 移动到当前工具姿态最小高度
arm_zmax            # 移动到当前工具姿态最大高度
arm_posture S2 1    # 切换工具姿态，保持当前 x/y/z
send ARM_IK_TEST_FLOW # 启动机械臂逆解测试流程，默认 8000ms/step
send ARM_IK_TEST_FLOW 1 8000 # 启动测试流程，指定 8000ms/step
send ARM_IK_TEST_FLOW 0 0 # 停止测试流程
arm_status          # 查询机械臂状态
arm_stop            # 机械臂停止并保持
```

机械臂目标点单位是 mm：

```text
arm_xyz X Y Z       # X/Y/Z 单位 mm；X/Y 只选 Y+/X+/X-，平面距离由底盘实现
```

工具姿态 state 按工具分别解释：

```text
TOOL: 0=S1, 1=S2, 2=gripper
S1: 0=准备吸取, 1=已吸取且姿态跟随最近 S2, 2=放置
S2: 0=准备吸取15deg, 1=竖直向下吸取, 2=短端同向平行, 3=Y+模板等效放置
gripper: 0=朝上, 1=朝下, 2=朝前/上电姿态
```

## 工具机构

```text
tool_on S2          # PE13 高电平，打开 S2 吸盘
tool_off S2         # PE13 低电平，关闭 S2 吸盘
tool_on gripper     # 夹爪舵机开
tool_off gripper    # 夹爪舵机关
arm_posture S1 0    # S1 准备吸取姿态
arm_posture S1 1    # S1 已吸取物块姿态，跟随最近 S2
arm_posture S1 2    # S1 放置姿态
arm_posture S2 0    # S2 准备吸取 15deg
arm_posture S2 1    # S2 竖直向下吸取
arm_posture S2 2    # S2 与曲柄短端同向平行
arm_posture S2 3    # S2 放置，S2Z+ // 当前工作空间等效 Y+
arm_posture gripper 0 # 夹爪朝上
arm_posture gripper 1 # 夹爪朝下
arm_posture gripper 2 # 夹爪朝前/上电状态
tool_status         # 查询工具姿态/机械臂状态
tool_stop           # 工具/机械臂停止并保持
```

兼容旧高度+yaw 目标仍可使用：

```text
tool_target TOOL STATE Z_MM YAW_RAD
send ARM_SET_TARGET 0 0 450 0  # S1 state0, target_z=450mm, yaw=0
send TOOL_SET_MODE 2 1 450 0   # gripper state1, target_z=450mm, yaw=0
```

## 上/下台阶机构

```text
climb_enable                    # 上/下台阶机构使能
climb_disable                   # 上/下台阶机构失能
climb_stop                      # 停止上/下台阶机构
climb_status                    # 查询上/下台阶状态，返回 flow=UPSTAIRS/DOWNSTAIRS
climb_tests                     # 列出所有 climb 测试动作编号

# 上台阶状态机
climb_step                      # 手动推进上台阶状态机一步
climb_step_wait 40              # 等当前动作完成，发送上台阶 step，再等待本步完成，超时 40s
climb_up_auto                   # 启动/继续上台阶自动流程
climb_up_auto_wait 180          # 启动上台阶自动流程并等待最终 DONE，超时 180s
climb_upstairs_step             # 上台阶 step 的明确别名
climb_upstairs_step_wait 40     # 上台阶等待后步进
climb_upstairs_auto             # 上台阶 auto 的明确别名
climb_upstairs_auto_wait 180    # 上台阶自动并等待 DONE

# 下台阶状态机
climb_down_step                 # 手动推进下台阶状态机一步
climb_down_step_wait 40         # 等当前动作完成，发送下台阶 step，再等待本步完成，超时 40s
climb_down_auto                 # 启动/继续下台阶自动流程
climb_down_auto_wait 180        # 启动下台阶自动流程并等待最终 DONE，超时 180s
climb_downstairs_step           # 手动推进下台阶状态机一步
climb_downstairs_step_wait 40   # 等当前动作完成，发送下台阶 step，再等待本步完成，超时 40s
climb_downstairs_auto           # 下台阶 auto 的明确别名
climb_downstairs_auto_wait 180  # 下台阶自动并等待 DONE
# Auto laser gate: upstairs auto starts forward immediately; firmware drives forward until valid X reaches 0 <= x < 35mm. Transient X=-1 is tolerated; continuous invalid X for 1000ms times out.
# Auto laser gate: downstairs auto starts forward immediately; h > 65mm detects the 50-53mm to 160mm+ jump, then chassis moves forward another 5mm before the normal downstairs flow. H=-1 is tolerated briefly; 1000ms invalid or 8000ms without trigger times out.

# 等待/转发
climb_wait 40                   # 仅等待当前 climb 动作完成，超时 40s
climb_wait_then CLIMB_UP_STEP   # 等当前动作完成后发送指定 USB 命令
climb_wait_then CLIMB_DOWN_STEP # 等当前动作完成后发送指定 USB 命令

# 原始测试动作入口
climb_test ACTION               # 发送指定测试动作，不等待完成
climb_test_wait ACTION [timeout_s]  # 发送指定测试动作并等待完成

# 立杆：四根
climb_all_legs_220 [timeout_s]       # 四根立杆目标到 220mm；带超时时间则等待完成
climb_all_legs_300 [timeout_s]       # 四根立杆目标到 300mm；带超时时间则等待完成
climb_all_legs_zero [timeout_s]      # 四根立杆统一回到 0mm；带超时时间则等待完成
climb_all_legs_up_10 [timeout_s]     # 四根立杆当前位置上升 10mm；带超时时间则等待完成
climb_all_legs_down_10 [timeout_s]   # 四根立杆当前位置下降 10mm；带超时时间则等待完成

# 立杆：前两根，ID1/ID4
climb_front_zero [timeout_s]     # 前两根立杆回到 0mm；带超时时间则等待完成
climb_front_220 [timeout_s]      # 前两根立杆目标到 220mm；带超时时间则等待完成
climb_front_minus_30 [timeout_s] # 前两根立杆目标到 -30mm；带超时时间则等待完成
climb_front_up_10 [timeout_s]   # 前两根立杆上升 10mm；带超时时间则等待完成
climb_front_down_10 [timeout_s] # 前两根立杆下降 10mm；带超时时间则等待完成

# 立杆：后两根，ID2/ID3
climb_rear_zero [timeout_s]     # 后两根立杆回到 0mm；带超时时间则等待完成
climb_rear_220 [timeout_s]      # 后两根立杆目标到 220mm；带超时时间则等待完成
climb_rear_minus_30 [timeout_s] # 后两根立杆目标到 -30mm；带超时时间则等待完成
climb_rear_up_10 [timeout_s]    # 后两根立杆上升 10mm；带超时时间则等待完成
climb_rear_down_10 [timeout_s]  # 后两根立杆下降 10mm；带超时时间则等待完成

# 前小驱动轮：FDCAN1 ID5/ID6，前两根立杆下方
climb_front_drive_forward_30 [timeout_s]    # 前小驱动轮前进 30mm；带超时时间则等待完成
climb_front_drive_forward_10 [timeout_s]    # 前小驱动轮前进 10mm；带超时时间则等待完成
climb_front_drive_backward_10 [timeout_s]   # 前小驱动轮后退 10mm；带超时时间则等待完成
climb_front_drive_backward_30 [timeout_s]   # 前小驱动轮后退 30mm；带超时时间则等待完成
climb_front_drive_forward_500 [timeout_s]   # 前小驱动轮前进 500mm；带超时时间则等待完成
climb_front_drive_backward_500 [timeout_s]  # 前小驱动轮后退 500mm；带超时时间则等待完成

# 后小驱动轮：FDCAN2 ID5/ID6，后两根立杆下方
climb_rear_drive_forward_30 [timeout_s]     # 后小驱动轮前进 30mm；带超时时间则等待完成
climb_rear_drive_forward_10 [timeout_s]     # 后小驱动轮前进 10mm；带超时时间则等待完成
climb_rear_drive_backward_10 [timeout_s]    # 后小驱动轮后退 10mm；带超时时间则等待完成
climb_rear_drive_backward_30 [timeout_s]    # 后小驱动轮后退 30mm；带超时时间则等待完成
climb_rear_drive_forward_500 [timeout_s]    # 后小驱动轮前进 500mm；带超时时间则等待完成
climb_rear_drive_backward_500 [timeout_s]   # 后小驱动轮后退 500mm；带超时时间则等待完成

# 整体小驱动轮：FDCAN1/FDCAN2 ID5/ID6，前后四个一起动
climb_all_drive_forward_30 [timeout_s]      # 前后小驱动轮一起前进 30mm；带超时时间则等待完成
climb_all_drive_forward_10 [timeout_s]      # 前后小驱动轮一起前进 10mm；带超时时间则等待完成
climb_all_drive_backward_10 [timeout_s]     # 前后小驱动轮一起后退 10mm；带超时时间则等待完成
climb_all_drive_backward_30 [timeout_s]     # 前后小驱动轮一起后退 30mm；带超时时间则等待完成
climb_all_drive_forward_500 [timeout_s]     # 前后小驱动轮一起前进 500mm；带超时时间则等待完成
climb_all_drive_backward_500 [timeout_s]    # 前后小驱动轮一起后退 500mm；带超时时间则等待完成

# 底盘麦轮
climb_chassis_forward_100 [timeout_s]       # 底盘前进 100mm；带超时时间则等待完成
climb_chassis_backward_100 [timeout_s]      # 底盘后退 100mm；带超时时间则等待完成
climb_chassis_forward_50 [timeout_s]        # 底盘前进 50mm；带超时时间则等待完成
climb_chassis_backward_50 [timeout_s]       # 底盘后退 50mm；带超时时间则等待完成
climb_chassis_forward_300 [timeout_s]       # 底盘前进 300mm；带超时时间则等待完成
climb_chassis_backward_300 [timeout_s]      # 底盘后退 300mm；带超时时间则等待完成
```

模拟旧 USART 三个控制字节：

```text
climb_ctrl ENABLE STEP AUTO     # 模拟旧 USART 的 enable/step/auto 三个控制位
```

例如：

```text
climb_ctrl 1 0 0                # 使能 climb，不触发 step/auto
climb_ctrl 1 1 0                # 使能 climb，并触发一次 step
climb_ctrl 1 0 1                # 使能 climb，并触发/继续 auto
```

## 任务流程 / 武器夹取对接

```text
send FLOW_WEAPON_GRAB       # 执行完整 weapon_grab_v1：含 J2 ccw5、J3 ccw1、J3 ccw1 优化尾段
weapon_dock_test            # 启动 WEAPON_DOCK_TEST_V1
weapon_dock_status          # 查询 FLOW_GET_STATUS
weapon_chassis_done         # 底盘移动完成，确认 checkpoint 1
weapon_dock_done            # 对接完成，确认 checkpoint 2

send FLOW_WEAPON_DOCK_TEST
send FLOW_CHASSIS_MOVE_DONE
send FLOW_DOCK_DONE
```

`FLOW_WEAPON_DOCK_TEST` 到达底盘移动检查点和对接完成检查点时会保持 `WAIT/HOST_CHECKPOINT_WAIT`，上位机发送对应确认命令后继续；底盘 `CHS_SET_POS/VEL` 命令不会重置该任务流。

## 自动调参

```text
tune_start 1        # 启动 yaw 自动调参，执行 1 轮
tune_status         # 查询 yaw 自动调参状态
tune_stop           # 停止 yaw 自动调参
```

底层命令等价写法：

```text
send YAW_TUNE_START 1   # 原生命令：启动 yaw 自动调参
query YAW_TUNE          # 原生命令：查询 yaw 自动调参状态
send YAW_TUNE_STOP      # 原生命令：停止 yaw 自动调参
```

## 原生命令名

如果不想用快捷命令，可以直接用 `send`：

```text
send SYS_ENABLE             # 系统使能
send CHS_SET_MODE 3         # 设置底盘模式为 WORLD_VEL
send CHS_SET_VEL 0.2 0 0 0  # 发送底盘速度指令
send ARM_SET_TARGET_XYZ 120 0 450 0 # 三维目标；Z 逆解，X/Y 只选空间
send ARM_SET_POSTURE 2 1    # gripper state1，夹爪朝下
send CLIMB_UP_STEP          # 上台阶手动推进一步
send CLIMB_UP_AUTO          # 上台阶自动执行/继续
send CLIMB_DOWN_STEP        # 下台阶手动推进一步
send CLIMB_DOWN_AUTO        # 下台阶自动执行/继续
send CLIMB_TEST_ACTION 10   # 执行 climb 测试动作 10：底盘前进 100mm
send FLOW_WEAPON_GRAB       # 执行完整武器夹取轨迹
send FLOW_WEAPON_DOCK_TEST  # 启动武器夹取+对接状态机
send FLOW_CHASSIS_MOVE_DONE # 确认底盘移动完成，继续 dock 流程
send FLOW_DOCK_DONE         # 确认对接完成，继续 dock 流程
send YAW_TUNE_START 1       # 启动 yaw 自动调参
send YAW_TUNE_GET_STATUS    # 查询 yaw 自动调参状态
send YAW_TUNE_STOP          # 停止 yaw 自动调参
```

查看完整底层 USB 命令号和打包结果：

```text
commands                    # 查看工具内置的完整 USB 命令表
pack YAW_TUNE_GET_STATUS    # 打印该命令的完整 USB 帧
pack CLIMB_DOWN_STEP        # 打印下台阶 step 的完整 USB 帧
```
