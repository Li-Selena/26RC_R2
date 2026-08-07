# Robot USB / USART 控制命令与使用方法

更新时间：2026-07-07

本文档对应当前工程实现，主要代码位置：

- USB 收发与 CRC：`BSP/Src/bsp_usb.c`
- USB 命令解析：`Components/Algorithm/Src/Data_Analysis.c`
- USB 状态回包：`Applications/Task/Src/PC_TX_Task.c`
- USART 遥控帧解析：`Components/Algorithm/Src/CRC.c`
- 上/下台阶状态机：`Applications/R2_user/Src/R2_climb.c`
- 上/下台阶参数与 debug 结构：`Applications/R2_user/Inc/R2_climb.h`

## 1. USB 帧格式

所有 USB 控制命令和状态回包都使用同一帧格式：

| 字段 | 长度 | 说明 |
|---|---:|---|
| HEAD1 | 1 | 固定 `0xA5` |
| HEAD2 | 1 | 固定 `0x5A` |
| LEN | 1 | DATA 字节数 |
| CMD | 1 | 命令字 |
| DATA | LEN | 数据区 |
| CRC_H | 1 | CRC16 高字节 |
| CRC_L | 1 | CRC16 低字节 |
| TAIL | 1 | 固定 `0xFF` |

CRC 规则：

- 算法：CRC16/Modbus，初值 `0xFFFF`，多项式 `0xA001`。
- 计算范围：`HEAD1 HEAD2 LEN CMD DATA`。
- 不包含：`CRC_H CRC_L TAIL`。
- 发送顺序：先发 CRC 高字节，再发 CRC 低字节。

数据规则：

- 带参数命令统一使用 `LEN=0x10`，DATA 为 4 个 `float32 little-endian`：`f0 f1 f2 f3`。
- 简单命令、停止命令、状态查询命令推荐使用 `LEN=0x00`。
- 简单命令也兼容 `LEN=0x10`，但没有必要。
- 状态回包使用同一帧格式，回包 `CMD` 与查询命令相同。
- 状态查询现在按请求顺序 FIFO 回包，`PC_TX_Task` 每 5ms 最多发送一个状态包，队列深度 16。

Python 组帧示例：

```python
import struct

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc

def pack_cmd(cmd: int, payload: bytes = b"") -> bytes:
    body = bytes([0xA5, 0x5A, len(payload), cmd]) + payload
    crc = crc16_modbus(body)
    return body + bytes([(crc >> 8) & 0xFF, crc & 0xFF, 0xFF])

def pack_empty(cmd: int) -> bytes:
    return pack_cmd(cmd)

def pack_4float(cmd: int, f0=0.0, f1=0.0, f2=0.0, f3=0.0) -> bytes:
    return pack_cmd(cmd, struct.pack("<4f", f0, f1, f2, f3))
```

## 2. USB 可用命令总表

### 2.1 System

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x00` | `SYS_DISABLE` | 空或 4float | 停止 yaw 调参、USB 底盘、上/下台阶；机械臂输出失能并回上电零位软件目标；清除底盘使能标志 |
| `0x01` | `SYS_ENABLE` | 空或 4float | 打开 USB 底盘使能标志并使能机械臂；机械臂 J1/J2 回固定准备姿态，J3 抱死上电电机零位；不会自动打开吸盘/夹爪或台阶状态机 |
| `0x02` | `SYS_SWITCH_SOURCE` | `f0=0/1` | 切换控制源，`0=USART`，`1=USB` |
| `0x05` | `SYS_STOP` | 空或 4float | 停止 yaw 调参、USB 底盘和上/下台阶；机械臂保持当前位置并停止输出 |
| `0x06` | `SYS_GET_STATUS` | 空或 4float | 查询系统状态 |

常用帧：

```text
SYS_DISABLE           A5 5A 00 00 FB 02 FF
SYS_ENABLE            A5 5A 00 01 3B C3 FF
SYS_STOP              A5 5A 00 05 F8 C2 FF
SYS_GET_STATUS        A5 5A 00 06 F9 82 FF
SYS_SWITCH_SOURCE USB A5 5A 10 02 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 38 4B FF
SYS_SWITCH_SOURCE USART
                      A5 5A 10 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 87 9F FF
```

使用方法：

1. 如果使用 USB 控制，先发 `SYS_SWITCH_SOURCE USB`。
2. 如需同时打开 USB 底盘和机械臂，发 `SYS_ENABLE`；工具执行器仍需用带 `f0=tool` 的 `TOOL_ENABLE` 单独打开。
3. 出现异常或要中断动作，发 `SYS_STOP` 或对应模块 `STOP`。
4. 若要回到串口遥控，发 `SYS_SWITCH_SOURCE USART`；固件会停止 yaw 调参、USB 底盘、上/下台阶并失能机械臂输出。

### 2.2 Chassis 底盘

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x10` | `CHS_DISABLE` | 空或 4float | 底盘失能并停止 USB 底盘控制器 |
| `0x11` | `CHS_ENABLE` | 空或 4float | 底盘使能 |
| `0x12` | `CHS_SET_MODE` | `f0=mode` | 设置底盘模式 `0..7` |
| `0x13` | `CHS_SET_VEL` | `f0=vx, f1=vy, f2=yaw_data, f3=0` | 设置速度目标 |
| `0x14` | `CHS_SET_POS` | `f0=dx, f1=dy, f2=yaw_data, f3=0` | 设置一次位置位移目标 |
| `0x15` | `CHS_STOP` | 空或 4float | 停止 USB 底盘控制器 |
| `0x16` | `CHS_GET_STATUS` | 空或 4float | 查询底盘状态 |

模式表：

| mode | 名称 | 坐标系 | 控制方式 | yaw |
|---:|---|---|---|---|
| `0` | `ROBOT_NO_YAW_VEL` | 机器人系 | 速度 | 锁 yaw |
| `1` | `ROBOT_VEL` | 机器人系 | 速度 | 可转向 |
| `2` | `WORLD_NO_YAW_VEL` | 世界系 | 速度 | 锁 yaw |
| `3` | `WORLD_VEL` | 世界系 | 速度 | 可转向 |
| `4` | `ROBOT_NO_YAW_POS` | 机器人系 | 位置 | 锁 yaw |
| `5` | `ROBOT_POS` | 机器人系 | 位置 | 可转向 |
| `6` | `WORLD_NO_YAW_POS` | 世界系 | 位置 | 锁 yaw |
| `7` | `WORLD_POS` | 世界系 | 位置 | 可转向 |

`yaw_data` 复用规则：

| 命令 | `ROBOT_NO_YAW` 模式 `0/4` | `WORLD_NO_YAW` 模式 `2/6` | 非 `NO_YAW` 模式 `1/3/5/7` |
|---|---|---|---|
| `CHS_SET_VEL` | `f2=target_yaw_robot_deg`，机器人系锁定角，`f3` 保留 | `f2=target_yaw_world_deg`，世界系锁定角，`f3` 保留 | `f2=vw_rad_s`，目标角速度，`f3` 保留 |
| `CHS_SET_POS` | `f2=target_yaw_robot_deg`，机器人系锁定角，`f3` 保留 | `f2=target_yaw_world_deg`，世界系锁定角，`f3` 保留 | `f2=dyaw_rad`，目标相对旋转量，`f3` 保留 |

注意：`NO_YAW` 不表示底盘内部 `vw` 永远为 0，而是上位机不再发送旋转速度/旋转位移；固件用当前模式坐标系下的 `target_yaw_*_deg` 和 IMU yaw 做闭环，抑制横移启停造成的累计偏航。

生效条件与限幅：

- `CHS_SET_MODE`、`CHS_SET_VEL`、`CHS_SET_POS` 只有当前控制源为 USB 且底盘使能标志 `Mecanum_control_flag=1` 时才会写入 USB 底盘控制器。
- `CHS_SET_VEL` 会限幅：平移速度矢量限制到 `2.0 m/s`（`sqrt(vx²+vy²) <= 2.0`），非 `NO_YAW` 模式下 `vw` 限制到约 `[-0.628, +0.628] rad/s`。
- `CHS_SET_POS` 当前不在 USB 解析层限幅，位置目标是否能执行由 `R2_Move_SetDist()` 和底盘状态机决定。

常用帧：

```text
CHS_DISABLE                 A5 5A 00 10 37 03 FF
CHS_ENABLE                  A5 5A 00 11 F7 C2 FF
CHS_STOP                    A5 5A 00 15 34 C3 FF
CHS_GET_STATUS              A5 5A 00 16 35 83 FF
CHS_SET_MODE WORLD_VEL=3    A5 5A 10 12 00 00 40 40 00 00 00 00 00 00 00 00 00 00 00 00 02 2D FF
CHS_SET_MODE WORLD_POS=7    A5 5A 10 12 00 00 E0 40 00 00 00 00 00 00 00 00 00 00 00 00 A2 8D FF
CHS_SET_VEL vx=0.4          A5 5A 10 13 CD CC CC 3E 00 00 00 00 00 00 00 00 00 00 00 00 F0 00 FF
CHS_SET_POS dx=1m           A5 5A 10 14 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 5C A5 FF
```

使用方法：

1. 切到 USB：`SYS_SWITCH_SOURCE USB`。
2. 打开底盘：`CHS_ENABLE`，或直接 `SYS_ENABLE`。
3. 速度控制：先 `CHS_SET_MODE` 到 `0..3`，再周期发送 `CHS_SET_VEL`；若是 `NO_YAW` 模式，每帧 `f2` 填当前模式坐标系下的期望锁定 yaw 角。
4. 位置控制：先 `CHS_SET_MODE` 到 `4..7`，再发送一次 `CHS_SET_POS`；若是 `NO_YAW` 模式，本帧 `f2` 填当前模式坐标系下的期望锁定 yaw 角。
5. 速度控制有 `100ms` 看门狗，`CHS_SET_VEL` 发送间隔应小于 `100ms`。
6. 位置运动执行中不要重复发送新位置命令，先查询 `pos_state`，或发 `CHS_STOP` 后再下新目标。

### 2.3 Arm 机械臂

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x20` | `ARM_DISABLE` | 空或 4float | 输出失能，软件目标回上电零位姿态 |
| `0x21` | `ARM_ENABLE` | 空或 4float | 使能机械臂；J1/J2 回固定准备姿态，J3 抱死上电电机零位 |
| `0x22` | `ARM_SET_WORKSPACE` | `f0=direction` | 只切换运动空间，`0=Y+`, `1=X+`, `2=X-`，保持当前工具/姿态/高度 |
| `0x23` | `ARM_SET_TARGET` | `f0=tool, f1=state, f2=target_z_mm, f3=approach_yaw_rad` | 兼容旧高度+yaw 目标，设置工具高度与姿态 |
| `0x24` | `ARM_SET_TARGET_XYZ` | `f0=x_mm, f1=y_mm, f2=z_mm, f3=0` | 三维坐标逆解；`z` 参与角度逆解，`x/y` 只参与运动空间选择 |
| `0x25` | `ARM_STOP` | 空或 4float | 保持当前编码器位置 |
| `0x26` | `ARM_GET_STATUS` | 空或 4float | 查询机械臂状态 |
| `0x27` | `ARM_IK_TEST_FLOW` | 空或 `f0=start/stop, f1=step_ms` | USB 触发逆解测试流程 |
| `0x28` | `ARM_HEIGHT_JOG` | `f0=delta_z_mm` | 锁定 J3 的高度点动；只通过 J2 升高/下降，正数上升、负数下降 |
| `0x29` | `ARM_HEIGHT_LIMIT` | `f0=0/1` | 移动到当前工具姿态 z 最小/最大高度，`0=min`, `1=max` |
| `0x2A` | `ARM_SET_POSTURE` | `f0=tool, f1=state` | 只切换工具姿态，保持当前 `x/y/z` 目标 |
| `0x2B` | `ARM_JOINT_JOG` | `f0=joint(2/3), f1=signed_delta_deg` | J2/J3 独立实际关节点动；目标关节按增量转动，其他关节保持当前反馈位置；从 YOZ 平面看且 X+ 指向屏幕外时，正数=逆时针，负数=顺时针 |

限制：

- `tool: 0=S1, 1=S2, 2=gripper`；`state` 按工具分别解释，不再跨工具共用含义。

| tool | state | 姿态/语义 |
|---|---:|---|
| `S1` | `0` | 准备吸取方块，`S1Z+ // Z0-` |
| `S1` | `1` | 已吸取物块，S1 偏置不变，姿态由最近一次成功的 S2 姿态决定 |
| `S1` | `2` | 放置方块，`S1Z+ // Z0-`，与 state0 姿态相同但流程语义不同 |
| `S2` | `0` | 准备吸取，吸盘平面与水平方块平面夹角 `15deg`，向 `Y0+` 倾斜 |
| `S2` | `1` | 吸取物块中，`S2Z+ // Z0-` |
| `S2` | `2` | `S2Z+` 与折弯点指向 J4 的曲柄短端同向平行，长端转动时持续保持平行 |
| `S2` | `3` | 放置方块，`S2Z+ // Y0+` |
| `gripper` | `0` | 夹爪朝上，`GZ+ // Y0+` |
| `gripper` | `1` | 夹爪朝下，`GZ+ // Y0-` |
| `gripper` | `2` | 夹爪朝前/上电状态，`GZ+ // Z0-` |

- 表中的 `Y0+ / Y0-` 是 `Y+` 工作空间模板定义；切到 `X+ / X-` 时，固件按当前 J1 yaw 等效旋转到本工作空间方向：模板 `Y+` 在 `Y+` 空间为 `Y0+`，在 `X+` 空间为 `X0+`，在 `X-` 空间为 `X0-`。`Z0+ / Z0-` 姿态仍保持世界竖直定义。
- 未显式调用工具姿态时，坐标系4保持上电朝向 `phi=pi`，J3 自动补偿 `theta3=pi-theta2`，并选择靠近当前 J3 的连续角度解，避免姿态补偿绕到反向等效角。
- `ARM_SET_TARGET_XYZ` 的 `z_mm` 参与机械臂角度逆解；`x_mm/y_mm` 只用于选择 `Y+ / X+ / X-` 运动空间，不要求机械臂在平面内到达该坐标。因机械结构导致不可达的平面坐标由底盘实现，机械臂逆解不实现底盘移动。
- `ARM_HEIGHT_JOG` 推荐上位机按钮步长：`+/-1`, `+/-5`, `+/-10`, `+/-20`, `+/-50`, `+/-100` mm。
- `ARM_JOINT_JOG` 推荐上位机按钮步长：`+/-1`, `+/-5`, `+/-10`, `+/-20`, `+/-30`, `+/-60`, `+/-90` deg；从 YOZ 平面看且 X+ 指向屏幕外时，`+` 为逆时针，`-` 为顺时针。
- `ARM_HEIGHT_LIMIT` 按当前工具姿态查询 z 限位；S1 的 `state=1` 会按最近一次成功的 S2 姿态决定限位。
- 底盘负责平面 `x/y`；机械臂只控制工具高度、工具使用/收纳姿态和 `approach_yaw_rad`。
- 机械臂三个关节电机均使用位置速度模式，目标速度由固件固定下发：J1=`2.0 rad/s`，J2=`2.5 rad/s`，J3=`3.0 rad/s`；上位机不需要也不能在 `ARM_SET_TARGET/TOOL_SET_MODE` 中下发速度。
- 当前 IK 基于修正零位 MDH、工具偏置和工具使用/收纳姿态；限位仅使用新模型配置表。
- 当前固件只使用新机械臂修正 MDH 模型，不再使用历史末端目标模型作为约束来源。
- `approach_yaw_rad` 作为 legacy J1 yaw hint，固件会吸附到 `Y+ / X+ / X-` 三个工作方向；当前固件已做水平坐标系反向修正，使上位机/状态包中的 `X+ / Y+` 与实际规定坐标一致。

常用帧：

```text
ARM_DISABLE                 A5 5A 00 20 23 03 FF
ARM_ENABLE                  A5 5A 00 21 E3 C2 FF
ARM_STOP                    A5 5A 00 25 20 C3 FF
ARM_GET_STATUS              A5 5A 00 26 21 83 FF
ARM_IK_TEST_FLOW start      send ARM_IK_TEST_FLOW
ARM_IK_TEST_FLOW 8s/step    send ARM_IK_TEST_FLOW 1 8000
ARM_IK_TEST_FLOW stop       send ARM_IK_TEST_FLOW 0 0
ARM_SET_TARGET S1,state0,z=450,yaw=0
ARM_SET_TARGET_XYZ x=120,y=0,z=450
ARM_SET_WORKSPACE X+
ARM_HEIGHT_JOG up20         send ARM_HEIGHT_JOG 20
ARM_HEIGHT_JOG down50       send ARM_HEIGHT_JOG -50
ARM_HEIGHT_LIMIT max        send ARM_HEIGHT_LIMIT 1
ARM_SET_POSTURE gripper up  send ARM_SET_POSTURE 2 0
ARM_JOINT_JOG J2 cw10       send ARM_JOINT_JOG 2 -10
ARM_JOINT_JOG J3 ccw90      send ARM_JOINT_JOG 3 90
```

使用方法：

1. 切到 USB：`SYS_SWITCH_SOURCE USB`。
2. 打开机械臂：`ARM_ENABLE`，或直接 `SYS_ENABLE`。
3. 发送 `ARM_SET_TARGET_XYZ x y z`。如果只想保持当前高度切换空间，发 `ARM_SET_WORKSPACE direction`；如果只想切换姿态，发 `ARM_SET_POSTURE tool state`。
4. 测试直接 `(x,y,z)` 逆解时发送 `ARM_IK_TEST_FLOW`。该流程只动机械臂，不发底盘位移；空载荷或 `f0=1` 启动，`f0=0` 停止，`f1` 可选每步停留 ms，默认 `8000ms`。
5. 查询 `ARM_GET_STATUS`，检查 IK 状态、目标角和工具世界坐标。
6. 停止时发 `ARM_STOP`。

失能/使能实物拓扑图（OYZ 侧视；历史图示仅作机构姿态参考，实际工作空间方向以固件坐标修正后的 `Y+ / X+ / X-` 为准）：

![机械臂失能/使能实物拓扑图](Mechanical_Arm_Model/arm_enable_disable_physical_topology.svg)

工具姿态简图（按工具解释 state）：

![工具姿态简图](Mechanical_Arm_Model/tool_posture_states.svg)

`ARM_IK_TEST_FLOW` 步骤：

USB 启动后 `ARM_ENABLE/SYS_ENABLE` 会让 J1/J2 回固定准备姿态，同时 J3 抱死上电电机零位。后续 `ARM_SET_WORKSPACE` 不再依赖逆解路径规划：固件记录当前 J1/J2/J3，保持 J3 锁定，把 J2 转到当前配置的 `alpha_max`，再转 J1 到目标空间 yaw，最后让 J2 回到记录角度；切换后 J2/J3 控制位置与切换前一致。

| 内部 step | 语义 | tool/state | 目标点 `(x,y,z)` mm |
|---|---|---|---|
| `P0` | 实际上电参考，不下发为 IK 目标 | motor zero | 以当前实车标定为准 |
| `0 / S0` | 启动预备：只动 J2 到 `Y+`，长端 `45deg`，变为凸型 | J3 hold power-on | `(0.000, 426.523, 380.304)` |
| `1` | `Y+`，长端斜向上 `45deg` | `S2/state1` | `(0.000, 261.959, 322.029)` |
| `2 / S1` | yaw 切换安全抬臂：`Y+`，长端 `80deg` | `S2/state1` | `(0.000, 77.155, 460.456)` |
| `3 / S2` | 安全切到 `X+`，长端 `80deg` | `S2/state1` | `(77.155, 0.000, 460.456)` |
| `4` | `X+`，长端水平 | `S2/state1` | `(328.885, 0.000, 35.905)` |
| `5 / S3` | yaw 切换安全抬臂：`X+`，长端 `80deg` | `S2/state1` | `(77.155, 0.000, 460.456)` |
| `6 / S4` | 安全切到 `X-`，长端 `80deg` | `S2/state1` | `(-77.155, 0.000, 460.456)` |
| `7` | `X-`，长端斜向下 `30deg` | `S2/state1` | `(-246.992, 0.000, -145.174)` |
| `8 / S5` | yaw 切换安全抬臂：`X-`，长端 `80deg` | `S2/state1` | `(-77.155, 0.000, 460.456)` |
| `9 / S6` | 安全切回 `Y+`，长端 `80deg` | `S2/state1` | `(0.000, 77.155, 460.456)` |
| `10` | 回到 `Y+`，长端斜向上 `45deg` | `S2/state1` | `(0.000, 261.959, 322.029)` |

### 2.4 Tool 工具

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x30` | `TOOL_DISABLE` | 空或 `f0=tool` | 空载荷兼容旧机械臂失能；带 4float 时关闭指定工具执行器 |
| `0x31` | `TOOL_ENABLE` | 空或 `f0=tool` | 空载荷兼容旧机械臂使能；带 4float 时开启指定工具执行器 |
| `0x32` | `TOOL_SET_MODE` | `f0=tool, f1=state, f2=target_z_mm, f3=approach_yaw_rad` | 与 `ARM_SET_TARGET` 相同，设置工具高度与姿态 |
| `0x35` | `TOOL_STOP` | 空或 4float | 工具停止 |
| `0x36` | `TOOL_GET_STATUS` | 空或 4float | 查询工具状态 |

常用帧：

```text
TOOL_DISABLE                A5 5A 00 30 EF 02 FF
TOOL_ENABLE                 A5 5A 00 31 2F C3 FF
TOOL_STOP                   A5 5A 00 35 EC C2 FF
TOOL_GET_STATUS             A5 5A 00 36 ED 82 FF
TOOL_SET_MODE S1,state0,z=450,yaw=0
```

使用方法：

1. 切到 USB：`SYS_SWITCH_SOURCE USB`。
2. 机械臂使能用 `ARM_ENABLE` 或 `SYS_ENABLE`；打开某个工具执行器时发带 4float 的 `TOOL_ENABLE`，`f0=tool`。
3. 发 `TOOL_SET_MODE tool state target_z_mm approach_yaw_rad`，其中 `tool: 0=S1, 1=S2, 2=gripper`，`state` 按上方工具表解释。
4. 查询 `TOOL_GET_STATUS` 判断 IK 状态、目标关节角和工具世界坐标。

`TOOL_ENABLE/DISABLE` 带 4float 时只使用 `f0=tool`：`1=S2` 通过 PE13 高/低电平控制唯一吸盘，`2=gripper` 控制夹爪；`f0=0` 的 S1 吸盘执行器已删除。运动学中的 S1 工具坐标仍保留。夹爪使用 `PE9/TIM1_CH1` 舵机 PWM，脉宽 `500..2500us`。

### 2.5 Robot 总状态

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x46` | `ROBOT_GET_STATUS` | 空或 4float | 查询整车总状态 |

常用帧：

```text
ROBOT_GET_STATUS            A5 5A 00 46 09 83 FF
```

建议上位机把 `ROBOT_GET_STATUS` 作为主状态查询命令，频率建议 `10..50Hz`。如果同时查询多个状态包，例如：

```text
ARM_GET_STATUS -> TOOL_GET_STATUS -> ROBOT_GET_STATUS -> CLIMB_GET_STATUS
```

则回包顺序也会是：

```text
0x26 -> 0x36 -> 0x46 -> 0x56
```

### 2.6 Yaw Auto Tune 自动调参

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x47` | `YAW_TUNE_START` | 空或 `f0=pass_count` | 启动 yaw 自动调参；空 payload 使用默认轮次 |
| `0x48` | `YAW_TUNE_STOP` | 空或 4float | 停止 yaw 自动调参，并停止 USB 底盘 |
| `0x49` | `YAW_TUNE_GET_STATUS` | 空或 4float | 查询 yaw 自动调参状态 |

常用帧：

```text
YAW_TUNE_START default    A5 5A 00 47 C9 42 FF
YAW_TUNE_START pass=1     A5 5A 10 47 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 BD 69 FF
YAW_TUNE_STOP             A5 5A 00 48 CD 02 FF
YAW_TUNE_GET_STATUS       A5 5A 00 49 0D C3 FF
```

使用方法：

1. 切到 USB 控制源，并使能底盘。
2. 发 `YAW_TUNE_START`，可选 `f0=pass_count` 指定轮次。
3. 周期查询 `YAW_TUNE_GET_STATUS`，观察 `state`、`phase`、误差和 PID 结果。
4. 需要中断时发 `YAW_TUNE_STOP`。

### 2.7 Climb 上/下台阶

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x50` | `CLIMB_DISABLE` | 空或 4float | 停止并复位上/下台阶状态机 |
| `0x51` | `CLIMB_ENABLE` | 空或 4float | 使能上/下台阶状态机 |
| `0x52` | `CLIMB_SET_CTRL` | `f0=enable, f1=step, f2=auto, f3=0` | 模拟 USART 三个控制字节 |
| `0x53` | `CLIMB_UP_STEP` | 空或 4float | 上台阶一次性步进请求，每发一次推进一个状态；兼容旧名 `CLIMB_STEP` |
| `0x54` | `CLIMB_UP_AUTO` | 空或 4float | 上台阶一次性自动执行请求，自动跑完整流程；兼容旧名 `CLIMB_AUTO` |
| `0x55` | `CLIMB_STOP` | 空或 4float | 停止并复位上/下台阶状态机 |
| `0x56` | `CLIMB_GET_STATUS` | 空或 4float | 查询上/下台阶状态 |
| `0x57` | `CLIMB_TEST_ACTION` | `f0=动作ID` | 单独执行一个上/下台阶机构调试动作，不推进完整状态机流程 |
| `0x58` | `CLIMB_DOWN_STEP` | 空或 4float | 下台阶一次性步进请求，每发一次推进一个状态；兼容旧名 `CLIMB_DOWNSTAIRS_STEP` |
| `0x59` | `CLIMB_DOWN_AUTO` | 空或 4float | 下台阶一次性自动执行请求，自动跑完整流程；兼容旧名 `CLIMB_DOWNSTAIRS_AUTO` |

常用帧：

```text
CLIMB_DISABLE               A5 5A 00 50 C7 02 FF
CLIMB_ENABLE                A5 5A 00 51 07 C3 FF
CLIMB_UP_STEP               A5 5A 00 53 C6 42 FF
CLIMB_UP_AUTO               A5 5A 00 54 04 03 FF
CLIMB_STOP                  A5 5A 00 55 C4 C2 FF
CLIMB_GET_STATUS            A5 5A 00 56 C5 82 FF
CLIMB_DOWN_STEP             A5 5A 00 58 01 03 FF
CLIMB_DOWN_AUTO             A5 5A 00 59 C1 C2 FF
CLIMB_SET_CTRL enable       A5 5A 10 52 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 29 77 FF
CLIMB_SET_CTRL step edge    A5 5A 10 52 00 00 80 3F 00 00 80 3F 00 00 00 00 00 00 00 00 30 65 FF
CLIMB_SET_CTRL auto edge    A5 5A 10 52 00 00 80 3F 00 00 00 00 00 00 80 3F 00 00 00 00 EC 7C FF
```

Auto laser gate:

- `CLIMB_UP_AUTO` and `CLIMB_SET_CTRL auto` start a forward mecanum search immediately. Firmware drives the mecanum chassis forward until valid X reaches `0 <= x < 35mm`, then starts the climb flow. Transient X invalid readings (`-1`) are tolerated, but continuous X invalid for `1000ms` enters `ERROR` with the `timeout` flag.
- `CLIMB_DOWN_AUTO` starts a forward mecanum search immediately. During `DOWN_LASER_APPROACH_H_GT_65`, valid height `h > 65mm` detects the 50-53mm to 160mm+ drop jump; transient height invalid readings (`-1`) are tolerated, continuous invalid height for `1000ms` enters `ERROR`, and no trigger within `8000ms` also times out. After trigger, the chassis moves forward another `5mm`, then enters `PREPARE_ALL_LEGS_MINUS_30` before the normal downstairs flow.

`CLIMB_SET_CTRL` 与 `CLIMB_UP_STEP` 的区别：

- `CLIMB_SET_CTRL` 是电平输入，等价于 USART 的 `climb_enable/climb_step/climb_auto` 三个字节。
- `CLIMB_SET_CTRL` 的 `step` 和 `auto` 只识别 `0->1` 上升沿；一直重复发送 `f1=1` 不会连续步进。
- `CLIMB_UP_STEP` / `CLIMB_UP_AUTO` 明确选择上台阶流程；`CLIMB_STEP` / `CLIMB_AUTO` 作为旧名兼容。
- `CLIMB_DOWN_STEP` / `CLIMB_DOWN_AUTO` 明确选择下台阶流程；`CLIMB_DOWNSTAIRS_STEP` / `CLIMB_DOWNSTAIRS_AUTO` 作为旧名兼容。
- step 命令是 USB 一次性命令；每收到一帧就触发一次步进，适合按钮点动。
- auto 命令是 USB 一次性命令；在 `IDLE/DONE` 从头自动执行上台阶时会先进入 `UP_LASER_APPROACH_X_LT_35`，麦轮底盘直线前进到 `x < 35mm`，然后进入 `PREPARE_ALL_LEGS_MINUS_30`；上台阶还会继续进入 `UP_PREPARE_CHASSIS_FORWARD_30`，麦轮底盘前进 `30mm` 后再进入 `STEP_01_*`；自动执行下台阶时会先进入 `DOWN_LASER_APPROACH_H_GT_65`，麦轮底盘直线前进等待 `h > 65mm` 突变，然后进入 `DOWN_PREPARE_CHASSIS_FORWARD_5` 再前进 `5mm`，之后进入 `PREPARE_ALL_LEGS_MINUS_30` 和 `STEP_01_*`；在已完成的中间状态从下一状态继续，在执行中的状态会到位后继续自动执行。
- 正在执行一种流程时不允许切换到另一种流程；误切会进入 `ERROR` 并置位 `flow_switch` 错误。
- 如果只是做 USB 上台阶步进调试，推荐使用 `CLIMB_ENABLE` + 多次 `CLIMB_UP_STEP`。
- USB 上/下台阶命令只有当前控制源为 USB 时生效；USART 上台阶命令只有 `UU_flag=0`、当前源为 USART 时输出到电机。

### 2.8 Task flow / weapon grab

| CMD | Name | DATA | Effect |
|---:|---|---|---|
| `0x66` | `FLOW_GET_STATUS` | empty or 4float | Query the outer task-flow state machine |
| `0x67` | `FLOW_WEAPON_GRAB` | empty or 4float | Run `weapon_grab_v1`: J3 ccw30, J2 cw5, J2 cw30, J2 cw1 x3, J3 ccw5, J2 cw1, J2 cw5, J2 cw1, J3 ccw1 x2, J2 ccw5, J3 ccw1 x2 |
| `0x68` | `FLOW_THROW_BLOCK` | 4float | `f0=1` X+ or `f0=2` X-; preserve the completed pickup pose while switching workspace, then turn off suction 2 |
| `0x69` | `FLOW_WEAPON_DOCK_TEST` | empty or 4float | Run the lower-computer weapon grab/dock test state machine; waits for host chassis movement and dock-complete confirmations |
| `0x6A` | `FLOW_CHASSIS_MOVE_DONE` | empty or 4float | Confirm checkpoint 1 and continue after host chassis movement |
| `0x6B` | `FLOW_DOCK_DONE` | empty or 4float | Confirm checkpoint 2 and continue after host dock decision |

Common frames and serial-tool calls:

```text
FLOW_WEAPON_GRAB       A5 5A 00 67 11 43 FF
FLOW_GET_STATUS        A5 5A 00 66 D1 82 FF
FLOW_THROW_BLOCK X+    A5 5A 10 68 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 84 45 FF
FLOW_THROW_BLOCK X-    A5 5A 10 68 00 00 00 40 00 00 00 00 00 00 00 00 00 00 00 00 FB 6F FF
FLOW_WEAPON_DOCK_TEST  A5 5A 00 69 D5 C2 FF
FLOW_CHASSIS_MOVE_DONE A5 5A 00 6A D4 82 FF
FLOW_DOCK_DONE         A5 5A 00 6B 14 43 FF

.\.venv\Scripts\python -m serial_tool send --port COM7 FLOW_WEAPON_GRAB --wait 0.2
.\.venv\Scripts\python -m serial_tool send --port COM7 THROW_BLOCK 1 --wait 0.2
.\.venv\Scripts\python -m serial_tool send --port COM7 THROW_BLOCK 2 --wait 0.2
.\.venv\Scripts\python -m serial_tool send --port COM7 FLOW_WEAPON_DOCK_TEST --wait 0.2
.\.venv\Scripts\python -m serial_tool send --port COM7 FLOW_CHASSIS_MOVE_DONE --wait 0.2
.\.venv\Scripts\python -m serial_tool send --port COM7 FLOW_DOCK_DONE --wait 0.2
.\.venv\Scripts\python -m serial_tool poll --port COM7 FLOW_GET_STATUS --rate 10
.\.venv\Scripts\python -m serial_tool block-flow-run --port COM7 PC_USB_Serial_Tool\weapon_grab_v1.json --timeout 20 --arm-settle 1.0
```

`FLOW_THROW_BLOCK` requires the completion marker from `S1_UP_V2` or `S1_DOWN_V2`. The workspace switch saves the current joint feedback and restores J2/J3 before releasing suction 2. The release consumes the marker, and a request without a marker returns task-flow `PRECONDITION` without moving the arm. The former `S1_UP_S2_UP_V2 / 0x62` command has been removed and merged into `S1_UP_V2 / 0x60`.

## 3. 上台阶动作流程

上台阶状态编号：

| state | 名称 | 动作 |
|---:|---|---|
| `0` | `IDLE` | 待机保持，四根立杆相对上电零位向上 `30mm`，目标 `-30mm`，上台阶使能后位置环开启 |
| `1..13` | `STEP_XX_*` | 按执行相邻关系压缩后的上台阶主流程；展开记录见 `CLIMB_ACTION_SEQUENCE.yaml` |
| `22` | `DONE` | 完成并保持待机位 `-30mm` |
| `23` | `ERROR` | 错误 |
| `24` | `PREPARE_ALL_LEGS_MINUS_30` | 执行上/下台阶前的预备状态，四根立杆先确认到待机位 `-30mm` |
| `25` | `UP_PREPARE_CHASSIS_FORWARD_30` | 上台阶专用预备状态，麦轮底盘直线前进 `30mm` |
| `26` | `UP_LASER_APPROACH_X_LT_35` | 上台阶自动触发前的接近状态，麦轮底盘直线前进到有效 X 满足 `0 <= x < 35mm`；X 短暂 `-1` 会等待，连续 `1000ms` 无效则超时 |
| `27` | `DOWN_LASER_APPROACH_H_GT_65` | 下台阶自动触发前的接近状态，麦轮底盘直线前进，检测到高度从约 `50-53mm` 突变到 `h > 65mm` 后触发；H 短暂 `-1` 会等待，连续 `1000ms` 无效则超时 |
| `28` | `DOWN_PREPARE_CHASSIS_FORWARD_5` | 下台阶触发后的预备状态，麦轮底盘再前进 `5mm` 后进入正式下台阶 |

动作参数：

| 参数 | 当前值 |
|---|---:|
| 上电接地位置 | `0mm` |
| 待机保持位置 | `-30mm` |
| 离地最大高度 | `30mm` |
| 撑地最大高度 | `260mm` |
| 高位抬升高度 | `220mm` |
| 前腿离地待机位 | `-30mm` |
| 前腿支撑过渡位 | `30mm` |
| 上台阶动作文件 | `PC_USB_Serial_Tool/upstairs_v4.json` |
| 保存动作总数 | `40` |
| 压缩后固件主流程状态数 | `13` |
| 执行前预备状态 | 上台阶：`state=26`，麦轮底盘直线前进到 `x < 35mm`；`state=24`，四根立杆目标 `-30mm`；`state=25`，麦轮底盘前进 `30mm`。下台阶：`state=27`，麦轮底盘前进等待 `h > 65mm` 突变；`state=28`，麦轮底盘再前进 `5mm`；`state=24`，四根立杆目标 `-30mm` |
| 上台阶固件压缩驱动段 | `ALL +90mm`、`ALL +30mm`、`REAR +120mm`、`ALL +500mm`、`ALL +270mm` |
| 小驱动轮超时估算 | `80mm/s + 1500ms` |
| 底盘最终前进 | `200mm` |
| 立杆到位容差 | `3mm` |
| 小驱动轮到位容差 | `5mm` |

状态完成条件：

| 状态类型 | 完成条件 |
|---|---|
| `PREPARE_ALL_LEGS_MINUS_30` | 四根立杆到待机位 `-30mm ± 3mm` |
| `UP_PREPARE_CHASSIS_FORWARD_30` | 底盘位置模式完成前进 `30mm` |
| `UP_LASER_APPROACH_X_LT_35` | 有效 X 满足 `0 <= x < 35mm`；X 连续无效 `1000ms` 触发超时保护 |
| `DOWN_LASER_APPROACH_H_GT_65` | 激光 H 有效在线且 `h > 65mm`，用于识别从约 `50-53mm` 到 `160mm+` 的高度突变；H 连续无效 `1000ms` 或接近超过 `8000ms` 触发超时保护 |
| `DOWN_PREPARE_CHASSIS_FORWARD_5` | 底盘位置模式完成前进 `5mm` |
| 立杆目标动作 | 对应立杆到目标位置 `±3mm` |
| 小驱动轮目标动作 | 当前驱动轮组完成本段 `xxx mm ± 5mm`；完整流程为前后四轮，测试动作按 `FRONT/REAR/ALL` 分组 |
| `CHASSIS_FORWARD_xxx` | 底盘位置模式返回 `R2_POS_DONE` |

下台阶压缩结构：

| 参数 | 当前值 |
|---|---:|
| 下台阶动作文件 | `PC_USB_Serial_Tool/downstairs_v4.json` |
| 保存动作总数 | `79` |
| 压缩后固件主流程状态数 | `15` |
| 展开记录 | `DOWNSTAIRS_ACTION_SEQUENCE.yaml` |
| 下台阶固件压缩驱动段 | `ALL +500mm`、`ALL +330mm`、`ALL +20mm`、`ALL +90mm`、`ALL +10mm`、`ALL +120mm` |
| 小驱动轮超时估算 | `80mm/s + 1500ms` |
| 结构体 | `s_downstairs_steps` |

`s_downstairs_steps` 是下台阶流程数据表；`CLIMB_UP_AUTO` / `CLIMB_UP_STEP` 执行上台阶 `s_main_steps`，`CLIMB_DOWN_AUTO` / `CLIMB_DOWN_STEP` 执行下台阶 `s_downstairs_steps`。旧名 `CLIMB_AUTO` / `CLIMB_STEP` 和 `CLIMB_DOWNSTAIRS_AUTO` / `CLIMB_DOWNSTAIRS_STEP` 继续兼容。

独立调试动作 `CLIMB_TEST_ACTION f0=动作ID`：

| 动作ID | 名称 | 动作 |
|---:|---|---|
| `1` | `ALL_LEGS_220` | 四根立杆目标设为 `220mm` |
| `2` | `ALL_LEGS_UP_10` | 四根立杆基于当前位置上升 `10mm` |
| `3` | `ALL_LEGS_DOWN_10` | 四根立杆基于当前位置下降 `10mm` |
| `4` | `REAR_DRIVE_FORWARD_30` | 后方小驱动轮前进 `30mm` |
| `5` | `REAR_DRIVE_FORWARD_10` | 后方小驱动轮前进 `10mm` |
| `6` | `REAR_DRIVE_BACKWARD_10` | 后方小驱动轮后退 `10mm` |
| `7` | `FRONT_ZERO` | 前两根立杆 ID1/ID4 回到 `0mm` |
| `8` | `FRONT_UP_10` | 前两根立杆 ID1/ID4 基于当前位置上升 `10mm` |
| `9` | `FRONT_DOWN_10` | 前两根立杆 ID1/ID4 基于当前位置下降 `10mm` |
| `10` | `CHASSIS_FORWARD_100` | 底盘麦轮前进 `100mm` |
| `11` | `CHASSIS_FORWARD_50` | 底盘麦轮前进 `50mm` |
| `12` | `CHASSIS_BACKWARD_50` | 底盘麦轮后退 `50mm` |
| `13` | `REAR_ZERO` | 后两根立杆 ID2/ID3 回到 `0mm` |
| `14` | `REAR_UP_10` | 后两根立杆 ID2/ID3 基于当前位置上升 `10mm` |
| `15` | `REAR_DOWN_10` | 后两根立杆 ID2/ID3 基于当前位置下降 `10mm` |
| `16` | `ALL_LEGS_ZERO` | 四根立杆统一回到 `0mm` |
| `17` | `REAR_DRIVE_BACKWARD_30` | 后方小驱动轮后退 `30mm` |
| `18` | `REAR_DRIVE_FORWARD_500` | 后方小驱动轮前进 `500mm` |
| `19` | `REAR_DRIVE_BACKWARD_500` | 后方小驱动轮后退 `500mm` |
| `20` | `CHASSIS_BACKWARD_100` | 底盘麦轮后退 `100mm` |
| `21` | `CHASSIS_FORWARD_300` | 底盘麦轮前进 `300mm` |
| `22` | `CHASSIS_BACKWARD_300` | 底盘麦轮后退 `300mm` |
| `23` | `FRONT_220` | 前两根立杆 ID1/ID4 目标到 `220mm` |
| `24` | `FRONT_MINUS_30` | 前两根立杆 ID1/ID4 目标到 `-30mm` |
| `25` | `REAR_220` | 后两根立杆 ID2/ID3 目标到 `220mm` |
| `26` | `REAR_MINUS_30` | 后两根立杆 ID2/ID3 目标到 `-30mm` |
| `27` | `FRONT_DRIVE_FORWARD_30` | 前方小驱动轮前进 `30mm` |
| `28` | `FRONT_DRIVE_FORWARD_10` | 前方小驱动轮前进 `10mm` |
| `29` | `FRONT_DRIVE_BACKWARD_10` | 前方小驱动轮后退 `10mm` |
| `30` | `FRONT_DRIVE_BACKWARD_30` | 前方小驱动轮后退 `30mm` |
| `31` | `FRONT_DRIVE_FORWARD_500` | 前方小驱动轮前进 `500mm` |
| `32` | `FRONT_DRIVE_BACKWARD_500` | 前方小驱动轮后退 `500mm` |
| `33` | `ALL_DRIVE_FORWARD_30` | 前后四个小驱动轮一起前进 `30mm` |
| `34` | `ALL_DRIVE_FORWARD_10` | 前后四个小驱动轮一起前进 `10mm` |
| `35` | `ALL_DRIVE_BACKWARD_10` | 前后四个小驱动轮一起后退 `10mm` |
| `36` | `ALL_DRIVE_BACKWARD_30` | 前后四个小驱动轮一起后退 `30mm` |
| `37` | `ALL_DRIVE_FORWARD_500` | 前后四个小驱动轮一起前进 `500mm` |
| `38` | `ALL_DRIVE_BACKWARD_500` | 前后四个小驱动轮一起后退 `500mm` |

旧 `DRIVE_*` / `climb_drive_*` 兼容名已删除；请使用 `REAR_DRIVE_*`、
`FRONT_DRIVE_*` 或 `ALL_DRIVE_*`。

尺度定义：

- 上电位置，即辅助轮和驱动轮接触地面的位置，为 `0mm`。
- 正方向表示立杆伸出、抬升底盘。
- 负方向表示立杆收回、离地。
- 离地目标禁止超过 `30mm`，即负向目标不能小于 `-30mm`。
- 撑地目标禁止超过 `260mm`，即正向目标不能大于 `260mm`。

USB 步进流程：

```text
1. 切换 USB 控制源
   A5 5A 10 02 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 38 4B FF

2. 使能上台阶
   A5 5A 00 51 07 C3 FF

3. 每次步进发送 CLIMB_UP_STEP
   A5 5A 00 53 C6 42 FF

4. 每次步进后查询状态，等待 state_done=1 再发下一次
   A5 5A 00 56 C5 82 FF

5. 当前上台阶完整顺序已压缩为 `13` 个固件状态：连续立杆 `10mm` 动作合并为目标位置，连续小驱动轮动作按 `ALL/FRONT/REAR` 驱动组压缩，重复底盘前进也合并为位置目标

6. 查询状态，确认 `state=22`；若使用 PC 工具解码，可同时看到 `state_name=DONE`
   A5 5A 00 56 C5 82 FF
```

USB 自动流程：

```text
1. SYS_SWITCH_SOURCE USB
2. CLIMB_ENABLE
3. CLIMB_UP_AUTO
4. 周期查询 CLIMB_GET_STATUS
5. state=22 表示完成，state=23 表示错误
```

可直接发送：

```text
SYS_SWITCH_SOURCE USB        A5 5A 10 02 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 38 4B FF
CLIMB_ENABLE                 A5 5A 00 51 07 C3 FF
CLIMB_UP_AUTO                A5 5A 00 54 04 03 FF
CLIMB_GET_STATUS             A5 5A 00 56 C5 82 FF
CLIMB_STOP                   A5 5A 00 55 C4 C2 FF
```

> 硬件更新：上/下台阶小驱动轮现在共有 4 个 M2006。FDCAN1 ID5/6
> 为前两根立杆下方的前左/前右驱动轮，FDCAN2 ID5/6 为后左/后右驱动轮。
> 前进时两条总线的 ID5 均为顺时针，ID6 均为逆时针；四个电机输出轴到轮子均为 1:1 传动。

## 4. 上台阶控制对象和电机链路

立杆升降电机走 `FDCAN2`。四个立杆升降电机按新车头方向定义，从右上角开始顺时针编号 `1..4`，编号即电机 ID。小驱动轮为 4 个 M2006：FDCAN1 ID5/6 是前两根立杆下方的前左/前右驱动轮，FDCAN2 ID5/6 是后左/后右驱动轮。

| 总线/ID | 控制对象 | 电机 | 位置 |
|---:|---|---|---|
| FDCAN2 `1` | 立杆 1 | M3508 | 新车头方向右上角，前右；立杆向下/撑地为逆时针 |
| FDCAN2 `2` | 立杆 2 | M3508 | 顺时针第 2 个，后右；立杆向下/撑地为顺时针 |
| FDCAN2 `3` | 立杆 3 | M3508 | 顺时针第 3 个，后左；立杆向下/撑地为逆时针 |
| FDCAN2 `4` | 立杆 4 | M3508 | 顺时针第 4 个，前左；立杆向下/撑地为顺时针 |
| FDCAN1 `5` | 前小驱动轮左 | M2006 | 俯视车头左侧；前进时电机输出轴顺时针 |
| FDCAN1 `6` | 前小驱动轮右 | M2006 | 俯视车头右侧；前进时电机输出轴逆时针 |
| FDCAN2 `5` | 后小驱动轮左 | M2006 | 后左；前进时电机输出轴顺时针 |
| FDCAN2 `6` | 后小驱动轮右 | M2006 | 后右；前进时电机输出轴逆时针 |

机械参数：

| 项 | 当前值 |
|---|---:|
| 编码器计数 | `8192 count/rev` |
| 立杆减速比 | `3591/187` |
| 齿轮齿数 | `18` |
| 齿条齿距 | `4.71mm` |
| 齿轮输出轴每转位移 | `84.78mm` |
| 立杆比例 | `1855.540 count/mm` |
| 小驱动轮直径 | `64mm` |
| 小驱动轮电机内部减速比 | `36`，M2006 内部减速 |
| 小驱动轮输出轴到轮子外部传动比 | `1:1` |
| 小驱动轮比例 | `1466.772 count/mm` |

方向参数约定：`+1` 对应立杆向下/撑地时电机逆时针，`-1` 对应立杆向下/撑地时电机顺时针。

| 参数 | 当前值 | 立杆向下/撑地旋向 |
|---|---:|---|
| `R2_CLIMB_LEG1_DIR` | `+1` | 逆时针 |
| `R2_CLIMB_LEG2_DIR` | `-1` | 顺时针 |
| `R2_CLIMB_LEG3_DIR` | `+1` | 逆时针 |
| `R2_CLIMB_LEG4_DIR` | `-1` | 顺时针 |
| `R2_CLIMB_DRIVE_LEFT_DIR` | `-1` | ID5 前进为顺时针 |
| `R2_CLIMB_DRIVE_RIGHT_DIR` | `+1` | ID6 前进为逆时针 |

方向检查：

- 正向向下/撑地目标会使 1/3 号立杆目标 count 增加，使 2/4 号立杆目标 count 减少。
- 若实车上 1/3 号正电流不是逆时针，或 2/4 号负电流不是顺时针，需要先修正对应 `R2_CLIMB_LEGx_DIR`，不要直接跑完整行程。
- 驱动轮正向前进目标会使 ID5 目标 count 减少，使 ID6 目标 count 增加。
- 若实车上 ID5 前进不是顺时针，或 ID6 前进不是逆时针，需要先修正对应 `R2_CLIMB_DRIVEx_DIR`。
- 首次实车测试建议先用小行程、低输出限幅点动，确认四根立杆都在抬升方向运动后再执行 `LIFT_HIGH`。

如果实车动作方向反了，优先改对应方向宏，不要改状态机距离。

Debug 可观察变量：

```c
g_r2_climb_debug_usart
g_r2_climb_debug_usb
g_r2_climb_debug_active
```

主要字段：

| 字段 | 含义 |
|---|---|
| `source` | `0=USART, 1=USB, 2=none` |
| `state` / `state_name` | 当前上台阶状态 |
| `enabled` | 上台阶功能是否使能 |
| `auto_run` | 是否处于自动执行 |
| `state_done` | 当前状态是否到位 |
| `error_flags` | 错误标志 |
| `is_motor_active` | 当前是否正在输出上台阶电机控制 |
| `pending_step` / `pending_auto` | 待处理请求 |
| `leg_pos_mm[4]` | 四根立杆当前位置 |
| `leg_target_mm[4]` | 四根立杆目标位置 |
| `drive_pos_mm[2]` | 当前驱动轮组的位置；前驱测试为前轮，后驱测试为后轮，完整流程为前后平均 |
| `drive_target_mm[2]` | 当前驱动轮组目标位置 |
| `leg_current[4]` | 四根立杆输出电流 |
| `front_drive_current[2]` / `drive_current[2]` | 前/后小驱动轮输出电流 |

## 5. USART 遥控控制帧

USART 使用 `UART10_Receive()` 解析，帧格式：

| 字段 | 长度 | 说明 |
|---|---:|---|
| HEAD | 1 | 固定 `0xA5` |
| DATA | 39 | 数据区 |
| CHECKSUM | 1 | DATA 40 字节累加和低 8 位 |
| TAIL | 1 | 固定 `0x5A` |

DATA 布局：

| 字节 | 含义 |
|---:|---|
| `0..7` | 底盘 8 种模式 one-hot，最多一个为 1；全 0 表示底盘停止 |
| `8` | `arm_flag`，`0=归零/保持`，`1=使能` |
| `9` | `UU_flag`，`0=USART源`，`1=USB源` |
| `10..12` | 保留；新机械臂工具目标通过 USB `ARM_SET_TARGET_XYZ`、`ARM_SET_TARGET` 或 `TOOL_SET_MODE` 设置 |
| `13` | `climb_enable`，`0=停止/复位`，`1=使能` |
| `14` | `climb_step`，上升沿触发一次步进 |
| `15` | `climb_auto`，上升沿触发自动执行 |
| `16..19` | `chassis param1`，VEL:`vx(m/s)` / POS:`dx(m)` |
| `20..23` | `chassis param2`，VEL:`vy(m/s)` / POS:`dy(m)` |
| `24..27` | `chassis yaw_data`，`ROBOT_NO_YAW`: `target_yaw_robot_deg`；`WORLD_NO_YAW`: `target_yaw_world_deg`；其它 VEL: `vw(rad/s)`；其它 POS: `dyaw(rad)` |
| `28..39` | 保留给新机械臂控制链路；当前固件不从 USART 帧解析机械臂目标 |

USART 侧新机械臂目标统一使用工具、高度、姿态和 yaw；平面移动由底盘应用处理。

USART 底盘 `chassis yaw_data` 与 USB 的 `yaw_data` 复用规则一致。`ROBOT_NO_YAW` 下目标角按机器人系解释，固件用进入机器人系锁定模式时的参考朝向换算到 IMU 闭环目标；`WORLD_NO_YAW` 下目标角按世界系绝对 yaw 解释。

USART 上台阶用法：

1. `UU_flag=0` 时当前帧才会写入 USART 底盘和上台阶控制器；`UU_flag=1` 会切到 USB 源并停止 USART 侧上台阶控制器。
2. `climb_enable=1` 使能。
3. 每次要步进时，让 `climb_step` 从 `0` 变为 `1`。
4. 如果要再次步进，必须先发一帧 `climb_step=0`，再发 `climb_step=1`。
5. 自动执行同理，`climb_auto` 需要 `0->1` 上升沿。
6. 自动执行启动后仍需持续发送 `UU_flag=0, climb_enable=1, climb_step=0, climb_auto=0` 的保活帧；USART 控制链路超过 `300ms` 无新帧会触发看门狗并停止 USART 上台阶。
7. `climb_enable=0` 会停止并复位上台阶状态机。

## 6. 状态回包简表

### 6.1 `SYS_GET_STATUS` 回包 `0x06`, `LEN=8`

| offset | 类型 | 含义 |
|---:|---|---|
| 0 | u8 | `USB_Task_flag` |
| 1 | u8 | `USART_Task_flag` |
| 2 | u8 | `Mecanum_control_flag` |
| 3 | u8 | reserved，当前为 `0` |
| 4 | u8 | `g_r2_arm_usb.enabled` |
| 5 | u8 | 底盘电机在线数 |
| 6 | u8 | USB 超时标志，bit0=USB 底盘速度看门狗超时，其它位保留 |
| 7 | u8 | reserved |

### 6.2 `CHS_GET_STATUS` 回包 `0x16`, `LEN=112`

| offset | 类型 | 含义 |
|---:|---|---|
| 0 | u8 | `mode`，见底盘 8 种模式表 |
| 1 | u8 | `pos_state`，`0=IDLE, 1=RUNNING, 2=DONE` |
| 2 | u8 | `emergency_stop` |
| 3 | u8 | reserved |
| 4 | f32 | `robot_vel.vx`，当前执行速度，m/s |
| 8 | f32 | `robot_vel.vy`，当前执行速度，m/s |
| 12 | f32 | `robot_vel.vw`，当前执行角速度，rad/s |
| 16 | f32 | `odom_x`，当前里程计，m |
| 20 | f32 | `odom_y`，当前里程计，m |
| 24 | f32 | `odom_yaw`，当前里程计 yaw，rad |
| 28 | f32 | `v_max` |
| 32 | f32 | `a_max` |
| 36 | f32 | `j_max` |
| 40 | f32 | `pos_progress` |
| 44 | f32 | `pos_err_x` |
| 48 | f32 | `pos_err_y` |
| 52 | f32 | `pos_err_yaw` |
| 56 | f32 | `nav.x_m` |
| 60 | f32 | `nav.y_m` |
| 64 | f32 | `nav.yaw_rad` |
| 68 | f32 | `nav.yaw_total_rad` |
| 72 | f32 | `nav.vx_mps` |
| 76 | f32 | `nav.vy_mps` |
| 80 | f32 | `nav.wz_radps` |
| 84 | u8 | `imu_online` |
| 85 | u8 | USB 超时标志，bit0=USB 底盘速度看门狗超时，其它位保留 |
| 86 | u8 | `status_flags`，见下表 |
| 87 | u8 | `error_flags`，见下表 |
| 88 | f32 | `target_vx`，当前目标速度，m/s |
| 92 | f32 | `target_vy`，当前目标速度，m/s |
| 96 | f32 | `target_vw`，当前目标角速度，rad/s |
| 100 | f32 | `target_dx`，当前位置目标，m |
| 104 | f32 | `target_dy`，当前位置目标，m |
| 108 | f32 | `target_dyaw`，当前位置目标，rad |

`CHS_GET_STATUS.status_flags`：

| bit | 含义 |
|---:|---|
| 0 | 底盘使能 |
| 1 | 当前为速度模式 |
| 2 | 当前为位置模式 |
| 3 | 位置运动执行中 |
| 4 | 底盘正在运动 |
| 5 | 位置运动完成 |
| 6 | IMU 在线 |
| 7 | 底盘电机全部在线 |

`CHS_GET_STATUS.error_flags`：

| bit | 含义 |
|---:|---|
| 0 | USB 底盘看门狗超时 |
| 1 | IMU 离线 |
| 2 | 底盘急停 |
| 3 | 底盘电机全部离线 |
| 4 | 底盘电机部分离线 |

### 6.3 `ARM_GET_STATUS` packet `0x26`, `LEN=80`

`TOOL_GET_STATUS (0x36)` uses the same 80-byte layout in the current firmware.

| offset | type | meaning |
|---:|---|---|
| 0 | u8 | `enabled` |
| 1 | u8 | `has_target` |
| 2 | u8 | `output_enabled` |
| 3 | u8 | `state`, `0=IDLE, 1=READY, 2=TARGET_VALID, 3=STOPPED, 4=ERROR` |
| 4 | u8 | `status_flags` |
| 5 | u8 | `error_flags` |
| 6 | u8 | `ik_status`, `0=OK, 1=NULL, 2=BAD_PARAM, 3=HEIGHT_UNREACHABLE, 4=JOINT_LIMIT, 5=LONG_LINK_LIMIT, 6=YAW_SWITCH_UNSAFE, 7=UNSUPPORTED_STATE, 8=BAD_DIRECTION` |
| 7 | u8 | `ik_reason`, `0=NONE, 1=BAD_TOOL, 2=BAD_STATE, 3=BAD_FLOAT, 4=HEIGHT_OUTSIDE_LINK, 5=J1_LIMIT, 6=J2_LIMIT, 7=J3_LIMIT, 8=BAD_DIRECTION, 9=UNSUPPORTED_STATE, 10=LONG_LINK_LIMIT, 11=YAW_SWITCH_UNSAFE` |
| 8 | u32 | `last_command_ms` |
| 12 | u32 | `last_update_ms` |
| 16 | u32 | `output_apply_count` |
| 20 | u8 | `tool`, `0=S1, 1=S2, 2=gripper` |
| 21 | u8 | `tool_state`，按当前 `tool` 解释：S1=`0/1/2`，S2=`0/1/2/3`，gripper=`0/1/2` |
| 22 | u8 | `target_direction`, `0=Y+`, `1=X+`, `2=X-` |
| 23 | u8 | `ik_test_flags`: low4=step, bit5=error, bit6=done, bit7=active |
| 24 | f32 | `target_z_mm` |
| 28 | f32 | `approach_yaw_rad` |
| 32 | f32 | `theta1_rad` |
| 36 | f32 | `theta2_rad` |
| 40 | f32 | `theta3_rad` |
| 44 | f32 | `tool_world_x_mm` |
| 48 | f32 | `tool_world_y_mm` |
| 52 | f32 | `tool_world_z_mm` |
| 56 | f32 | `j4_world_z_mm` |
| 60 | f32 | `height_error_mm = target_z_mm - tool_world_z_mm` |
| 64 | f32 | `motor_feedback_rad[0]`, raw J1 motor feedback angle |
| 68 | f32 | `motor_feedback_rad[1]`, raw J2 motor feedback angle |
| 72 | f32 | `motor_feedback_rad[2]`, raw J3 motor feedback angle |
| 76 | u8 | `motor_feedback_ok_mask`, bit0=J1, bit1=J2, bit2=J3 |

`status_flags`: bit0=enabled, bit1=has_target, bit2=output_enabled, bit3=IK_OK, bit4=active.

`error_flags`: bit0=IK_NOT_OK, bit1=JOINT_LIMIT, bit2=HEIGHT_UNREACHABLE, bit3=BAD_PARAM, bit4=UNSAFE, bit5=UNSUPPORTED.

### 6.4 `TOOL_GET_STATUS` packet `0x36`, `LEN=80`

This packet is intentionally identical to `ARM_GET_STATUS` for the new coupled tool/arm model.

### 6.5 `YAW_TUNE_GET_STATUS` 回包 `0x49`, `LEN=64`

| offset | 类型 | 含义 |
|---:|---|---|
| 0 | u8 | `state` |
| 1 | u8 | `segment_index` |
| 2 | u8 | `segment_count` |
| 3 | u8 | `fail_reason` |
| 4 | u32 | `tick_ms` |
| 8 | u32 | `segment_elapsed_ms` |
| 12 | f32 | `yaw_error_deg` |
| 16 | f32 | `yaw_error_abs_max_deg` |
| 20 | f32 | `yaw_rate_error_rms_dps` |
| 24 | f32 | `gyro_z_abs_max_dps` |
| 28 | f32 | `score` |
| 32 | f32 | `angle_kp` |
| 36 | f32 | `angle_kd` |
| 40 | f32 | `rate_kp` |
| 44 | f32 | `rate_ki` |
| 48 | f32 | `rate_kd` |
| 52 | f32 | `pos_kp_yaw` |
| 56 | f32 | `last_adjust` |
| 60 | u8 | `pass_index` |
| 61 | u8 | `pass_count` |
| 62 | u8 | `active_mode` |
| 63 | u8 | `phase` |

注意：当前 USB 状态包没有打包 `angle_ki`，需要看完整调参内部状态时以固件结构体或调试视图为准。

### 6.6 `ROBOT_GET_STATUS` 回包 `0x46`, `LEN=253`

| offset | 类型 | 含义 |
|---:|---|---|
| 0 | u8 | 协议版本，当前为 `5` |
| 1 | u8 | active source，`0=USART, 1=USB, 2=none` |
| 2 | u8 | enable flags，bit0 底盘，bit3 上/下台阶，bit4 机械臂 |
| 3 | u8 | executing flags，bit0 底盘运动，bit1 位置运动，bit4 台阶电机输出，bit5 yaw 自动调参，bit6 机械臂电机输出，bit7 任一执行中 |
| 4 | u8 | error flags |
| 5 | u8 | online flags，bit0 USB近期有命令，bit1 USART近期有命令，bit2 IMU在线，bit3底盘电机在线，bit4机械臂状态已更新，bit5台阶电机在线，bit6激光在线 |
| 6 | u8 | USB 最近命令 CMD |
| 7 | u8 | USB 最近命令 LEN |
| 8 | u32 | USB 命令计数 |
| 12 | u32 | USB 最近命令 tick |
| 16 | u32 | USART 成功帧计数 |
| 20 | u32 | USART 最近帧 tick |
| 24 | u32 | USART 校验失败计数 |
| 28 | u8 | USART 最近校验是否 OK |
| 29 | u8 | USART 当前底盘模式 |
| 30 | u8 | reserved，当前为 `0` |
| 31 | u8 | reserved，当前为 `0` |
| 32 | f32 | `nav.x_m` |
| 36 | f32 | `nav.y_m` |
| 40 | f32 | `nav.yaw_total_rad` |
| 44 | f32 | `nav.vx_mps` |
| 48 | f32 | `nav.vy_mps` |
| 52 | f32 | `nav.wz_radps` |
| 56 | f32 | `chs.odom_x` |
| 60 | f32 | `chs.odom_y` |
| 64 | f32 | `chs.odom_yaw` |
| 68 | f32 | `chs.robot_vel.vx` |
| 72 | f32 | `chs.robot_vel.vy` |
| 76 | f32 | `chs.robot_vel.vw` |
| 80..93 | u8 | reserved，当前为 `0`；机械臂详细状态见 offset `184..235` |
| 94 | u8 | 底盘 `pos_state` |
| 95 | u8 | timeout flags |
| 96 | u8 | climb active source，`0=USART, 1=USB, 2=none` |
| 97 | u8 | climb `state` |
| 98 | u8 | climb `enabled` |
| 99 | u8 | climb `auto_run` |
| 100 | u8 | climb `state_done` |
| 101 | u8 | climb `error_flags`，bit0 超时，bit1 参数未配置，bit2 测试动作错误，bit3 流程切换错误，bit4 自动修补失败 |
| 102 | u8 | climb `pending_step` |
| 103 | u8 | climb `pending_auto` |
| 104 | u8 | climb motor active |
| 105 | u8 | 上/下台阶电机在线数，统计 FDCAN2 ID1..6 + FDCAN1 ID5..6，共 8 个 |
| 106 | u8 | climb test action |
| 107 | u8 | climb flags，bit0 test active，bit1 test chassis active，bit2 downstairs flow |
| 108 | u32 | climb 当前状态已运行时间 ms |
| 112 | u32 | climb 最近更新时间 ms |
| 116 | u8 | laser valid flags |
| 117 | u8 | laser online flags |
| 118 | u8 | laser waiting flags |
| 119 | u8 | laser all valid |
| 120 | u8 | laser all online |
| 124 | u32 | laser update tick |
| 128 | i32 | laser x_pos distance mm |
| 132 | i32 | laser y_pos distance mm |
| 136 | i32 | laser height distance mm |
| 140 | u8 | yaw tune state |
| 141 | u8 | yaw tune fail reason |
| 142 | u8 | yaw tune phase |
| 143 | u8 | yaw tune active mode |
| 144 | u32 | yaw tune tick ms |
| 148 | u32 | yaw tune segment elapsed ms |
| 152 | f32 | yaw tune yaw error deg |
| 156 | f32 | yaw tune score |
| 160 | f32 | `chassis.target_vx` |
| 164 | f32 | `chassis.target_vy` |
| 168 | f32 | `chassis.target_vw` |
| 172 | f32 | `chassis.target_dx` |
| 176 | f32 | `chassis.target_dy` |
| 180 | f32 | `chassis.target_dyaw` |
| 184 | u8 | `arm.enabled` |
| 185 | u8 | `arm.has_target` |
| 186 | u8 | `arm.output_enabled` |
| 187 | u8 | `arm.state` |
| 188 | u8 | `arm.status_flags` |
| 189 | u8 | `arm.error_flags` |
| 190 | u8 | `arm.ik_status` |
| 191 | u8 | `arm.ik_reason` |
| 192 | f32 | `arm.target_z_mm` |
| 196 | f32 | `arm.approach_yaw_rad` |
| 200 | f32 | `arm.theta1_rad` |
| 204 | f32 | `arm.theta2_rad` |
| 208 | f32 | `arm.theta3_rad` |
| 212 | f32 | `arm.tool_world_x_mm` |
| 216 | f32 | `arm.tool_world_y_mm` |
| 220 | f32 | `arm.tool_world_z_mm` |
| 224 | u32 | `arm.last_update_ms` |
| 228 | u32 | `arm.output_apply_count` |
| 232 | u8 | `arm.tool`，`0=S1, 1=S2, 2=gripper` |
| 233 | u8 | `arm.tool_state`，按当前 `arm.tool` 解释：S1=`0/1/2`，S2=`0/1/2/3`，gripper=`0/1/2` |
| 234 | u8 | `arm.target_direction`，`0=Y+`, `1=X+`, `2=X-` |
| 235 | u8 | `arm.ik_test_flags`: low4=step, bit5=error, bit6=done, bit7=active |
| 236..237 | u8 | 保留 |
| 238 | u8 | climb `error_flags` 摘要 |
| 239 | u8 | `active_source_stale` |
| 240 | f32 | `arm.motor_feedback_rad[0]`, raw J1 motor feedback angle |
| 244 | f32 | `arm.motor_feedback_rad[1]`, raw J2 motor feedback angle |
| 248 | f32 | `arm.motor_feedback_rad[2]`, raw J3 motor feedback angle |
| 252 | u8 | `arm.motor_feedback_ok_mask`, bit0=J1, bit1=J2, bit2=J3 |

`ROBOT_GET_STATUS.error_flags`：

| bit | 含义 |
|---:|---|
| 0 | USB 底盘超时 |
| 1 | 保留 |
| 2 | 保留 |
| 3 | IMU 离线 |
| 4 | 底盘急停 |
| 5 | 机械臂错误 |
| 6 | 保留 |
| 7 | 当前控制源命令不新鲜 |

### 6.7 `CLIMB_GET_STATUS` 回包 `0x56`, `LEN=69`

| offset | 类型 | 含义 |
|---:|---|---|
| 0 | u8 | `state` |
| 1 | u8 | `enabled` |
| 2 | u8 | `auto_run` |
| 3 | u8 | `state_done` |
| 4 | u8 | `error_flags`，bit0 超时，bit1 参数未配置，bit2 测试动作错误，bit3 流程切换错误，bit4 自动修补失败 |
| 5 | u8 | active source，`0=USART, 1=USB, 2=none` |
| 6 | u8 | 上/下台阶电机在线数，统计 FDCAN2 ID1..6 + FDCAN1 ID5..6，共 8 个 |
| 7 | u8 | `test_action` |
| 8 | u32 | 当前状态已运行时间 ms |
| 12 | u32 | 最近更新时间 ms |
| 16 | f32 | `leg_pos_mm[0]`，立杆1 前右 |
| 20 | f32 | `leg_pos_mm[1]`，立杆2 后右 |
| 24 | f32 | `leg_pos_mm[2]`，立杆3 后左 |
| 28 | f32 | `leg_pos_mm[3]`，立杆4 前左 |
| 32 | f32 | `leg_target_mm[0]` |
| 36 | f32 | `leg_target_mm[1]` |
| 40 | f32 | `leg_target_mm[2]` |
| 44 | f32 | `leg_target_mm[3]` |
| 48 | f32 | `drive_pos_mm[0]`，当前驱动轮组左侧位置；前驱测试为前左，后驱测试为后左，完整流程为前后左平均 |
| 52 | f32 | `drive_pos_mm[1]`，当前驱动轮组右侧位置；前驱测试为前右，后驱测试为后右，完整流程为前后右平均 |
| 56 | f32 | `drive_target_mm[0]`，当前驱动轮组左侧目标 |
| 60 | f32 | `drive_target_mm[1]`，当前驱动轮组右侧目标 |
| 64 | u8 | `flow`，`0=UPSTAIRS, 1=DOWNSTAIRS` |
| 65 | u8 | `status_flags`: bit0=motor_output_active, bit1=leg_busy, bit2=drive_busy, bit3=test_chassis_active, bit4=pending_step, bit5=pending_auto, bit6=pending_test_action, bit7=ready_for_next |
| 66 | u8 | `leg_reached_mask`: bit0..3 = leg1..4 reached target tolerance |
| 67 | u8 | `drive_reached_mask`: bit0..1 = left/right drive reached target tolerance；前驱测试只判断前驱轮，后驱测试只判断后驱轮，完整流程要求同侧前后驱动轮都到位 |
| 68 | u8 | `summary_state`: `0=NOT_READY, 1=READY, 2=RUNNING, 3=DONE, 4=ERROR, 5=PAUSED` |

`ready_for_next` is set only when the climb state is done/idle/done-state, no pending command is queued, no chassis test action is active, and all leg/drive reached masks are complete.

上位机主控制建议优先看 `summary_state`：`READY` 可发下一条单步/动作命令，`RUNNING` 继续轮询，`DONE` 表示完整上/下台阶流程结束，`ERROR` 需要停止并读取 `error_flags`，`PAUSED` 表示自动流程在中断点等待 `CLIMB_AUTO_RESUME`，`NOT_READY` 表示尚未到达可安全继续的待机/到位状态。

上/下台阶状态判断建议：

- `state=22`：流程完成。
- `state=23`：错误，读取 `error_flags`。
- `state=24`：执行前立杆预备状态，四根立杆目标 `-30mm`；上台阶到位后进入 `state=25`，下台阶到位后进入 `STEP_01_*`。
- `state=25`：上台阶执行前底盘预备状态，麦轮底盘前进 `30mm`，完成后再进入 `STEP_01_*`。
- `state=26`：上台阶自动触发前接近状态，麦轮底盘直线前进，检测到 `x < 35mm` 后进入 `PREPARE_ALL_LEGS_MINUS_30`。
- `state=27`：下台阶自动触发前接近状态，麦轮底盘直线前进，检测到 `h > 65mm` 的高度突变后进入 `DOWN_PREPARE_CHASSIS_FORWARD_5`。
- `state=28`：下台阶触发后的预备状态，麦轮底盘再前进 `5mm` 后进入 `PREPARE_ALL_LEGS_MINUS_30`。
- `state=29`：自动下台阶流程首次故障后的单片机内部修补状态，四根立杆收回到 `-30mm`。自动上台阶失败不会进入该状态，会直接进入 `ERROR`。
- `state=30`：自动下台阶流程首次故障后的单片机内部修补状态，麦轮底盘后退 `200mm`；完成后按原下台阶流程重新自动运行。
- 手动步进时，通常等待 `state_done=1` 后再发下一次 step 命令。
- 上台阶步进用 `CLIMB_UP_STEP`，下台阶步进用 `CLIMB_DOWN_STEP`。
- 如果你明确要强制推进，step 命令会直接进入下一状态，不要求 `state_done=1`。

### 6.8 上位机错误处理建议

通用原则：上位机收到任一 `error_flags != 0` 后，应先停止继续下发新的运动目标，再按模块查询专属状态包确认目标值、实际值和状态位。`ROBOT_GET_STATUS` 适合做 10..50Hz 总览轮询；出现异常时再补查 `CHS/ARM/TOOL/CLIMB_GET_STATUS`。

| 错误来源 | 触发字段 | 上位机建议处理 |
|---|---|---|
| USB 底盘超时 | `ROBOT.error_flags.bit0` 或 `CHS.error_flags.bit0` | 立即停止发速度目标，发送 `CHS_STOP` 或 `SYS_STOP`；确认 USB 链路恢复后，重新 `SYS_SWITCH_SOURCE USB`、`CHS_ENABLE`，速度模式下保持 `<100ms` 周期发送 `CHS_SET_VEL`。 |
| IMU 离线 | `ROBOT.error_flags.bit3` 或 `CHS.error_flags.bit1` | 禁止继续使用 `WORLD_*`、`NO_YAW` 锁航向和自动台阶流程；提示检查 IMU/INS，必要时切到机器人系低速手动模式。 |
| 底盘急停 | `ROBOT.error_flags.bit4` 或 `CHS.error_flags.bit2` | 显示急停状态；若是用户主动停止，可等待下一条 `CHS_SET_VEL`/`CHS_SET_POS` 解除；若非主动停止，先人工确认现场安全。 |
| 底盘电机离线 | `CHS.error_flags.bit3/bit4` | 停止底盘运动；提示检查 FDCAN1 电机反馈、电源和 ID，在线数恢复前不要进入自动流程。 |
| 机械臂 IK 不可达 | `ARM.ik_status=3` 或 `ARM.error_flags.bit2` | 调整 `target_z_mm` 或切换工具/姿态；底盘继续负责 x/y。 |
| 机械臂限位 | `ARM.ik_status=4` 或 `ARM.error_flags.bit1` | 按 `ik_reason` 判断 J1/J2/J3 限位，调整 yaw、高度或工具状态。 |
| 机械臂参数异常 | `ARM.ik_status=1/2/8` 或 `ARM.error_flags.bit3` | 停止继续下发目标，检查 `tool/state/target_z_mm/approach_yaw_rad` 是否非法、NaN/Inf 或方向参数错误。 |
| 机械臂不安全/不支持 | `ARM.ik_status=5/6/7` 或 `ARM.error_flags.bit4/bit5` | 调整 yaw、长连杆姿态或工具状态；当前 USB 状态包不包含机械臂电机在线位，不能按此字段判断电机离线。 |
| 工具/机械臂 IK 错误 | `TOOL_GET_STATUS` 与 `ARM_GET_STATUS` 同布局；检查 `ik_status` 和 `error_flags` | 按机械臂 IK/限位规则处理；`TOOL_SET_MODE` 只是按工具编号换算目标点，平面移动仍由底盘负责。 |
| 工具执行器无独立反馈 | 吸盘/夹爪只由 `TOOL_ENABLE/DISABLE f0=tool` 输出开关 | 状态包不包含吸盘真空、夹爪角度或工具到位位；如怀疑卡滞，关闭具体执行器并人工确认。 |
| 上/下台阶超时 | `CLIMB.error_flags.bit0` | 自动上台阶超时直接进入 `ERROR`，不再二次尝试；自动下台阶首次超时仍会先执行一次修补：收腿到 `-30mm`、底盘后退 `200mm`、按原下台阶流程重跑。若仍失败，上位机收到 `ERROR` 后再人工/策略处理。 |
| 上/下台阶参数未配置 | `CLIMB.error_flags.bit1` | 禁止执行台阶流程；提示检查 `R2_CLIMB_PARAM_CONFIGURED`、count/mm、方向宏和机械参数。 |
| 上/下台阶测试动作错误 | `CLIMB.error_flags.bit2` | 检查 `test_action` 是否合法，以及该动作是否需要底盘控制器/激光条件；发送 `CLIMB_STOP` 后重新选择动作。 |
| 上/下台阶流程切换错误 | `CLIMB.error_flags.bit3` | 当前流程未结束时不要从上台阶切到下台阶或反向切换；先 `CLIMB_STOP`，等待 `state=IDLE` 后再启动目标流程。 |
| 下台阶自动修补失败 | `CLIMB.error_flags.bit4` | 单片机已经修补并重跑一次仍失败，或修补过程自身失败；停止继续自动下发，交由上位机/人工确认现场状态。自动上台阶已取消内部修补，首次故障会直接报 `ERROR`。 |
| 当前控制源不新鲜 | `ROBOT.error_flags.bit7` | 提示当前控制源保活不足；若使用 USB，恢复周期发送命令或切换到 USART；自动流程中不要忽略此位。 |

## 7. 最小可用流程汇总

USB 上台阶手动步进最小流程：

```text
A5 5A 10 02 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 38 4B FF
A5 5A 00 51 07 C3 FF
A5 5A 00 53 C6 42 FF
A5 5A 00 56 C5 82 FF
```

之后每次要推进一个状态，继续发送：

```text
A5 5A 00 53 C6 42 FF
A5 5A 00 56 C5 82 FF
```

USB 上台阶自动执行最小流程：

```text
A5 5A 10 02 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 38 4B FF
A5 5A 00 51 07 C3 FF
A5 5A 00 54 04 03 FF
A5 5A 00 56 C5 82 FF
```

USB 下台阶手动步进最小流程：

```text
A5 5A 10 02 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 38 4B FF
A5 5A 00 51 07 C3 FF
A5 5A 00 58 01 03 FF
A5 5A 00 56 C5 82 FF
```

之后每次要推进一个下台阶状态，继续发送：

```text
A5 5A 00 58 01 03 FF
A5 5A 00 56 C5 82 FF
```

USB 下台阶自动执行最小流程：

```text
A5 5A 10 02 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 38 4B FF
A5 5A 00 51 07 C3 FF
A5 5A 00 59 C1 C2 FF
A5 5A 00 56 C5 82 FF
```

USB 多状态查询顺序测试：

```text
A5 5A 00 26 21 83 FF
A5 5A 00 36 ED 82 FF
A5 5A 00 46 09 83 FF
A5 5A 00 56 C5 82 FF
```

期望回包顺序：

```text
0x26 -> 0x36 -> 0x46 -> 0x56
```
