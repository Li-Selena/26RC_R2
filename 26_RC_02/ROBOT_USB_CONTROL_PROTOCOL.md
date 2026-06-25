# Robot USB / USART 控制命令与使用方法

更新时间：2026-06-17

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
| `0x00` | `SYS_DISABLE` | 空或 4float | 停止 USB 底盘、上/下台阶，机械臂保持当前位置，工具保持，关闭底盘/机械臂/工具使能标志 |
| `0x01` | `SYS_ENABLE` | 空或 4float | 打开底盘、机械臂、工具使能标志 |
| `0x02` | `SYS_SWITCH_SOURCE` | `f0=0/1` | 切换控制源，`0=USART`，`1=USB` |
| `0x05` | `SYS_STOP` | 空或 4float | 停止 USB 底盘和上/下台阶，工具 stop，机械臂保持当前位置 |
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
2. 如需统一打开底盘、机械臂、工具，发 `SYS_ENABLE`。
3. 出现异常或要中断动作，发 `SYS_STOP` 或对应模块 `STOP`。
4. 若要回到串口遥控，发 `SYS_SWITCH_SOURCE USART`。

### 2.2 Chassis 底盘

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x10` | `CHS_DISABLE` | 空或 4float | 底盘失能并停止 USB 底盘控制器 |
| `0x11` | `CHS_ENABLE` | 空或 4float | 底盘使能 |
| `0x12` | `CHS_SET_MODE` | `f0=mode` | 设置底盘模式 `0..7` |
| `0x13` | `CHS_SET_VEL` | `f0=vx, f1=vy, f2=vw, f3=lock_yaw_deg` | 设置速度目标 |
| `0x14` | `CHS_SET_POS` | `f0=dx, f1=dy, f2=dyaw, f3=0` | 设置一次位置位移目标 |
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
3. 速度控制：先 `CHS_SET_MODE` 到 `0..3`，再周期发送 `CHS_SET_VEL`。
4. 位置控制：先 `CHS_SET_MODE` 到 `4..7`，再发送一次 `CHS_SET_POS`。
5. 速度控制有 `100ms` 看门狗，`CHS_SET_VEL` 发送间隔应小于 `100ms`。
6. 位置运动执行中不要重复发送新位置命令，先查询 `pos_state`，或发 `CHS_STOP` 后再下新目标。

### 2.3 Arm 机械臂

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x20` | `ARM_DISABLE` | 空或 4float | 机械臂失能并保持当前位置 |
| `0x21` | `ARM_ENABLE` | 空或 4float | 机械臂使能 |
| `0x23` | `ARM_SET_TARGET` | `f0=x, f1=y, f2=z, f3=0` | 设置末端目标点，单位 mm |
| `0x25` | `ARM_STOP` | 空或 4float | 保持当前编码器位置 |
| `0x26` | `ARM_GET_STATUS` | 空或 4float | 查询机械臂状态 |

限制：

- 坐标系：正前方为 `+X`，左侧为 `+Y`，竖直向上为 `+Z`。
- J1 角度定义：从 `+X` 向 `+Y` 转动为正，范围到 `+180deg`；从 `+X` 向 `-Y` 转动为负，范围到 `-180deg`。
- 固件不再对 `x/y/z` 做矩形 clamp；超出机械臂安全运动空间的目标会被拒绝，保持上一组安全输出或当前位置。
- 目标点必须通过当前 IK 安全空间检查：`theta2/j3` 联动保护、50mm 内缩目标空间、低位 J1 锁平面规则。
- `z > 250mm` 时 J1 可旋转；`z <= 250mm` 时目标必须在模型 `XZ` 平面内，容差 `|y| <= 5mm`。
- J1 输出限制为 `[-180, 180] deg`；J1 正负由 `atan2(y, x)` 决定，`+Y` 一侧为正角，`-Y` 一侧为负角。目标在 `-X` 后方边界时，固件按上一组有效 J1 输出所在侧选择 `+180` 或 `-180`，避免边界符号抖动。
- USB 机械臂命令超时：`300ms`

常用帧：

```text
ARM_DISABLE                 A5 5A 00 20 23 03 FF
ARM_ENABLE                  A5 5A 00 21 E3 C2 FF
ARM_STOP                    A5 5A 00 25 20 C3 FF
ARM_GET_STATUS              A5 5A 00 26 21 83 FF
ARM_SET_TARGET 200,0,180    A5 5A 10 23 00 00 48 43 00 00 00 00 00 00 34 43 00 00 00 00 2D 25 FF
```

使用方法：

1. 切到 USB：`SYS_SWITCH_SOURCE USB`。
2. 打开机械臂：`ARM_ENABLE`，或直接 `SYS_ENABLE`。
3. 发送 `ARM_SET_TARGET`。
4. 查询 `ARM_GET_STATUS`，检查 IK 状态、目标角和实际角误差。
5. 停止时发 `ARM_STOP`。

### 2.4 Tool 工具

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x30` | `TOOL_DISABLE` | 空或 4float | 工具失能并保持 |
| `0x31` | `TOOL_ENABLE` | 空或 4float | 工具使能 |
| `0x32` | `TOOL_SET_MODE` | `f0=dev` | 选择工具，`0=夹爪/Clamp`，`1=吸盘/Chuck` |
| `0x33` | `TOOL_ACTION` | `f0=act` | 执行动作，`0=闭合/关闭`，`1=打开/张开` |
| `0x35` | `TOOL_STOP` | 空或 4float | 工具停止 |
| `0x36` | `TOOL_GET_STATUS` | 空或 4float | 查询工具状态 |

USB 工具动作超时：`2000ms`。

常用帧：

```text
TOOL_DISABLE                A5 5A 00 30 EF 02 FF
TOOL_ENABLE                 A5 5A 00 31 2F C3 FF
TOOL_STOP                   A5 5A 00 35 EC C2 FF
TOOL_GET_STATUS             A5 5A 00 36 ED 82 FF
TOOL_SET_MODE clamp=0       A5 5A 10 32 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 88 8B FF
TOOL_SET_MODE chuck=1       A5 5A 10 32 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 37 5F FF
TOOL_ACTION close=0         A5 5A 10 33 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 18 DA FF
TOOL_ACTION open=1          A5 5A 10 33 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 A7 0E FF
```

使用方法：

1. 切到 USB：`SYS_SWITCH_SOURCE USB`。
2. 打开工具：`TOOL_ENABLE`，或直接 `SYS_ENABLE`。
3. 发 `TOOL_SET_MODE` 选择吸盘或夹爪。
4. 发 `TOOL_ACTION` 执行闭合或打开。
5. 查询 `TOOL_GET_STATUS` 判断是否完成。

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
3. 周期查询 `YAW_TUNE_GET_STATUS`，观察 `state/state_name`、误差和 PID 结果。
4. 需要中断时发 `YAW_TUNE_STOP`。

### 2.7 Climb 上/下台阶

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x50` | `CLIMB_DISABLE` | 空或 4float | 停止并复位上/下台阶状态机 |
| `0x51` | `CLIMB_ENABLE` | 空或 4float | 使能上/下台阶状态机 |
| `0x52` | `CLIMB_SET_CTRL` | `f0=enable, f1=step, f2=auto, f3=0` | 模拟 USART 三个控制字节 |
| `0x53` | `CLIMB_STEP` | 空或 4float | 上台阶一次性步进请求，每发一次推进一个状态 |
| `0x54` | `CLIMB_AUTO` | 空或 4float | 上台阶一次性自动执行请求，自动跑完整流程 |
| `0x55` | `CLIMB_STOP` | 空或 4float | 停止并复位上/下台阶状态机 |
| `0x56` | `CLIMB_GET_STATUS` | 空或 4float | 查询上/下台阶状态 |
| `0x57` | `CLIMB_TEST_ACTION` | `f0=动作ID` | 单独执行一个上台阶调试动作，不改变完整状态机流程 |
| `0x58` | `CLIMB_DOWNSTAIRS_STEP` | 空或 4float | 下台阶一次性步进请求，每发一次推进一个状态 |
| `0x59` | `CLIMB_DOWNSTAIRS_AUTO` | 空或 4float | 下台阶一次性自动执行请求，自动跑完整流程 |

常用帧：

```text
CLIMB_DISABLE               A5 5A 00 50 C7 02 FF
CLIMB_ENABLE                A5 5A 00 51 07 C3 FF
CLIMB_STEP                  A5 5A 00 53 C6 42 FF
CLIMB_AUTO                  A5 5A 00 54 04 03 FF
CLIMB_STOP                  A5 5A 00 55 C4 C2 FF
CLIMB_GET_STATUS            A5 5A 00 56 C5 82 FF
CLIMB_DOWNSTAIRS_STEP       A5 5A 00 58 01 03 FF
CLIMB_DOWNSTAIRS_AUTO       A5 5A 00 59 C1 C2 FF
CLIMB_SET_CTRL enable       A5 5A 10 52 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 29 77 FF
CLIMB_SET_CTRL step edge    A5 5A 10 52 00 00 80 3F 00 00 80 3F 00 00 00 00 00 00 00 00 30 65 FF
CLIMB_SET_CTRL auto edge    A5 5A 10 52 00 00 80 3F 00 00 00 00 00 00 80 3F 00 00 00 00 EC 7C FF
```

`CLIMB_SET_CTRL` 与 `CLIMB_STEP` 的区别：

- `CLIMB_SET_CTRL` 是电平输入，等价于 USART 的 `climb_enable/climb_step/climb_auto` 三个字节。
- `CLIMB_SET_CTRL` 的 `step` 和 `auto` 只识别 `0->1` 上升沿；一直重复发送 `f1=1` 不会连续步进。
- `CLIMB_STEP` / `CLIMB_AUTO` 明确选择上台阶流程。
- `CLIMB_DOWNSTAIRS_STEP` / `CLIMB_DOWNSTAIRS_AUTO` 明确选择下台阶流程。
- step 命令是 USB 一次性命令；每收到一帧就触发一次步进，适合按钮点动。
- auto 命令是 USB 一次性命令；在 `IDLE/DONE` 从头执行，在已完成的中间状态从下一状态继续，在执行中的状态会到位后继续自动执行。
- 正在执行一种流程时不允许切换到另一种流程；误切会进入 `ERROR` 并置位 `flow_switch` 错误。
- 如果只是做 USB 步进调试，推荐使用 `CLIMB_ENABLE` + 多次 `CLIMB_STEP`。
- USB 上/下台阶命令只有当前控制源为 USB 时生效；USART 上台阶命令只有 `UU_flag=0`、当前源为 USART 时输出到电机。

## 3. 上台阶动作流程

上台阶状态编号：

| state | 名称 | 动作 |
|---:|---|---|
| `0` | `IDLE` | 待机保持，四根立杆相对上电零位向上 `10mm`，目标 `-10mm`，上台阶使能后位置环开启 |
| `1..11` | `STEP_XX_*` | 按执行相邻关系压缩后的上台阶主流程；展开记录见 `CLIMB_ACTION_SEQUENCE.yaml` |
| `22` | `DONE` | 完成并保持待机位 `-10mm` |
| `23` | `ERROR` | 错误 |

动作参数：

| 参数 | 当前值 |
|---|---:|
| 上电接地位置 | `0mm` |
| 待机保持位置 | `-10mm` |
| 离地最大高度 | `10mm` |
| 撑地最大高度 | `240mm` |
| 高位抬升高度 | `220mm` |
| 前腿离地待机位 | `-10mm` |
| 前腿支撑过渡位 | `30mm` |
| 上台阶动作文件 | `PC_USB_Serial_Tool/upstairs_v02.json` |
| 保存动作总数 | `83` |
| 压缩后固件状态数 | `11` |
| 小驱动轮驱动块 | `120mm / 360mm / 1530mm` |
| 小驱动轮超时估算 | `80mm/s + 1500ms` |
| 底盘最终前进 | `200mm` |
| 立杆到位容差 | `3mm` |
| 小驱动轮到位容差 | `5mm` |

状态完成条件：

| 状态类型 | 完成条件 |
|---|---|
| 立杆目标动作 | 对应立杆到目标位置 `±3mm` |
| `DRIVE_FORWARD_xxx` | ID5/ID6 小驱动轮完成本段 `xxx mm ± 5mm` |
| `CHASSIS_FORWARD_xxx` | 底盘位置模式返回 `R2_POS_DONE` |

下台阶压缩结构：

| 参数 | 当前值 |
|---|---:|
| 下台阶动作文件 | `PC_USB_Serial_Tool/downstairs_v01.json` |
| 保存动作总数 | `69` |
| 压缩后固件状态数 | `14` |
| 展开记录 | `DOWNSTAIRS_ACTION_SEQUENCE.yaml` |
| 后驱动轮后退块 | `120mm / 300mm / 500mm / 1500mm` |
| 后驱动轮超时估算 | `80mm/s + 1500ms` |
| 结构体 | `s_downstairs_steps` |

`s_downstairs_steps` 是下台阶流程数据表；`CLIMB_AUTO` / `CLIMB_STEP` 执行上台阶 `s_main_steps`，`CLIMB_DOWNSTAIRS_AUTO` / `CLIMB_DOWNSTAIRS_STEP` 执行下台阶 `s_downstairs_steps`。

独立调试动作 `CLIMB_TEST_ACTION f0=动作ID`：

| 动作ID | 名称 | 动作 |
|---:|---|---|
| `1` | `ALL_LEGS_220` | 四根立杆目标设为 `220mm` |
| `2` | `ALL_LEGS_UP_10` | 四根立杆基于当前位置上升 `10mm` |
| `3` | `ALL_LEGS_DOWN_10` | 四根立杆基于当前位置下降 `10mm` |
| `4` | `DRIVE_FORWARD_30` | 后方小驱动轮前进 `30mm` |
| `5` | `DRIVE_FORWARD_10` | 后方小驱动轮前进 `10mm` |
| `6` | `DRIVE_BACKWARD_10` | 后方小驱动轮后退 `10mm` |
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
| `17` | `DRIVE_BACKWARD_30` | 后方小驱动轮后退 `30mm` |
| `18` | `DRIVE_FORWARD_500` | 后方小驱动轮前进 `500mm` |
| `19` | `DRIVE_BACKWARD_500` | 后方小驱动轮后退 `500mm` |
| `20` | `CHASSIS_BACKWARD_100` | 底盘麦轮后退 `100mm` |
| `21` | `CHASSIS_FORWARD_300` | 底盘麦轮前进 `300mm` |
| `22` | `CHASSIS_BACKWARD_300` | 底盘麦轮后退 `300mm` |

尺度定义：

- 上电位置，即辅助轮和驱动轮接触地面的位置，为 `0mm`。
- 正方向表示立杆伸出、抬升底盘。
- 负方向表示立杆收回、离地。
- 离地目标禁止超过 `10mm`，即负向目标不能小于 `-10mm`。
- 撑地目标禁止超过 `240mm`，即正向目标不能大于 `240mm`。

USB 步进流程：

```text
1. 切换 USB 控制源
   A5 5A 10 02 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 38 4B FF

2. 使能上台阶
   A5 5A 00 51 07 C3 FF

3. 每次步进发送 CLIMB_STEP
   A5 5A 00 53 C6 42 FF

4. 每次步进后查询状态，等待 state_done=1 再发下一次
   A5 5A 00 56 C5 82 FF

5. 当前完整顺序已压缩为 `11` 个固件状态：连续立杆 `10mm` 动作合并为 `30mm` 目标，连续后驱动轮前进合并为 `1530mm`，重复后腿下压和底盘前进也各合并为一段

6. 查询状态，确认 state=22 或 state_name=DONE
   A5 5A 00 56 C5 82 FF
```

USB 自动流程：

```text
1. SYS_SWITCH_SOURCE USB
2. CLIMB_ENABLE
3. CLIMB_AUTO
4. 周期查询 CLIMB_GET_STATUS
5. state=22 表示完成，state=23 表示错误
```

可直接发送：

```text
SYS_SWITCH_SOURCE USB        A5 5A 10 02 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 38 4B FF
CLIMB_ENABLE                 A5 5A 00 51 07 C3 FF
CLIMB_AUTO                   A5 5A 00 54 04 03 FF
CLIMB_GET_STATUS             A5 5A 00 56 C5 82 FF
CLIMB_STOP                   A5 5A 00 55 C4 C2 FF
```

## 4. 上台阶控制对象和电机链路

上台阶电机全部走 `FDCAN2`。四个立杆升降电机按新车头方向定义，从右上角开始顺时针编号 `1..4`，编号即电机 ID。后两根立杆带的小驱动轮从车尾看，左侧为 ID `5`，右侧为 ID `6`。

| FDCAN2 ID | 控制对象 | 电机 | 位置 |
|---:|---|---|---|
| `1` | 立杆 1 | M3508 | 新车头方向右上角，前右；立杆向下/撑地为逆时针 |
| `2` | 立杆 2 | M3508 | 顺时针第 2 个，后右；立杆向下/撑地为顺时针 |
| `3` | 立杆 3 | M3508 | 顺时针第 3 个，后左；立杆向下/撑地为逆时针 |
| `4` | 立杆 4 | M3508 | 顺时针第 4 个，前左；立杆向下/撑地为顺时针 |
| `5` | 小驱动轮左 | M2006 | 从车尾看左侧；前进为逆时针 |
| `6` | 小驱动轮右 | M2006 | 从车尾看右侧；前进为顺时针 |

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
| 小驱动轮减速比 | `36` |
| 小驱动轮比例 | `1466.772 count/mm` |

方向参数约定：`+1` 对应立杆向下/撑地时电机逆时针，`-1` 对应立杆向下/撑地时电机顺时针。

| 参数 | 当前值 | 立杆向下/撑地旋向 |
|---|---:|---|
| `R2_CLIMB_LEG1_DIR` | `+1` | 逆时针 |
| `R2_CLIMB_LEG2_DIR` | `-1` | 顺时针 |
| `R2_CLIMB_LEG3_DIR` | `+1` | 逆时针 |
| `R2_CLIMB_LEG4_DIR` | `-1` | 顺时针 |
| `R2_CLIMB_DRIVE_LEFT_DIR` | `+1` | ID5 前进为逆时针 |
| `R2_CLIMB_DRIVE_RIGHT_DIR` | `-1` | ID6 前进为顺时针 |

方向检查：

- 正向向下/撑地目标会使 1/3 号立杆目标 count 增加，使 2/4 号立杆目标 count 减少。
- 若实车上 1/3 号正电流不是逆时针，或 2/4 号负电流不是顺时针，需要先修正对应 `R2_CLIMB_LEGx_DIR`，不要直接跑完整行程。
- 驱动轮正向前进目标会使 ID5 目标 count 增加，使 ID6 目标 count 减少。
- 若实车上 ID5 正电流不是逆时针，或 ID6 负电流不是顺时针，需要先修正对应 `R2_CLIMB_DRIVEx_DIR`。
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
| `drive_pos_mm[2]` | 两个后方小驱动轮当前位置 |
| `drive_target_mm[2]` | 两个后方小驱动轮目标位置 |
| `leg_current[4]` | 四根立杆输出电流 |
| `drive_current[2]` | 两个小驱动轮输出电流 |

## 5. USART 遥控控制帧

USART 使用 `UART10_Receive()` 解析，帧格式：

| 字段 | 长度 | 说明 |
|---|---:|---|
| HEAD | 1 | 固定 `0xA5` |
| DATA | 39 | 数据区 |
| CHECKSUM | 1 | DATA 39 字节累加和低 8 位 |
| TAIL | 1 | 固定 `0x5A` |

DATA 布局：

| 字节 | 含义 |
|---:|---|
| `0..7` | 底盘 8 种模式 one-hot，最多一个为 1；全 0 表示底盘停止 |
| `8` | `arm_flag`，`0=归零/保持`，`1=使能` |
| `9` | `UU_flag`，`0=USART源`，`1=USB源` |
| `10` | `tool_flag`，`0=夹爪`，`1=吸盘` |
| `11` | `tooluse_flag`，`0=闭合`，`1=打开` |
| `12` | `climb_enable`，`0=停止/复位`，`1=使能` |
| `13` | `climb_step`，上升沿触发一次步进 |
| `14` | `climb_auto`，上升沿触发自动执行 |
| `15..18` | `chassis param1`，VEL:`vx(m/s)` / POS:`dx(m)` |
| `19..22` | `chassis param2`，VEL:`vy(m/s)` / POS:`dy(m)` |
| `23..26` | `chassis param3`，VEL:`vw(rad/s)` / POS:`dyaw(rad)` |
| `27..30` | `arm_x(mm)` |
| `31..34` | `arm_y(mm)` |
| `35..38` | `arm_z(mm)` |

USART 机械臂目标点同样不再做旧矩形 clamp。`UU_flag=0` 且 `arm_flag=1` 时，`arm_x/y/z` 必须通过当前 IK 安全空间检查后才会写入 USART 机械臂目标；检查失败时保持上一组输出，并通过机械臂 IK 状态回包返回错误原因。

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
| 3 | u8 | `Arm_control_flag` |
| 4 | u8 | `Tool_control_flag` |
| 5 | u8 | 底盘电机在线数 |
| 6 | u8 | USB 超时标志，bit0 底盘，bit1 机械臂，bit2 工具 |
| 7 | u8 | `test_action`，最近一次/当前独立调试动作 ID，`0=NONE` |

### 6.2 `ARM_GET_STATUS` 回包 `0x26`, `LEN=40`

| offset | 类型 | 含义 |
|---:|---|---|
| 0 | u8 | `has_last_valid` |
| 1 | u8 | IK 状态，`0=OK, 1=UNREACHABLE, 2=UNSAFE, 3=PARAM_ERR` |
| 2 | u8 | 动作采纳状态，`0=APPLY_NEW, 1=HOLD_LAST, 2=KEEP_CURRENT` |
| 3 | u8 | reserved |
| 4 | f32 | `model_theta1(rad)` |
| 8 | f32 | `model_theta2(rad)` |
| 12 | f32 | `model_theta3(rad)` |
| 16 | f32 | `motor_j1_target(deg)` |
| 20 | f32 | `motor_j2_target(deg)` |
| 24 | f32 | `motor_j3_target(deg)` |
| 28 | f32 | `actual_j1_deg` |
| 32 | f32 | `actual_j2_deg` |
| 36 | f32 | `actual_j3_deg` |

### 6.3 `TOOL_GET_STATUS` 回包 `0x36`, `LEN=16`

| offset | 类型 | 含义 |
|---:|---|---|
| 0 | u8 | 当前工具，`0=夹爪, 1=吸盘` |
| 1 | u8 | `clamp.state` |
| 2 | u8 | `clamp.run_status`，`0=IDLE, 1=MOVING, 2=ERROR` |
| 3 | u8 | `clamp.safe_flag` |
| 4 | f32 | `clamp.real_angle` |
| 8 | u8 | `chuck.state` |
| 9 | u8 | `chuck.run_status` |
| 10 | u8 | `chuck.safe_flag` |
| 11 | u8 | active source，`0=USART, 1=USB` |
| 12 | f32 | `chuck.real_angle` |

### 6.4 `ROBOT_GET_STATUS` 回包 `0x46`, `LEN=160`

| offset | 类型 | 含义 |
|---:|---|---|
| 0 | u8 | 协议版本，当前为 `1` |
| 1 | u8 | active source，`0=USART, 1=USB, 2=none` |
| 2 | u8 | enable flags，bit0 底盘，bit1 机械臂，bit2 工具 |
| 3 | u8 | executing flags，bit0 底盘运动，bit1 位置运动，bit2 机械臂运动，bit3 工具运动，bit4 任一执行中 |
| 4 | u8 | error flags |
| 5 | u8 | online flags，bit0 USB近期有命令，bit1 USART近期有命令，bit2 IMU在线，bit3底盘电机在线，bit4机械臂电机在线 |
| 6 | u8 | USB 最近命令 CMD |
| 7 | u8 | USB 最近命令 LEN |
| 8 | u32 | USB 命令计数 |
| 12 | u32 | USB 最近命令 tick |
| 16 | u32 | USART 成功帧计数 |
| 20 | u32 | USART 最近帧 tick |
| 24 | u32 | USART 校验失败计数 |
| 28 | u8 | USART 最近校验是否 OK |
| 29 | u8 | USART 当前底盘模式 |
| 30 | u8 | 机械臂 IK 状态 |
| 31 | u8 | 机械臂动作采纳状态 |
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
| 80 | f32 | `arm_err_j1` |
| 84 | f32 | `arm_err_j2` |
| 88 | f32 | `arm_err_j3` |
| 92 | u8 | 当前工具 |
| 93 | u8 | 当前工具运行状态 |
| 94 | u8 | 底盘 `pos_state` |
| 95 | u8 | timeout flags |
| 96 | u8 | climb active source，`0=USART, 1=USB, 2=none` |
| 97 | u8 | climb `state` |
| 98 | u8 | climb `enabled` |
| 99 | u8 | climb `auto_run` |
| 100 | u8 | climb `state_done` |
| 101 | u8 | climb `error_flags`，bit0 超时，bit1 参数未配置，bit2 测试动作错误，bit3 流程切换错误 |
| 102 | u8 | climb `pending_step` |
| 103 | u8 | climb `pending_auto` |
| 104 | u8 | climb motor active |
| 105 | u8 | FDCAN2 上/下台阶电机在线数，统计 ID1..6 |
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

`ROBOT_GET_STATUS.error_flags`：

| bit | 含义 |
|---:|---|
| 0 | USB 底盘超时 |
| 1 | USB 机械臂超时 |
| 2 | USB 工具超时 |
| 3 | IMU 离线 |
| 4 | 底盘急停 |
| 5 | 机械臂 IK 非 OK |
| 6 | 工具错误 |
| 7 | 当前控制源命令不新鲜 |

### 6.5 `CLIMB_GET_STATUS` 回包 `0x56`, `LEN=68`

| offset | 类型 | 含义 |
|---:|---|---|
| 0 | u8 | `state` |
| 1 | u8 | `enabled` |
| 2 | u8 | `auto_run` |
| 3 | u8 | `state_done` |
| 4 | u8 | `error_flags`，bit0 超时，bit1 参数未配置，bit2 测试动作错误，bit3 流程切换错误 |
| 5 | u8 | active source，`0=USART, 1=USB, 2=none` |
| 6 | u8 | FDCAN2 上/下台阶电机在线数，统计 ID1..6 |
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
| 48 | f32 | `drive_pos_mm[0]`，从车尾看左侧小驱动轮 |
| 52 | f32 | `drive_pos_mm[1]`，从车尾看右侧小驱动轮 |
| 56 | f32 | `drive_target_mm[0]`，从车尾看左侧小驱动轮 |
| 60 | f32 | `drive_target_mm[1]`，从车尾看右侧小驱动轮 |
| 64 | u8 | `flow`，`0=UPSTAIRS, 1=DOWNSTAIRS` |

上/下台阶状态判断建议：

- `state=22`：流程完成。
- `state=23`：错误，读取 `error_flags`。
- 手动步进时，通常等待 `state_done=1` 后再发下一次 step 命令。
- 上台阶步进用 `CLIMB_STEP`，下台阶步进用 `CLIMB_DOWNSTAIRS_STEP`。
- 如果你明确要强制推进，step 命令会直接进入下一状态，不要求 `state_done=1`。

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
