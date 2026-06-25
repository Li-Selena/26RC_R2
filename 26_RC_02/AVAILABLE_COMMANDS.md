# 可用命令清单

来源：

- `Components/Algorithm/Inc/Data_Analysis.h`
- `Components/Algorithm/Src/Data_Analysis.c`
- `ROBOT_USB_CONTROL_PROTOCOL.md`

## USB 帧格式

```text
A5 5A LEN CMD DATA... CRC_H CRC_L FF
```

- `HEAD1=0xA5`
- `HEAD2=0x5A`
- `TAIL=0xFF`
- CRC：CRC16/Modbus，初值 `0xFFFF`，多项式 `0xA001`
- CRC 计算范围：`HEAD1 HEAD2 LEN CMD DATA`
- CRC 发送顺序：高字节在前，低字节在后
- 带参数命令使用 `LEN=0x10`，`DATA=<4 个 float32 little-endian>`
- 无参数、停止、状态查询命令推荐使用 `LEN=0x00`

## System

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x00` | `SYS_DISABLE` | 空或 4float | 停止 USB 底盘、上/下台阶；机械臂和工具保持；关闭底盘、机械臂、工具使能 |
| `0x01` | `SYS_ENABLE` | 空或 4float | 打开底盘、机械臂、工具使能 |
| `0x02` | `SYS_SWITCH_SOURCE` | `f0=0/1` | 切换控制源，`0=USART`，`1=USB` |
| `0x05` | `SYS_STOP` | 空或 4float | 停止 USB 底盘和上/下台阶；工具停止；机械臂保持当前位置 |
| `0x06` | `SYS_GET_STATUS` | 空或 4float | 查询系统状态 |

```text
SYS_DISABLE              A5 5A 00 00 FB 02 FF
SYS_ENABLE               A5 5A 00 01 3B C3 FF
SYS_STOP                 A5 5A 00 05 F8 C2 FF
SYS_GET_STATUS           A5 5A 00 06 F9 82 FF
SYS_SWITCH_SOURCE USB    A5 5A 10 02 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 38 4B FF
SYS_SWITCH_SOURCE USART  A5 5A 10 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 87 9F FF
```

## Chassis 底盘

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x10` | `CHS_DISABLE` | 空或 4float | 底盘失能并停止 USB 底盘控制器 |
| `0x11` | `CHS_ENABLE` | 空或 4float | 底盘使能 |
| `0x12` | `CHS_SET_MODE` | `f0=mode` | 设置底盘模式 `0..7` |
| `0x13` | `CHS_SET_VEL` | `f0=vx, f1=vy, f2=vw, f3=lock_yaw_deg` | 设置速度目标 |
| `0x14` | `CHS_SET_POS` | `f0=dx, f1=dy, f2=dyaw, f3=0` | 设置一次位置位移目标 |
| `0x15` | `CHS_STOP` | 空或 4float | 停止 USB 底盘控制器 |
| `0x16` | `CHS_GET_STATUS` | 空或 4float | 查询底盘状态 |

底盘模式：

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

注意：

- 速度控制要先设置 `mode=0..3`，再周期发送 `CHS_SET_VEL`。
- 位置控制要先设置 `mode=4..7`，再发送一次 `CHS_SET_POS`。
- 速度控制有 `100ms` 看门狗，`CHS_SET_VEL` 发送间隔应小于 `100ms`。

## Arm 机械臂

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x20` | `ARM_DISABLE` | 空或 4float | 机械臂失能并保持当前位置 |
| `0x21` | `ARM_ENABLE` | 空或 4float | 机械臂使能 |
| `0x23` | `ARM_SET_TARGET` | `f0=x, f1=y, f2=z, f3=0` | 设置末端目标点，单位 mm |
| `0x25` | `ARM_STOP` | 空或 4float | 保持当前编码器位置 |
| `0x26` | `ARM_GET_STATUS` | 空或 4float | 查询机械臂状态 |

```text
ARM_DISABLE                 A5 5A 00 20 23 03 FF
ARM_ENABLE                  A5 5A 00 21 E3 C2 FF
ARM_STOP                    A5 5A 00 25 20 C3 FF
ARM_GET_STATUS              A5 5A 00 26 21 83 FF
ARM_SET_TARGET 200,0,180    A5 5A 10 23 00 00 48 43 00 00 00 00 00 00 34 43 00 00 00 00 2D 25 FF
```

限制：

- `x`: `[-500, 500] mm`
- `y`: `[-500, 500] mm`
- `z`: `[50, 500] mm`
- USB 机械臂命令超时：`300ms`

## Tool 工具

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x30` | `TOOL_DISABLE` | 空或 4float | 工具失能并保持 |
| `0x31` | `TOOL_ENABLE` | 空或 4float | 工具使能 |
| `0x32` | `TOOL_SET_MODE` | `f0=dev` | 选择工具，`0=夹爪/Clamp`，`1=吸盘/Chuck` |
| `0x33` | `TOOL_ACTION` | `f0=act` | 执行动作，`0=闭合/关闭`，`1=打开/张开` |
| `0x35` | `TOOL_STOP` | 空或 4float | 工具停止 |
| `0x36` | `TOOL_GET_STATUS` | 空或 4float | 查询工具状态 |

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

USB 工具动作超时：`2000ms`。

## Robot 总状态

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x46` | `ROBOT_GET_STATUS` | 空或 4float | 查询整车总状态 |

```text
ROBOT_GET_STATUS            A5 5A 00 46 09 83 FF
```

建议上位机把 `ROBOT_GET_STATUS` 作为主状态查询命令，频率建议 `10..50Hz`。

## Yaw Auto Tune 自动调参

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x47` | `YAW_TUNE_START` | 空或 `f0=pass_count` | 启动 yaw 自动调参；空 payload 使用默认轮次 |
| `0x48` | `YAW_TUNE_STOP` | 空或 4float | 停止 yaw 自动调参，并停止 USB 底盘 |
| `0x49` | `YAW_TUNE_GET_STATUS` | 空或 4float | 查询 yaw 自动调参状态 |

```text
YAW_TUNE_START default    A5 5A 00 47 C9 42 FF
YAW_TUNE_START pass=1     A5 5A 10 47 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 BD 69 FF
YAW_TUNE_STOP             A5 5A 00 48 CD 02 FF
YAW_TUNE_GET_STATUS       A5 5A 00 49 0D C3 FF
```

注意：

- `YAW_TUNE_START` 只有当前控制源为 USB、且底盘已使能时生效。
- 启动调参会解除 USB 底盘速度看门狗；停止调参会同时 `R2_Move_Stop()`。
- `YAW_TUNE_GET_STATUS` 状态回包为 `CMD=0x49`。

## Climb 上/下台阶

| CMD | 名称 | DATA | 作用 |
|---:|---|---|---|
| `0x50` | `CLIMB_DISABLE` | 空或 4float | 停止并复位上/下台阶状态机 |
| `0x51` | `CLIMB_ENABLE` | 空或 4float | 使能上/下台阶状态机 |
| `0x52` | `CLIMB_SET_CTRL` | `f0=enable, f1=step, f2=auto, f3=0` | 模拟 USART 三个控制字节 |
| `0x53` | `CLIMB_STEP` | 空或 4float | 上台阶一次性步进请求，每发一次推进一个状态 |
| `0x54` | `CLIMB_AUTO` | 空或 4float | 上台阶一次性自动执行请求，自动跑完整流程 |
| `0x55` | `CLIMB_STOP` | 空或 4float | 停止并复位上/下台阶状态机 |
| `0x56` | `CLIMB_GET_STATUS` | 空或 4float | 查询上/下台阶状态 |
| `0x57` | `CLIMB_TEST_ACTION` | `f0=动作ID` | 单独执行一个上台阶调试动作 |
| `0x58` | `CLIMB_DOWNSTAIRS_STEP` | 空或 4float | 下台阶一次性步进请求，每发一次推进一个状态 |
| `0x59` | `CLIMB_DOWNSTAIRS_AUTO` | 空或 4float | 下台阶一次性自动执行请求，自动跑完整流程 |

```text
CLIMB_DISABLE               A5 5A 00 50 C7 02 FF
CLIMB_ENABLE                A5 5A 00 51 07 C3 FF
CLIMB_STEP                  A5 5A 00 53 C6 42 FF
CLIMB_AUTO                  A5 5A 00 54 04 03 FF
CLIMB_STOP                  A5 5A 00 55 C4 C2 FF
CLIMB_GET_STATUS            A5 5A 00 56 C5 82 FF
CLIMB_DOWNSTAIRS_STEP       A5 5A 00 58 01 03 FF
CLIMB_DOWNSTAIRS_AUTO       A5 5A 00 59 C1 C2 FF
CLIMB_TEST_ACTION 1         A5 5A 10 57 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 78 64 FF
CLIMB_TEST_ACTION 16        A5 5A 10 57 00 00 80 41 00 00 00 00 00 00 00 00 00 00 00 00 C6 CD FF
CLIMB_SET_CTRL enable       A5 5A 10 52 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 29 77 FF
CLIMB_SET_CTRL step edge    A5 5A 10 52 00 00 80 3F 00 00 80 3F 00 00 00 00 00 00 00 00 30 65 FF
CLIMB_SET_CTRL auto edge    A5 5A 10 52 00 00 80 3F 00 00 00 00 00 00 80 3F 00 00 00 00 EC 7C FF
```

注意：

- `CLIMB_SET_CTRL` 是电平输入，等价于 USART 的 `climb_enable/climb_step/climb_auto`。
- `CLIMB_SET_CTRL` 的 `step` 和 `auto` 只识别 `0->1` 上升沿。
- `CLIMB_STEP` / `CLIMB_AUTO` 明确选择上台阶流程。
- `CLIMB_DOWNSTAIRS_STEP` / `CLIMB_DOWNSTAIRS_AUTO` 明确选择下台阶流程。
- `step` 是一次性命令，适合按钮点动。
- `auto` 是一次性命令；在 `IDLE/DONE` 从头执行，在已完成的中间状态从下一状态继续。
- `CLIMB_TEST_ACTION` 不推进完整状态机，只单独执行指定动作；动作完成后看 `CLIMB_GET_STATUS.state_done` 和各目标/当前位置。
- `IDLE` 和 `DONE` 在上台阶使能后会让四根立杆位置环保持待机位：相对上电零位向上 `10mm`，即目标 `-10mm`。
- USB 上/下台阶命令只有当前控制源为 USB 时生效；USART 上台阶命令只有 `UU_flag=0`、当前源为 USART 时写入控制器并输出到电机。
- USART 自动执行发出 `climb_auto` 上升沿后，仍需持续发送 `UU_flag=0, climb_enable=1` 的保活帧；超过 `300ms` 无 USART 帧会触发看门狗停止。

## 单条可复制命令

本节只列固定帧命令，不包含需要下发可变数据的命令，例如速度、位置、机械臂目标点、工具选择/动作参数、底盘模式等。

### SYS_DISABLE

```text
A5 5A 00 00 FB 02 FF
```

### SYS_ENABLE

```text
A5 5A 00 01 3B C3 FF
```

### SYS_STOP

```text
A5 5A 00 05 F8 C2 FF
```

### SYS_GET_STATUS

```text
A5 5A 00 06 F9 82 FF
```

### CHS_DISABLE

```text
A5 5A 00 10 37 03 FF
```

### CHS_ENABLE

```text
A5 5A 00 11 F7 C2 FF
```

### CHS_STOP

```text
A5 5A 00 15 34 C3 FF
```

### CHS_GET_STATUS

```text
A5 5A 00 16 35 83 FF
```

### ARM_DISABLE

```text
A5 5A 00 20 23 03 FF
```

### ARM_ENABLE

```text
A5 5A 00 21 E3 C2 FF
```

### ARM_STOP

```text
A5 5A 00 25 20 C3 FF
```

### ARM_GET_STATUS

```text
A5 5A 00 26 21 83 FF
```

### TOOL_DISABLE

```text
A5 5A 00 30 EF 02 FF
```

### TOOL_ENABLE

```text
A5 5A 00 31 2F C3 FF
```

### TOOL_STOP

```text
A5 5A 00 35 EC C2 FF
```

### TOOL_GET_STATUS

```text
A5 5A 00 36 ED 82 FF
```

### ROBOT_GET_STATUS

```text
A5 5A 00 46 09 83 FF
```

### YAW_TUNE_START

```text
A5 5A 00 47 C9 42 FF
```

### YAW_TUNE_STOP

```text
A5 5A 00 48 CD 02 FF
```

### YAW_TUNE_GET_STATUS

```text
A5 5A 00 49 0D C3 FF
```

### CLIMB_DISABLE

```text
A5 5A 00 50 C7 02 FF
```

### CLIMB_ENABLE

```text
A5 5A 00 51 07 C3 FF
```

### CLIMB_STEP

```text
A5 5A 00 53 C6 42 FF
```

### CLIMB_AUTO

```text
A5 5A 00 54 04 03 FF
```

### CLIMB_STOP

```text
A5 5A 00 55 C4 C2 FF
```

### CLIMB_GET_STATUS

```text
A5 5A 00 56 C5 82 FF
```

### CLIMB_DOWNSTAIRS_STEP

```text
A5 5A 00 58 01 03 FF
```

### CLIMB_DOWNSTAIRS_AUTO

```text
A5 5A 00 59 C1 C2 FF
```

## 推荐启动顺序

USB 控制底盘、机械臂或工具前，建议先发：

```text
SYS_SWITCH_SOURCE USB    A5 5A 10 02 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 38 4B FF
SYS_ENABLE               A5 5A 00 01 3B C3 FF
```

紧急停止或退出动作：

```text
SYS_STOP                 A5 5A 00 05 F8 C2 FF
```

回到 USART 遥控：

```text
SYS_SWITCH_SOURCE USART  A5 5A 10 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 87 9F FF
```
