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

在 shell 里可以让上位机先等待当前上台阶动作完成，再发送下一条 USB 命令：

```text
climb_wait 20
climb_step_wait 40
climb_auto_wait 180
climb_wait_then CLIMB_STEP
climb_tests
climb_test CHASSIS_FORWARD_100
climb_test_wait FRONT_UP_10 10
climb_chassis_forward_100
climb_all_legs_220 5
```

等待逻辑会轮询 `CLIMB_GET_STATUS`，看到 `state_done=1`、`IDLE` 或 `DONE` 后继续；如果状态变成 `ERROR` 会直接报错。

## 3. 自动调参指令

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

## 4. 底盘指令示例

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
vx vy vw lock_yaw_deg
```

速度控制有 100ms 看门狗，持续运动时需要周期发送速度命令。

## 5. 单条命令发送

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
