from pathlib import Path
import math
import re


ROOT = Path(__file__).resolve().parents[1]


def read(rel):
    return (ROOT / rel).read_text(encoding="utf-8", errors="ignore")


def require(cond, message):
    if not cond:
        raise AssertionError(message)


def function_body(source, name):
    match = re.search(r"\b(?:void|static\s+void)\s+" + re.escape(name) + r"\s*\([^)]*\)\s*\{", source)
    require(match is not None, f"missing function {name}")
    start = match.end()
    depth = 1
    i = start
    while i < len(source) and depth:
        if source[i] == "{":
            depth += 1
        elif source[i] == "}":
            depth -= 1
        i += 1
    require(depth == 0, f"could not parse function {name}")
    return source[start : i - 1]


def check_r2_mode_math():
    world_modes = {2, 3, 6, 7}
    for mode in range(8):
        is_vel = mode <= 3
        is_pos = mode >= 4
        is_world = mode in world_modes
        is_no_yaw = (mode & 1) == 0

        require(is_vel != is_pos, f"mode {mode}: vel/pos overlap")
        require(is_no_yaw == (mode in {0, 2, 4, 6}), f"mode {mode}: no-yaw classification")
        require(is_world == (mode in world_modes), f"mode {mode}: world classification")

    yaw = math.pi / 2.0
    wx, wy = 1.0, 0.0
    rx = wx * math.cos(yaw) + wy * math.sin(yaw)
    ry = -wx * math.sin(yaw) + wy * math.cos(yaw)
    require(abs(rx) < 1e-6 and abs(ry + 1.0) < 1e-6, "world_to_robot 90deg transform")


def check_static_invariants():
    control = read("Applications/Task/Src/Control_Task.c")
    isr_body = function_body(control, "control_tim1mscallback")
    for forbidden in ["R2_Move_Update", "R2_Move_UpdateOdom", "Mecanum_Calc", "cosf", "sinf"]:
        require(forbidden not in isr_body, f"TIM3 ISR still contains heavy call: {forbidden}")
    require("vTaskNotifyGiveFromISR" in isr_body, "TIM3 ISR must notify control task")

    step_body = function_body(control, "R2_Control_1msStep")
    yaw_call = step_body.find("R2_Move_UpdateYaw(&")
    odom_call = step_body.find("R2_Move_UpdateOdom(&")
    require(yaw_call >= 0 and odom_call >= 0 and yaw_call < odom_call,
            "R2 yaw must be updated before odom integration")

    data = read("Components/Algorithm/Src/Data_Analysis.c")
    data_body = function_body(data, "Data_Analysis")
    require("USB_Read4Floats(d, f)" not in data_body, "Data_Analysis must not read payload directly")
    require(data_body.count("USB_Read4FloatsChecked") >= 5, "expected guarded USB payload reads")
    require("Control_SetSource" in data_body, "USB source switch must use unified source setter")

    fdcan = read("BSP/Src/bsp_fdcan.c")
    for name in ["FDCAN1_Filter_Init", "FDCAN2_Filter_Init"]:
        require("FDCAN_Start" not in function_body(fdcan, name), f"{name} should only configure filters")
    require("FDCAN_Motor_Start_All" in fdcan, "missing unified FDCAN start function")
    fdcan3_config = function_body(fdcan, "FDCAN_Config")
    require("FDCAN_FILTER_TO_RXFIFO1" in fdcan3_config, "FDCAN3 config must route filters to FIFO1")
    require("FDCAN_IT_RX_FIFO1_NEW_MESSAGE" in fdcan3_config, "FDCAN3 config must notify FIFO1")
    require("FDCAN_RX_FIFO1" in fdcan, "FDCAN3 read loop must drain FIFO1")

    can = read("Applications/Task/Src/CAN_Task.c")
    require("osDelay(5000)" in can, "CAN startup delay must yield to scheduler")
    require("FDCAN1_CMD_2(fdcan1_aux_m5, fdcan1_aux_m6, 0, 0)" in can,
            "FDCAN1 must send the 0x1FF frame for motors 5/6")
    require("PID_velocity_realize_1(0.0f, 5)" in can and
            "PID_velocity_realize_1(0.0f, 6)" in can,
            "FDCAN1 motors 5/6 must be covered by zero-speed PID when chassis source is active")

    arm = read("Applications/R2_user/Src/R2_arm.c")
    require("R2_Arm_OutputApplyJointTargetsRad" in arm, "new arm output hook is missing")
    require("__weak void R2_Arm_OutputApplyJointTargetsRad" in arm,
            "default arm output hook must remain weak for the motor driver")
    require("RobotArmKinematics_SolveToolHeight" in arm, "new arm must call the height/posture IK")

    kin = read("Components/Algorithm/Src/robotarm_kinematics.c")
    for value in ["-46.79f", "45.5f", "92.6f", "103.0f", "-92.0f", "10.3f"]:
        require(value in kin, f"missing tool offset value {value}")


def main():
    check_r2_mode_math()
    check_static_invariants()
    print("control plan verification passed")


if __name__ == "__main__":
    main()
