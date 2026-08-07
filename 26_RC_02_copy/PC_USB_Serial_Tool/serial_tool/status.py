from __future__ import annotations

import struct
from typing import Any, Dict, Iterable, List

from .commands import command_name
from .protocol import UsbFrame, bytes_to_hex


RAD_TO_DEG = 57.29577951308232
SOURCE_NAMES = {0: "USART", 1: "USB", 2: "NONE"}
CHASSIS_MODES = {
    0: "ROBOT_NO_YAW_VEL",
    1: "ROBOT_VEL",
    2: "WORLD_NO_YAW_VEL",
    3: "WORLD_VEL",
    4: "ROBOT_NO_YAW_POS",
    5: "ROBOT_POS",
    6: "WORLD_NO_YAW_POS",
    7: "WORLD_POS",
}
CHASSIS_POS_STATE = {0: "IDLE", 1: "RUNNING", 2: "DONE"}
CLIMB_FLOW_NAMES = {0: "UPSTAIRS", 1: "DOWNSTAIRS"}
CLIMB_SUMMARY_STATES = {
    0: "NOT_READY",
    1: "READY",
    2: "RUNNING",
    3: "DONE",
    4: "ERROR",
    5: "PAUSED",
}
TASK_FLOW_IDS = {
    0: "NONE",
    1: "S1_UP_V2",
    2: "S1_DOWN_V2",
    4: "S1_UP_S2_DOWN_V1",
    5: "S1_DOWN_S2_UP_V1",
    6: "S1_DOWN_S2_DOWN_V1",
    7: "WEAPON_GRAB_V1",
    8: "THROW_BLOCK_V1",
    9: "WEAPON_DOCK_TEST_V1",
}
TASK_FLOW_STATES = {0: "IDLE", 1: "ISSUE", 2: "WAIT", 3: "DONE", 4: "ERROR"}
TASK_FLOW_ERRORS = {
    0: "NONE",
    1: "BAD_FLOW",
    2: "ARM",
    3: "CLIMB",
    4: "POSTURE",
    5: "PRECONDITION",
}
TASK_FLOW_OPS = {
    0: "NONE",
    1: "CLIMB_ACTION",
    2: "ARM_JOG",
    3: "TOOL_SET",
    4: "POSTURE_CHECK",
    5: "CLIMB_GATE",
    6: "ARM_ENABLE_HOME",
    7: "CLIMB_AUTO_PAUSE",
    8: "CLIMB_AUTO_RESUME",
    9: "ARM_WORKSPACE_SWITCH",
    10: "TOOL_OFF_HELD",
    11: "HOST_CHECKPOINT_WAIT",
}
TASK_FLOW_S1_ENDS = {0: "NONE", 1: "S1_UP_END", 2: "S1_DOWN_END"}
CLIMB_UPSTAIRS_STEP_NAMES = [
    "STEP_01_ALL_LEGS_220",
    "STEP_02_ALL_DRIVE_FORWARD_30_X3",
    "STEP_03_ALL_DRIVE_FORWARD_10_X3",
    "STEP_04_ALL_LEGS_DOWN_10_X5",
    "STEP_05_FRONT_MINUS_30",
    "STEP_06_REAR_DRIVE_FORWARD_30_X4",
    "STEP_07_FRONT_UP_10_X2",
    "STEP_08_ALL_LEGS_UP_10_X6",
    "STEP_09_ALL_DRIVE_FORWARD_200_PAUSE",
    "STEP_10_ALL_DRIVE_FORWARD_300_RESUME",
    "STEP_11_ALL_DRIVE_FORWARD_30_X9",
    "STEP_12_ALL_LEGS_ZERO",
    "STEP_13_ALL_LEGS_DOWN_10_X2",
    "STEP_14_CHASSIS_FORWARD_100_X2",
]
CLIMB_STEP_NAMES = CLIMB_UPSTAIRS_STEP_NAMES
CLIMB_DOWNSTAIRS_STEP_NAMES = [
    "DOWN_01_FRONT_UP_10_X23",
    "DOWN_02_REAR_ZERO",
    "DOWN_03_REAR_UP_10",
    "DOWN_04_ALL_LEGS_UP_10",
    "DOWN_05_ALL_DRIVE_FORWARD_500",
    "DOWN_06_ALL_DRIVE_FORWARD_30_X11",
    "DOWN_07_ALL_DRIVE_FORWARD_10_X2",
    "DOWN_08_ALL_LEGS_DOWN_10_X6",
    "DOWN_09_ALL_DRIVE_FORWARD_30_X3",
    "DOWN_10_ALL_DRIVE_FORWARD_10",
    "DOWN_11_REAR_UP_10_X19",
    "DOWN_12_ALL_DRIVE_FORWARD_30_X4",
    "DOWN_13_ALL_LEGS_ZERO",
    "DOWN_14_ALL_LEGS_DOWN_10_X3",
    "DOWN_15_CHASSIS_FORWARD_100",
]


def climb_state_name(state: int, flow: int = 0) -> str:
    if state == 0:
        return "IDLE"
    if state == 22:
        return "DONE"
    if state == 23:
        return "ERROR"
    if state == 24:
        return "PREPARE_ALL_LEGS_MINUS_30"
    if state == 25:
        return "UP_PREPARE_CHASSIS_FORWARD_30"
    if state == 26:
        return "UP_LASER_APPROACH_X_LT_35"
    if state == 27:
        return "DOWN_LASER_APPROACH_H_GT_65"
    if state == 28:
        return "DOWN_PREPARE_CHASSIS_FORWARD_5"
    if state == 29:
        return "RECOVER_ALL_LEGS_MINUS_30"
    if state == 30:
        return "RECOVER_CHASSIS_BACKWARD_200"
    names = CLIMB_DOWNSTAIRS_STEP_NAMES if flow == 1 else CLIMB_UPSTAIRS_STEP_NAMES
    if 1 <= state <= len(names):
        return names[state - 1]
    return "UNKNOWN"


CLIMB_STATES = {
    0: "IDLE",
    22: "DONE",
    23: "ERROR",
    24: "PREPARE_ALL_LEGS_MINUS_30",
    25: "UP_PREPARE_CHASSIS_FORWARD_30",
    26: "UP_LASER_APPROACH_X_LT_35",
    27: "DOWN_LASER_APPROACH_H_GT_65",
    28: "DOWN_PREPARE_CHASSIS_FORWARD_5",
    29: "RECOVER_ALL_LEGS_MINUS_30",
    30: "RECOVER_CHASSIS_BACKWARD_200",
}
CLIMB_STATES.update({i + 1: name for i, name in enumerate(CLIMB_UPSTAIRS_STEP_NAMES)})
CLIMB_TEST_ACTIONS = {
    0: "NONE",
    1: "ALL_LEGS_220",
    2: "ALL_LEGS_UP_10",
    3: "ALL_LEGS_DOWN_10",
    4: "REAR_DRIVE_FORWARD_30",
    5: "REAR_DRIVE_FORWARD_10",
    6: "REAR_DRIVE_BACKWARD_10",
    7: "FRONT_ZERO",
    8: "FRONT_UP_10",
    9: "FRONT_DOWN_10",
    10: "CHASSIS_FORWARD_100",
    11: "CHASSIS_FORWARD_50",
    12: "CHASSIS_BACKWARD_50",
    13: "REAR_ZERO",
    14: "REAR_UP_10",
    15: "REAR_DOWN_10",
    16: "ALL_LEGS_ZERO",
    17: "REAR_DRIVE_BACKWARD_30",
    18: "REAR_DRIVE_FORWARD_500",
    19: "REAR_DRIVE_BACKWARD_500",
    20: "CHASSIS_BACKWARD_100",
    21: "CHASSIS_FORWARD_300",
    22: "CHASSIS_BACKWARD_300",
    23: "FRONT_220",
    24: "FRONT_MINUS_30",
    25: "REAR_220",
    26: "REAR_MINUS_30",
    27: "FRONT_DRIVE_FORWARD_30",
    28: "FRONT_DRIVE_FORWARD_10",
    29: "FRONT_DRIVE_BACKWARD_10",
    30: "FRONT_DRIVE_BACKWARD_30",
    31: "FRONT_DRIVE_FORWARD_500",
    32: "FRONT_DRIVE_BACKWARD_500",
    33: "ALL_DRIVE_FORWARD_30",
    34: "ALL_DRIVE_FORWARD_10",
    35: "ALL_DRIVE_BACKWARD_10",
    36: "ALL_DRIVE_BACKWARD_30",
    37: "ALL_DRIVE_FORWARD_500",
    38: "ALL_DRIVE_BACKWARD_500",
    39: "ALL_LEGS_300",
}
YAW_TUNE_STATES = {0: "IDLE", 1: "RUNNING", 2: "DONE", 3: "FAILED", 4: "STOPPED"}
YAW_TUNE_FAILS = {
    0: "NONE",
    1: "IMU_OFFLINE",
    2: "SOURCE",
    3: "MOTOR",
    4: "SETDIST",
    5: "SAFETY",
    6: "TIMEOUT",
    7: "BAD_ARG",
}
YAW_TUNE_PHASES = {0: "IDLE", 1: "START", 2: "RUN", 3: "SETTLE"}
IK_STATUS = {
    0: "OK",
    1: "NULL",
    2: "BAD_PARAM",
    3: "HEIGHT_UNREACHABLE",
    4: "JOINT_LIMIT",
    5: "LONG_LINK_LIMIT",
    6: "YAW_SWITCH_UNSAFE",
    7: "UNSUPPORTED_STATE",
    8: "BAD_DIRECTION",
}
IK_REASON = {
    0: "NONE",
    1: "BAD_TOOL",
    2: "BAD_STATE",
    3: "BAD_FLOAT",
    4: "HEIGHT_OUTSIDE_LINK",
    5: "J1_LIMIT",
    6: "J2_LIMIT",
    7: "J3_LIMIT",
    8: "BAD_DIRECTION",
    9: "UNSUPPORTED_STATE",
    10: "LONG_LINK_LIMIT",
    11: "YAW_SWITCH_UNSAFE",
}
ARM_STATE = {0: "IDLE", 1: "READY", 2: "TARGET_VALID", 3: "STOPPED", 4: "ERROR"}
ARM_DIRECTION = {0: "Y+", 1: "X+", 2: "X-"}
TOOL_DEV = {0: "S1", 1: "S2", 2: "GRIPPER"}
TOOL_STATE_BY_TOOL = {
    0: {0: "S1_PREPARE", 1: "S1_CARRY_BY_S2", 2: "S1_PLACE"},
    1: {0: "S2_PREPARE_15DEG", 1: "S2_SUCTION_DOWN", 2: "S2_SHORT_PARALLEL", 3: "S2_PLACE_Y_POS"},
    2: {0: "GRIPPER_UP", 1: "GRIPPER_DOWN", 2: "GRIPPER_FORWARD"},
}


def _tool_state_name(tool: int, state: int) -> str:
    return TOOL_STATE_BY_TOOL.get(tool, {}).get(state, "UNKNOWN")


def _u8(data: bytes, offset: int) -> int:
    _need(data, offset + 1)
    return data[offset]


def _u16(data: bytes, offset: int) -> int:
    _need(data, offset + 2)
    return struct.unpack_from("<H", data, offset)[0]


def _u32(data: bytes, offset: int) -> int:
    _need(data, offset + 4)
    return struct.unpack_from("<I", data, offset)[0]


def _i32(data: bytes, offset: int) -> int:
    _need(data, offset + 4)
    return struct.unpack_from("<i", data, offset)[0]


def _f32(data: bytes, offset: int) -> float:
    _need(data, offset + 4)
    return struct.unpack_from("<f", data, offset)[0]


def _need(data: bytes, size: int) -> None:
    if len(data) < size:
        raise ValueError(f"payload too short: need {size}, got {len(data)}")


def _flags(value: int, names: Iterable[str]) -> Dict[str, bool]:
    return {name: bool(value & (1 << idx)) for idx, name in enumerate(names)}


def _arm_ik_test_flags(value: int) -> Dict[str, Any]:
    return {
        "step": value & 0x0F,
        "error": bool(value & 0x20),
        "done": bool(value & 0x40),
        "active": bool(value & 0x80),
    }


def _float_list(data: bytes) -> List[float]:
    return [_f32(data, offset) for offset in range(0, len(data) - 3, 4)]


def decode_usb_frame(frame: UsbFrame) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "cmd": f"0x{frame.cmd:02X}",
        "cmd_id": frame.cmd,
        "cmd_name": command_name(frame.cmd),
        "len": frame.length,
        "raw_hex": frame.hex(),
    }

    try:
        result["payload"] = decode_payload(frame.cmd, frame.payload)
    except ValueError as exc:
        result["payload_error"] = str(exc)
        result["payload_hex"] = bytes_to_hex(frame.payload)

    return result


def decode_payload(cmd: int, payload: bytes) -> Dict[str, Any]:
    if cmd == 0x06:
        return _decode_sys_status(payload)
    if cmd == 0x16:
        return _decode_chassis_status(payload)
    if cmd == 0x26:
        return _decode_arm_status(payload)
    if cmd == 0x36:
        return _decode_tool_status(payload)
    if cmd == 0x46:
        return _decode_robot_status(payload)
    if cmd == 0x49:
        return _decode_yaw_tune_status(payload)
    if cmd == 0x56:
        return _decode_climb_status(payload)
    if cmd == 0x66:
        return _decode_task_flow_status(payload)
    if cmd == 0x90:
        return _decode_arm_ik_result(payload)
    return _decode_generic_payload(payload)


def _decode_sys_status(p: bytes) -> Dict[str, Any]:
    _need(p, 8)
    timeout = _u8(p, 6)
    return {
        "usb_task": _u8(p, 0),
        "usart_task": _u8(p, 1),
        "chassis_enabled": _u8(p, 2),
        "arm_enabled": _u8(p, 3),
        "tool_enabled": _u8(p, 4),
        "chassis_motor_online_count": _u8(p, 5),
        "usb_timeout_flags": _flags(timeout, ["chassis", "arm", "tool"]),
        "reserved": _u8(p, 7),
    }


def _decode_chassis_status(p: bytes) -> Dict[str, Any]:
    _need(p, 56)
    mode = _u8(p, 0)
    pos_state = _u8(p, 1)
    data: Dict[str, Any] = {
        "mode": mode,
        "mode_name": CHASSIS_MODES.get(mode, "UNKNOWN"),
        "pos_state": pos_state,
        "pos_state_name": CHASSIS_POS_STATE.get(pos_state, "UNKNOWN"),
        "emergency_stop": bool(_u8(p, 2)),
        "robot_vel": {"vx": _f32(p, 4), "vy": _f32(p, 8), "vw": _f32(p, 12)},
        "odom": {"x": _f32(p, 16), "y": _f32(p, 20), "yaw": _f32(p, 24)},
        "limits": {"v_max": _f32(p, 28), "a_max": _f32(p, 32), "j_max": _f32(p, 36)},
        "position": {
            "progress": _f32(p, 40),
            "err_x": _f32(p, 44),
            "err_y": _f32(p, 48),
            "err_yaw": _f32(p, 52),
        },
    }

    if len(p) >= 88:
        timeout = _u8(p, 85)
        data.update(
            {
                "nav": {
                    "x_m": _f32(p, 56),
                    "y_m": _f32(p, 60),
                    "yaw_rad": _f32(p, 64),
                    "yaw_total_rad": _f32(p, 68),
                    "vx_mps": _f32(p, 72),
                    "vy_mps": _f32(p, 76),
                    "wz_radps": _f32(p, 80),
                },
                "imu_online": bool(_u8(p, 84)),
                "usb_timeout_flags": _flags(timeout, ["chassis", "arm", "tool"]),
                "status_flags": _flags(
                    _u8(p, 86),
                    [
                        "enabled",
                        "vel_mode",
                        "pos_mode",
                        "pos_running",
                        "moving",
                        "pos_done",
                        "imu_online",
                        "motors_online",
                    ],
                ),
                "error_flags": _flags(
                    _u8(p, 87),
                    [
                        "usb_chassis_timeout",
                        "imu_offline",
                        "emergency_stop",
                        "all_motors_offline",
                        "partial_motor_offline",
                    ],
                ),
            }
        )
    if len(p) >= 112:
        data.update(
            {
                "target_vel": {"vx": _f32(p, 88), "vy": _f32(p, 92), "vw": _f32(p, 96)},
                "target_pos": {"dx": _f32(p, 100), "dy": _f32(p, 104), "dyaw": _f32(p, 108)},
            }
        )
    return data


def _decode_arm_status(p: bytes) -> Dict[str, Any]:
    _need(p, 64)
    state = _u8(p, 3)
    status_flags = _u8(p, 4)
    error_flags = _u8(p, 5)
    ik = _u8(p, 6)
    reason = _u8(p, 7)
    tool = _u8(p, 20)
    tool_state = _u8(p, 21)
    target_direction = _u8(p, 22)
    ik_test_flags = _u8(p, 23)
    theta_rad = [_f32(p, 32), _f32(p, 36), _f32(p, 40)]
    tool_world_mm = [_f32(p, 44), _f32(p, 48), _f32(p, 52)]
    data: Dict[str, Any] = {
        "enabled": bool(_u8(p, 0)),
        "has_target": bool(_u8(p, 1)),
        "output_enabled": bool(_u8(p, 2)),
        "state": state,
        "state_name": ARM_STATE.get(state, "UNKNOWN"),
        "status_flags": _flags(
            status_flags,
            ["enabled", "has_target", "output_enabled", "ik_ok", "active"],
        ),
        "error_flags": _flags(
            error_flags,
            ["ik", "joint_limit", "height_unreachable", "bad_param", "unsafe", "unsupported"],
        ),
        "ik_status": ik,
        "ik_status_name": IK_STATUS.get(ik, "UNKNOWN"),
        "ik_reason": reason,
        "ik_reason_name": IK_REASON.get(reason, "UNKNOWN"),
        "last_command_ms": _u32(p, 8),
        "last_update_ms": _u32(p, 12),
        "output_apply_count": _u32(p, 16),
        "tool": tool,
        "tool_name": TOOL_DEV.get(tool, "UNKNOWN"),
        "tool_state": tool_state,
        "tool_state_name": _tool_state_name(tool, tool_state),
        "target_direction": target_direction,
        "target_direction_name": ARM_DIRECTION.get(target_direction, "UNKNOWN"),
        "ik_test": _arm_ik_test_flags(ik_test_flags),
        "target_z_mm": _f32(p, 24),
        "approach_yaw_rad": _f32(p, 28),
        "theta_rad": theta_rad,
        "theta_deg": [x * RAD_TO_DEG for x in theta_rad],
        "tool_world_mm": tool_world_mm,
        "j4_z_mm": _f32(p, 56),
        "height_error_mm": _f32(p, 60),
    }
    if len(p) >= 80:
        motor_feedback_rad = [_f32(p, 64), _f32(p, 68), _f32(p, 72)]
        data.update(
            {
                "motor_feedback_rad": motor_feedback_rad,
                "motor_feedback_deg": [x * RAD_TO_DEG for x in motor_feedback_rad],
                "motor_feedback_ok": _flags(_u8(p, 76), ["j1", "j2", "j3"]),
            }
        )
    return data


def _decode_tool_status(p: bytes) -> Dict[str, Any]:
    data = _decode_arm_status(p)
    data["status_kind"] = "tool_pose"
    return data


def _decode_robot_status(p: bytes) -> Dict[str, Any]:
    _need(p, 96)
    protocol_version = _u8(p, 0)
    source = _u8(p, 1)
    enable_flags = _u8(p, 2)
    executing_flags = _u8(p, 3)
    error_flags = _u8(p, 4)
    online_flags = _u8(p, 5)
    pos_state = _u8(p, 94)
    if protocol_version >= 5:
        executing_names = ["chassis_moving", "chassis_pos_running", "task_flow_active", "reserved3", "climb_motor_active", "yaw_tune_running", "arm_active", "any"]
    elif protocol_version >= 2:
        executing_names = ["chassis_moving", "chassis_pos_running", "reserved2", "reserved3", "climb_motor_active", "yaw_tune_running", "arm_active", "any"]
    else:
        executing_names = ["chassis_moving", "chassis_pos_running", "arm_active", "reserved3", "any"]
    data: Dict[str, Any] = {
        "protocol_version": protocol_version,
        "active_source": source,
        "active_source_name": SOURCE_NAMES.get(source, "UNKNOWN"),
        "enable_flags": _flags(enable_flags, ["chassis", "reserved1", "reserved2", "climb", "arm"]),
        "executing_flags": _flags(executing_flags, executing_names),
        "error_flags": _flags(
            error_flags,
            [
                "usb_chassis_timeout",
                "reserved1",
                "reserved2",
                "imu_offline",
                "chassis_emergency_stop",
                "arm_error",
                "reserved6",
                "active_source_stale",
            ],
        ),
        "online_flags": _flags(online_flags, ["usb_recent", "usart_recent", "imu", "chassis_motors", "arm_motors", "climb_motors", "laser"]),
        "usb_rx": {"last_cmd": _u8(p, 6), "last_len": _u8(p, 7), "count": _u32(p, 8), "last_tick": _u32(p, 12)},
        "usart_rx": {
            "frame_count": _u32(p, 16),
            "last_frame_tick": _u32(p, 20),
            "checksum_fail_count": _u32(p, 24),
            "checksum_ok": bool(_u8(p, 28)),
            "mode": _u8(p, 29),
        },
        "arm": {},
        "nav": {"x_m": _f32(p, 32), "y_m": _f32(p, 36), "yaw_total_rad": _f32(p, 40), "vx_mps": _f32(p, 44), "vy_mps": _f32(p, 48), "wz_radps": _f32(p, 52)},
        "chassis": {
            "odom_x": _f32(p, 56),
            "odom_y": _f32(p, 60),
            "odom_yaw": _f32(p, 64),
            "robot_vel": {"vx": _f32(p, 68), "vy": _f32(p, 72), "vw": _f32(p, 76)},
            "pos_state": pos_state,
            "pos_state_name": CHASSIS_POS_STATE.get(pos_state, "UNKNOWN"),
        },
        "arm_error_deg": [_f32(p, 80), _f32(p, 84), _f32(p, 88)],
        "timeout_flags": _flags(_u8(p, 95), ["usb_chassis", "reserved1", "reserved2", "usart_control"]),
    }

    if len(p) >= 160:
        climb_state = _u8(p, 97)
        test_action = _u8(p, 106)
        test_flags = _u8(p, 107)
        climb_flow = 1 if (test_flags & 0x04) else 0
        yaw_state = _u8(p, 140)
        yaw_fail = _u8(p, 141)
        yaw_phase = _u8(p, 142)
        yaw_mode = _u8(p, 143)
        data.update(
            {
                "climb": {
                    "active_source": _u8(p, 96),
                    "active_source_name": SOURCE_NAMES.get(_u8(p, 96), "UNKNOWN"),
                    "flow": climb_flow,
                    "flow_name": CLIMB_FLOW_NAMES.get(climb_flow, "UNKNOWN"),
                    "state": climb_state,
                    "state_name": climb_state_name(climb_state, climb_flow),
                    "enabled": bool(_u8(p, 98)),
                    "auto_run": bool(_u8(p, 99)),
                    "state_done": bool(_u8(p, 100)),
                    "error_flags": _flags(_u8(p, 101), ["timeout", "param_not_configured", "test_action", "flow_switch", "recovery_failed"]),
                    "pending_step": bool(_u8(p, 102)),
                    "pending_auto": bool(_u8(p, 103)),
                    "motor_active": bool(_u8(p, 104)),
                    "climb_motor_online_count": _u8(p, 105),
                    "fdcan2_motor_online_count": _u8(p, 105),
                    "test_action": test_action,
                    "test_action_name": CLIMB_TEST_ACTIONS.get(test_action, "UNKNOWN"),
                    "test_active": bool(test_flags & 0x01),
                    "test_chassis_active": bool(test_flags & 0x02),
                    "elapsed_ms": _u32(p, 108),
                    "last_update_ms": _u32(p, 112),
                },
                "laser": {
                    "valid_flags": _flags(_u8(p, 116), ["x_pos", "y_pos", "height"]),
                    "online_flags": _flags(_u8(p, 117), ["x_pos", "y_pos", "height"]),
                    "waiting_flags": _flags(_u8(p, 118), ["x_pos", "y_pos", "height"]),
                    "all_valid": bool(_u8(p, 119)),
                    "all_online": bool(_u8(p, 120)),
                    "update_tick": _u32(p, 124),
                    "distance_mm": {
                        "x_pos": _i32(p, 128),
                        "y_pos": _i32(p, 132),
                        "height": _i32(p, 136),
                    },
                },
                "yaw_tune": {
                    "state": yaw_state,
                    "state_name": YAW_TUNE_STATES.get(yaw_state, "UNKNOWN"),
                    "fail_reason": yaw_fail,
                    "fail_reason_name": YAW_TUNE_FAILS.get(yaw_fail, "UNKNOWN"),
                    "phase": yaw_phase,
                    "phase_name": YAW_TUNE_PHASES.get(yaw_phase, "UNKNOWN"),
                    "active_mode": yaw_mode,
                    "active_mode_name": CHASSIS_MODES.get(yaw_mode, "UNKNOWN"),
                    "tick_ms": _u32(p, 144),
                    "segment_elapsed_ms": _u32(p, 148),
                    "yaw_error_deg": _f32(p, 152),
                    "score": _f32(p, 156),
                },
            }
        )

    if len(p) >= 240:
        arm_theta_rad = [_f32(p, 200), _f32(p, 204), _f32(p, 208)]
        arm_tool_world_mm = [_f32(p, 212), _f32(p, 216), _f32(p, 220)]
        data["chassis"].update(
            {
                "target_vel": {"vx": _f32(p, 160), "vy": _f32(p, 164), "vw": _f32(p, 168)},
                "target_pos": {"dx": _f32(p, 172), "dy": _f32(p, 176), "dyaw": _f32(p, 180)},
            }
        )
        data["arm"].update(
            {
                "enabled": bool(_u8(p, 184)),
                "has_target": bool(_u8(p, 185)),
                "output_enabled": bool(_u8(p, 186)),
                "state": _u8(p, 187),
                "state_name": ARM_STATE.get(_u8(p, 187), "UNKNOWN"),
                "status_flags": _flags(
                    _u8(p, 188),
                    ["enabled", "has_target", "output_enabled", "ik_ok", "active"],
                ),
                "error_flags": _flags(
                    _u8(p, 189),
                    ["ik", "joint_limit", "height_unreachable", "bad_param", "unsafe", "unsupported"],
                ),
                "ik_status": _u8(p, 190),
                "ik_status_name": IK_STATUS.get(_u8(p, 190), "UNKNOWN"),
                "ik_reason": _u8(p, 191),
                "ik_reason_name": IK_REASON.get(_u8(p, 191), "UNKNOWN"),
                "target_z_mm": _f32(p, 192),
                "approach_yaw_rad": _f32(p, 196),
                "theta_rad": arm_theta_rad,
                "theta_deg": [x * RAD_TO_DEG for x in arm_theta_rad],
                "tool_world_mm": arm_tool_world_mm,
                "last_update_ms": _u32(p, 224),
                "output_apply_count": _u32(p, 228),
                "tool": _u8(p, 232),
                "tool_name": TOOL_DEV.get(_u8(p, 232), "UNKNOWN"),
                "tool_state": _u8(p, 233),
                "tool_state_name": _tool_state_name(_u8(p, 232), _u8(p, 233)),
                "target_direction": _u8(p, 234),
                "target_direction_name": ARM_DIRECTION.get(_u8(p, 234), "UNKNOWN"),
                "ik_test": _arm_ik_test_flags(_u8(p, 235)),
            }
        )
        data["climb_summary_error_flags"] = _flags(_u8(p, 238), ["timeout", "param_not_configured", "test_action", "flow_switch", "recovery_failed"])
        data["active_source_stale"] = bool(_u8(p, 239))

    if len(p) >= 253:
        motor_feedback_rad = [_f32(p, 240), _f32(p, 244), _f32(p, 248)]
        data["arm"].update(
            {
                "motor_feedback_rad": motor_feedback_rad,
                "motor_feedback_deg": [x * RAD_TO_DEG for x in motor_feedback_rad],
                "motor_feedback_ok": _flags(_u8(p, 252), ["j1", "j2", "j3"]),
            }
        )

    return data


def _decode_climb_status(p: bytes) -> Dict[str, Any]:
    _need(p, 64)
    state = _u8(p, 0)
    source = _u8(p, 5)
    error_flags = _u8(p, 4)
    test_action = _u8(p, 7)
    flow = _u8(p, 64) if len(p) >= 65 else 0
    status_flags = _u8(p, 65) if len(p) >= 68 else 0
    leg_reached_mask = _u8(p, 66) if len(p) >= 68 else 0
    drive_reached_mask = _u8(p, 67) if len(p) >= 68 else 0
    summary_state = _u8(p, 68) if len(p) >= 69 else None
    return {
        "state": state,
        "state_name": climb_state_name(state, flow),
        "summary_state": summary_state,
        "summary_state_name": (
            CLIMB_SUMMARY_STATES.get(summary_state, "UNKNOWN")
            if summary_state is not None
            else None
        ),
        "flow": flow,
        "flow_name": CLIMB_FLOW_NAMES.get(flow, "UNKNOWN"),
        "enabled": bool(_u8(p, 1)),
        "auto_run": bool(_u8(p, 2)),
        "state_done": bool(_u8(p, 3)),
        "error_flags": _flags(error_flags, ["timeout", "param_not_configured", "test_action", "flow_switch", "recovery_failed"]),
        "active_source": source,
        "active_source_name": SOURCE_NAMES.get(source, "UNKNOWN"),
        "climb_motor_online_count": _u8(p, 6),
        "fdcan2_motor_online_count": _u8(p, 6),
        "test_action": test_action,
        "test_action_name": CLIMB_TEST_ACTIONS.get(test_action, "UNKNOWN"),
        "elapsed_ms": _u32(p, 8),
        "last_update_ms": _u32(p, 12),
        "leg_pos_mm": [_f32(p, 16), _f32(p, 20), _f32(p, 24), _f32(p, 28)],
        "leg_target_mm": [_f32(p, 32), _f32(p, 36), _f32(p, 40), _f32(p, 44)],
        "drive_pos_mm": [_f32(p, 48), _f32(p, 52)],
        "drive_target_mm": [_f32(p, 56), _f32(p, 60)],
        "status_flags": _flags(
            status_flags,
            [
                "motor_output_active",
                "leg_busy",
                "drive_busy",
                "test_chassis_active",
                "pending_step",
                "pending_auto",
                "pending_test_or_auto_pause",
                "ready_for_next",
            ],
        ),
        "leg_reached_mask": leg_reached_mask,
        "drive_reached_mask": drive_reached_mask,
        "leg_reached": [bool(leg_reached_mask & (1 << i)) for i in range(4)],
        "drive_reached": [bool(drive_reached_mask & (1 << i)) for i in range(2)],
    }


def _decode_task_flow_status(p: bytes) -> Dict[str, Any]:
    _need(p, 28)
    flow_id = _u8(p, 1)
    state = _u8(p, 2)
    error = _u8(p, 3)
    current_op = _u8(p, 4)
    completed_s1_end = _u8(p, 5)
    flow_arg = _u8(p, 7)
    data = {
        "protocol_version": _u8(p, 0),
        "flow_id": flow_id,
        "flow_name": TASK_FLOW_IDS.get(flow_id, "UNKNOWN"),
        "state": state,
        "state_name": TASK_FLOW_STATES.get(state, "UNKNOWN"),
        "error": error,
        "error_name": TASK_FLOW_ERRORS.get(error, "UNKNOWN"),
        "current_op": current_op,
        "current_op_name": TASK_FLOW_OPS.get(current_op, "UNKNOWN"),
        "completed_s1_end": completed_s1_end,
        "completed_s1_end_name": TASK_FLOW_S1_ENDS.get(completed_s1_end, "UNKNOWN"),
        "active": bool(_u8(p, 6)),
        "flow_arg": flow_arg,
        "flow_arg_name": ARM_DIRECTION.get(flow_arg, "UNKNOWN") if flow_id == 8 else None,
        "entry_index": _u16(p, 8),
        "entry_count": _u16(p, 10),
        "repeat_index": _u16(p, 12),
        "repeat_count": _u16(p, 14),
        "completed_steps": _u32(p, 16),
        "step_start_ms": _u32(p, 20),
        "last_update_ms": _u32(p, 24),
    }
    if len(p) >= 30:
        data.update(
            {
                "host_checkpoint_pending": _u8(p, 28),
                "host_checkpoint_ack": _u8(p, 29),
            }
        )
    return data


def _decode_yaw_tune_status(p: bytes) -> Dict[str, Any]:
    _need(p, 64)
    state = _u8(p, 0)
    fail = _u8(p, 3)
    mode = _u8(p, 62)
    phase = _u8(p, 63)
    return {
        "state": state,
        "state_name": YAW_TUNE_STATES.get(state, "UNKNOWN"),
        "segment_index": _u8(p, 1),
        "segment_count": _u8(p, 2),
        "fail_reason": fail,
        "fail_reason_name": YAW_TUNE_FAILS.get(fail, "UNKNOWN"),
        "tick_ms": _u32(p, 4),
        "segment_elapsed_ms": _u32(p, 8),
        "yaw_error_deg": _f32(p, 12),
        "yaw_error_abs_max_deg": _f32(p, 16),
        "yaw_rate_error_rms_dps": _f32(p, 20),
        "gyro_z_abs_max_dps": _f32(p, 24),
        "score": _f32(p, 28),
        "pid": {
            "angle_kp": _f32(p, 32),
            "angle_kd": _f32(p, 36),
            "rate_kp": _f32(p, 40),
            "rate_ki": _f32(p, 44),
            "rate_kd": _f32(p, 48),
            "pos_kp_yaw": _f32(p, 52),
        },
        "last_adjust": _f32(p, 56),
        "pass_index": _u8(p, 60),
        "pass_count": _u8(p, 61),
        "active_mode": mode,
        "active_mode_name": CHASSIS_MODES.get(mode, "UNKNOWN"),
        "phase": phase,
        "phase_name": YAW_TUNE_PHASES.get(phase, "UNKNOWN"),
    }


def _decode_arm_ik_result(p: bytes) -> Dict[str, Any]:
    _need(p, 2)
    status = _u8(p, 0)
    reason = _u8(p, 1)
    data = {
        "status": status,
        "status_name": IK_STATUS.get(status, "UNKNOWN"),
        "reason": reason,
        "reason_name": IK_REASON.get(reason, "UNKNOWN"),
    }
    if len(p) >= 14:
        data.update(
            {
                "tool": _u8(p, 2),
                "tool_name": TOOL_DEV.get(_u8(p, 2), "UNKNOWN"),
                "tool_state": _u8(p, 3),
                "tool_state_name": _tool_state_name(_u8(p, 2), _u8(p, 3)),
                "target_z_mm": _f32(p, 4),
                "approach_yaw_rad": _f32(p, 8),
                "joint_count": _u8(p, 12),
            }
        )
    return data


def _decode_generic_payload(p: bytes) -> Dict[str, Any]:
    out: Dict[str, Any] = {"payload_hex": bytes_to_hex(p)}
    if len(p) and len(p) % 4 == 0:
        out["float32_le"] = _float_list(p)
    return out
