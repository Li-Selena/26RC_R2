from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

from .protocol import pack_4float_frame, pack_usb_frame


@dataclass(frozen=True)
class UsbCommand:
    name: str
    cmd: int
    needs_float_payload: bool = False
    doc: str = ""
    aliases: Tuple[str, ...] = ()
    optional_float_payload: bool = False


COMMANDS: Tuple[UsbCommand, ...] = (
    UsbCommand("SYS_DISABLE", 0x00, doc="Disable all USB-controlled modules."),
    UsbCommand("SYS_ENABLE", 0x01, doc="Enable chassis, arm and tool."),
    UsbCommand("SYS_SWITCH_SOURCE", 0x02, True, "f0=0 USART, f0=1 USB.", ("SOURCE",)),
    UsbCommand("SYS_STOP", 0x05, doc="Stop USB chassis and hold the arm target."),
    UsbCommand("SYS_GET_STATUS", 0x06, doc="Query system status.", aliases=("SYS_STATUS",)),
    UsbCommand("CHS_DISABLE", 0x10),
    UsbCommand("CHS_ENABLE", 0x11),
    UsbCommand("CHS_SET_MODE", 0x12, True, "f0=mode 0..7.", ("CHASSIS_SET_MODE",)),
    UsbCommand("CHS_SET_VEL", 0x13, True, "f0=vx, f1=vy, f2=yaw_data, f3=0. ROBOT_NO_YAW: robot-frame target_yaw_deg; WORLD_NO_YAW: world-frame target_yaw_deg; other modes: vw_rad_s.", ("VEL", "CHASSIS_SET_VEL")),
    UsbCommand("CHS_SET_POS", 0x14, True, "f0=dx, f1=dy, f2=yaw_data, f3=0. ROBOT_NO_YAW: robot-frame target_yaw_deg; WORLD_NO_YAW: world-frame target_yaw_deg; other modes: dyaw_rad.", ("POS", "CHASSIS_SET_POS")),
    UsbCommand("CHS_STOP", 0x15),
    UsbCommand("CHS_GET_STATUS", 0x16, aliases=("CHS_STATUS", "CHASSIS_STATUS")),
    UsbCommand("ARM_DISABLE", 0x20),
    UsbCommand("ARM_ENABLE", 0x21),
    UsbCommand("ARM_SET_WORKSPACE", 0x22, True, "f0=direction 0:Y+ 1:X+ 2:X-. Keep current tool/state/z.", ("ARM_SPACE", "ARM_WORKSPACE")),
    UsbCommand("ARM_SET_TARGET", 0x23, True, "Legacy: f0=tool, f1=tool-local state, f2=target_z_mm, f3=approach_yaw_rad.", ("ARM_TARGET",)),
    UsbCommand("ARM_SET_TARGET_XYZ", 0x24, True, "f0=x_mm, f1=y_mm, f2=z_mm. x/y choose arm workspace only; z solves J2/J3.", ("ARM_XYZ", "ARM_TARGET_XYZ")),
    UsbCommand("ARM_STOP", 0x25),
    UsbCommand("ARM_GET_STATUS", 0x26, aliases=("ARM_STATUS", "ARM_STATE", "ARM_QUERY")),
    UsbCommand("ARM_IK_TEST_FLOW", 0x27, doc="Start/stop arm IK XYZ test flow. Empty=start default, optional f0=1 start/f0=0 stop, f1=step_ms.", aliases=("ARM_IK_TEST", "IK_TEST_FLOW"), optional_float_payload=True),
    UsbCommand("ARM_HEIGHT_JOG", 0x28, True, "f0=delta_z_mm. Locks J3 and moves height with J2 only.", ("ARM_Z_JOG", "ARM_JOG_Z")),
    UsbCommand("ARM_HEIGHT_LIMIT", 0x29, True, "f0=0 move to current tool min z, f0=1 move to max z.", ("ARM_Z_LIMIT", "ARM_HEIGHT_MINMAX")),
    UsbCommand("ARM_SET_POSTURE", 0x2A, True, "f0=tool, f1=tool-local state. Keep current xyz.", ("ARM_POSTURE", "TOOL_POSTURE")),
    UsbCommand("ARM_JOINT_JOG", 0x2B, True, "f0=joint 2/3, f1=signed delta_deg. +deg is CCW viewed on YOZ with X+ out of screen.", ("ARM_JOG_JOINT", "ARM_ACTUAL_JOG")),
    UsbCommand("TOOL_DISABLE", 0x30, doc="Empty disables the arm; f0=1 turns off suction 2 on PE13, f0=2 closes the gripper. S1 actuator removed."),
    UsbCommand("TOOL_ENABLE", 0x31, doc="Empty enables the arm; f0=1 turns on suction 2 on PE13, f0=2 opens the gripper. S1 actuator removed."),
    UsbCommand("TOOL_SET_MODE", 0x32, True, "Same payload as ARM_SET_TARGET."),
    UsbCommand("TOOL_STOP", 0x35),
    UsbCommand("TOOL_GET_STATUS", 0x36, aliases=("TOOL_STATUS",)),
    UsbCommand("ROBOT_GET_STATUS", 0x46, aliases=("ROBOT_STATUS", "STATUS")),
    UsbCommand("YAW_TUNE_START", 0x47, doc="Start yaw auto tune. Optional f0=pass_count.", aliases=("AUTOTUNE_START", "TUNE_START"), optional_float_payload=True),
    UsbCommand("YAW_TUNE_STOP", 0x48, doc="Stop yaw auto tune.", aliases=("AUTOTUNE_STOP", "TUNE_STOP")),
    UsbCommand("YAW_TUNE_GET_STATUS", 0x49, aliases=("YAW_TUNE_STATUS", "AUTOTUNE_STATUS", "TUNE_STATUS")),
    UsbCommand("CLIMB_DISABLE", 0x50),
    UsbCommand("CLIMB_ENABLE", 0x51),
    UsbCommand("CLIMB_SET_CTRL", 0x52, True, "f0=enable, f1=step, f2=auto."),
    UsbCommand("CLIMB_UP_STEP", 0x53, aliases=("CLIMB_STEP", "CLIMB_UPSTAIRS_STEP", "UPSTAIRS_STEP")),
    UsbCommand("CLIMB_UP_AUTO", 0x54, aliases=("CLIMB_AUTO", "CLIMB_RUN", "CLIMB_UP_RUN", "CLIMB_UPSTAIRS_AUTO", "CLIMB_UPSTAIRS_RUN", "UPSTAIRS_AUTO", "UPSTAIRS_RUN")),
    UsbCommand("CLIMB_STOP", 0x55),
    UsbCommand("CLIMB_GET_STATUS", 0x56, aliases=("CLIMB_STATUS",)),
    UsbCommand("CLIMB_TEST_ACTION", 0x57, True, "f0=action id.", aliases=("CLIMB_ACTION", "CLIMB_TEST")),
    UsbCommand("CLIMB_DOWN_STEP", 0x58, aliases=("CLIMB_DOWNSTAIRS_STEP", "DOWNSTAIRS_STEP")),
    UsbCommand("CLIMB_DOWN_AUTO", 0x59, aliases=("CLIMB_DOWNSTAIRS_AUTO", "CLIMB_DOWNSTAIRS_RUN", "CLIMB_DOWN_RUN", "DOWNSTAIRS_AUTO", "DOWNSTAIRS_RUN")),
    UsbCommand("CLIMB_UP_GATE", 0x5A, doc="Run only the upstairs auto laser gate: UP_LASER_APPROACH_X_LT_35.", aliases=("CLIMB_UP_LASER_GATE", "UPSTAIRS_GATE", "UP_LASER_GATE")),
    UsbCommand("CLIMB_DOWN_GATE", 0x5B, doc="Run only the downstairs auto laser gate: DOWN_LASER_APPROACH_H_GT_65 plus 5mm approach.", aliases=("CLIMB_DOWN_LASER_GATE", "DOWNSTAIRS_GATE", "DOWN_LASER_GATE")),
    UsbCommand("CLIMB_UP_AUTO_PAUSE", 0x5C, doc="Run upstairs auto and pause at the v2 interrupt point.", aliases=("CLIMB_UP_PAUSE", "UPSTAIRS_AUTO_PAUSE", "UP_AUTO_PAUSE")),
    UsbCommand("CLIMB_DOWN_AUTO_PAUSE", 0x5D, doc="Run downstairs auto and pause at the v2 interrupt point.", aliases=("CLIMB_DOWN_PAUSE", "DOWNSTAIRS_AUTO_PAUSE", "DOWN_AUTO_PAUSE")),
    UsbCommand("CLIMB_AUTO_RESUME", 0x5E, doc="Resume a climb auto flow paused at the v2 interrupt point.", aliases=("CLIMB_RESUME", "AUTO_RESUME")),
    UsbCommand("FLOW_S1_UP", 0x60, doc="Run the built-in s1_up_v2 task-flow state machine.", aliases=("S1_UP", "S1_UP_V1", "S1_UP_V2", "S1_UP_FLOW", "RUN_S1_UP")),
    UsbCommand("FLOW_S1_DOWN", 0x61, doc="Run the built-in s1_down_v2 task-flow state machine.", aliases=("S1_DOWN", "S1_DOWN_V1", "S1_DOWN_V2", "S1_DOWN_FLOW", "RUN_S1_DOWN")),
    UsbCommand("FLOW_S1_UP_S2_DOWN", 0x63, doc="Run s1_up_s2_down_v1 after FLOW_S1_UP has completed.", aliases=("S1_UP_S2_DOWN", "S1_UP_S2_DOWN_V1", "S1_UP_S2_DOWN_FLOW", "RUN_S1_UP_S2_DOWN")),
    UsbCommand("FLOW_S1_DOWN_S2_UP", 0x64, doc="Run s1_down_s2_up_v1 after FLOW_S1_DOWN has completed.", aliases=("S1_DOWN_S2_UP", "S1_DOWN_S2_UP_V1", "S1_DOWN_S2_UP_FLOW", "RUN_S1_DOWN_S2_UP")),
    UsbCommand("FLOW_S1_DOWN_S2_DOWN", 0x65, doc="Run s1_down_s2_down_v1 after FLOW_S1_DOWN has completed.", aliases=("S1_DOWN_S2_DOWN", "S1_DOWN_S2_DOWN_V1", "S1_DOWN_S2_DOWN_FLOW", "RUN_S1_DOWN_S2_DOWN")),
    UsbCommand("FLOW_GET_STATUS", 0x66, doc="Query the outer S1/S2 task-flow state machine.", aliases=("FLOW_STATUS", "TASK_FLOW_STATUS", "TASK_STATUS")),
    UsbCommand("FLOW_WEAPON_GRAB", 0x67, doc="Run the weapon_grab_v1 arm grab task-flow state machine.", aliases=("WEAPON_GRAB", "WEAPON_GRAB_V1", "RUN_WEAPON_GRAB", "ARM_GRAB", "ARM_GRAB_V1")),
    UsbCommand("FLOW_THROW_BLOCK", 0x68, True, "Throw the held block toward f0=1 X+ or f0=2 X-. Requires completed S1_UP_V2 or S1_DOWN_V2; both use suction 2.", ("THROW_BLOCK", "THROW_BLOCK_V1", "BLOCK_THROW", "RUN_THROW_BLOCK")),
    UsbCommand("FLOW_WEAPON_DOCK_TEST", 0x69, doc="Run the lower-computer weapon grab/dock test state machine; waits for host chassis and dock checkpoints.", aliases=("WEAPON_DOCK_TEST", "WEAPON_DOCK_TEST_V1", "RUN_WEAPON_DOCK_TEST")),
    UsbCommand("FLOW_CHASSIS_MOVE_DONE", 0x6A, doc="Confirm weapon dock checkpoint 1: host chassis movement completed.", aliases=("CHASSIS_MOVE_DONE", "WEAPON_CHASSIS_DONE", "WEAPON_CHASSIS_MOVE_DONE")),
    UsbCommand("FLOW_DOCK_DONE", 0x6B, doc="Confirm weapon dock checkpoint 2: host docking decision completed.", aliases=("DOCK_DONE", "WEAPON_DOCK_DONE", "WEAPON_DOCK_COMPLETE")),
    UsbCommand("ARM_IK_RESULT", 0x90, doc="Async arm IK result from firmware."),
)

COMMAND_BY_NAME: Dict[str, UsbCommand] = {}
COMMAND_BY_ID: Dict[int, UsbCommand] = {}

for command in COMMANDS:
    COMMAND_BY_ID[command.cmd] = command
    COMMAND_BY_NAME[command.name] = command
    for alias in command.aliases:
        COMMAND_BY_NAME[alias] = command


STATUS_ALIASES = {
    "SYS": "SYS_GET_STATUS",
    "SYSTEM": "SYS_GET_STATUS",
    "CHS": "CHS_GET_STATUS",
    "CHASSIS": "CHS_GET_STATUS",
    "ARM": "ARM_GET_STATUS",
    "TOOL": "TOOL_GET_STATUS",
    "ROBOT": "ROBOT_GET_STATUS",
    "YAW_TUNE": "YAW_TUNE_GET_STATUS",
    "AUTOTUNE": "YAW_TUNE_GET_STATUS",
    "TUNE": "YAW_TUNE_GET_STATUS",
    "CLIMB": "CLIMB_GET_STATUS",
    "FLOW": "FLOW_GET_STATUS",
    "TASK_FLOW": "FLOW_GET_STATUS",
    "TASK": "FLOW_GET_STATUS",
}


def climb_test_shortcut_action(token: str) -> Optional[int]:
    key = token.strip().lower()
    if not key.startswith("climb_"):
        return None

    # Import lazily to keep status decoding free to import command_name above.
    from .status import CLIMB_TEST_ACTIONS

    action_by_shortcut = {
        f"climb_{name.lower()}": action
        for action, name in CLIMB_TEST_ACTIONS.items()
        if action != 0
    }
    return action_by_shortcut.get(key)


def build_climb_test_shortcut_sequence(token: str) -> Optional[Tuple[bytes, bytes, bytes]]:
    action = climb_test_shortcut_action(token)
    if action is None:
        return None

    return (
        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
        build_usb_command("CLIMB_ENABLE"),
        build_usb_command("CLIMB_TEST_ACTION", [float(action)]),
    )


def command_name(cmd: int) -> str:
    info = COMMAND_BY_ID.get(cmd)
    return info.name if info else f"CMD_0x{cmd:02X}"


def list_commands() -> List[UsbCommand]:
    return list(COMMANDS)


def resolve_command(token: str) -> UsbCommand:
    key = token.strip().upper()
    key = STATUS_ALIASES.get(key, key)
    if key in COMMAND_BY_NAME:
        return COMMAND_BY_NAME[key]

    try:
        if key.startswith("0X"):
            cmd = int(key, 16)
        else:
            cmd = int(key, 10)
    except ValueError as exc:
        raise ValueError(f"unknown command: {token}") from exc

    if not 0 <= cmd <= 0xFF:
        raise ValueError("numeric command must be 0..255")
    return UsbCommand(f"CMD_0x{cmd:02X}", cmd)


def build_usb_command(token: str, values: Optional[Iterable[float]] = None) -> bytes:
    shortcut_action = climb_test_shortcut_action(token)
    if shortcut_action is not None:
        vals = list(values or [])
        if vals:
            raise ValueError(f"{token} does not accept float payload values; use CLIMB_TEST_ACTION {shortcut_action}")
        return pack_4float_frame(resolve_command("CLIMB_TEST_ACTION").cmd, [float(shortcut_action)])

    command = resolve_command(token)
    vals = list(values or [])

    if vals:
        return pack_4float_frame(command.cmd, vals)
    if command.needs_float_payload:
        raise ValueError(f"{command.name} needs a float payload")
    return pack_usb_frame(command.cmd)


def build_query_command(token: str) -> bytes:
    key = token.strip().upper()
    key = STATUS_ALIASES.get(key, key)
    return build_usb_command(key)
