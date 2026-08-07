from __future__ import annotations

from dataclasses import dataclass
from math import pi
from typing import Any, Dict, Iterable, List, Optional, Tuple

from .commands import build_usb_command, resolve_command
from .status import CLIMB_TEST_ACTIONS


ARM_JOINT_JOG_STEPS_DEG = (1, 5, 10, 20, 30, 60, 90)
ARM_HEIGHT_JOG_STEPS_MM = (1, 5, 10, 20, 50, 100)
BASE_TRANSLATE_STEPS_MM = (10, 50, 100)
BASE_ROBOT_POS_MODE = 5.0

WAIT_NONE = "none"
WAIT_ARM = "arm"
WAIT_ARM_STATUS = "arm_status"
WAIT_CHASSIS = "chassis"
WAIT_CLIMB = "climb"
WAIT_CLIMB_FINAL = "climb_final"
WAIT_FLOW = "flow"
WAIT_SETTLE = "settle"

DEFAULT_ARM_BLOCK_TIMEOUT_S = 1.0
DEFAULT_ARM_TARGET_BLOCK_TIMEOUT_S = 3.0
DEFAULT_BASE_BLOCK_TIMEOUT_S = 10.0
DEFAULT_CLIMB_BLOCK_TIMEOUT_S = 40.0
DEFAULT_TOOL_SETTLE_S = 0.3


@dataclass(frozen=True)
class BlockCommand:
    block_id: str
    category: str
    label: str
    frames: Tuple[Tuple[str, Tuple[float, ...]], ...]
    doc: str = ""
    record_status: bool = False
    wait_kind: str = WAIT_NONE
    default_timeout_s: float = 0.0


def _frame(name: str, *values: float) -> Tuple[str, Tuple[float, ...]]:
    return name, tuple(float(v) for v in values)


def _single(
    block_id: str,
    category: str,
    label: str,
    name: str,
    *values: float,
    doc: str = "",
    wait_kind: str = WAIT_NONE,
    default_timeout_s: float = 0.0,
) -> BlockCommand:
    return BlockCommand(
        block_id,
        category,
        label,
        (_frame(name, *values),),
        doc,
        wait_kind=wait_kind,
        default_timeout_s=default_timeout_s,
    )


def _base_pos_block(block_id: str, label: str, dx_m: float, dy_m: float, dyaw_rad: float) -> BlockCommand:
    return BlockCommand(
        block_id,
        "base",
        label,
        (
            _frame("SYS_SWITCH_SOURCE", 1.0),
            _frame("CHS_ENABLE"),
            _frame("CHS_SET_MODE", BASE_ROBOT_POS_MODE),
            _frame("CHS_SET_POS", dx_m, dy_m, dyaw_rad),
        ),
        "Robot-frame chassis position move.",
        wait_kind=WAIT_CHASSIS,
        default_timeout_s=DEFAULT_BASE_BLOCK_TIMEOUT_S,
    )


def _climb_test_block(action: int, name: str) -> BlockCommand:
    block_id = f"climb_{name.lower()}"
    label = "Climb " + name.lower().replace("_", " ")
    return BlockCommand(
        block_id,
        "climb",
        label,
        (
            _frame("SYS_SWITCH_SOURCE", 1.0),
            _frame("CLIMB_ENABLE"),
            _frame("CLIMB_TEST_ACTION", float(action)),
        ),
        "Run one climb test action and wait for CLIMB_GET_STATUS.ready/state_done.",
        wait_kind=WAIT_CLIMB,
        default_timeout_s=DEFAULT_CLIMB_BLOCK_TIMEOUT_S,
    )


def _climb_gate_block(block_id: str, label: str, command_name: str, doc: str) -> BlockCommand:
    return BlockCommand(
        block_id,
        "climb",
        label,
        (
            _frame("SYS_SWITCH_SOURCE", 1.0),
            _frame("CLIMB_ENABLE"),
            _frame(command_name),
        ),
        doc,
        wait_kind=WAIT_CLIMB,
        default_timeout_s=DEFAULT_CLIMB_BLOCK_TIMEOUT_S,
    )


def _climb_auto_control_block(
    block_id: str,
    label: str,
    command_name: str,
    doc: str,
    wait_kind: str,
    timeout_s: float,
) -> BlockCommand:
    return BlockCommand(
        block_id,
        "climb",
        label,
        (
            _frame("SYS_SWITCH_SOURCE", 1.0),
            _frame("CLIMB_ENABLE"),
            _frame(command_name),
        ),
        doc,
        wait_kind=wait_kind,
        default_timeout_s=timeout_s,
    )


def _arm_joint_target_block(
    block_id: str,
    label: str,
    j2_delta_deg: float,
    j3_delta_deg: float,
    doc: str,
) -> BlockCommand:
    return BlockCommand(
        block_id,
        "arm_joint_target",
        label,
        (
            _frame("ARM_JOINT_JOG", 2.0, j2_delta_deg),
            _frame("ARM_JOINT_JOG", 3.0, j3_delta_deg),
        ),
        doc,
        wait_kind=WAIT_ARM,
        default_timeout_s=DEFAULT_ARM_TARGET_BLOCK_TIMEOUT_S,
    )


def _build_blocks() -> Tuple[BlockCommand, ...]:
    blocks: List[BlockCommand] = []

    for joint in (2, 3):
        for step in ARM_JOINT_JOG_STEPS_DEG:
            blocks.append(
                _single(
                    f"arm_j{joint}_cw_{step}deg",
                    "arm_joint",
                    f"J{joint} CW {step} deg",
                    "ARM_JOINT_JOG",
                    joint,
                    -step,
                    doc="CW viewed on YOZ plane with X+ out of screen.",
                    wait_kind=WAIT_ARM,
                    default_timeout_s=DEFAULT_ARM_BLOCK_TIMEOUT_S,
                )
            )
            blocks.append(
                _single(
                    f"arm_j{joint}_ccw_{step}deg",
                    "arm_joint",
                    f"J{joint} CCW {step} deg",
                    "ARM_JOINT_JOG",
                    joint,
                    step,
                    doc="CCW viewed on YOZ plane with X+ out of screen.",
                    wait_kind=WAIT_ARM,
                    default_timeout_s=DEFAULT_ARM_BLOCK_TIMEOUT_S,
                )
            )

    for step in ARM_HEIGHT_JOG_STEPS_MM:
        blocks.append(
            _single(
                f"arm_up_{step}mm",
                "arm_height",
                f"Arm up {step} mm",
                "ARM_HEIGHT_JOG",
                step,
                wait_kind=WAIT_ARM,
                default_timeout_s=DEFAULT_ARM_BLOCK_TIMEOUT_S,
            )
        )
        blocks.append(
            _single(
                f"arm_down_{step}mm",
                "arm_height",
                f"Arm down {step} mm",
                "ARM_HEIGHT_JOG",
                -step,
                wait_kind=WAIT_ARM,
                default_timeout_s=DEFAULT_ARM_BLOCK_TIMEOUT_S,
            )
        )

    for tool_id, tool_name in ((1, "s2"), (2, "gripper")):
        blocks.append(
            _single(
                f"tool_{tool_name}_on",
                "tool",
                f"{tool_name.upper()} on",
                "TOOL_ENABLE",
                tool_id,
                wait_kind=WAIT_SETTLE,
                default_timeout_s=DEFAULT_TOOL_SETTLE_S,
            )
        )
        blocks.append(
            _single(
                f"tool_{tool_name}_off",
                "tool",
                f"{tool_name.upper()} off",
                "TOOL_DISABLE",
                tool_id,
                wait_kind=WAIT_SETTLE,
                default_timeout_s=DEFAULT_TOOL_SETTLE_S,
            )
        )

    for direction_id, direction_name in ((0, "y_pos"), (1, "x_pos"), (2, "x_neg")):
        blocks.append(
            _single(
                f"arm_space_{direction_name}",
                "arm_space",
                f"Switch arm workspace {direction_name.upper()}",
                "ARM_SET_WORKSPACE",
                direction_id,
                doc="Keep current tool, state, and z while switching work space.",
                wait_kind=WAIT_ARM,
                default_timeout_s=DEFAULT_ARM_BLOCK_TIMEOUT_S,
            )
        )

    for step_mm in BASE_TRANSLATE_STEPS_MM:
        d = float(step_mm) * 0.001
        blocks.extend(
            (
                _base_pos_block(f"base_forward_{step_mm}mm", f"Base forward {step_mm} mm", 0.0, d, 0.0),
                _base_pos_block(f"base_back_{step_mm}mm", f"Base back {step_mm} mm", 0.0, -d, 0.0),
                _base_pos_block(f"base_left_{step_mm}mm", f"Base left {step_mm} mm", -d, 0.0, 0.0),
                _base_pos_block(f"base_right_{step_mm}mm", f"Base right {step_mm} mm", d, 0.0, 0.0),
            )
        )

    blocks.extend(
        (
            _base_pos_block("base_cw_90deg", "Base CW 90 deg", 0.0, 0.0, -0.5 * pi),
            _base_pos_block("base_ccw_90deg", "Base CCW 90 deg", 0.0, 0.0, 0.5 * pi),
        )
    )

    blocks.append(
        BlockCommand(
            "arm_show_record_status",
            "status",
            "Show and record arm pose",
            (_frame("ARM_GET_STATUS"),),
            "Read current point, model joint angles, and motor feedback angles.",
            record_status=True,
            wait_kind=WAIT_ARM_STATUS,
            default_timeout_s=DEFAULT_ARM_BLOCK_TIMEOUT_S,
        )
    )

    blocks.extend(
        (
            _climb_gate_block(
                "climb_up_laser_gate",
                "Climb up laser gate",
                "CLIMB_UP_GATE",
                "Run the upstairs auto gate: legs to -30, chassis 0.08 m/s, wait for 0 <= x_pos < 35.",
            ),
            _climb_gate_block(
                "climb_down_laser_gate",
                "Climb down laser gate",
                "CLIMB_DOWN_GATE",
                "Run the downstairs auto gate: wait for height > 65, then advance 5 mm.",
            ),
            _climb_auto_control_block(
                "climb_up_auto_pause",
                "Climb up auto pause",
                "CLIMB_UP_AUTO_PAUSE",
                "Run full upstairs auto and pause after the first 200 mm of the 500 mm drive segment.",
                WAIT_CLIMB,
                DEFAULT_CLIMB_BLOCK_TIMEOUT_S,
            ),
            _climb_auto_control_block(
                "climb_down_auto_pause",
                "Climb down auto pause",
                "CLIMB_DOWN_AUTO_PAUSE",
                "Run full downstairs auto and pause after DOWN_05_ALL_DRIVE_FORWARD_500.",
                WAIT_CLIMB,
                DEFAULT_CLIMB_BLOCK_TIMEOUT_S,
            ),
            _climb_auto_control_block(
                "climb_auto_resume",
                "Climb auto resume",
                "CLIMB_AUTO_RESUME",
                "Resume the paused climb auto flow and wait for final DONE.",
                WAIT_CLIMB_FINAL,
                180.0,
            ),
        )
    )

    blocks.extend(
        (
            _arm_joint_target_block(
                "arm_s1_up_v1_end_from_enable",
                "S1 up v1 arm end from enable",
                5.0,
                -185.0,
                "Move from the S1 enable-end pose to the recorded s1_up_v1 arm end pose.",
            ),
            _arm_joint_target_block(
                "arm_s1_down_v1_end_from_enable",
                "S1 down v1 arm end from enable",
                5.0,
                -190.0,
                "Move from the S1 enable-end pose to the recorded s1_down_v1 arm end pose.",
            ),
        )
    )

    for action, name in CLIMB_TEST_ACTIONS.items():
        if action != 0:
            blocks.append(_climb_test_block(action, name))

    return tuple(blocks)


BLOCK_COMMANDS: Tuple[BlockCommand, ...] = _build_blocks()
BLOCK_BY_ID: Dict[str, BlockCommand] = {block.block_id: block for block in BLOCK_COMMANDS}


def list_block_commands() -> List[BlockCommand]:
    return list(BLOCK_COMMANDS)


def resolve_block_command(token: str) -> BlockCommand:
    key = token.strip().lower()
    if key not in BLOCK_BY_ID:
        raise ValueError(f"unknown block command: {token}")
    return BLOCK_BY_ID[key]


def is_block_command(token: str) -> bool:
    return token.strip().lower() in BLOCK_BY_ID


def build_block_command_sequence(token: str) -> Tuple[bytes, ...]:
    block = resolve_block_command(token)
    frames = []
    for name, values in block.frames:
        frames.append(build_usb_command(name, values if values else None))
    return tuple(frames)


def block_command_frame_records(block: BlockCommand) -> List[Dict[str, Any]]:
    records: List[Dict[str, Any]] = []
    for name, values in block.frames:
        command = resolve_command(name)
        records.append(
            {
                "command": command.name,
                "cmd": command.cmd,
                "values": list(values),
                "has_float_payload": bool(values),
            }
        )
    return records


def build_block_flow_step(
    block_id: str,
    index: int,
    note: Optional[str] = None,
    snapshot: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    block = resolve_block_command(block_id)
    step: Dict[str, Any] = {
        "index": int(index),
        "block_id": block.block_id,
        "category": block.category,
        "label": block.label,
        "record_status": bool(block.record_status),
        "wait_kind": block.wait_kind,
        "timeout_s": block.default_timeout_s,
        "frames": block_command_frame_records(block),
    }
    if block.doc:
        step["doc"] = block.doc
    if note:
        step["note"] = note
    if snapshot is not None:
        step["snapshot"] = snapshot
    return step


def build_interrupt_placeholder_step(
    index: int,
    slot: int,
    group: Optional[str] = None,
    note: Optional[str] = None,
) -> Dict[str, Any]:
    step: Dict[str, Any] = {
        "index": int(index),
        "block_id": f"interrupt_slot_{int(slot):03d}",
        "category": "interrupt",
        "label": f"Interrupt slot {int(slot):03d}",
        "record_status": False,
        "wait_kind": WAIT_NONE,
        "timeout_s": 0.0,
        "frames": [],
        "interrupt_slot": int(slot),
        "interrupt_status": "empty",
        "placeholder": True,
    }
    if group:
        step["interrupt_group"] = group
    if note:
        step["note"] = note
    return step


def is_interrupt_placeholder_step(step: Dict[str, Any]) -> bool:
    if not isinstance(step, dict):
        return False
    if bool(step.get("placeholder")) and step.get("interrupt_slot") is not None:
        return True
    return str(step.get("interrupt_status") or "").strip().lower() == "empty"


def fill_interrupt_slot(
    flow: Dict[str, Any],
    slot: int,
    block_id: str,
    note: Optional[str] = None,
) -> Dict[str, Any]:
    steps = flow.get("steps")
    if not isinstance(steps, list):
        raise ValueError("flow steps must be a list")

    slot = int(slot)
    if slot <= 0:
        raise ValueError("slot must be a positive integer")

    for index, step in enumerate(steps):
        if not isinstance(step, dict):
            continue
        if int(step.get("interrupt_slot") or 0) != slot:
            continue

        group = step.get("interrupt_group")
        old_note = step.get("note")
        replacement = build_block_flow_step(block_id, int(step.get("index") or (index + 1)), note=note)
        replacement["interrupt_slot"] = slot
        replacement["interrupt_status"] = "filled"
        replacement["placeholder"] = False
        if group:
            replacement["interrupt_group"] = group
        if old_note and note is None:
            replacement["slot_note"] = old_note
        steps[index] = replacement
        flow["step_count"] = len(steps)
        return replacement

    raise ValueError(f"interrupt slot {slot} not found")


def clear_interrupt_slot(flow: Dict[str, Any], slot: int) -> Dict[str, Any]:
    steps = flow.get("steps")
    if not isinstance(steps, list):
        raise ValueError("flow steps must be a list")

    slot = int(slot)
    if slot <= 0:
        raise ValueError("slot must be a positive integer")

    for index, step in enumerate(steps):
        if not isinstance(step, dict):
            continue
        if int(step.get("interrupt_slot") or 0) != slot:
            continue

        group = step.get("interrupt_group")
        note = step.get("slot_note") or step.get("note")
        replacement = build_interrupt_placeholder_step(
            int(step.get("index") or (index + 1)),
            slot,
            group=str(group) if group else None,
            note=str(note) if note else None,
        )
        steps[index] = replacement
        flow["step_count"] = len(steps)
        return replacement

    raise ValueError(f"interrupt slot {slot} not found")


def list_interrupt_slots(flow: Dict[str, Any]) -> List[Dict[str, Any]]:
    steps = flow.get("steps")
    if not isinstance(steps, list):
        raise ValueError("flow steps must be a list")

    slots: List[Dict[str, Any]] = []
    for index, step in enumerate(steps, 1):
        if not isinstance(step, dict) or step.get("interrupt_slot") is None:
            continue
        status = str(step.get("interrupt_status") or "filled")
        if is_interrupt_placeholder_step(step):
            status = "empty"
        slots.append(
            {
                "index": int(step.get("index") or index),
                "slot": int(step.get("interrupt_slot") or 0),
                "group": step.get("interrupt_group"),
                "status": status,
                "block_id": step.get("block_id"),
                "label": step.get("label"),
            }
        )
    return slots


def clear_all_interrupt_slots(flow: Dict[str, Any]) -> int:
    slots = list_interrupt_slots(flow)
    for slot in slots:
        clear_interrupt_slot(flow, int(slot["slot"]))
    return len(slots)


def build_block_flow_data(
    name: str,
    created_at: Optional[str],
    steps: Iterable[Dict[str, Any]],
) -> Dict[str, Any]:
    step_list = list(steps)
    return {
        "type": "r2_block_flow",
        "version": 1,
        "name": name,
        "created_at": created_at,
        "step_count": len(step_list),
        "steps": step_list,
    }


def _c_identifier(text: str) -> str:
    chars = []
    for ch in text:
        if ch.isalnum() or ch == "_":
            chars.append(ch)
        else:
            chars.append("_")
    ident = "".join(chars).strip("_") or "r2_block_flow"
    if ident[0].isdigit():
        ident = f"flow_{ident}"
    return ident


def _c_float(value: float) -> str:
    return f"{float(value):.9g}f"


def block_flow_to_c_source(flow: Dict[str, Any], symbol: Optional[str] = None) -> str:
    if flow.get("type") != "r2_block_flow":
        raise ValueError("not an r2_block_flow object")
    steps = flow.get("steps")
    if not isinstance(steps, list):
        raise ValueError("flow steps must be a list")

    flow_name = str(flow.get("name") or "r2_block_flow")
    array_name = _c_identifier(symbol or flow_name)
    lines = [
        "/* Auto-generated from PC_USB_Serial_Tool block flow. */",
        "#include <stdint.h>",
        "",
        "typedef struct",
        "{",
        "    uint8_t cmd;",
        "    uint8_t has_float_payload;",
        "    float f[4];",
        "} R2_BlockFlowFrame_t;",
        "",
        "typedef struct",
        "{",
        "    const char *block_id;",
        "    uint8_t frame_count;",
        "    R2_BlockFlowFrame_t frames[4];",
        "} R2_BlockFlowStep_t;",
        "",
        f"#define {array_name.upper()}_STEP_COUNT {len(steps)}U",
        f"static const R2_BlockFlowStep_t {array_name}[{array_name.upper()}_STEP_COUNT] =",
        "{",
    ]

    for step in steps:
        if not isinstance(step, dict):
            raise ValueError("each flow step must be an object")
        block_id = str(step.get("block_id") or "")
        frames = step.get("frames")
        if not isinstance(frames, list):
            raise ValueError(f"step {block_id or '?'} frames must be a list")
        if len(frames) > 4:
            raise ValueError(f"step {block_id or '?'} has more than 4 frames")

        lines.append("    {")
        lines.append(f"        \"{block_id}\",")
        lines.append(f"        {len(frames)}U,")
        lines.append("        {")
        for frame in frames:
            if not isinstance(frame, dict):
                raise ValueError(f"step {block_id or '?'} has an invalid frame")
            cmd = int(frame.get("cmd", 0))
            values = list(frame.get("values") or [])
            padded = (values + [0.0, 0.0, 0.0, 0.0])[:4]
            has_payload = 1 if frame.get("has_float_payload") else 0
            value_text = ", ".join(_c_float(float(v)) for v in padded)
            lines.append(f"            {{0x{cmd:02X}U, {has_payload}U, {{{value_text}}}}},")
        for _ in range(4 - len(frames)):
            lines.append("            {0x00U, 0U, {0.0f, 0.0f, 0.0f, 0.0f}},")
        lines.append("        },")
        lines.append("    },")

    lines.extend(["};", ""])
    return "\n".join(lines)


def arm_status_snapshot(payload: Dict[str, Any], note: Optional[str] = None) -> Dict[str, Any]:
    arm = payload.get("arm") if isinstance(payload.get("arm"), dict) else payload
    record: Dict[str, Any] = {
        "type": "r2_arm_pose_snapshot",
        "version": 1,
        "tool_world_mm": arm.get("tool_world_mm"),
        "theta_rad": arm.get("theta_rad"),
        "theta_deg": arm.get("theta_deg"),
        "motor_feedback_rad": arm.get("motor_feedback_rad"),
        "motor_feedback_deg": arm.get("motor_feedback_deg"),
        "motor_feedback_ok": arm.get("motor_feedback_ok"),
        "tool": arm.get("tool"),
        "tool_name": arm.get("tool_name"),
        "tool_state": arm.get("tool_state"),
        "tool_state_name": arm.get("tool_state_name"),
        "target_direction": arm.get("target_direction"),
        "target_direction_name": arm.get("target_direction_name"),
        "last_update_ms": arm.get("last_update_ms"),
        "output_apply_count": arm.get("output_apply_count"),
    }
    if note:
        record["note"] = note
    return record
