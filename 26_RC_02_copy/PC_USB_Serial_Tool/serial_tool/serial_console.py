from __future__ import annotations

import importlib
import json
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple

from . import block_commands as _block_commands
from .block_commands import (
    DEFAULT_ARM_BLOCK_TIMEOUT_S,
    DEFAULT_CLIMB_BLOCK_TIMEOUT_S,
    DEFAULT_TOOL_SETTLE_S,
    WAIT_ARM,
    WAIT_ARM_STATUS,
    WAIT_CHASSIS,
    WAIT_CLIMB,
    WAIT_CLIMB_FINAL,
    WAIT_FLOW,
    WAIT_NONE,
    WAIT_SETTLE,
    arm_status_snapshot,
    block_flow_to_c_source,
    build_block_flow_data,
    build_block_flow_step,
    build_block_command_sequence,
    clear_all_interrupt_slots,
    clear_interrupt_slot,
    fill_interrupt_slot,
    is_block_command,
    is_interrupt_placeholder_step,
    list_interrupt_slots,
    list_block_commands,
    resolve_block_command,
)
from .commands import build_climb_test_shortcut_sequence, build_query_command, build_usb_command, list_commands
from .protocol import UsbFrame, UsbStreamParser, bytes_to_hex, parse_hex, pack_4float_frame, pack_usb_frame
from .status import CLIMB_TEST_ACTIONS, decode_usb_frame


CLIMB_DONE_STATE = 22
CLIMB_ERROR_STATE = 23
DEFAULT_CLIMB_WAIT_TIMEOUT_S = 40.0
DEFAULT_CLIMB_FINAL_TIMEOUT_S = 180.0
DEFAULT_CLIMB_POLL_INTERVAL_S = 0.1
CLIMB_COMMAND_SETTLE_S = 0.05
DEFAULT_BLOCK_FLOW_TIMEOUT_S = DEFAULT_CLIMB_BLOCK_TIMEOUT_S
DEFAULT_BLOCK_FLOW_ARM_SETTLE_S = DEFAULT_ARM_BLOCK_TIMEOUT_S
DEFAULT_BLOCK_FLOW_POLL_INTERVAL_S = 0.1
ARM_FEEDBACK_STILL_TOL_DEG = 0.5
ARM_FEEDBACK_SETTLE_STABLE_SAMPLES = 3
ARM_FEEDBACK_POLL_INTERVAL_S = 0.1
FLOW_AUTOSAVE_FILE = ".climb_flow_autosave.json"
BLOCK_FLOW_AUTOSAVE_FILE = ".block_flow_autosave.json"
TEST_FLOW_FILES = {
    "s1_up": "s1_up_v2.json",
    "s1_down": "s1_down_v2.json",
    "s1_up_s2_down": "s1_up_s2_down_v2.json",
    "s1_down_s2_up": "s1_down_s2_up_v2.json",
    "s1_down_s2_down": "s1_down_s2_down_v2.json",
}
S1_PREFIX_FLOW_KEYS = {"s1_up_v2", "s1_down_v2"}
ARM_JOG_STEPS_MM = (1, 5, 10, 20, 50, 100, 200, 500)
ARM_JOINT_JOG_STEPS_DEG = (1, 5, 10, 20, 30, 60, 90)
CLIMB_TEST_ACTION_BY_NAME = {name: action for action, name in CLIMB_TEST_ACTIONS.items()}
CLIMB_TEST_SHORTCUTS = {
    f"climb_{name.lower()}": action
    for action, name in CLIMB_TEST_ACTIONS.items()
    if action != 0
}


def _refresh_block_commands() -> None:
    module = importlib.reload(_block_commands)
    globals()["block_flow_to_c_source"] = module.block_flow_to_c_source
    globals()["build_block_flow_data"] = module.build_block_flow_data
    globals()["build_block_flow_step"] = module.build_block_flow_step
    globals()["build_block_command_sequence"] = module.build_block_command_sequence
    globals()["clear_all_interrupt_slots"] = module.clear_all_interrupt_slots
    globals()["clear_interrupt_slot"] = module.clear_interrupt_slot
    globals()["fill_interrupt_slot"] = module.fill_interrupt_slot
    globals()["is_block_command"] = module.is_block_command
    globals()["is_interrupt_placeholder_step"] = module.is_interrupt_placeholder_step
    globals()["list_interrupt_slots"] = module.list_interrupt_slots
    globals()["list_block_commands"] = module.list_block_commands
    globals()["resolve_block_command"] = module.resolve_block_command


def _is_block_command_live(token: str) -> bool:
    _refresh_block_commands()
    return is_block_command(token)


def list_serial_ports() -> List[str]:
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise RuntimeError("pyserial is not installed. Run: python -m pip install -r requirements.txt") from exc

    ports = []
    for port in list_ports.comports():
        desc = port.description or ""
        hwid = port.hwid or ""
        ports.append(f"{port.device}\t{desc}\t{hwid}")
    return ports


def open_serial(port: str, baud: int, timeout: float = 0.05):
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("pyserial is not installed. Run: python -m pip install -r requirements.txt") from exc

    return serial.Serial(port=port, baudrate=baud, timeout=timeout)


def print_decoded_frame(frame, json_lines: bool = False) -> None:
    decoded = decode_usb_frame(frame)
    if json_lines:
        print(json.dumps(decoded, ensure_ascii=False, separators=(",", ":")), flush=True)
    else:
        print(json.dumps(decoded, ensure_ascii=False, indent=2), flush=True)


def monitor_serial(port: str, baud: int, json_lines: bool = False, raw_rx: bool = False) -> None:
    parser = UsbStreamParser()
    with open_serial(port, baud) as ser:
        print(f"Opened {ser.port} @ {baud}. Press Ctrl+C to stop.", flush=True)
        try:
            while True:
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue
                if raw_rx:
                    print(f"RX {bytes_to_hex(chunk)}", flush=True)
                for frame in parser.feed(chunk):
                    print_decoded_frame(frame, json_lines=json_lines)
        except KeyboardInterrupt:
            print("\nStopped.", flush=True)


def send_and_read(port: str, baud: int, frame: bytes, wait_s: float = 0.0, json_lines: bool = False) -> None:
    send_sequence_and_read(port, baud, [frame], wait_s=wait_s, json_lines=json_lines)


def send_sequence_and_read(
    port: str,
    baud: int,
    frames: Iterable[bytes],
    wait_s: float = 0.0,
    json_lines: bool = False,
    delay_s: float = 0.02,
) -> None:
    parser = UsbStreamParser()
    with open_serial(port, baud) as ser:
        for frame in frames:
            ser.write(frame)
            ser.flush()
            print(f"TX {bytes_to_hex(frame)}", flush=True)
            if delay_s > 0:
                time.sleep(delay_s)

        end = time.monotonic() + max(0.0, wait_s)
        while time.monotonic() < end:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                for parsed in parser.feed(chunk):
                    print_decoded_frame(parsed, json_lines=json_lines)


def poll_serial(port: str, baud: int, command: str, rate: float, count: int = 0, json_lines: bool = False) -> None:
    if rate <= 0:
        raise ValueError("rate must be > 0")

    frame = build_query_command(command)
    interval = 1.0 / rate
    parser = UsbStreamParser()
    sent = 0

    with open_serial(port, baud) as ser:
        print(f"Polling {command} at {rate:g} Hz on {ser.port}. Press Ctrl+C to stop.", flush=True)
        next_send = time.monotonic()
        try:
            while count <= 0 or sent < count:
                now = time.monotonic()
                if now >= next_send:
                    ser.write(frame)
                    ser.flush()
                    sent += 1
                    next_send = now + interval

                chunk = ser.read(ser.in_waiting or 1)
                if chunk:
                    for parsed in parser.feed(chunk):
                        print_decoded_frame(parsed, json_lines=json_lines)
                time.sleep(0.001)
        except KeyboardInterrupt:
            print("\nStopped.", flush=True)


class SerialShell:
    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.parser = UsbStreamParser()
        self.stop_event = threading.Event()
        self.ser = None
        self.reader_thread: Optional[threading.Thread] = None
        self.frame_queue: "queue.Queue[UsbFrame]" = queue.Queue()
        self.flow_name: Optional[str] = None
        self.flow_started_at: Optional[str] = None
        self.flow_steps: List[Dict[str, Any]] = []
        self.last_flow_candidate: Optional[Dict[str, Any]] = None
        self.flow_autosave_path = Path(FLOW_AUTOSAVE_FILE)
        self.block_flow_name: Optional[str] = None
        self.block_flow_started_at: Optional[str] = None
        self.block_flow_steps: List[Dict[str, Any]] = []
        self.last_block_flow_candidate: Optional[Dict[str, Any]] = None
        self.block_flow_autosave_path = Path(BLOCK_FLOW_AUTOSAVE_FILE)
        self.test_flow_path: Optional[Path] = None
        self.test_flow_data: Optional[Dict[str, Any]] = None
        self.test_flow_name: Optional[str] = None
        self.test_interrupt_bounds: Optional[Tuple[int, int]] = None
        self.test_next_slot: Optional[int] = None
        self.test_executed_slots: Set[int] = set()
        self.test_timeout_s: float = DEFAULT_BLOCK_FLOW_TIMEOUT_S
        self.test_arm_settle_s: float = DEFAULT_BLOCK_FLOW_ARM_SETTLE_S
        self.completed_s1_prefixes: Set[str] = set()

    def run(self) -> None:
        self.ser = open_serial(self.port, self.baud)
        print(f"Opened {self.ser.port} @ {self.baud}. Type 'help' for commands.", flush=True)
        self._print_flow_autosave_hint()
        self._print_block_flow_autosave_hint()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        try:
            while not self.stop_event.is_set():
                try:
                    line = input("> ")
                except EOFError:
                    break
                if not self._handle_line(line):
                    break
        finally:
            self.stop_event.set()
            if self.reader_thread:
                self.reader_thread.join(timeout=0.5)
            if self.ser:
                self.ser.close()

    def _reader_loop(self) -> None:
        assert self.ser is not None
        while not self.stop_event.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception as exc:  # pragma: no cover - serial hardware path
                print(f"\nSerial read error: {exc}", file=sys.stderr, flush=True)
                self.stop_event.set()
                return
            if not chunk:
                continue
            for frame in self.parser.feed(chunk):
                self.frame_queue.put(frame)
                print()
                print_decoded_frame(frame)
                print("> ", end="", flush=True)

    def _write(self, frame: bytes) -> None:
        assert self.ser is not None
        self.ser.write(frame)
        self.ser.flush()
        print(f"TX {bytes_to_hex(frame)}", flush=True)

    def _write_sequence(self, frames: Iterable[bytes], delay_s: float = 0.02) -> None:
        for frame in frames:
            self._write(frame)
            time.sleep(delay_s)

    def _handle_line(self, line: str) -> bool:
        parts = line.strip().split()
        if not parts:
            return True

        op = parts[0].lower()
        args = parts[1:]

        try:
            if op in {"exit", "quit"}:
                return False
            if op == "help":
                self._print_help()
            elif op == "commands":
                self._print_commands()
            elif op == "blocks":
                self._print_blocks()
            elif op == "block":
                self._run_block(args)
            elif op == "arm_snapshot":
                self._arm_snapshot(args)
            elif op == "block_flow_start":
                self._block_flow_start(args)
            elif op == "block_flow_add":
                self._block_flow_add(args)
            elif op in {"block_flow_confirm", "block_flow_save"}:
                self._block_flow_confirm(args)
            elif op in {"block_flow_discard", "block_flow_drop"}:
                self._block_flow_discard()
            elif op == "block_flow_show":
                self._block_flow_show()
            elif op == "block_flow_export":
                self._block_flow_export(args)
            elif op == "block_flow_export_c":
                self._block_flow_export_c(args)
            elif op == "block_flow_run":
                self._block_flow_run_command(args)
            elif op == "weapon_dock_test":
                if args:
                    raise ValueError("usage: weapon_dock_test")
                self._write(build_usb_command("FLOW_WEAPON_DOCK_TEST"))
            elif op == "weapon_dock_test_wait":
                timeout_s = self._optional_timeout(args, DEFAULT_BLOCK_FLOW_TIMEOUT_S)
                self._write(build_usb_command("FLOW_WEAPON_DOCK_TEST"))
                time.sleep(CLIMB_COMMAND_SETTLE_S)
                self._wait_for_task_flow(timeout_s, expected_flow_id=9)
            elif op == "weapon_dock_status":
                if args:
                    raise ValueError("usage: weapon_dock_status")
                self._write(build_usb_command("FLOW_GET_STATUS"))
            elif op == "weapon_chassis_done":
                if args:
                    raise ValueError("usage: weapon_chassis_done")
                self._write(build_usb_command("FLOW_CHASSIS_MOVE_DONE"))
            elif op == "weapon_dock_done":
                if args:
                    raise ValueError("usage: weapon_dock_done")
                self._write(build_usb_command("FLOW_DOCK_DONE"))
            elif op in {"block_flow_slots", "interrupt_slots"}:
                self._block_flow_slots(args)
            elif op in {"block_flow_slot_set", "interrupt_set"}:
                self._block_flow_slot_set(args)
            elif op in {"block_flow_slot_clear", "interrupt_clear"}:
                self._block_flow_slot_clear(args)
            elif op in {"block_flow_slot_clear_all", "interrupt_clear_all"}:
                self._block_flow_slot_clear_all(args)
            elif op == "block_flow_recover":
                self._block_flow_recover(args)
            elif op == "block_flow_autosave":
                self._block_flow_autosave_command(args)
            elif op == "block_flow_clear":
                self._block_flow_clear()
            elif op == "test":
                self._test_start(args)
            elif op == "test_status":
                self._test_status()
            elif op == "test_show":
                self._test_show(args)
            elif op == "test_continue":
                self._test_continue(args)
            elif op == "test_export":
                self._test_export(args)
            elif op == "test_clear":
                self._test_clear()
            elif op == "flow_start":
                self._flow_start(args)
            elif op == "flow_save":
                self._flow_save(args)
            elif op in {"flow_confirm", "flow_ask_save"}:
                self._flow_confirm(args)
            elif op == "flow_add":
                self._flow_add(args)
            elif op == "flow_show":
                self._flow_show()
            elif op == "flow_export":
                self._flow_export(args)
            elif op == "flow_clear":
                self._flow_clear()
            elif op == "flow_recover":
                self._flow_recover(args)
            elif op == "flow_autosave":
                self._flow_autosave_command(args)
            elif op == "pack":
                self._pack(args)
            elif op == "send":
                self._send_named(args)
            elif op == "query":
                self._query(args)
            elif op in {"raw", "hex"}:
                self._write(parse_hex(" ".join(args)))
            elif op == "usb":
                self._write(build_usb_command("SYS_SWITCH_SOURCE", [1.0]))
            elif op == "usart":
                self._write(build_usb_command("SYS_SWITCH_SOURCE", [0.0]))
            elif op == "enable":
                self._write(build_usb_command("SYS_ENABLE"))
            elif op == "disable":
                self._write(build_usb_command("SYS_DISABLE"))
            elif op == "stop":
                self._write(build_usb_command("SYS_STOP"))
            elif op == "status":
                self._write(build_usb_command("ROBOT_GET_STATUS"))
            elif op == "chs_enable":
                self._write(build_usb_command("CHS_ENABLE"))
            elif op == "chs_disable":
                self._write(build_usb_command("CHS_DISABLE"))
            elif op == "chs_stop":
                self._write(build_usb_command("CHS_STOP"))
            elif op == "chs_status":
                self._write(build_usb_command("CHS_GET_STATUS"))
            elif op == "chs_mode":
                self._write(build_usb_command("CHS_SET_MODE", self._need_floats(args, 1, "chs_mode MODE")))
            elif op == "vel":
                values = self._need_floats(args, 3, "vel VX VY YAW_DATA")
                if len(values) < 4:
                    values.append(0.0)
                self._write(build_usb_command("CHS_SET_VEL", values))
            elif op == "pos":
                self._write(build_usb_command("CHS_SET_POS", self._need_floats(args, 3, "pos DX DY YAW_DATA")))
            elif op == "arm_enable":
                self._write(build_usb_command("ARM_ENABLE"))
            elif op == "arm_disable":
                self._write(build_usb_command("ARM_DISABLE"))
            elif op == "arm_stop":
                self._write(build_usb_command("ARM_STOP"))
            elif op in {"arm_status", "arm_state", "arm_query"}:
                self._write(build_usb_command("ARM_GET_STATUS"))
            elif op in {"throw_block", "block_throw"}:
                if len(args) != 1:
                    raise ValueError("usage: throw_block X+|X-")
                direction = self._parse_direction(args[0])
                if direction not in (1, 2):
                    raise ValueError("throw direction must be X+ or X-")
                self._write(build_usb_command("FLOW_THROW_BLOCK", [float(direction)]))
            elif op == "arm_target":
                self._write(build_usb_command("ARM_SET_TARGET", self._need_floats(args, 4, "arm_target TOOL STATE Z_MM YAW_RAD")))
            elif op in {"arm_space", "arm_workspace"}:
                if len(args) != 1:
                    raise ValueError("usage: arm_space Y+|X+|X-")
                self._write(build_usb_command("ARM_SET_WORKSPACE", [float(self._parse_direction(args[0]))]))
            elif op in {"arm_xyz", "xyz"}:
                self._write(build_usb_command("ARM_SET_TARGET_XYZ", self._need_floats(args, 3, "arm_xyz X_MM Y_MM Z_MM")))
            elif op in {"arm_z_jog", "arm_jog_z"}:
                self._write(build_usb_command("ARM_HEIGHT_JOG", self._need_floats(args, 1, "arm_z_jog DELTA_Z_MM")))
            elif op in {"arm_up", "arm_raise"}:
                self._write(build_usb_command("ARM_HEIGHT_JOG", [self._parse_jog_amount(args, positive=True)]))
            elif op in {"arm_down", "arm_lower"}:
                self._write(build_usb_command("ARM_HEIGHT_JOG", [self._parse_jog_amount(args, positive=False)]))
            elif op in {"arm_zmin", "arm_min"}:
                self._write(build_usb_command("ARM_HEIGHT_LIMIT", [0.0]))
            elif op in {"arm_zmax", "arm_max"}:
                self._write(build_usb_command("ARM_HEIGHT_LIMIT", [1.0]))
            elif op == "arm_posture":
                if len(args) != 2:
                    raise ValueError("usage: arm_posture TOOL STATE")
                self._write(build_usb_command("ARM_SET_POSTURE", [float(self._parse_tool(args[0])), float(args[1])]))
            elif op in {f"arm_up{step}" for step in ARM_JOG_STEPS_MM}:
                self._write(build_usb_command("ARM_HEIGHT_JOG", [float(op.removeprefix("arm_up"))]))
            elif op in {f"arm_down{step}" for step in ARM_JOG_STEPS_MM}:
                self._write(build_usb_command("ARM_HEIGHT_JOG", [-float(op.removeprefix("arm_down"))]))
            elif op in {"arm_j2_cw", "arm_j2_ccw", "arm_j3_cw", "arm_j3_ccw"}:
                joint = 2 if "_j2_" in op else 3
                sign = -1.0 if op.endswith("_cw") else 1.0
                self._write(build_usb_command("ARM_JOINT_JOG", [float(joint), sign * self._parse_joint_jog_amount(args)]))
            elif op in {
                f"arm_j{joint}_{direction}{step}"
                for joint in (2, 3)
                for direction in ("cw", "ccw")
                for step in ARM_JOINT_JOG_STEPS_DEG
            }:
                name_parts = op.split("_")
                joint = int(name_parts[1].removeprefix("j"))
                direction_step = name_parts[2]
                sign = -1.0 if direction_step.startswith("cw") else 1.0
                step = float(direction_step.removeprefix("ccw").removeprefix("cw"))
                self._write(build_usb_command("ARM_JOINT_JOG", [float(joint), sign * step]))
            elif op == "tool_enable":
                self._write(build_usb_command("TOOL_ENABLE", self._optional_tool_payload(args, "tool_enable [TOOL]")))
            elif op == "tool_on":
                self._write(build_usb_command("TOOL_ENABLE", self._optional_tool_payload(args, "tool_on [TOOL]")))
            elif op == "tool_disable":
                self._write(build_usb_command("TOOL_DISABLE", self._optional_tool_payload(args, "tool_disable [TOOL]")))
            elif op == "tool_off":
                self._write(build_usb_command("TOOL_DISABLE", self._optional_tool_payload(args, "tool_off [TOOL]")))
            elif op == "tool_stop":
                self._write(build_usb_command("TOOL_STOP"))
            elif op == "tool_status":
                self._write(build_usb_command("TOOL_GET_STATUS"))
            elif op == "tool_target":
                self._write(build_usb_command("TOOL_SET_MODE", self._need_floats(args, 4, "tool_target TOOL STATE Z_MM YAW_RAD")))
            elif op == "tool_s2":
                values = self._need_floats(args, 2, "tool_s2 Z_MM YAW_RAD")
                self._write(build_usb_command("TOOL_SET_MODE", [1.0, 1.0, values[0], values[1]]))
            elif op == "tool_gripper":
                values = self._need_floats(args, 2, "tool_gripper Z_MM YAW_RAD")
                self._write(build_usb_command("TOOL_SET_MODE", [2.0, 2.0, values[0], values[1]]))
            elif op == "tool_stow":
                values = self._need_floats(args, 3, "tool_stow TOOL Z_MM YAW_RAD")
                self._write(build_usb_command("TOOL_SET_MODE", [values[0], 0.0, values[1], values[2]]))
            elif op == "tune_start":
                pass_count = float(args[0]) if args else 1.0
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("SYS_ENABLE"),
                        build_usb_command("CHS_ENABLE"),
                        build_usb_command("YAW_TUNE_START", [pass_count]),
                        build_usb_command("YAW_TUNE_GET_STATUS"),
                    ]
                )
            elif op == "tune_status":
                self._write(build_usb_command("YAW_TUNE_GET_STATUS"))
            elif op == "tune_stop":
                self._write_sequence(
                    [
                        build_usb_command("YAW_TUNE_STOP"),
                        build_usb_command("YAW_TUNE_GET_STATUS"),
                    ]
                )
            elif op in {"climb_auto", "climb_run", "climb_up_auto", "climb_up_run", "climb_upstairs_auto", "climb_upstairs_run"}:
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_UP_AUTO"),
                        build_usb_command("CLIMB_GET_STATUS"),
                    ]
                )
            elif op in {"climb_auto_wait", "climb_run_wait", "climb_up_auto_wait", "climb_up_run_wait", "climb_upstairs_auto_wait", "climb_upstairs_run_wait"}:
                timeout_s = self._optional_timeout(args, DEFAULT_CLIMB_FINAL_TIMEOUT_S)
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_UP_AUTO"),
                    ]
                )
                time.sleep(CLIMB_COMMAND_SETTLE_S)
                self._wait_for_climb(timeout_s, final_state=True)
            elif op in {"climb_down_auto", "climb_down_run", "climb_downstairs_auto", "climb_downstairs_run"}:
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_DOWN_AUTO"),
                        build_usb_command("CLIMB_GET_STATUS"),
                    ]
                )
            elif op in {"climb_down_auto_wait", "climb_down_run_wait", "climb_downstairs_auto_wait", "climb_downstairs_run_wait"}:
                timeout_s = self._optional_timeout(args, DEFAULT_CLIMB_FINAL_TIMEOUT_S)
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_DOWN_AUTO"),
                    ]
                )
                time.sleep(CLIMB_COMMAND_SETTLE_S)
                self._wait_for_climb(timeout_s, final_state=True)
            elif op in {"climb_step", "climb_upstairs_step"}:
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_UP_STEP"),
                        build_usb_command("CLIMB_GET_STATUS"),
                    ]
                )
            elif op in {"climb_down_step", "climb_downstairs_step"}:
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_DOWN_STEP"),
                        build_usb_command("CLIMB_GET_STATUS"),
                    ]
                )
            elif op in {"climb_step_wait", "climb_next", "climb_up_step_wait", "climb_up_next", "climb_upstairs_step_wait", "climb_upstairs_next"}:
                timeout_s = self._optional_timeout(args)
                self._wait_for_climb(timeout_s, final_state=False)
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_UP_STEP"),
                    ]
                )
                time.sleep(CLIMB_COMMAND_SETTLE_S)
                self._wait_for_climb(timeout_s, final_state=False)
            elif op in {"climb_down_step_wait", "climb_down_next", "climb_downstairs_step_wait", "climb_downstairs_next"}:
                timeout_s = self._optional_timeout(args)
                self._wait_for_climb(timeout_s, final_state=False)
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_DOWN_STEP"),
                    ]
                )
                time.sleep(CLIMB_COMMAND_SETTLE_S)
                self._wait_for_climb(timeout_s, final_state=False)
            elif op == "climb_wait":
                self._wait_for_climb(self._optional_timeout(args), final_state=False)
            elif op == "climb_wait_then":
                if not args:
                    raise ValueError("usage: climb_wait_then NAME [float...]")
                self._wait_for_climb(DEFAULT_CLIMB_WAIT_TIMEOUT_S, final_state=False)
                self._send_named(args)
            elif op in {"climb_tests", "climb_test_list"}:
                self._print_climb_tests()
            elif op == "climb_test":
                self._climb_test(args, wait=False)
            elif op == "climb_test_wait":
                self._climb_test(args, wait=True)
            elif op in CLIMB_TEST_SHORTCUTS:
                timeout_s = self._optional_timeout(args) if args else DEFAULT_CLIMB_WAIT_TIMEOUT_S
                self._write_climb_test_action(CLIMB_TEST_SHORTCUTS[op], wait=bool(args), timeout_s=timeout_s)
            elif op == "climb_enable":
                self._write(build_usb_command("CLIMB_ENABLE"))
            elif op == "climb_disable":
                self._write(build_usb_command("CLIMB_DISABLE"))
            elif op == "climb_stop":
                self._write(build_usb_command("CLIMB_STOP"))
            elif op == "climb_status":
                self._write(build_usb_command("CLIMB_GET_STATUS"))
            elif op == "climb_ctrl":
                self._write(build_usb_command("CLIMB_SET_CTRL", self._need_floats(args, 3, "climb_ctrl ENABLE STEP AUTO")))
            elif "activate" in op and (".venv" in op or "scripts" in op):
                print("You are already inside the serial shell. Type 'exit' first, then run PowerShell commands.", flush=True)
            elif _is_block_command_live(op):
                if self.block_flow_name is not None:
                    self._block_flow_add([op] + args)
                elif self._test_is_active():
                    self._test_block([op] + args)
                else:
                    if args:
                        raise ValueError(f"{op} does not accept arguments unless a block flow is active")
                    self._execute_block(op)
            else:
                self._send_direct_or_unknown(parts)
        except Exception as exc:
            print(f"Error: {exc}", flush=True)
        return True

    def _pack(self, args: List[str]) -> None:
        if not args:
            raise ValueError("usage: pack NAME [float...]")
        frame = build_usb_command(args[0], [float(x) for x in args[1:]])
        print(bytes_to_hex(frame), flush=True)

    def _send_named(self, args: List[str]) -> None:
        if not args:
            raise ValueError("usage: send NAME [float...]")
        if _is_block_command_live(args[0]):
            if len(args) > 1:
                raise ValueError(f"{args[0]} does not accept float payload values")
            self._write_sequence(build_block_command_sequence(args[0]))
            return
        frames = build_climb_test_shortcut_sequence(args[0])
        if frames is not None:
            if len(args) > 1:
                raise ValueError(f"{args[0]} does not accept float payload values; use {args[0]} [timeout_s]")
            self._write_sequence(frames)
            return
        frame = build_usb_command(args[0], [float(x) for x in args[1:]])
        self._write(frame)

    def _send_direct_or_unknown(self, parts: List[str]) -> None:
        try:
            values = [float(x) for x in parts[1:]]
            frame = build_usb_command(parts[0], values)
        except ValueError:
            print(f"Unknown command: {parts[0].lower()}. Type 'help'.", flush=True)
            return
        self._write(frame)

    def _query(self, args: List[str]) -> None:
        if not args:
            raise ValueError("usage: query ROBOT|TUNE|CLIMB|SYS|CHS|ARM|TOOL")
        self._write(build_query_command(args[0]))

    @staticmethod
    def _print_blocks() -> None:
        _refresh_block_commands()
        for block in list_block_commands():
            marker = " record" if block.record_status else ""
            print(f"{block.block_id:<28} {block.category:<12} {block.label}{marker}", flush=True)

    def _run_block(self, args: List[str]) -> None:
        if not args:
            raise ValueError("usage: block BLOCK_ID [note...]")
        if self.block_flow_name is not None:
            self._block_flow_add(args)
            return
        if self._test_is_active():
            self._test_block(args)
            return
        if len(args) != 1:
            raise ValueError("usage: block BLOCK_ID")
        _refresh_block_commands()
        self._execute_block(args[0])

    def _arm_snapshot(self, args: List[str]) -> None:
        if len(args) > 2:
            raise ValueError("usage: arm_snapshot [file.json] [note]")
        payload = self._request_arm_status_payload()
        note = args[1] if len(args) == 2 else None
        record = arm_status_snapshot(payload, note=note)
        text = json.dumps(record, ensure_ascii=False, indent=2)
        if args:
            path = Path(args[0])
            path.write_text(text + "\n", encoding="utf-8")
            print(f"Arm snapshot saved: {path}", flush=True)
        else:
            print(text, flush=True)

    def _request_arm_status_payload(self, timeout_s: float = 1.0) -> Dict[str, Any]:
        self._drain_frames()
        self._write(build_usb_command("ARM_GET_STATUS"))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                frame = self.frame_queue.get(timeout=min(0.05, max(0.0, deadline - time.monotonic())))
            except queue.Empty:
                continue

            decoded = decode_usb_frame(frame)
            cmd_id = decoded.get("cmd_id")
            payload = decoded.get("payload")
            if cmd_id in {0x26, 0x36} and isinstance(payload, dict):
                return payload
            if cmd_id == 0x46 and isinstance(payload, dict) and isinstance(payload.get("arm"), dict):
                return payload["arm"]

        raise TimeoutError("timeout waiting for ARM_GET_STATUS reply")

    def _execute_block(self, block_id: str, wait: bool = False) -> Optional[Dict[str, Any]]:
        _refresh_block_commands()
        block = resolve_block_command(block_id)
        frames = build_block_command_sequence(block.block_id)
        if not block.record_status:
            self._write_sequence(frames)
            if wait:
                step = build_block_flow_step(block.block_id, 1)
                return self._wait_after_block_flow_step(
                    step,
                    timeout_s=DEFAULT_BLOCK_FLOW_TIMEOUT_S,
                    arm_settle_s=DEFAULT_BLOCK_FLOW_ARM_SETTLE_S,
                )
            return None

        if len(frames) > 1:
            self._write_sequence(frames[:-1])
        payload = self._request_arm_status_payload()
        snapshot = arm_status_snapshot(payload)
        print(json.dumps(snapshot, ensure_ascii=False, indent=2), flush=True)
        return snapshot

    @staticmethod
    def _frame_record_to_bytes(frame: Dict[str, Any]) -> bytes:
        command = frame.get("command")
        values = list(frame.get("values") or [])
        has_payload = bool(frame.get("has_float_payload"))
        if command:
            return build_usb_command(str(command), values if has_payload else None)

        cmd = int(frame.get("cmd", 0))
        if has_payload:
            return pack_4float_frame(cmd, values)
        return pack_usb_frame(cmd)

    @classmethod
    def _block_flow_step_frames(cls, step: Dict[str, Any]) -> List[bytes]:
        records = step.get("frames")
        if not isinstance(records, list):
            block_id = str(step.get("block_id") or "")
            if not block_id:
                raise ValueError("block flow step must have frames or block_id")
            return list(build_block_command_sequence(block_id))
        return [cls._frame_record_to_bytes(frame) for frame in records if isinstance(frame, dict)]

    @staticmethod
    def _block_flow_step_wait_kind(step: Dict[str, Any]) -> str:
        raw_wait = str(step.get("wait_kind") or "").strip().lower()
        if raw_wait in {WAIT_NONE, WAIT_ARM, WAIT_ARM_STATUS, WAIT_CHASSIS, WAIT_CLIMB, WAIT_CLIMB_FINAL, WAIT_FLOW, WAIT_SETTLE}:
            return raw_wait

        category = str(step.get("category") or "").strip().lower()
        frames = step.get("frames") if isinstance(step.get("frames"), list) else []
        commands = {str(frame.get("command") or "").upper() for frame in frames if isinstance(frame, dict)}
        cmd_ids = {int(frame.get("cmd", -1)) for frame in frames if isinstance(frame, dict)}

        if (
            category == "climb"
            or commands & {"CLIMB_TEST_ACTION", "CLIMB_UP_GATE", "CLIMB_DOWN_GATE", "CLIMB_UP_AUTO_PAUSE", "CLIMB_DOWN_AUTO_PAUSE", "CLIMB_AUTO_RESUME"}
            or cmd_ids & {0x57, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E}
        ):
            return WAIT_CLIMB
        if category == "base" or "CHS_SET_POS" in commands or 0x14 in cmd_ids:
            return WAIT_CHASSIS
        if category == "status" or bool(step.get("record_status")):
            return WAIT_ARM_STATUS
        if category in {"arm_height", "arm_joint", "arm_space"}:
            return WAIT_ARM
        if category == "tool":
            return WAIT_SETTLE
        if commands & {"ARM_SET_WORKSPACE", "ARM_SET_TARGET", "ARM_SET_TARGET_XYZ", "ARM_HEIGHT_JOG", "ARM_HEIGHT_LIMIT", "ARM_SET_POSTURE", "ARM_JOINT_JOG"}:
            return WAIT_ARM
        if commands & {"TOOL_ENABLE", "TOOL_DISABLE"}:
            return WAIT_SETTLE
        return WAIT_NONE

    @staticmethod
    def _block_flow_step_timeout(step: Dict[str, Any], fallback_s: float) -> float:
        value = step.get("timeout_s")
        try:
            timeout_s = float(value)
        except (TypeError, ValueError):
            timeout_s = 0.0
        if timeout_s > 0.0:
            return timeout_s
        return fallback_s

    @staticmethod
    def _block_flow_step_repeat_count(step: Dict[str, Any]) -> int:
        value = step.get("repeat_count", 1)
        try:
            repeat_count = int(value)
        except (TypeError, ValueError):
            raise ValueError("block flow step repeat_count must be an integer")
        if repeat_count <= 0:
            raise ValueError("block flow step repeat_count must be > 0")
        return repeat_count

    def _wait_after_block_flow_step(
        self,
        step: Dict[str, Any],
        timeout_s: float,
        arm_settle_s: float,
    ) -> Optional[Dict[str, Any]]:
        wait_kind = self._block_flow_step_wait_kind(step)
        effective_timeout_s = self._block_flow_step_timeout(step, timeout_s)

        if wait_kind == WAIT_CLIMB:
            time.sleep(CLIMB_COMMAND_SETTLE_S)
            return self._wait_for_climb(effective_timeout_s, final_state=False)
        if wait_kind == WAIT_CLIMB_FINAL:
            time.sleep(CLIMB_COMMAND_SETTLE_S)
            return self._wait_for_climb(effective_timeout_s, final_state=True)
        if wait_kind == WAIT_FLOW:
            time.sleep(CLIMB_COMMAND_SETTLE_S)
            expected_flow_id = step.get("expected_flow_id")
            return self._wait_for_task_flow(
                effective_timeout_s,
                int(expected_flow_id) if expected_flow_id is not None else None,
            )
        if wait_kind == WAIT_CHASSIS:
            time.sleep(CLIMB_COMMAND_SETTLE_S)
            return self._wait_for_chassis_pos(effective_timeout_s)
        if wait_kind == WAIT_ARM:
            arm_timeout_s = max(effective_timeout_s, timeout_s, arm_settle_s)
            return self._wait_for_arm_settle(arm_timeout_s, arm_settle_s, step=step)
        if wait_kind == WAIT_ARM_STATUS:
            payload = self._request_arm_status_payload(timeout_s=min(1.0, max(0.1, effective_timeout_s)))
            snapshot = arm_status_snapshot(payload)
            print(json.dumps(snapshot, ensure_ascii=False, indent=2), flush=True)
            return payload
        if wait_kind == WAIT_SETTLE:
            settle_s = self._block_flow_step_timeout(step, DEFAULT_TOOL_SETTLE_S)
            time.sleep(max(0.0, settle_s))
            return None
        return None

    def _block_flow_start(self, args: List[str]) -> None:
        if len(args) > 1:
            raise ValueError("usage: block_flow_start [name]")
        self.block_flow_name = args[0] if args else time.strftime("block_flow_%Y%m%d_%H%M%S")
        self.block_flow_started_at = time.strftime("%Y-%m-%d %H:%M:%S")
        self.block_flow_steps = []
        self.last_block_flow_candidate = None
        self._block_flow_autosave()
        print(f"Block flow recording started: {self.block_flow_name}", flush=True)

    def _block_flow_add(self, args: List[str]) -> None:
        if not args:
            raise ValueError("usage: block_flow_add BLOCK_ID [note...]")
        if self.block_flow_name is None:
            self._block_flow_start([])

        _refresh_block_commands()
        block_id = args[0]
        note = " ".join(args[1:]) if len(args) > 1 else None
        snapshot = self._execute_block(block_id, wait=True)
        self.last_block_flow_candidate = build_block_flow_step(
            block_id,
            len(self.block_flow_steps) + 1,
            note=note,
            snapshot=snapshot,
        )
        self._block_flow_autosave()
        print(
            f"Candidate block flow step {self.last_block_flow_candidate['index']}: "
            f"{self.last_block_flow_candidate['block_id']}",
            flush=True,
        )
        answer = input("Save this block step to the current flow? [y/N] ").strip().lower()
        if answer in {"y", "yes"}:
            self._block_flow_save_candidate([])
        else:
            self._block_flow_discard()

    def _block_flow_save_candidate(self, args: List[str]) -> None:
        if self.last_block_flow_candidate is None:
            raise ValueError("no block flow candidate to save yet")
        step = json.loads(json.dumps(self.last_block_flow_candidate, ensure_ascii=False))
        if args:
            step["note"] = " ".join(args)
        step["index"] = len(self.block_flow_steps) + 1
        self.block_flow_steps.append(step)
        self.last_block_flow_candidate = None
        self._block_flow_autosave()
        print(f"Saved block flow step {step['index']}: {step['block_id']}", flush=True)

    def _block_flow_confirm(self, args: List[str]) -> None:
        if self.last_block_flow_candidate is None:
            raise ValueError("no block flow candidate to confirm yet")
        candidate = self.last_block_flow_candidate
        note = " ".join(args)
        note_text = f" note={note}" if note else ""
        print(f"Candidate: {candidate['block_id']}{note_text}", flush=True)
        answer = input("Save this block step to the current flow? [y/N] ").strip().lower()
        if answer in {"y", "yes"}:
            self._block_flow_save_candidate(args)
        else:
            self._block_flow_discard()

    def _block_flow_discard(self) -> None:
        if self.last_block_flow_candidate is None:
            print("No block flow candidate to discard.", flush=True)
            return
        self.last_block_flow_candidate = None
        self._block_flow_autosave()
        print("Discarded last block flow candidate.", flush=True)

    def _block_flow_show(self) -> None:
        if not self.block_flow_steps:
            print("Block flow is empty.", flush=True)
        else:
            for step in self.block_flow_steps:
                note = f"  # {step['note']}" if step.get("note") else ""
                print(
                    f"{step['index']:02d}. {step['block_id']} [{step.get('category', '')}] "
                    f"frames={len(step.get('frames', []))}{note}",
                    flush=True,
                )
        if self.last_block_flow_candidate is not None:
            print(
                f"Pending candidate: {self.last_block_flow_candidate['block_id']} "
                "use block_flow_confirm or block_flow_discard.",
                flush=True,
            )

    def _block_flow_data(self) -> Dict[str, Any]:
        return build_block_flow_data(
            self.block_flow_name or "block_flow",
            self.block_flow_started_at,
            self.block_flow_steps,
        )

    def _block_flow_autosave_data(self) -> Dict[str, Any]:
        data = self._block_flow_data()
        data["autosave"] = True
        data["autosaved_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        if self.last_block_flow_candidate is not None:
            data["last_block_flow_candidate"] = self.last_block_flow_candidate
        return data

    def _block_flow_autosave(self) -> None:
        if self.block_flow_name is None and not self.block_flow_steps and self.last_block_flow_candidate is None:
            return
        text = json.dumps(self._block_flow_autosave_data(), ensure_ascii=False, indent=2)
        self.block_flow_autosave_path.write_text(text + "\n", encoding="utf-8")

    def _print_block_flow_autosave_hint(self) -> None:
        if not self.block_flow_autosave_path.exists():
            return
        try:
            data = json.loads(self.block_flow_autosave_path.read_text(encoding="utf-8"))
            step_count = int(data.get("step_count", 0))
            name = str(data.get("name", "block_flow"))
        except Exception:
            step_count = -1
            name = "unknown"
        detail = f"{step_count} saved step(s)" if step_count >= 0 else "unreadable"
        print(
            f"Block flow autosave found: {self.block_flow_autosave_path} ({name}, {detail}). "
            "Use block_flow_recover to restore it.",
            flush=True,
        )

    def _block_flow_recover(self, args: List[str]) -> None:
        if len(args) > 1:
            raise ValueError("usage: block_flow_recover [file.json]")
        path = Path(args[0]) if args else self.block_flow_autosave_path
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("type") != "r2_block_flow":
            raise ValueError(f"not an r2 block flow file: {path}")

        steps = data.get("steps", [])
        if not isinstance(steps, list):
            raise ValueError(f"invalid block flow steps in {path}")

        restored_steps = json.loads(json.dumps(steps, ensure_ascii=False))
        for index, step in enumerate(restored_steps, 1):
            if isinstance(step, dict):
                step["index"] = index

        candidate = data.get("last_block_flow_candidate")
        self.block_flow_name = str(data.get("name") or "block_flow")
        self.block_flow_started_at = data.get("created_at")
        self.block_flow_steps = restored_steps
        self.last_block_flow_candidate = candidate if isinstance(candidate, dict) else None
        self._block_flow_autosave()
        print(
            f"Recovered block flow '{self.block_flow_name}' from {path}: "
            f"{len(self.block_flow_steps)} step(s).",
            flush=True,
        )
        if self.last_block_flow_candidate is not None:
            print(
                "Recovered one unsaved candidate. Use block_flow_confirm or block_flow_discard.",
                flush=True,
            )

    def _block_flow_autosave_command(self, args: List[str]) -> None:
        if len(args) > 1:
            raise ValueError("usage: block_flow_autosave [file.json]")
        if args:
            self.block_flow_autosave_path = Path(args[0])
        self._block_flow_autosave()
        print(f"Block flow autosave file: {self.block_flow_autosave_path}", flush=True)

    def _block_flow_export(self, args: List[str]) -> None:
        if len(args) > 1:
            raise ValueError("usage: block_flow_export [file.json]")
        data = self._block_flow_data()
        text = json.dumps(data, ensure_ascii=False, indent=2)
        if args:
            Path(args[0]).write_text(text + "\n", encoding="utf-8")
            self._block_flow_autosave()
            print(f"Block flow exported: {args[0]}", flush=True)
        else:
            print(text, flush=True)

    def _block_flow_export_c(self, args: List[str]) -> None:
        if len(args) > 2:
            raise ValueError("usage: block_flow_export_c [file.c] [symbol]")
        path = Path(args[0]) if args else None
        symbol = args[1] if len(args) == 2 else (path.stem if path else None)
        text = block_flow_to_c_source(self._block_flow_data(), symbol=symbol)
        if path:
            path.write_text(text, encoding="utf-8")
            print(f"Block flow C struct exported: {path}", flush=True)
        else:
            print(text, flush=True)

    def _block_flow_run_command(self, args: List[str]) -> None:
        if len(args) > 3:
            raise ValueError("usage: block_flow_run [file.json] [timeout_s] [arm_settle_s]")

        path: Optional[Path] = None
        timeout_s = DEFAULT_BLOCK_FLOW_TIMEOUT_S
        arm_settle_s = DEFAULT_BLOCK_FLOW_ARM_SETTLE_S
        numeric_start = 0

        if args:
            candidate = Path(args[0])
            if candidate.exists() or args[0].lower().endswith(".json"):
                path = candidate
                numeric_start = 1

        if len(args) > numeric_start:
            timeout_s = float(args[numeric_start])
        if len(args) > numeric_start + 1:
            arm_settle_s = float(args[numeric_start + 1])

        if path is not None:
            data = json.loads(path.read_text(encoding="utf-8"))
        else:
            data = self._block_flow_data()

        self._run_block_flow_data(data, timeout_s=timeout_s, arm_settle_s=arm_settle_s, path=path)

    @staticmethod
    def _read_block_flow_file(path: Path) -> Dict[str, Any]:
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("type") != "r2_block_flow":
            raise ValueError(f"not an r2 block flow file: {path}")
        if not isinstance(data.get("steps"), list):
            raise ValueError(f"invalid block flow steps in {path}")
        return data

    @staticmethod
    def _write_block_flow_file(path: Path, data: Dict[str, Any]) -> None:
        steps = data.get("steps")
        if isinstance(steps, list):
            for index, step in enumerate(steps, 1):
                if isinstance(step, dict):
                    step["index"] = index
            data["step_count"] = len(steps)
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    @staticmethod
    def _normalize_flow_key(value: Any) -> Optional[str]:
        raw = str(value or "").strip().lower()
        if not raw or raw == "none":
            return None
        if raw.endswith(".json"):
            raw = raw[:-5]
        if raw in {"s1_up", "s1_down"}:
            raw = f"{raw}_v2"
        return raw

    @classmethod
    def _flow_key(cls, data: Dict[str, Any], path: Optional[Path] = None) -> Optional[str]:
        key = cls._normalize_flow_key(data.get("name"))
        if key is None and path is not None:
            key = cls._normalize_flow_key(path.stem)
        return key

    @classmethod
    def _required_s1_prefix(cls, data: Dict[str, Any]) -> Optional[str]:
        metadata = data.get("metadata")
        if not isinstance(metadata, dict):
            return None
        return cls._normalize_flow_key(metadata.get("requires_s1_prefix"))

    def _check_flow_preconditions(self, data: Dict[str, Any], path: Optional[Path] = None) -> None:
        required = self._required_s1_prefix(data)
        if required is None:
            return
        if required in self.completed_s1_prefixes:
            return
        flow = self._flow_key(data, path) or str(path or "block_flow")
        raise ValueError(
            f"flow {flow} requires {required} to complete in this serial-tool session first"
        )

    def _mark_flow_completed(self, data: Dict[str, Any], path: Optional[Path] = None) -> None:
        key = self._flow_key(data, path)
        if key not in S1_PREFIX_FLOW_KEYS:
            return
        self.completed_s1_prefixes.add(key)
        print(f"S1 precondition state=SET prefix={key}", flush=True)

    @staticmethod
    def _same_path(left: Path, right: Path) -> bool:
        try:
            return left.resolve() == right.resolve()
        except OSError:
            return str(left) == str(right)

    def _sync_test_flow_after_slot_edit(
        self,
        path: Path,
        data: Dict[str, Any],
        touched_slots: Optional[Set[int]] = None,
    ) -> None:
        if (self.test_flow_path is None) or (not self._test_is_active()):
            return
        if not self._same_path(path, self.test_flow_path):
            return

        self.test_flow_data = data
        self.test_interrupt_bounds = self._interrupt_bounds(data)
        self.test_next_slot = self._next_empty_interrupt_slot(data)
        if touched_slots:
            self.test_executed_slots.difference_update(touched_slots)

    def _block_flow_slots(self, args: List[str]) -> None:
        if len(args) != 1:
            raise ValueError("usage: block_flow_slots FILE.json")
        path = Path(args[0])
        data = self._read_block_flow_file(path)
        slots = list_interrupt_slots(data)
        if not slots:
            print(f"No interrupt slots in {path}.", flush=True)
            return
        for slot in slots:
            block_id = slot.get("block_id") or ""
            label = slot.get("label") or ""
            print(
                f"slot {int(slot['slot']):03d} step={int(slot['index'])} "
                f"status={slot['status']} block={block_id} label={label}",
                flush=True,
            )

    def _block_flow_slot_set(self, args: List[str]) -> None:
        if len(args) < 3:
            raise ValueError("usage: block_flow_slot_set FILE.json SLOT BLOCK_ID [note...]")
        path = Path(args[0])
        slot = int(args[1])
        block_id = args[2]
        note = " ".join(args[3:]) if len(args) > 3 else None
        _refresh_block_commands()
        data = self._read_block_flow_file(path)
        step = fill_interrupt_slot(data, slot, block_id, note=note)
        self._write_block_flow_file(path, data)
        self._sync_test_flow_after_slot_edit(path, data, {slot})
        print(
            f"Filled {path} slot {slot:03d}: {step['block_id']} "
            f"wait={step.get('wait_kind', WAIT_NONE)}",
            flush=True,
        )

    def _block_flow_slot_clear(self, args: List[str]) -> None:
        if len(args) != 2:
            raise ValueError("usage: block_flow_slot_clear FILE.json SLOT")
        path = Path(args[0])
        slot = int(args[1])
        data = self._read_block_flow_file(path)
        clear_interrupt_slot(data, slot)
        self._write_block_flow_file(path, data)
        self._sync_test_flow_after_slot_edit(path, data, {slot})
        print(f"Cleared {path} slot {slot:03d}.", flush=True)

    def _block_flow_slot_clear_all(self, args: List[str]) -> None:
        if len(args) != 1:
            raise ValueError("usage: block_flow_slot_clear_all FILE.json")
        path = Path(args[0])
        data = self._read_block_flow_file(path)
        touched_slots = {int(slot["slot"]) for slot in list_interrupt_slots(data)}
        count = clear_all_interrupt_slots(data)
        self._write_block_flow_file(path, data)
        self._sync_test_flow_after_slot_edit(path, data, touched_slots)
        print(f"Cleared {count} interrupt slots in {path}.", flush=True)

    @staticmethod
    def _test_flow_root() -> Path:
        return Path(__file__).resolve().parent.parent

    @classmethod
    def _resolve_test_flow_path(cls, token: str) -> Path:
        raw = token.strip()
        key = raw.lower()
        if key.endswith(".json"):
            key = key[:-5]
        if key.endswith("_v2"):
            base_key = key[:-3]
        else:
            base_key = key

        names: List[str] = []
        if raw.lower().endswith(".json"):
            names.append(raw)
        else:
            names.append(f"{raw}.json")
        if base_key in TEST_FLOW_FILES:
            names.append(TEST_FLOW_FILES[base_key])
        if key in TEST_FLOW_FILES:
            names.append(TEST_FLOW_FILES[key])

        candidates: List[Path] = []
        for name in names:
            path = Path(name)
            candidates.append(path)
            candidates.append(cls._test_flow_root() / name)

        seen = set()
        for path in candidates:
            text = str(path)
            if text in seen:
                continue
            seen.add(text)
            if path.exists():
                return path

        valid = ", ".join(sorted(TEST_FLOW_FILES))
        raise ValueError(f"unknown test flow: {token}. Use one of: {valid}")

    @staticmethod
    def _interrupt_bounds(data: Dict[str, Any]) -> Tuple[int, int]:
        steps = data.get("steps")
        if not isinstance(steps, list):
            raise ValueError("block flow steps must be a list")
        indices = [
            index
            for index, step in enumerate(steps)
            if isinstance(step, dict) and step.get("interrupt_slot") is not None
        ]
        if not indices:
            raise ValueError("flow has no interrupt slots")
        return min(indices), max(indices) + 1

    @staticmethod
    def _next_empty_interrupt_slot(data: Dict[str, Any], start_slot: int = 1) -> Optional[int]:
        slots = list_interrupt_slots(data)
        for slot in sorted(slots, key=lambda item: int(item["slot"])):
            slot_number = int(slot["slot"])
            if slot_number < start_slot:
                continue
            if str(slot.get("status") or "") == "empty":
                return slot_number
        return None

    def _test_is_active(self) -> bool:
        return self.test_flow_data is not None and self.test_interrupt_bounds is not None

    def _test_start(self, args: List[str]) -> None:
        if not args:
            valid = "|".join(sorted(TEST_FLOW_FILES))
            raise ValueError(f"usage: test {valid} [timeout_s] [arm_settle_s]")
        if len(args) > 3:
            raise ValueError("usage: test FLOW [timeout_s] [arm_settle_s]")

        path = self._resolve_test_flow_path(args[0])
        data = self._read_block_flow_file(path)
        self._check_flow_preconditions(data, path)
        start, end = self._interrupt_bounds(data)
        timeout_s = float(args[1]) if len(args) >= 2 else DEFAULT_BLOCK_FLOW_TIMEOUT_S
        arm_settle_s = float(args[2]) if len(args) >= 3 else DEFAULT_BLOCK_FLOW_ARM_SETTLE_S
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be > 0")
        if arm_settle_s < 0.0:
            raise ValueError("arm_settle_s must be >= 0")

        flow_name = str(data.get("name") or path.stem)
        steps = data["steps"]
        print(
            f"Test flow state=RUN_PREFIX name={flow_name} file={path} "
            f"prefix_steps={start} interrupt_steps={end - start}",
            flush=True,
        )
        self._drain_frames()
        self._run_block_flow_steps(
            flow_name,
            steps,
            0,
            start,
            timeout_s=timeout_s,
            arm_settle_s=arm_settle_s,
        )

        self.test_flow_path = path
        self.test_flow_data = data
        self.test_flow_name = flow_name
        self.test_interrupt_bounds = (start, end)
        self.test_next_slot = self._next_empty_interrupt_slot(data)
        self.test_executed_slots = set()
        self.test_timeout_s = timeout_s
        self.test_arm_settle_s = arm_settle_s

        slot_text = f"{self.test_next_slot:03d}" if self.test_next_slot is not None else "none"
        print(
            f"Test flow state=PAUSED_BEFORE_INTERRUPT name={flow_name} next_slot={slot_text}. "
            "Type a block id to test it, then choose whether to save it.",
            flush=True,
        )

    def _test_status(self) -> None:
        if not self._test_is_active():
            print("No active test flow.", flush=True)
            return
        assert self.test_flow_data is not None
        assert self.test_interrupt_bounds is not None
        start, end = self.test_interrupt_bounds
        slots = list_interrupt_slots(self.test_flow_data)
        filled = sum(1 for slot in slots if str(slot.get("status") or "") == "filled")
        empty = sum(1 for slot in slots if str(slot.get("status") or "") == "empty")
        next_slot = f"{self.test_next_slot:03d}" if self.test_next_slot is not None else "none"
        print(
            f"Test flow name={self.test_flow_name} file={self.test_flow_path} "
            f"interrupt_steps={start + 1}-{end} filled={filled} empty={empty} "
            f"next_slot={next_slot} executed_slots={sorted(self.test_executed_slots)}",
            flush=True,
        )

    def _test_block(self, args: List[str]) -> None:
        if not self._test_is_active():
            raise ValueError("no active test flow; start with: test FLOW")
        if not args:
            raise ValueError("usage: block BLOCK_ID [note...]")
        assert self.test_flow_data is not None
        assert self.test_flow_path is not None

        block_id = args[0]
        note = " ".join(args[1:]) if len(args) > 1 else None
        slot = self.test_next_slot
        if slot is None:
            print("No empty interrupt slot remains; executing block without saving.", flush=True)

        snapshot = self._execute_block(block_id, wait=True)
        if slot is None:
            return

        answer = input(f"Save block '{block_id}' to interrupt slot {slot:03d}? [y/N] ").strip().lower()
        if answer not in {"y", "yes"}:
            print("Executed only; not saved to the flow.", flush=True)
            return

        step = fill_interrupt_slot(self.test_flow_data, slot, block_id, note=note)
        if snapshot is not None:
            step["snapshot"] = snapshot
        step["executed_in_test_session"] = True
        step["tested_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        self.test_executed_slots.add(slot)
        self.test_next_slot = self._next_empty_interrupt_slot(self.test_flow_data, start_slot=slot + 1)
        self._write_block_flow_file(self.test_flow_path, self.test_flow_data)
        next_slot = f"{self.test_next_slot:03d}" if self.test_next_slot is not None else "none"
        print(f"Saved slot {slot:03d}: {step['block_id']}. next_slot={next_slot}", flush=True)

    def _test_continue(self, args: List[str]) -> None:
        if not self._test_is_active():
            raise ValueError("no active test flow; start with: test FLOW")
        mode = args[0].lower() if args else "skip"
        if mode not in {"skip", "rest", "suffix", "slots", "with_slots", "run_slots"}:
            raise ValueError("usage: test_continue [skip|slots]")

        assert self.test_flow_data is not None
        assert self.test_interrupt_bounds is not None
        assert self.test_flow_path is not None
        steps = self.test_flow_data["steps"]
        start, end = self.test_interrupt_bounds
        flow_name = self.test_flow_name or str(self.test_flow_data.get("name") or "test_flow")

        if mode in {"slots", "with_slots", "run_slots"}:
            print("Test flow state=RUN_INTERRUPT_SLOTS filled slots only; empty slots skipped.", flush=True)
            self._run_block_flow_steps(
                flow_name,
                steps,
                start,
                end,
                timeout_s=self.test_timeout_s,
                arm_settle_s=self.test_arm_settle_s,
                skip_slots=self.test_executed_slots,
            )
        else:
            print("Test flow state=SKIP_INTERRUPT_SLOTS running suffix only.", flush=True)

        print("Test flow state=RUN_SUFFIX", flush=True)
        self._run_block_flow_steps(
            flow_name,
            steps,
            end,
            len(steps),
            timeout_s=self.test_timeout_s,
            arm_settle_s=self.test_arm_settle_s,
        )
        self._write_block_flow_file(self.test_flow_path, self.test_flow_data)
        print(f"Test flow state=DONE updated={self.test_flow_path}", flush=True)
        self._mark_flow_completed(self.test_flow_data, self.test_flow_path)
        self._test_show(["slots"])

    def _test_show(self, args: List[str]) -> None:
        if not self._test_is_active():
            raise ValueError("no active test flow; start with: test FLOW")
        assert self.test_flow_data is not None
        mode = args[0].lower() if args else "slots"
        steps = self.test_flow_data.get("steps")
        if not isinstance(steps, list):
            raise ValueError("block flow steps must be a list")

        if mode == "all":
            for index, step in enumerate(steps, 1):
                if not isinstance(step, dict):
                    continue
                slot = step.get("interrupt_slot")
                extra = ""
                if slot is not None:
                    status = "empty" if is_interrupt_placeholder_step(step) else "filled"
                    extra = f" slot={int(slot):03d} {status}"
                print(f"{index:03d}. {step.get('block_id', '?')}{extra}", flush=True)
            return

        if mode not in {"slots", "interrupts"}:
            raise ValueError("usage: test_show [slots|all]")
        for slot in list_interrupt_slots(self.test_flow_data):
            print(
                f"slot {int(slot['slot']):03d} step={int(slot['index'])} "
                f"status={slot['status']} block={slot.get('block_id') or ''}",
                flush=True,
            )

    def _test_export(self, args: List[str]) -> None:
        if not self._test_is_active():
            raise ValueError("no active test flow; start with: test FLOW")
        if len(args) > 1:
            raise ValueError("usage: test_export [file.json]")
        assert self.test_flow_data is not None
        assert self.test_flow_path is not None
        path = Path(args[0]) if args else self.test_flow_path
        self._write_block_flow_file(path, self.test_flow_data)
        print(f"Generated test flow: {path}", flush=True)
        self._test_show(["all"])

    def _test_clear(self) -> None:
        self.test_flow_path = None
        self.test_flow_data = None
        self.test_flow_name = None
        self.test_interrupt_bounds = None
        self.test_next_slot = None
        self.test_executed_slots = set()
        print("Test flow session cleared.", flush=True)

    def _run_block_flow_steps(
        self,
        flow_name: str,
        steps: List[Dict[str, Any]],
        start_index: int,
        end_index: int,
        timeout_s: float,
        arm_settle_s: float,
        skip_slots: Optional[Set[int]] = None,
    ) -> None:
        total = len(steps)
        skip_slots = skip_slots or set()
        for zero_index in range(start_index, end_index):
            step = steps[zero_index]
            index = zero_index + 1
            if not isinstance(step, dict):
                raise ValueError(f"block flow step {index} is not an object")

            block_id = str(step.get("block_id") or f"step_{index}")
            slot_raw = step.get("interrupt_slot")
            if slot_raw is not None and int(slot_raw) in skip_slots:
                print(
                    f"Block flow state=SKIP_TESTED_SLOT step={index}/{total} "
                    f"slot={int(slot_raw):03d}",
                    flush=True,
                )
                continue
            if is_interrupt_placeholder_step(step):
                print(
                    f"Block flow state=SKIP_EMPTY_SLOT step={index}/{total} "
                    f"slot={int(step.get('interrupt_slot') or 0):03d}",
                    flush=True,
                )
                continue
            if bool(step.get("skip_on_run")):
                print(
                    f"Block flow state=SKIP_DISABLED_STEP step={index}/{total} block={block_id}",
                    flush=True,
                )
                continue

            wait_kind = self._block_flow_step_wait_kind(step)
            frames = self._block_flow_step_frames(step)
            if not frames:
                if bool(step.get("placeholder")) or str(step.get("category") or "").strip().lower() == "tool_interface":
                    print(
                        f"Block flow state=SKIP_NOOP_STEP step={index}/{total} block={block_id}",
                        flush=True,
                    )
                    continue
                raise ValueError(f"block flow step {index} has no frames")

            repeat_count = self._block_flow_step_repeat_count(step)
            print(
                f"Block flow state=SEND_STEP name={flow_name} step={index}/{total} "
                f"block={block_id} wait={wait_kind} repeat={repeat_count}",
                flush=True,
            )
            for repeat_index in range(repeat_count):
                if repeat_count > 1:
                    print(
                        f"Block flow state=SEND_REPEAT step={index}/{total} "
                        f"block={block_id} repeat={repeat_index + 1}/{repeat_count}",
                        flush=True,
                    )
                self._write_sequence(frames)
                print(
                    f"Block flow state=WAIT_STEP step={index}/{total} block={block_id} "
                    f"repeat={repeat_index + 1}/{repeat_count}",
                    flush=True,
                )
                self._wait_after_block_flow_step(
                    step,
                    timeout_s=timeout_s,
                    arm_settle_s=arm_settle_s,
                )

    def _run_block_flow_data(
        self,
        data: Dict[str, Any],
        timeout_s: float = DEFAULT_BLOCK_FLOW_TIMEOUT_S,
        arm_settle_s: float = DEFAULT_BLOCK_FLOW_ARM_SETTLE_S,
        path: Optional[Path] = None,
    ) -> None:
        if data.get("type") != "r2_block_flow":
            raise ValueError("not an r2_block_flow object")
        steps = data.get("steps")
        if not isinstance(steps, list):
            raise ValueError("block flow steps must be a list")
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be > 0")

        self._check_flow_preconditions(data, path)
        flow_name = str(data.get("name") or "block_flow")
        print(f"Block flow state=IDLE name={flow_name} steps={len(steps)}", flush=True)
        self._drain_frames()
        self._run_block_flow_steps(
            flow_name,
            steps,
            0,
            len(steps),
            timeout_s=timeout_s,
            arm_settle_s=arm_settle_s,
        )
        print(f"Block flow state=DONE name={flow_name}", flush=True)
        self._mark_flow_completed(data, path)

    def _block_flow_clear(self) -> None:
        self.block_flow_name = None
        self.block_flow_started_at = None
        self.block_flow_steps = []
        self.last_block_flow_candidate = None
        try:
            self.block_flow_autosave_path.unlink()
        except FileNotFoundError:
            pass
        print("Block flow cleared.", flush=True)

    def _climb_test(self, args: List[str], wait: bool) -> None:
        if not args:
            raise ValueError("usage: climb_test ACTION | climb_test_wait ACTION [timeout_s]")
        if wait and len(args) > 2:
            raise ValueError("usage: climb_test_wait ACTION [timeout_s]")
        if not wait and len(args) > 1:
            raise ValueError("usage: climb_test ACTION")

        action = self._parse_climb_test_action(args[0])
        timeout_s = float(args[1]) if wait and len(args) == 2 else DEFAULT_CLIMB_WAIT_TIMEOUT_S
        self._write_climb_test_action(action, wait=wait, timeout_s=timeout_s)

    def _write_climb_test_action(
        self,
        action: int,
        wait: bool,
        timeout_s: float = DEFAULT_CLIMB_WAIT_TIMEOUT_S,
    ) -> Optional[Dict[str, Any]]:
        name = CLIMB_TEST_ACTIONS.get(action, "UNKNOWN")
        print(f"Climb test action: {action} {name}", flush=True)
        frames = [
            build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
            build_usb_command("CLIMB_ENABLE"),
            build_usb_command("CLIMB_TEST_ACTION", [float(action)]),
        ]
        if not wait:
            frames.append(build_usb_command("CLIMB_GET_STATUS"))
        self._write_sequence(frames)
        result: Optional[Dict[str, Any]] = None
        if wait:
            time.sleep(CLIMB_COMMAND_SETTLE_S)
            result = self._wait_for_climb(timeout_s, final_state=False)
        self.last_flow_candidate = self._build_flow_step(action, wait, timeout_s, result)
        self._flow_autosave()
        print("Use flow_confirm [note] to choose whether to save this action.", flush=True)
        return result

    def _build_flow_step(
        self,
        action: int,
        wait: bool,
        timeout_s: float,
        result: Optional[Dict[str, Any]],
    ) -> Dict[str, Any]:
        name = CLIMB_TEST_ACTIONS.get(action, "UNKNOWN")
        step: Dict[str, Any] = {
            "command": "CLIMB_TEST_ACTION",
            "action_id": action,
            "action_name": name,
            "values": [float(action)],
            "wait": bool(wait),
            "timeout_s": float(timeout_s),
            "done_condition": "CLIMB_GET_STATUS.state_done == 1",
        }
        if result:
            step["result"] = self._summarize_climb_result(result)
        return step

    @staticmethod
    def _summarize_climb_result(payload: Dict[str, Any]) -> Dict[str, Any]:
        keep = (
            "state",
            "state_name",
            "state_done",
            "error_flags",
            "elapsed_ms",
            "test_action",
            "test_action_name",
            "leg_pos_mm",
            "leg_target_mm",
            "drive_pos_mm",
            "drive_target_mm",
        )
        return {key: payload[key] for key in keep if key in payload}

    def _flow_start(self, args: List[str]) -> None:
        self.flow_name = args[0] if args else time.strftime("climb_flow_%Y%m%d_%H%M%S")
        self.flow_started_at = time.strftime("%Y-%m-%d %H:%M:%S")
        self.flow_steps = []
        self.last_flow_candidate = None
        print(f"Flow recording started: {self.flow_name}", flush=True)

    def _flow_save(self, args: List[str]) -> None:
        if self.last_flow_candidate is None:
            raise ValueError("no climb test action to save yet")
        if self.flow_name is None:
            self.flow_name = time.strftime("climb_flow_%Y%m%d_%H%M%S")
            self.flow_started_at = time.strftime("%Y-%m-%d %H:%M:%S")
            self.flow_steps = []
            print(f"Flow recording started: {self.flow_name}", flush=True)

        step = json.loads(json.dumps(self.last_flow_candidate, ensure_ascii=False))
        if args:
            step["note"] = " ".join(args)
        step["index"] = len(self.flow_steps) + 1
        self.flow_steps.append(step)
        self.last_flow_candidate = None
        self._flow_autosave()
        print(f"Saved flow step {step['index']}: {step['action_id']} {step['action_name']}", flush=True)

    def _flow_confirm(self, args: List[str]) -> None:
        if self.last_flow_candidate is None:
            raise ValueError("no climb test action to confirm yet")

        candidate = self.last_flow_candidate
        note = " ".join(args)
        note_text = f" note={note}" if note else ""
        print(
            f"Candidate: {candidate['action_id']} {candidate['action_name']}{note_text}",
            flush=True,
        )
        answer = input("Save this action to the current flow? [y/N] ").strip().lower()
        if answer in {"y", "yes"}:
            self._flow_save(args)
        else:
            self.last_flow_candidate = None
            self._flow_autosave()
            print("Discarded last flow candidate.", flush=True)

    def _flow_add(self, args: List[str]) -> None:
        if not args:
            raise ValueError("usage: flow_add ACTION [timeout_s] [note...]")
        action = self._parse_climb_test_action(args[0])
        timeout_s = DEFAULT_CLIMB_WAIT_TIMEOUT_S
        note_start = 1
        if len(args) >= 2:
            try:
                timeout_s = float(args[1])
                note_start = 2
            except ValueError:
                note_start = 1
        self.last_flow_candidate = self._build_flow_step(action, True, timeout_s, None)
        self._flow_save(args[note_start:])

    def _flow_show(self) -> None:
        if not self.flow_steps:
            print("Flow is empty.", flush=True)
            return
        for step in self.flow_steps:
            note = f"  # {step['note']}" if step.get("note") else ""
            print(
                f"{step['index']:02d}. {step['action_id']} {step['action_name']} "
                f"wait={int(bool(step.get('wait')))} timeout={step.get('timeout_s')}s{note}",
                flush=True,
            )

    def _flow_data(self) -> Dict[str, Any]:
        return {
            "type": "r2_climb_debug_flow",
            "version": 1,
            "name": self.flow_name or "climb_flow",
            "created_at": self.flow_started_at,
            "step_count": len(self.flow_steps),
            "steps": self.flow_steps,
        }

    def _flow_autosave_data(self) -> Dict[str, Any]:
        data = self._flow_data()
        data["autosave"] = True
        data["autosaved_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        if self.last_flow_candidate is not None:
            data["last_flow_candidate"] = self.last_flow_candidate
        return data

    def _flow_autosave(self) -> None:
        if self.flow_name is None and not self.flow_steps and self.last_flow_candidate is None:
            return
        text = json.dumps(self._flow_autosave_data(), ensure_ascii=False, indent=2)
        with self.flow_autosave_path.open("w", encoding="utf-8") as f:
            f.write(text)
            f.write("\n")

    def _print_flow_autosave_hint(self) -> None:
        if not self.flow_autosave_path.exists():
            return
        try:
            data = json.loads(self.flow_autosave_path.read_text(encoding="utf-8"))
            step_count = int(data.get("step_count", 0))
            name = str(data.get("name", "climb_flow"))
        except Exception:
            step_count = -1
            name = "unknown"
        detail = f"{step_count} saved step(s)" if step_count >= 0 else "unreadable"
        print(
            f"Flow autosave found: {self.flow_autosave_path} ({name}, {detail}). "
            "Use flow_recover to restore it.",
            flush=True,
        )

    def _flow_recover(self, args: List[str]) -> None:
        if len(args) > 1:
            raise ValueError("usage: flow_recover [file.json]")
        path = Path(args[0]) if args else self.flow_autosave_path
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("type") != "r2_climb_debug_flow":
            raise ValueError(f"not an r2 climb flow file: {path}")

        steps = data.get("steps", [])
        if not isinstance(steps, list):
            raise ValueError(f"invalid flow steps in {path}")

        restored_steps = json.loads(json.dumps(steps, ensure_ascii=False))
        for index, step in enumerate(restored_steps, 1):
            if isinstance(step, dict):
                step["index"] = index

        candidate = data.get("last_flow_candidate")
        self.flow_name = str(data.get("name") or "climb_flow")
        self.flow_started_at = data.get("created_at")
        self.flow_steps = restored_steps
        self.last_flow_candidate = candidate if isinstance(candidate, dict) else None
        self._flow_autosave()
        print(
            f"Recovered flow '{self.flow_name}' from {path}: {len(self.flow_steps)} step(s).",
            flush=True,
        )
        if self.last_flow_candidate is not None:
            print(
                "Recovered one unsaved candidate. Use flow_confirm [note] or flow_save [note] to keep it.",
                flush=True,
            )

    def _flow_autosave_command(self, args: List[str]) -> None:
        if len(args) > 1:
            raise ValueError("usage: flow_autosave [file.json]")
        if args:
            self.flow_autosave_path = Path(args[0])
        self._flow_autosave()
        print(f"Flow autosave file: {self.flow_autosave_path}", flush=True)

    def _flow_export(self, args: List[str]) -> None:
        data = self._flow_data()
        text = json.dumps(data, ensure_ascii=False, indent=2)
        if args:
            path = args[0]
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)
                f.write("\n")
            self._flow_autosave()
            print(f"Flow exported: {path}", flush=True)
        else:
            print(text, flush=True)

    def _flow_clear(self) -> None:
        self.flow_name = None
        self.flow_started_at = None
        self.flow_steps = []
        self.last_flow_candidate = None
        try:
            self.flow_autosave_path.unlink()
        except FileNotFoundError:
            pass
        print("Flow cleared.", flush=True)

    @staticmethod
    def _parse_climb_test_action(token: str) -> int:
        text = token.strip()
        try:
            value = int(text, 0)
        except ValueError:
            key = text.upper().replace("-", "_")
            for prefix in ("R2_CLIMB_TEST_", "CLIMB_TEST_", "TEST_"):
                if key.startswith(prefix):
                    key = key[len(prefix):]
                    break
            if key not in CLIMB_TEST_ACTION_BY_NAME:
                raise ValueError(f"unknown climb test action: {token}") from None
            value = CLIMB_TEST_ACTION_BY_NAME[key]

        if value not in CLIMB_TEST_ACTIONS:
            raise ValueError(f"unknown climb test action id: {value}")
        if value == 0:
            raise ValueError("action 0 is NONE; use climb_stop if you want to stop")
        return value

    @staticmethod
    def _print_climb_tests() -> None:
        for action, name in CLIMB_TEST_ACTIONS.items():
            print(f"{action:2d} {name}", flush=True)

    def _drain_frames(self) -> None:
        while True:
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                return

    def _wait_for_chassis_pos(self, timeout_s: float) -> Dict[str, Any]:
        if timeout_s <= 0:
            raise ValueError("timeout must be > 0")

        self._drain_frames()
        deadline = time.monotonic() + timeout_s
        next_query = 0.0
        last_payload: Optional[Dict[str, Any]] = None
        print(f"Waiting for chassis position state DONE/IDLE (timeout {timeout_s:g}s)...", flush=True)

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_query:
                self._write(build_usb_command("CHS_GET_STATUS"))
                next_query = now + DEFAULT_BLOCK_FLOW_POLL_INTERVAL_S

            try:
                frame = self.frame_queue.get(timeout=min(0.05, max(0.0, deadline - time.monotonic())))
            except queue.Empty:
                continue

            decoded = decode_usb_frame(frame)
            if decoded.get("cmd_id") != 0x16:
                continue
            payload = decoded.get("payload")
            if not isinstance(payload, dict):
                continue

            last_payload = payload
            pos_state = int(payload.get("pos_state", -1))
            pos_state_name = str(payload.get("pos_state_name", "UNKNOWN"))
            progress = payload.get("position", {}).get("progress", "?") if isinstance(payload.get("position"), dict) else "?"
            print(
                f"chassis: pos_state={pos_state_name}({pos_state}) progress={progress}",
                flush=True,
            )

            if bool(payload.get("emergency_stop")):
                raise RuntimeError("chassis entered emergency stop")
            if pos_state in {0, 2} or pos_state_name in {"IDLE", "DONE"}:
                print("Chassis position move is done.", flush=True)
                return payload

        summary = self._format_chassis_payload(last_payload)
        raise TimeoutError(f"timeout waiting for chassis position after {timeout_s:g}s. last={summary}")

    @staticmethod
    def _arm_wait_joint_indexes(step: Optional[Dict[str, Any]]) -> Tuple[int, ...]:
        if not isinstance(step, dict):
            return (1, 2)

        joints: Set[int] = set()
        frames = step.get("frames")
        if not isinstance(frames, list):
            return (1, 2)

        for frame in frames:
            if not isinstance(frame, dict):
                continue
            command = str(frame.get("command") or "").strip().upper()
            values = frame.get("values")
            if not isinstance(values, list):
                values = []

            if command == "ARM_JOINT_JOG" and values:
                try:
                    joint = int(float(values[0]))
                except (TypeError, ValueError):
                    continue
                if joint in (2, 3):
                    joints.add(joint - 1)
            elif command in {"ARM_HEIGHT_JOG", "ARM_HEIGHT_LIMIT", "ARM_SET_POSTURE", "ARM_SET_TARGET", "ARM_SET_TARGET_XYZ"}:
                joints.update((1, 2))
            elif command in {"ARM_ENABLE", "ARM_SET_WORKSPACE"}:
                joints.update((0, 1, 2))

        return tuple(sorted(joints)) if joints else (1, 2)

    @staticmethod
    def _arm_feedback_values_deg(
        payload: Dict[str, Any],
        joint_indexes: Tuple[int, ...],
    ) -> Optional[List[Tuple[int, float]]]:
        feedback_deg = payload.get("motor_feedback_deg")
        feedback_ok = payload.get("motor_feedback_ok")
        if not isinstance(feedback_deg, list) or not isinstance(feedback_ok, dict):
            return None
        if len(feedback_deg) < 3:
            return None

        names = ("j1", "j2", "j3")
        values: List[Tuple[int, float]] = []
        for index in joint_indexes:
            if index < 0 or index >= len(names):
                continue
            if not bool(feedback_ok.get(names[index])):
                return None
            values.append((index, float(feedback_deg[index])))
        return values or None

    @staticmethod
    def _arm_feedback_deltas_deg(
        current: List[Tuple[int, float]],
        previous: Optional[List[Tuple[int, float]]],
    ) -> Optional[List[Tuple[int, float]]]:
        if previous is None:
            return None
        previous_by_index = {index: value for index, value in previous}
        deltas: List[Tuple[int, float]] = []
        for index, value in current:
            if index not in previous_by_index:
                return None
            deltas.append((index, abs(value - previous_by_index[index])))
        return deltas

    @staticmethod
    def _format_arm_feedback_values(values: List[Tuple[int, float]], suffix: str = "") -> str:
        return " ".join(f"J{index + 1}={value:.2f}{suffix}" for index, value in values)

    def _wait_for_arm_settle(
        self,
        timeout_s: float,
        settle_s: float,
        step: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        if timeout_s <= 0:
            raise ValueError("timeout must be > 0")

        deadline = time.monotonic() + timeout_s
        min_accept_at = time.monotonic() + max(0.0, settle_s)
        joint_indexes = self._arm_wait_joint_indexes(step)
        stable_samples = 0
        last_log_at = 0.0
        first = True
        last_payload: Optional[Dict[str, Any]] = None
        previous_values: Optional[List[Tuple[int, float]]] = None
        last_values: Optional[List[Tuple[int, float]]] = None
        last_deltas: Optional[List[Tuple[int, float]]] = None

        while time.monotonic() < deadline:
            request_timeout = min(1.0, max(0.1, deadline - time.monotonic()))
            payload = self._request_arm_status_payload(timeout_s=request_timeout)
            self._raise_if_arm_error(payload)
            last_payload = payload

            values = self._arm_feedback_values_deg(payload, joint_indexes)
            now = time.monotonic()
            if first:
                print(
                    f"arm: state={payload.get('state_name', 'UNKNOWN')} "
                    f"ik={payload.get('ik_status_name', 'UNKNOWN')} "
                    f"settle={settle_s:g}s still_tol={ARM_FEEDBACK_STILL_TOL_DEG:g}deg/sample",
                    flush=True,
                )
                first = False

            if values is None:
                if now >= min_accept_at:
                    print("arm: feedback settle check unavailable; fixed settle elapsed.", flush=True)
                    return payload
                time.sleep(min(ARM_FEEDBACK_POLL_INTERVAL_S, max(0.0, deadline - time.monotonic())))
                continue

            deltas = self._arm_feedback_deltas_deg(values, previous_values)
            previous_values = values
            last_values = values
            last_deltas = deltas

            if (now - last_log_at) >= 1.0:
                detail = self._format_arm_feedback_values(values, "deg")
                if deltas is not None:
                    detail += " delta " + self._format_arm_feedback_values(deltas, "deg")
                print(
                    f"arm: waiting feedback settle {detail} "
                    f"stable={stable_samples}/{ARM_FEEDBACK_SETTLE_STABLE_SAMPLES}",
                    flush=True,
                )
                last_log_at = now

            max_delta = max((delta for _, delta in deltas), default=None) if deltas is not None else None
            if now >= min_accept_at and max_delta is not None and max_delta <= ARM_FEEDBACK_STILL_TOL_DEG:
                stable_samples += 1
                if stable_samples >= ARM_FEEDBACK_SETTLE_STABLE_SAMPLES:
                    print(
                        f"Arm feedback settled: {self._format_arm_feedback_values(values, 'deg')}",
                        flush=True,
                    )
                    return payload
            else:
                stable_samples = 0

            time.sleep(min(ARM_FEEDBACK_POLL_INTERVAL_S, max(0.0, deadline - time.monotonic())))

        if last_payload is not None:
            if last_values is not None:
                detail = self._format_arm_feedback_values(last_values, "deg")
                if last_deltas is not None:
                    detail += " delta " + self._format_arm_feedback_values(last_deltas, "deg")
                raise TimeoutError(
                    f"timeout waiting for arm feedback settle after {timeout_s:g}s: {detail}"
                )
        raise TimeoutError(f"timeout waiting for arm feedback settle after {timeout_s:g}s")

    @staticmethod
    def _raise_if_arm_error(payload: Dict[str, Any]) -> None:
        state = int(payload.get("state", -1))
        ik_status = int(payload.get("ik_status", 0))
        error_flags = payload.get("error_flags")
        has_error_flag = any(bool(v) for v in error_flags.values()) if isinstance(error_flags, dict) else False
        if state == 4 or ik_status != 0 or has_error_flag:
            raise RuntimeError(
                "arm status error: "
                f"state={payload.get('state_name', state)} "
                f"ik={payload.get('ik_status_name', ik_status)} "
                f"reason={payload.get('ik_reason_name', payload.get('ik_reason'))}"
            )

    def _wait_for_climb(self, timeout_s: float, final_state: bool) -> Dict[str, Any]:
        if timeout_s <= 0:
            raise ValueError("timeout must be > 0")

        self._drain_frames()
        deadline = time.monotonic() + timeout_s
        next_query = 0.0
        last_payload: Optional[Dict[str, Any]] = None
        mode = "DONE" if final_state else "ready"
        print(f"Waiting for climb {mode} (timeout {timeout_s:g}s)...", flush=True)

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_query:
                self._write(build_usb_command("CLIMB_GET_STATUS"))
                next_query = now + DEFAULT_CLIMB_POLL_INTERVAL_S

            try:
                frame = self.frame_queue.get(timeout=min(0.05, max(0.0, deadline - time.monotonic())))
            except queue.Empty:
                continue

            decoded = decode_usb_frame(frame)
            if decoded.get("cmd_id") != 0x56:
                continue
            payload = decoded.get("payload")
            if not isinstance(payload, dict):
                continue

            last_payload = payload
            state = int(payload.get("state", -1))
            state_name = str(payload.get("state_name", "UNKNOWN"))
            flow_name = str(payload.get("flow_name", "UPSTAIRS"))
            state_done = bool(payload.get("state_done"))
            auto_run = bool(payload.get("auto_run"))
            summary_name = payload.get("summary_state_name")
            elapsed = payload.get("elapsed_ms", "?")
            print(
                f"climb: flow={flow_name} summary={summary_name or 'LEGACY'} state={state_name}({state}) "
                f"done={int(state_done)} auto={int(auto_run)} elapsed={elapsed}ms",
                flush=True,
            )

            if summary_name == "ERROR" or state == CLIMB_ERROR_STATE or state_name == "ERROR":
                raise RuntimeError(f"climb entered ERROR, flags={payload.get('error_flags')}")
            if final_state:
                if summary_name == "DONE" or state == CLIMB_DONE_STATE or state_name == "DONE":
                    print("Climb final state is DONE.", flush=True)
                    return payload
            elif summary_name in {"READY", "DONE"} or state in {0, CLIMB_DONE_STATE} or state_done:
                print("Climb is ready for the next command.", flush=True)
                return payload

        summary = self._format_climb_payload(last_payload)
        raise TimeoutError(f"timeout waiting for climb {mode} after {timeout_s:g}s. last={summary}")

    def _wait_for_task_flow(
        self,
        timeout_s: float,
        expected_flow_id: Optional[int] = None,
    ) -> Dict[str, Any]:
        if timeout_s <= 0:
            raise ValueError("timeout must be > 0")

        self._drain_frames()
        deadline = time.monotonic() + timeout_s
        next_query = 0.0
        last_payload: Optional[Dict[str, Any]] = None
        print(f"Waiting for task flow DONE (timeout {timeout_s:g}s)...", flush=True)

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_query:
                self._write(build_usb_command("FLOW_GET_STATUS"))
                next_query = now + DEFAULT_BLOCK_FLOW_POLL_INTERVAL_S

            try:
                frame = self.frame_queue.get(
                    timeout=min(0.05, max(0.0, deadline - time.monotonic()))
                )
            except queue.Empty:
                continue

            decoded = decode_usb_frame(frame)
            if decoded.get("cmd_id") != 0x66:
                continue
            payload = decoded.get("payload")
            if not isinstance(payload, dict):
                continue

            last_payload = payload
            flow_id = int(payload.get("flow_id", -1))
            state = int(payload.get("state", -1))
            state_name = str(payload.get("state_name", "UNKNOWN"))
            error_name = str(payload.get("error_name", "NONE"))
            print(
                f"task flow: name={payload.get('flow_name', 'UNKNOWN')} "
                f"state={state_name} step={payload.get('entry_index', '?')}/"
                f"{payload.get('entry_count', '?')}",
                flush=True,
            )

            if expected_flow_id is not None and flow_id != expected_flow_id:
                continue
            if state == 4 or state_name == "ERROR":
                raise RuntimeError(
                    f"task flow entered ERROR: {error_name}, op={payload.get('current_op_name')}"
                )
            if state == 3 or state_name == "DONE":
                print("Task flow state is DONE.", flush=True)
                return payload

        raise TimeoutError(
            f"timeout waiting for task flow DONE after {timeout_s:g}s. last={last_payload}"
        )

    @staticmethod
    def _format_climb_payload(payload: Optional[Dict[str, Any]]) -> str:
        if not payload:
            return "no CLIMB_GET_STATUS reply"
        return (
            f"summary={payload.get('summary_state_name', 'LEGACY')}, "
            f"state={payload.get('state_name', 'UNKNOWN')}({payload.get('state', '?')}), "
            f"flow={payload.get('flow_name', 'UPSTAIRS')}, "
            f"done={int(bool(payload.get('state_done')))}, "
            f"auto={int(bool(payload.get('auto_run')))}, "
            f"errors={payload.get('error_flags')}"
        )

    @staticmethod
    def _format_chassis_payload(payload: Optional[Dict[str, Any]]) -> str:
        if not payload:
            return "no CHS_GET_STATUS reply"
        return (
            f"pos_state={payload.get('pos_state_name', 'UNKNOWN')}({payload.get('pos_state', '?')}), "
            f"emergency={int(bool(payload.get('emergency_stop')))}"
        )

    @staticmethod
    def _optional_timeout(args: List[str], default: float = DEFAULT_CLIMB_WAIT_TIMEOUT_S) -> float:
        if not args:
            return default
        if len(args) != 1:
            raise ValueError("usage: command [timeout_s]")
        return float(args[0])

    @staticmethod
    def _need_floats(args: List[str], count: int, usage: str) -> List[float]:
        if len(args) < count:
            raise ValueError(f"usage: {usage}")
        return [float(x) for x in args]

    @staticmethod
    def _parse_direction(token: str) -> int:
        key = token.strip().upper().replace(" ", "")
        mapping = {
            "0": 0,
            "Y+": 0,
            "+Y": 0,
            "YPOS": 0,
            "Y_POS": 0,
            "1": 1,
            "X+": 1,
            "+X": 1,
            "XPOS": 1,
            "X_POS": 1,
            "2": 2,
            "X-": 2,
            "-X": 2,
            "XNEG": 2,
            "X_NEG": 2,
        }
        if key not in mapping:
            raise ValueError("direction must be Y+, X+, or X-")
        return mapping[key]

    @staticmethod
    def _parse_tool(token: str) -> int:
        key = token.strip().upper()
        mapping = {
            "0": 0,
            "S1": 0,
            "SUCTION1": 0,
            "1": 1,
            "S2": 1,
            "SUCTION2": 1,
            "2": 2,
            "G": 2,
            "GRIPPER": 2,
            "CLAW": 2,
        }
        if key not in mapping:
            raise ValueError("tool must be S1, S2, gripper, or 0/1/2")
        return mapping[key]

    @classmethod
    def _optional_tool_payload(cls, args: List[str], usage: str) -> List[float]:
        if not args:
            return []
        if len(args) != 1:
            raise ValueError(f"usage: {usage}")
        tool = cls._parse_tool(args[0])
        if tool == 0:
            raise ValueError("S1 suction was removed; use S2")
        return [float(tool)]

    @staticmethod
    def _parse_jog_amount(args: List[str], positive: bool) -> float:
        if len(args) != 1:
            raise ValueError("usage: arm_up|arm_down 1|5|10|20|50|100|200|500")
        raw_value = float(args[0])
        value = int(raw_value)
        if raw_value != float(value):
            raise ValueError("height jog step must be an integer millimeter value")
        if value not in ARM_JOG_STEPS_MM:
            raise ValueError("height jog step must be one of 1, 5, 10, 20, 50, 100, 200, 500 mm")
        return float(value if positive else -value)

    @staticmethod
    def _parse_joint_jog_amount(args: List[str]) -> float:
        if len(args) != 1:
            raise ValueError("usage: arm_j2_cw|arm_j2_ccw|arm_j3_cw|arm_j3_ccw 1|5|10|20|30|60|90")
        raw_value = float(args[0])
        value = int(raw_value)
        if raw_value != float(value):
            raise ValueError("joint jog step must be an integer degree value")
        if value not in ARM_JOINT_JOG_STEPS_DEG:
            raise ValueError("joint jog step must be one of 1, 5, 10, 20, 30, 60, 90 deg")
        return float(value)

    def _print_commands(self) -> None:
        for command in list_commands():
            payload = "4float" if command.needs_float_payload else ("optional" if command.optional_float_payload else "empty")
            print(f"{command.name:<22} 0x{command.cmd:02X} {payload:<7} {command.doc}", flush=True)

    @staticmethod
    def _print_help() -> None:
        print(
            "\n".join(
                [
                    "help",
                    "commands",
                    "blocks | block BLOCK_ID [note...] | arm_snapshot [file.json] [note]",
                    "block_flow_start [name] then type BLOCK_ID [note...] | block_flow_confirm [note...] | block_flow_discard | block_flow_show",
                    "block_flow_export [file.json] | block_flow_export_c [file.c] [symbol] | block_flow_run [file.json] [timeout_s] [arm_settle_s]",
                    "block_flow_slots FILE.json | block_flow_slot_set FILE.json SLOT BLOCK_ID [note...] | block_flow_slot_clear FILE.json SLOT | interrupt_clear_all FILE.json",
                    "block_flow_recover [file.json] | block_flow_clear",
                    "test FLOW [timeout_s] [arm_settle_s] | test_status | test_continue [skip|slots] | test_show [slots|all] | test_export [file.json] | test_clear",
                    "flow_start [name] | flow_confirm [note] | flow_save [note] | flow_add ACTION [timeout_s] [note...]",
                    "flow_show | flow_export [file.json] | flow_recover [file.json] | flow_autosave [file.json] | flow_clear",
                    "pack NAME [float...]",
                    "send NAME [float...]",
                    "NAME [float...] also works, for example TOOL_SET_MODE 0 1 450 0",
                    "query ROBOT|FLOW|TUNE|CLIMB|SYS|CHS|ARM|TOOL",
                    "raw A5 5A ...",
                    "usb | usart | enable | disable | stop | status",
                    "chs_enable | chs_disable | chs_mode MODE | vel VX VY YAW_DATA | pos DX DY YAW_DATA | chs_stop | chs_status",
                    "arm_enable | arm_disable | arm_xyz X Y Z | arm_space Y+|X+|X- | arm_posture TOOL STATE | arm_stop | arm_status/arm_state",
                    "arm_up/down 1|5|10|20|50|100|200|500, or arm_up20/arm_down20 | arm_zmin | arm_zmax",
                    "arm_j2_cw/ccw 1|5|10|20|30|60|90 | arm_j3_cw/ccw 1|5|10|20|30|60|90 | arm_j2_cw10/arm_j3_ccw90",
                    "tool_on/off S2|GRIPPER | tool_status | legacy tool_target TOOL STATE Z_MM YAW_RAD, tool_s2/tool_gripper Z_MM YAW_RAD",
                    "climb_enable | climb_step | climb_up_auto/climb_auto | climb_ctrl ENABLE STEP AUTO | climb_stop | climb_status",
                    "climb_wait [timeout_s] | climb_step_wait [timeout_s] | climb_up_auto_wait/climb_auto_wait [timeout_s] | climb_wait_then NAME [float...]",
                    "climb_down_step/climb_downstairs_step | climb_down_step_wait [timeout_s] | climb_down_auto/climb_downstairs_auto | climb_down_auto_wait [timeout_s]",
                    "climb_tests | climb_test ACTION | climb_test_wait ACTION [timeout_s]",
                    "climb_<action_name_lower> [timeout_s], for example climb_all_legs_220 or climb_all_legs_zero 10",
                    "S1_UP | S1_DOWN | S1_UP_S2_DOWN | S1_DOWN_S2_UP | S1_DOWN_S2_DOWN",
                    "weapon_dock_test | weapon_dock_status | weapon_chassis_done | weapon_dock_done",
                    "throw_block X+|X- (requires completed S1_UP_V2 or S1_DOWN_V2)",
                    "tune_start [pass_count] | tune_status | tune_stop",
                    "exit",
                ]
            ),
            flush=True,
        )


def run_block_flow_file(
    port: str,
    baud: int,
    path: Path,
    timeout_s: float = DEFAULT_BLOCK_FLOW_TIMEOUT_S,
    arm_settle_s: float = DEFAULT_BLOCK_FLOW_ARM_SETTLE_S,
) -> None:
    data = json.loads(path.read_text(encoding="utf-8"))
    shell = SerialShell(port, baud)
    shell.ser = open_serial(port, baud)
    print(f"Opened {shell.ser.port} @ {baud}. Running block flow {path}.", flush=True)
    shell.reader_thread = threading.Thread(target=shell._reader_loop, daemon=True)
    shell.reader_thread.start()
    try:
        shell._run_block_flow_data(data, timeout_s=timeout_s, arm_settle_s=arm_settle_s, path=path)
    finally:
        shell.stop_event.set()
        if shell.reader_thread:
            shell.reader_thread.join(timeout=0.5)
        if shell.ser:
            shell.ser.close()
