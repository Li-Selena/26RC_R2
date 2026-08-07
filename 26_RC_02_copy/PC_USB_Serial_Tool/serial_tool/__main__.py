from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .block_commands import (
    block_flow_to_c_source,
    build_block_command_sequence,
    clear_all_interrupt_slots,
    clear_interrupt_slot,
    fill_interrupt_slot,
    is_block_command,
    list_block_commands,
    list_interrupt_slots,
)
from .commands import build_climb_test_shortcut_sequence, build_query_command, build_usb_command, list_commands
from .protocol import bytes_to_hex, parse_hex
from .serial_console import SerialShell, list_serial_ports, monitor_serial, poll_serial, run_block_flow_file, send_and_read, send_sequence_and_read
from .usart_remote import build_remote_frame


def _add_serial_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", required=True, help="Serial port, for example COM7.")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate. USB CDC can keep the default.")


def cmd_ports(_args: argparse.Namespace) -> int:
    ports = list_serial_ports()
    if not ports:
        print("No serial ports found.")
        return 0
    for port in ports:
        print(port)
    return 0


def cmd_commands(_args: argparse.Namespace) -> int:
    for command in list_commands():
        payload = "4float" if command.needs_float_payload else ("optional" if command.optional_float_payload else "empty")
        print(f"{command.name:<22} 0x{command.cmd:02X} {payload:<7} {command.doc}")
    return 0


def cmd_blocks(_args: argparse.Namespace) -> int:
    for block in list_block_commands():
        marker = " record" if block.record_status else ""
        print(f"{block.block_id:<28} {block.category:<12} {block.label}{marker}")
    return 0


def cmd_block_flow_c(args: argparse.Namespace) -> int:
    data = json.loads(Path(args.input).read_text(encoding="utf-8"))
    text = block_flow_to_c_source(data, symbol=args.symbol)
    Path(args.output).write_text(text, encoding="utf-8")
    print(f"Block flow C struct exported: {args.output}")
    return 0


def cmd_block_flow_run(args: argparse.Namespace) -> int:
    run_block_flow_file(
        args.port,
        args.baud,
        Path(args.input),
        timeout_s=args.timeout,
        arm_settle_s=args.arm_settle,
    )
    return 0


def _read_block_flow(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("type") != "r2_block_flow":
        raise ValueError(f"not an r2_block_flow file: {path}")
    if not isinstance(data.get("steps"), list):
        raise ValueError(f"invalid block flow steps in {path}")
    return data


def _write_block_flow(path: Path, data: dict) -> None:
    steps = data.get("steps")
    if isinstance(steps, list):
        for index, step in enumerate(steps, 1):
            if isinstance(step, dict):
                step["index"] = index
        data["step_count"] = len(steps)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def cmd_block_flow_slots(args: argparse.Namespace) -> int:
    data = _read_block_flow(Path(args.input))
    for slot in list_interrupt_slots(data):
        print(
            f"slot {int(slot['slot']):03d}\tstep {int(slot['index'])}\t"
            f"{slot['status']}\t{slot.get('block_id') or ''}\t{slot.get('label') or ''}"
        )
    return 0


def cmd_block_flow_slot_set(args: argparse.Namespace) -> int:
    path = Path(args.input)
    data = _read_block_flow(path)
    output = Path(args.output) if args.output else path
    note = " ".join(args.note) if args.note else None
    step = fill_interrupt_slot(data, args.slot, args.block_id, note=note)
    _write_block_flow(output, data)
    print(f"Filled {output} slot {args.slot:03d}: {step['block_id']}")
    return 0


def cmd_block_flow_slot_clear(args: argparse.Namespace) -> int:
    path = Path(args.input)
    data = _read_block_flow(path)
    output = Path(args.output) if args.output else path
    clear_interrupt_slot(data, args.slot)
    _write_block_flow(output, data)
    print(f"Cleared {output} slot {args.slot:03d}.")
    return 0


def cmd_block_flow_slot_clear_all(args: argparse.Namespace) -> int:
    path = Path(args.input)
    data = _read_block_flow(path)
    output = Path(args.output) if args.output else path
    count = clear_all_interrupt_slots(data)
    _write_block_flow(output, data)
    print(f"Cleared {count} interrupt slots in {output}.")
    return 0


def cmd_pack(args: argparse.Namespace) -> int:
    frame = build_usb_command(args.name, args.values)
    print(bytes_to_hex(frame))
    return 0


def cmd_send(args: argparse.Namespace) -> int:
    if args.raw:
        send_and_read(args.port, args.baud, parse_hex(args.raw), wait_s=args.wait, json_lines=args.jsonl)
        return 0

    if args.name and is_block_command(args.name):
        if args.values:
            raise ValueError(f"{args.name} does not accept positional float values")
        send_sequence_and_read(args.port, args.baud, build_block_command_sequence(args.name), wait_s=args.wait, json_lines=args.jsonl)
        return 0

    shortcut_frames = build_climb_test_shortcut_sequence(args.name)
    if shortcut_frames is not None:
        if args.values:
            raise ValueError(
                f"{args.name} does not accept positional float values in send; "
                "use --wait SECONDS to read replies after sending"
            )
        send_sequence_and_read(args.port, args.baud, shortcut_frames, wait_s=args.wait, json_lines=args.jsonl)
        return 0

    frame = build_usb_command(args.name, args.values)
    send_and_read(args.port, args.baud, frame, wait_s=args.wait, json_lines=args.jsonl)
    return 0


def cmd_monitor(args: argparse.Namespace) -> int:
    monitor_serial(args.port, args.baud, json_lines=args.jsonl, raw_rx=args.raw_rx)
    return 0


def cmd_poll(args: argparse.Namespace) -> int:
    poll_serial(args.port, args.baud, args.name, args.rate, count=args.count, json_lines=args.jsonl)
    return 0


def cmd_shell(args: argparse.Namespace) -> int:
    SerialShell(args.port, args.baud).run()
    return 0


def _remote_kwargs(args: argparse.Namespace):
    return {
        "mode": args.mode,
        "arm_enable": args.arm_enable,
        "source_usb": args.source_usb,
        "climb_enable": args.climb_enable,
        "climb_step": args.climb_step,
        "climb_auto": args.climb_auto,
        "chassis": args.chassis,
    }


def cmd_remote_pack(args: argparse.Namespace) -> int:
    frame = build_remote_frame(**_remote_kwargs(args))
    print(bytes_to_hex(frame))
    return 0


def cmd_remote_send(args: argparse.Namespace) -> int:
    frame = build_remote_frame(**_remote_kwargs(args))
    send_and_read(args.port, args.baud, frame, wait_s=args.wait, json_lines=args.jsonl)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot USB serial protocol tool.")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("ports", help="List serial ports.")
    p.set_defaults(func=cmd_ports)

    p = sub.add_parser("commands", help="List known USB commands.")
    p.set_defaults(func=cmd_commands)

    p = sub.add_parser("blocks", help="List ready-to-use flow block commands.")
    p.set_defaults(func=cmd_blocks)

    p = sub.add_parser("block-flow-c", help="Convert an exported block flow JSON file to C structs.")
    p.add_argument("input", help="Input r2_block_flow JSON file.")
    p.add_argument("output", help="Output .c/.h file.")
    p.add_argument("--symbol", help="C array symbol name. Defaults to the flow name.")
    p.set_defaults(func=cmd_block_flow_c)

    p = sub.add_parser("block-flow-run", help="Run an exported arm/base/climb block flow JSON file.")
    _add_serial_args(p)
    p.add_argument("input", help="Input r2_block_flow JSON file.")
    p.add_argument("--timeout", type=float, default=40.0, help="Fallback per-step timeout in seconds.")
    p.add_argument("--arm-settle", type=float, default=1.0, help="Arm settle time after arm blocks.")
    p.set_defaults(func=cmd_block_flow_run)

    p = sub.add_parser("block-flow-slots", help="List interrupt slots in a block flow JSON file.")
    p.add_argument("input", help="Input r2_block_flow JSON file.")
    p.set_defaults(func=cmd_block_flow_slots)

    p = sub.add_parser("block-flow-slot-set", help="Fill one interrupt slot with an existing block command.")
    p.add_argument("input", help="Input r2_block_flow JSON file; overwritten unless --output is used.")
    p.add_argument("slot", type=int, help="Interrupt slot number, for example 1.")
    p.add_argument("block_id", help="Existing block command id, for example arm_j2_cw_10deg.")
    p.add_argument("note", nargs="*", help="Optional note saved on the filled slot.")
    p.add_argument("--output", help="Write to a different file instead of overwriting input.")
    p.set_defaults(func=cmd_block_flow_slot_set)

    p = sub.add_parser("block-flow-slot-clear", help="Clear one interrupt slot back to an empty placeholder.")
    p.add_argument("input", help="Input r2_block_flow JSON file; overwritten unless --output is used.")
    p.add_argument("slot", type=int, help="Interrupt slot number, for example 1.")
    p.add_argument("--output", help="Write to a different file instead of overwriting input.")
    p.set_defaults(func=cmd_block_flow_slot_clear)

    p = sub.add_parser("block-flow-slot-clear-all", help="Clear all interrupt slots back to empty placeholders.")
    p.add_argument("input", help="Input r2_block_flow JSON file; overwritten unless --output is used.")
    p.add_argument("--output", help="Write to a different file instead of overwriting input.")
    p.set_defaults(func=cmd_block_flow_slot_clear_all)

    p = sub.add_parser("pack", help="Pack a USB frame and print hex.")
    p.add_argument("name", help="Command name or numeric command id.")
    p.add_argument("values", nargs="*", type=float, help="Optional float payload values, padded to 4 floats.")
    p.set_defaults(func=cmd_pack)

    p = sub.add_parser("send", help="Send one USB command or raw hex frame.")
    _add_serial_args(p)
    p.add_argument("name", nargs="?", help="Command name or numeric command id.")
    p.add_argument("values", nargs="*", type=float)
    p.add_argument("--raw", help="Raw hex bytes to send instead of command name.")
    p.add_argument("--wait", type=float, default=0.0, help="Seconds to read replies after sending.")
    p.add_argument("--jsonl", action="store_true", help="Print one JSON object per line.")
    p.set_defaults(func=cmd_send)

    p = sub.add_parser("monitor", help="Read and decode incoming USB frames.")
    _add_serial_args(p)
    p.add_argument("--jsonl", action="store_true")
    p.add_argument("--raw-rx", action="store_true", help="Also print raw RX chunks.")
    p.set_defaults(func=cmd_monitor)

    p = sub.add_parser("poll", help="Periodically send a status query and decode replies.")
    _add_serial_args(p)
    p.add_argument("name", help="Status target, for example ROBOT, CLIMB, SYS.")
    p.add_argument("--rate", type=float, default=10.0, help="Polling rate in Hz.")
    p.add_argument("--count", type=int, default=0, help="Stop after count sends; 0 means forever.")
    p.add_argument("--jsonl", action="store_true")
    p.set_defaults(func=cmd_poll)

    p = sub.add_parser("shell", help="Interactive send/read shell.")
    _add_serial_args(p)
    p.set_defaults(func=cmd_shell)

    for name, func, needs_serial in (
        ("remote-pack", cmd_remote_pack, False),
        ("remote-send", cmd_remote_send, True),
    ):
        p = sub.add_parser(name, help="Build/send USART remote control frame.")
        if needs_serial:
            _add_serial_args(p)
            p.add_argument("--wait", type=float, default=0.0)
            p.add_argument("--jsonl", action="store_true")
        p.add_argument("--mode", type=int, choices=range(8), help="Chassis mode 0..7. Omit for all-zero stop mode.")
        p.add_argument("--arm-enable", type=int, choices=[0, 1], default=0)
        p.add_argument("--source-usart", dest="source_usb", action="store_const", const=0, default=0)
        p.add_argument("--source-usb", dest="source_usb", action="store_const", const=1)
        p.add_argument("--climb-enable", type=int, choices=[0, 1], default=0)
        p.add_argument("--climb-step", type=int, choices=[0, 1], default=0)
        p.add_argument("--climb-auto", type=int, choices=[0, 1], default=0)
        p.add_argument("--chassis", nargs=3, type=float, default=[0.0, 0.0, 0.0], metavar=("P1", "P2", "P3"))
        p.set_defaults(func=func)

    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if getattr(args, "raw", None) is None and getattr(args, "cmd", "") == "send" and not args.name:
        parser.error("send requires NAME or --raw")
    try:
        return args.func(args)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
