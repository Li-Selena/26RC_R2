import struct
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from serial_tool.block_commands import (
    WAIT_ARM,
    WAIT_CHASSIS,
    WAIT_CLIMB,
    WAIT_CLIMB_FINAL,
    WAIT_FLOW,
    WAIT_NONE,
    block_flow_to_c_source,
    build_block_command_sequence,
    build_block_flow_data,
    build_block_flow_step,
    build_interrupt_placeholder_step,
    clear_all_interrupt_slots,
    clear_interrupt_slot,
    fill_interrupt_slot,
    is_interrupt_placeholder_step,
    list_interrupt_slots,
    list_block_commands,
)
from serial_tool.commands import build_climb_test_shortcut_sequence, build_usb_command, command_name
from serial_tool.protocol import UsbStreamParser, bytes_to_hex, crc16_modbus, pack_usb_frame
from serial_tool.serial_console import SerialShell
from serial_tool.status import CLIMB_TEST_ACTIONS, decode_usb_frame
from serial_tool.usart_remote import build_remote_frame


class ProtocolTests(unittest.TestCase):
    def test_known_empty_frames(self):
        self.assertEqual(bytes_to_hex(build_usb_command("SYS_DISABLE")), "A5 5A 00 00 FB 02 FF")
        self.assertEqual(bytes_to_hex(build_usb_command("SYS_ENABLE")), "A5 5A 00 01 3B C3 FF")
        self.assertEqual(bytes_to_hex(build_usb_command("SYS_STOP")), "A5 5A 00 05 F8 C2 FF")
        self.assertEqual(bytes_to_hex(build_usb_command("SYS_GET_STATUS")), "A5 5A 00 06 F9 82 FF")
        self.assertEqual(bytes_to_hex(build_usb_command("CLIMB_GET_STATUS")), "A5 5A 00 56 C5 82 FF")

    def test_task_flow_command_aliases(self):
        cases = (
            ("S1_UP", 0x60, "FLOW_S1_UP"),
            ("S1_UP_V2", 0x60, "FLOW_S1_UP"),
            ("S1_DOWN", 0x61, "FLOW_S1_DOWN"),
            ("S1_DOWN_V2", 0x61, "FLOW_S1_DOWN"),
            ("S1_UP_S2_DOWN", 0x63, "FLOW_S1_UP_S2_DOWN"),
            ("S1_DOWN_S2_UP", 0x64, "FLOW_S1_DOWN_S2_UP"),
            ("S1_DOWN_S2_DOWN", 0x65, "FLOW_S1_DOWN_S2_DOWN"),
            ("TASK_FLOW_STATUS", 0x66, "FLOW_GET_STATUS"),
            ("WEAPON_GRAB", 0x67, "FLOW_WEAPON_GRAB"),
        )
        for alias, cmd, canonical in cases:
            with self.subTest(alias=alias):
                parsed = UsbStreamParser().feed(build_usb_command(alias))[0]
                self.assertEqual(parsed.cmd, cmd)
                self.assertEqual(command_name(cmd), canonical)

    def test_removed_s1_up_s2_up_command_is_rejected(self):
        with self.assertRaises(ValueError):
            build_usb_command("S1_UP_S2_UP_V2")

    def test_known_4float_frame(self):
        frame = build_usb_command("SYS_SWITCH_SOURCE", [1.0])
        self.assertEqual(
            bytes_to_hex(frame),
            "A5 5A 10 02 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 38 4B FF",
        )

    def test_throw_block_direction_frames(self):
        x_pos = UsbStreamParser().feed(build_usb_command("THROW_BLOCK", [1.0]))[0]
        x_neg = UsbStreamParser().feed(build_usb_command("THROW_BLOCK", [2.0]))[0]

        self.assertEqual(x_pos.cmd, 0x68)
        self.assertEqual(x_neg.cmd, 0x68)
        self.assertEqual(command_name(x_pos.cmd), "FLOW_THROW_BLOCK")
        self.assertEqual(struct.unpack("<4f", x_pos.payload)[0], 1.0)
        self.assertEqual(struct.unpack("<4f", x_neg.payload)[0], 2.0)

    def test_stream_parser_split_frame(self):
        frame = pack_usb_frame(0x46)
        parser = UsbStreamParser()
        out = parser.feed(b"\x00\x11" + frame[:3])
        self.assertEqual(out, [])
        out = parser.feed(frame[3:] + b"\x99")
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0].cmd, 0x46)
        self.assertEqual(out[0].payload, b"")

    def test_decode_robot_status_payload(self):
        payload = bytearray(96)
        payload[0] = 1
        payload[1] = 1
        payload[2] = 0b00000111
        payload[5] = 0b00011101
        struct.pack_into("<I", payload, 8, 12)
        struct.pack_into("<f", payload, 32, 1.25)
        frame = pack_usb_frame(0x46, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["payload"]["active_source_name"], "USB")
        self.assertTrue(decoded["payload"]["enable_flags"]["chassis"])
        self.assertEqual(decoded["payload"]["usb_rx"]["count"], 12)
        self.assertAlmostEqual(decoded["payload"]["nav"]["x_m"], 1.25)

    def test_decode_robot_status_v2_extension(self):
        payload = bytearray(160)
        payload[0] = 2
        payload[1] = 1
        payload[2] = 0b00001111
        payload[3] = 0b10110000
        payload[5] = 0b01000000
        payload[96] = 1
        payload[97] = 5
        payload[98] = 1
        payload[100] = 1
        payload[105] = 6
        payload[106] = 10
        payload[107] = 0b11
        struct.pack_into("<I", payload, 108, 2345)
        payload[116] = 0b111
        payload[117] = 0b101
        payload[119] = 1
        struct.pack_into("<I", payload, 124, 9876)
        struct.pack_into("<i", payload, 128, 123)
        struct.pack_into("<i", payload, 132, -1)
        struct.pack_into("<i", payload, 136, 456)
        payload[140] = 1
        payload[142] = 2
        payload[143] = 3
        struct.pack_into("<I", payload, 144, 4321)
        struct.pack_into("<f", payload, 152, -2.5)
        struct.pack_into("<f", payload, 156, 8.75)
        frame = pack_usb_frame(0x46, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)["payload"]
        self.assertEqual(decoded["protocol_version"], 2)
        self.assertTrue(decoded["enable_flags"]["climb"])
        self.assertTrue(decoded["executing_flags"]["climb_motor_active"])
        self.assertTrue(decoded["executing_flags"]["yaw_tune_running"])
        self.assertTrue(decoded["executing_flags"]["any"])
        self.assertEqual(decoded["climb"]["state_name"], "STEP_05_FRONT_MINUS_30")
        self.assertEqual(decoded["climb"]["test_action_name"], "CHASSIS_FORWARD_100")
        self.assertTrue(decoded["climb"]["test_chassis_active"])
        self.assertEqual(decoded["laser"]["distance_mm"]["y_pos"], -1)
        self.assertEqual(decoded["laser"]["update_tick"], 9876)
        self.assertEqual(decoded["yaw_tune"]["state_name"], "RUNNING")
        self.assertEqual(decoded["yaw_tune"]["active_mode_name"], "WORLD_VEL")
        self.assertAlmostEqual(decoded["yaw_tune"]["score"], 8.75)

    def test_decode_robot_status_downstairs_flow(self):
        payload = bytearray(160)
        payload[0] = 2
        payload[96] = 1
        payload[97] = 4
        payload[107] = 0b100
        frame = pack_usb_frame(0x46, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)["payload"]
        self.assertEqual(decoded["climb"]["flow_name"], "DOWNSTAIRS")
        self.assertEqual(decoded["climb"]["state_name"], "DOWN_04_ALL_LEGS_UP_10")

    def test_decode_yaw_tune_status_payload(self):
        payload = bytearray(64)
        payload[0] = 1
        payload[1] = 2
        payload[2] = 9
        payload[3] = 0
        struct.pack_into("<I", payload, 4, 1234)
        struct.pack_into("<I", payload, 8, 456)
        struct.pack_into("<f", payload, 12, -1.5)
        struct.pack_into("<f", payload, 28, 3.25)
        struct.pack_into("<f", payload, 32, 1.1)
        struct.pack_into("<f", payload, 40, 0.8)
        payload[60] = 0
        payload[61] = 1
        payload[62] = 3
        payload[63] = 2
        frame = pack_usb_frame(0x49, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["cmd_name"], "YAW_TUNE_GET_STATUS")
        self.assertEqual(decoded["payload"]["state_name"], "RUNNING")
        self.assertEqual(decoded["payload"]["segment_index"], 2)
        self.assertAlmostEqual(decoded["payload"]["yaw_error_deg"], -1.5)
        self.assertAlmostEqual(decoded["payload"]["score"], 3.25)
        self.assertEqual(decoded["payload"]["active_mode_name"], "WORLD_VEL")
        self.assertEqual(decoded["payload"]["phase_name"], "RUN")

    def test_decode_extended_mechanism_status_payloads(self):
        chassis = bytearray(112)
        chassis[0] = 3
        chassis[1] = 1
        chassis[84] = 1
        chassis[86] = 0b11010011
        chassis[87] = 0b10000
        struct.pack_into("<f", chassis, 88, 0.4)
        struct.pack_into("<f", chassis, 100, 1.25)
        frame = pack_usb_frame(0x16, bytes(chassis))
        decoded = decode_usb_frame(UsbStreamParser().feed(frame)[0])["payload"]
        self.assertEqual(decoded["mode_name"], "WORLD_VEL")
        self.assertTrue(decoded["status_flags"]["moving"])
        self.assertTrue(decoded["error_flags"]["partial_motor_offline"])
        self.assertAlmostEqual(decoded["target_vel"]["vx"], 0.4)
        self.assertAlmostEqual(decoded["target_pos"]["dx"], 1.25)

        arm = bytearray(80)
        arm[0] = 1
        arm[1] = 1
        arm[2] = 1
        arm[3] = 2
        arm[4] = 0b00011111
        arm[5] = 0b00000101
        arm[6] = 3
        arm[7] = 4
        arm[20] = 1
        arm[21] = 1
        arm[22] = 1
        arm[23] = 0x82
        struct.pack_into("<I", arm, 8, 123)
        struct.pack_into("<I", arm, 16, 7)
        struct.pack_into("<f", arm, 24, 450.0)
        struct.pack_into("<f", arm, 28, 0.5)
        struct.pack_into("<f", arm, 32, 0.1)
        struct.pack_into("<f", arm, 44, 12.0)
        struct.pack_into("<f", arm, 52, 450.0)
        struct.pack_into("<f", arm, 60, 2.0)
        struct.pack_into("<f", arm, 64, 0.11)
        struct.pack_into("<f", arm, 68, -0.22)
        struct.pack_into("<f", arm, 72, 0.33)
        arm[76] = 0b101
        frame = pack_usb_frame(0x26, bytes(arm))
        decoded = decode_usb_frame(UsbStreamParser().feed(frame)[0])["payload"]
        self.assertEqual(decoded["state_name"], "TARGET_VALID")
        self.assertEqual(decoded["ik_status_name"], "HEIGHT_UNREACHABLE")
        self.assertEqual(decoded["ik_reason_name"], "HEIGHT_OUTSIDE_LINK")
        self.assertTrue(decoded["status_flags"]["active"])
        self.assertTrue(decoded["error_flags"]["height_unreachable"])
        self.assertEqual(decoded["tool_name"], "S2")
        self.assertEqual(decoded["tool_state_name"], "S2_SUCTION_DOWN")
        self.assertEqual(decoded["target_direction_name"], "X+")
        self.assertTrue(decoded["ik_test"]["active"])
        self.assertEqual(decoded["ik_test"]["step"], 2)
        self.assertAlmostEqual(decoded["target_z_mm"], 450.0)
        self.assertAlmostEqual(decoded["theta_deg"][0], 0.1 * 57.29577951308232, places=6)
        self.assertAlmostEqual(decoded["tool_world_mm"][2], 450.0)
        self.assertAlmostEqual(decoded["height_error_mm"], 2.0)
        self.assertAlmostEqual(decoded["motor_feedback_rad"][1], -0.22)
        self.assertFalse(decoded["motor_feedback_ok"]["j2"])
        self.assertTrue(decoded["motor_feedback_ok"]["j3"])

        tool = bytearray(arm)
        tool[20] = 2
        frame = pack_usb_frame(0x36, bytes(tool))
        decoded = decode_usb_frame(UsbStreamParser().feed(frame)[0])["payload"]
        self.assertEqual(decoded["status_kind"], "tool_pose")
        self.assertEqual(decoded["tool_name"], "GRIPPER")
        self.assertAlmostEqual(decoded["target_z_mm"], 450.0)

    def test_decode_robot_status_v3_targets(self):
        payload = bytearray(240)
        payload[0] = 3
        payload[1] = 1
        struct.pack_into("<f", payload, 160, 0.4)
        struct.pack_into("<f", payload, 172, 1.25)
        payload[184] = 1
        payload[185] = 1
        payload[186] = 1
        payload[187] = 2
        payload[188] = 0b00011111
        payload[190] = 0
        payload[191] = 0
        struct.pack_into("<f", payload, 192, 450.0)
        struct.pack_into("<f", payload, 196, 0.25)
        struct.pack_into("<f", payload, 200, 0.1)
        struct.pack_into("<f", payload, 208, 0.3)
        struct.pack_into("<f", payload, 212, 100.0)
        struct.pack_into("<f", payload, 220, 450.0)
        struct.pack_into("<I", payload, 224, 1234)
        struct.pack_into("<I", payload, 228, 6)
        payload[232] = 2
        payload[233] = 1
        payload[234] = 2
        payload[235] = 0x44
        payload[238] = 0x08
        payload[239] = 1
        frame = pack_usb_frame(0x46, bytes(payload))
        decoded = decode_usb_frame(UsbStreamParser().feed(frame)[0])["payload"]
        self.assertEqual(decoded["protocol_version"], 3)
        self.assertAlmostEqual(decoded["chassis"]["target_vel"]["vx"], 0.4)
        self.assertAlmostEqual(decoded["chassis"]["target_pos"]["dx"], 1.25)
        self.assertEqual(decoded["arm"]["state_name"], "TARGET_VALID")
        self.assertEqual(decoded["arm"]["tool_name"], "GRIPPER")
        self.assertEqual(decoded["arm"]["tool_state_name"], "GRIPPER_DOWN")
        self.assertEqual(decoded["arm"]["target_direction_name"], "X-")
        self.assertTrue(decoded["arm"]["ik_test"]["done"])
        self.assertEqual(decoded["arm"]["ik_test"]["step"], 4)
        self.assertAlmostEqual(decoded["arm"]["target_z_mm"], 450.0)
        self.assertAlmostEqual(decoded["arm"]["theta_rad"][2], 0.3)
        self.assertAlmostEqual(decoded["arm"]["tool_world_mm"][2], 450.0)
        self.assertEqual(decoded["arm"]["output_apply_count"], 6)
        self.assertTrue(decoded["climb_summary_error_flags"]["flow_switch"])
        self.assertTrue(decoded["active_source_stale"])

    def test_decode_robot_status_v5_arm_motor_feedback(self):
        payload = bytearray(253)
        payload[0] = 5
        payload[3] = 0b00000100
        payload[184] = 1
        payload[185] = 1
        payload[187] = 2
        struct.pack_into("<f", payload, 200, 0.1)
        struct.pack_into("<f", payload, 204, -0.2)
        struct.pack_into("<f", payload, 208, 0.3)
        struct.pack_into("<f", payload, 212, 100.0)
        struct.pack_into("<f", payload, 216, 20.0)
        struct.pack_into("<f", payload, 220, 450.0)
        struct.pack_into("<f", payload, 240, 1.1)
        struct.pack_into("<f", payload, 244, 2.2)
        struct.pack_into("<f", payload, 248, 3.3)
        payload[252] = 0b111

        frame = pack_usb_frame(0x46, bytes(payload))
        decoded = decode_usb_frame(UsbStreamParser().feed(frame)[0])["payload"]

        self.assertEqual(decoded["protocol_version"], 5)
        self.assertAlmostEqual(decoded["arm"]["theta_deg"][1], -0.2 * 57.29577951308232, places=6)
        self.assertAlmostEqual(decoded["arm"]["motor_feedback_rad"][2], 3.3)
        self.assertTrue(decoded["executing_flags"]["task_flow_active"])
        self.assertTrue(decoded["arm"]["motor_feedback_ok"]["j1"])
        self.assertTrue(decoded["arm"]["motor_feedback_ok"]["j2"])
        self.assertTrue(decoded["arm"]["motor_feedback_ok"]["j3"])

    def test_decode_task_flow_status_payload(self):
        payload = bytearray(28)
        payload[0] = 1
        payload[1] = 4
        payload[2] = 4
        payload[3] = 5
        payload[4] = 4
        payload[5] = 1
        payload[6] = 0
        struct.pack_into("<H", payload, 8, 3)
        struct.pack_into("<H", payload, 10, 24)
        struct.pack_into("<H", payload, 12, 2)
        struct.pack_into("<H", payload, 14, 11)
        struct.pack_into("<I", payload, 16, 42)
        struct.pack_into("<I", payload, 20, 1234)
        struct.pack_into("<I", payload, 24, 5678)

        frame = pack_usb_frame(0x66, bytes(payload))
        decoded = decode_usb_frame(UsbStreamParser().feed(frame)[0])["payload"]

        self.assertEqual(decoded["protocol_version"], 1)
        self.assertEqual(decoded["flow_name"], "S1_UP_S2_DOWN_V1")
        self.assertEqual(decoded["state_name"], "ERROR")
        self.assertEqual(decoded["error_name"], "PRECONDITION")
        self.assertEqual(decoded["current_op_name"], "POSTURE_CHECK")
        self.assertEqual(decoded["completed_s1_end_name"], "S1_UP_END")
        self.assertEqual(decoded["entry_index"], 3)
        self.assertEqual(decoded["entry_count"], 24)
        self.assertEqual(decoded["repeat_index"], 2)
        self.assertEqual(decoded["repeat_count"], 11)
        self.assertEqual(decoded["completed_steps"], 42)
        self.assertEqual(decoded["last_update_ms"], 5678)

    def test_decode_task_flow_v2_names_and_ops(self):
        payload = bytearray(28)
        payload[0] = 1
        payload[1] = 1
        payload[2] = 2
        payload[3] = 0
        payload[4] = 6
        struct.pack_into("<H", payload, 10, 12)

        frame = pack_usb_frame(0x66, bytes(payload))
        decoded = decode_usb_frame(UsbStreamParser().feed(frame)[0])["payload"]

        self.assertEqual(decoded["flow_name"], "S1_UP_V2")
        self.assertEqual(decoded["current_op_name"], "ARM_ENABLE_HOME")

    def test_decode_throw_block_flow_status(self):
        payload = bytearray(28)
        payload[0] = 1
        payload[1] = 8
        payload[2] = 2
        payload[4] = 9
        payload[5] = 2
        payload[7] = 1

        frame = pack_usb_frame(0x66, bytes(payload))
        decoded = decode_usb_frame(UsbStreamParser().feed(frame)[0])["payload"]

        self.assertEqual(decoded["flow_name"], "THROW_BLOCK_V1")
        self.assertEqual(decoded["current_op_name"], "ARM_WORKSPACE_SWITCH")
        self.assertEqual(decoded["completed_s1_end_name"], "S1_DOWN_END")
        self.assertEqual(decoded["flow_arg_name"], "X+")

    def test_decode_climb_status_payload(self):
        payload = bytearray(64)
        payload[0] = 5
        payload[1] = 1
        payload[2] = 0
        payload[3] = 1
        payload[4] = 0b100
        payload[5] = 1
        payload[6] = 8
        payload[7] = 10
        struct.pack_into("<I", payload, 8, 2345)
        struct.pack_into("<f", payload, 16, 12.5)
        struct.pack_into("<f", payload, 32, 220.0)
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["cmd_name"], "CLIMB_GET_STATUS")
        self.assertEqual(decoded["payload"]["state_name"], "STEP_05_FRONT_MINUS_30")
        self.assertTrue(decoded["payload"]["state_done"])
        self.assertTrue(decoded["payload"]["error_flags"]["test_action"])
        self.assertEqual(decoded["payload"]["active_source_name"], "USB")
        self.assertEqual(decoded["payload"]["climb_motor_online_count"], 8)
        self.assertEqual(decoded["payload"]["fdcan2_motor_online_count"], 8)
        self.assertEqual(decoded["payload"]["test_action_name"], "CHASSIS_FORWARD_100")
        self.assertEqual(decoded["payload"]["elapsed_ms"], 2345)
        self.assertAlmostEqual(decoded["payload"]["leg_pos_mm"][0], 12.5)
        self.assertAlmostEqual(decoded["payload"]["leg_target_mm"][0], 220.0)

    def test_decode_climb_prepare_status_payload(self):
        payload = bytearray(64)
        payload[0] = 24
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["payload"]["state_name"], "PREPARE_ALL_LEGS_MINUS_30")

        payload[0] = 25
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["payload"]["state_name"], "UP_PREPARE_CHASSIS_FORWARD_30")

        payload[0] = 26
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["payload"]["state_name"], "UP_LASER_APPROACH_X_LT_35")

        payload[0] = 27
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["payload"]["state_name"], "DOWN_LASER_APPROACH_H_GT_65")

        payload[0] = 28
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["payload"]["state_name"], "DOWN_PREPARE_CHASSIS_FORWARD_5")

    def test_climb_test_action_frame(self):
        frame = build_usb_command("CLIMB_TEST_ACTION", [16.0])
        parsed = UsbStreamParser().feed(frame)[0]
        self.assertEqual(parsed.cmd, 0x57)
        self.assertAlmostEqual(struct.unpack_from("<f", parsed.payload, 0)[0], 16.0)

        frame = build_usb_command("CLIMB_TEST_ACTION", [23.0])
        parsed = UsbStreamParser().feed(frame)[0]
        self.assertEqual(CLIMB_TEST_ACTIONS[23], "FRONT_220")
        self.assertAlmostEqual(struct.unpack_from("<f", parsed.payload, 0)[0], 23.0)

        self.assertEqual(CLIMB_TEST_ACTIONS[4], "REAR_DRIVE_FORWARD_30")
        self.assertEqual(CLIMB_TEST_ACTIONS[27], "FRONT_DRIVE_FORWARD_30")
        self.assertEqual(CLIMB_TEST_ACTIONS[33], "ALL_DRIVE_FORWARD_30")
        self.assertEqual(CLIMB_TEST_ACTIONS[39], "ALL_LEGS_300")
        self.assertEqual(SerialShell._parse_climb_test_action("FRONT_DRIVE_FORWARD_30"), 27)
        self.assertEqual(SerialShell._parse_climb_test_action("REAR_DRIVE_FORWARD_30"), 4)
        self.assertEqual(SerialShell._parse_climb_test_action("ALL_DRIVE_FORWARD_30"), 33)
        with self.assertRaises(ValueError):
            SerialShell._parse_climb_test_action("DRIVE_FORWARD_30")

        frame = build_usb_command("climb_all_drive_forward_30")
        parsed = UsbStreamParser().feed(frame)[0]
        self.assertEqual(parsed.cmd, 0x57)
        self.assertAlmostEqual(struct.unpack_from("<f", parsed.payload, 0)[0], 33.0)

        frames = build_climb_test_shortcut_sequence("climb_front_drive_forward_30")
        self.assertIsNotNone(frames)
        parsed = [UsbStreamParser().feed(frame)[0] for frame in frames]
        self.assertEqual([frame.cmd for frame in parsed], [0x02, 0x51, 0x57])
        self.assertAlmostEqual(struct.unpack_from("<f", parsed[2].payload, 0)[0], 27.0)

    def test_climb_downstairs_command_frames(self):
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_UP_STEP"))[0]
        self.assertEqual(parsed.cmd, 0x53)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_STEP"))[0]
        self.assertEqual(parsed.cmd, 0x53)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_UP_AUTO"))[0]
        self.assertEqual(parsed.cmd, 0x54)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_AUTO"))[0]
        self.assertEqual(parsed.cmd, 0x54)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_DOWN_STEP"))[0]
        self.assertEqual(parsed.cmd, 0x58)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_DOWNSTAIRS_STEP"))[0]
        self.assertEqual(parsed.cmd, 0x58)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_DOWN_AUTO"))[0]
        self.assertEqual(parsed.cmd, 0x59)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_DOWNSTAIRS_AUTO"))[0]
        self.assertEqual(parsed.cmd, 0x59)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_DOWNSTAIRS_RUN"))[0]
        self.assertEqual(parsed.cmd, 0x59)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_UP_GATE"))[0]
        self.assertEqual(parsed.cmd, 0x5A)
        parsed = UsbStreamParser().feed(build_usb_command("UP_LASER_GATE"))[0]
        self.assertEqual(parsed.cmd, 0x5A)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_DOWN_GATE"))[0]
        self.assertEqual(parsed.cmd, 0x5B)
        parsed = UsbStreamParser().feed(build_usb_command("DOWN_LASER_GATE"))[0]
        self.assertEqual(parsed.cmd, 0x5B)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_UP_AUTO_PAUSE"))[0]
        self.assertEqual(parsed.cmd, 0x5C)
        parsed = UsbStreamParser().feed(build_usb_command("UP_AUTO_PAUSE"))[0]
        self.assertEqual(parsed.cmd, 0x5C)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_DOWN_AUTO_PAUSE"))[0]
        self.assertEqual(parsed.cmd, 0x5D)
        parsed = UsbStreamParser().feed(build_usb_command("DOWN_AUTO_PAUSE"))[0]
        self.assertEqual(parsed.cmd, 0x5D)
        parsed = UsbStreamParser().feed(build_usb_command("CLIMB_AUTO_RESUME"))[0]
        self.assertEqual(parsed.cmd, 0x5E)
        parsed = UsbStreamParser().feed(build_usb_command("AUTO_RESUME"))[0]
        self.assertEqual(parsed.cmd, 0x5E)

    def test_arm_ik_test_flow_command_frames(self):
        parsed = UsbStreamParser().feed(build_usb_command("ARM_IK_TEST_FLOW"))[0]
        self.assertEqual(parsed.cmd, 0x27)
        self.assertEqual(parsed.payload, b"")

        parsed = UsbStreamParser().feed(build_usb_command("ARM_IK_TEST", [1.0, 8000.0]))[0]
        self.assertEqual(parsed.cmd, 0x27)
        values = struct.unpack("<4f", parsed.payload)
        self.assertEqual(values[0], 1.0)
        self.assertEqual(values[1], 8000.0)

    def test_arm_xyz_workspace_height_and_posture_frames(self):
        parsed = UsbStreamParser().feed(build_usb_command("ARM_SET_WORKSPACE", [1.0]))[0]
        self.assertEqual(parsed.cmd, 0x22)
        self.assertEqual(struct.unpack("<4f", parsed.payload)[0], 1.0)

        parsed = UsbStreamParser().feed(build_usb_command("ARM_XYZ", [100.0, 20.0, 450.0]))[0]
        self.assertEqual(parsed.cmd, 0x24)
        values = struct.unpack("<4f", parsed.payload)
        self.assertEqual(values[:3], (100.0, 20.0, 450.0))

        parsed = UsbStreamParser().feed(build_usb_command("ARM_HEIGHT_JOG", [20.0]))[0]
        self.assertEqual(parsed.cmd, 0x28)
        self.assertEqual(struct.unpack("<4f", parsed.payload)[0], 20.0)

        parsed = UsbStreamParser().feed(build_usb_command("ARM_JOINT_JOG", [2.0, -10.0]))[0]
        self.assertEqual(parsed.cmd, 0x2B)
        values = struct.unpack("<4f", parsed.payload)
        self.assertEqual(values[:2], (2.0, -10.0))

        parsed = UsbStreamParser().feed(build_usb_command("ARM_HEIGHT_LIMIT", [1.0]))[0]
        self.assertEqual(parsed.cmd, 0x29)
        self.assertEqual(struct.unpack("<4f", parsed.payload)[0], 1.0)

        parsed = UsbStreamParser().feed(build_usb_command("ARM_POSTURE", [2.0, 0.0]))[0]
        self.assertEqual(parsed.cmd, 0x2A)
        values = struct.unpack("<4f", parsed.payload)
        self.assertEqual(values[:2], (2.0, 0.0))

        self.assertEqual(SerialShell._parse_direction("X-"), 2)
        self.assertEqual(SerialShell._parse_tool("gripper"), 2)
        with self.assertRaisesRegex(ValueError, "S1 suction was removed"):
            SerialShell._optional_tool_payload(["S1"], "tool_on [TOOL]")
        self.assertEqual(SerialShell._parse_jog_amount(["50"], positive=False), -50.0)
        self.assertEqual(SerialShell._parse_joint_jog_amount(["30"]), 30.0)

    def test_arm_block_command_frames(self):
        block_ids = {block.block_id for block in list_block_commands()}
        self.assertIn("arm_j2_cw_10deg", block_ids)
        self.assertIn("arm_j2_cw_60deg", block_ids)
        self.assertIn("arm_j3_ccw_90deg", block_ids)
        self.assertIn("arm_space_y_pos", block_ids)
        self.assertIn("arm_space_x_pos", block_ids)
        self.assertIn("arm_space_x_neg", block_ids)
        self.assertIn("arm_show_record_status", block_ids)
        self.assertIn("base_forward_10mm", block_ids)
        self.assertIn("base_left_50mm", block_ids)
        self.assertIn("base_right_100mm", block_ids)
        self.assertIn("climb_all_legs_220", block_ids)
        self.assertIn("climb_front_drive_forward_30", block_ids)
        self.assertIn("climb_up_laser_gate", block_ids)
        self.assertIn("climb_down_laser_gate", block_ids)
        self.assertIn("climb_up_auto_pause", block_ids)
        self.assertIn("climb_down_auto_pause", block_ids)
        self.assertIn("climb_auto_resume", block_ids)
        self.assertIn("tool_s2_on", block_ids)
        self.assertNotIn("tool_s1_on", block_ids)

        frames = build_block_command_sequence("arm_j2_cw_10deg")
        parsed = UsbStreamParser().feed(frames[0])[0]
        self.assertEqual(parsed.cmd, 0x2B)
        self.assertEqual(struct.unpack("<4f", parsed.payload)[:2], (2.0, -10.0))

        frames = build_block_command_sequence("arm_j3_ccw_90deg")
        parsed = UsbStreamParser().feed(frames[0])[0]
        self.assertEqual(parsed.cmd, 0x2B)
        self.assertEqual(struct.unpack("<4f", parsed.payload)[:2], (3.0, 90.0))

        frames = build_block_command_sequence("arm_space_x_pos")
        parsed = UsbStreamParser().feed(frames[0])[0]
        self.assertEqual(parsed.cmd, 0x22)
        self.assertEqual(struct.unpack("<4f", parsed.payload)[0], 1.0)

        frames = build_block_command_sequence("base_forward_10mm")
        parsed = UsbStreamParser().feed(b"".join(frames))
        self.assertEqual([frame.cmd for frame in parsed], [0x02, 0x11, 0x12, 0x14])
        self.assertAlmostEqual(struct.unpack("<4f", parsed[-1].payload)[1], 0.01)

        frames = build_block_command_sequence("base_ccw_90deg")
        parsed = UsbStreamParser().feed(b"".join(frames))
        self.assertEqual([frame.cmd for frame in parsed], [0x02, 0x11, 0x12, 0x14])
        self.assertAlmostEqual(struct.unpack("<4f", parsed[-1].payload)[2], 1.5707963705062866)

        frames = build_block_command_sequence("climb_all_legs_220")
        parsed = UsbStreamParser().feed(b"".join(frames))
        self.assertEqual([frame.cmd for frame in parsed], [0x02, 0x51, 0x57])
        self.assertAlmostEqual(struct.unpack("<4f", parsed[-1].payload)[0], 1.0)

        frames = build_block_command_sequence("climb_up_laser_gate")
        parsed = UsbStreamParser().feed(b"".join(frames))
        self.assertEqual([frame.cmd for frame in parsed], [0x02, 0x51, 0x5A])

        frames = build_block_command_sequence("climb_down_laser_gate")
        parsed = UsbStreamParser().feed(b"".join(frames))
        self.assertEqual([frame.cmd for frame in parsed], [0x02, 0x51, 0x5B])

        frames = build_block_command_sequence("climb_up_auto_pause")
        parsed = UsbStreamParser().feed(b"".join(frames))
        self.assertEqual([frame.cmd for frame in parsed], [0x02, 0x51, 0x5C])

        frames = build_block_command_sequence("climb_down_auto_pause")
        parsed = UsbStreamParser().feed(b"".join(frames))
        self.assertEqual([frame.cmd for frame in parsed], [0x02, 0x51, 0x5D])

        frames = build_block_command_sequence("climb_auto_resume")
        parsed = UsbStreamParser().feed(b"".join(frames))
        self.assertEqual([frame.cmd for frame in parsed], [0x02, 0x51, 0x5E])

    def test_block_flow_struct_export(self):
        step = build_block_flow_step("base_forward_50mm", 1, note="forward")
        flow = build_block_flow_data("demo_flow", "2026-07-07 00:00:00", [step])

        self.assertEqual(flow["type"], "r2_block_flow")
        self.assertEqual(flow["step_count"], 1)
        self.assertEqual(flow["steps"][0]["frames"][3]["cmd"], 0x14)
        self.assertAlmostEqual(flow["steps"][0]["frames"][3]["values"][1], 0.05)
        self.assertEqual(flow["steps"][0]["wait_kind"], WAIT_CHASSIS)

        source = block_flow_to_c_source(flow, symbol="g_demo_flow")
        self.assertIn("#define G_DEMO_FLOW_STEP_COUNT 1U", source)
        self.assertIn("static const R2_BlockFlowStep_t g_demo_flow", source)
        self.assertIn('"base_forward_50mm"', source)
        self.assertIn("{0x14U, 1U", source)

    def test_mixed_block_flow_wait_kind_inference(self):
        climb_step = build_block_flow_step("climb_front_220", 1)
        self.assertEqual(climb_step["wait_kind"], WAIT_CLIMB)
        self.assertEqual(SerialShell._block_flow_step_wait_kind(climb_step), WAIT_CLIMB)

        climb_gate_step = build_block_flow_step("climb_up_laser_gate", 2)
        self.assertEqual(climb_gate_step["wait_kind"], WAIT_CLIMB)
        self.assertEqual(SerialShell._block_flow_step_wait_kind(climb_gate_step), WAIT_CLIMB)

        climb_resume_step = build_block_flow_step("climb_auto_resume", 3)
        self.assertEqual(climb_resume_step["wait_kind"], WAIT_CLIMB_FINAL)
        self.assertEqual(SerialShell._block_flow_step_wait_kind(climb_resume_step), WAIT_CLIMB_FINAL)

        arm_step = build_block_flow_step("arm_down_20mm", 4)
        self.assertEqual(SerialShell._block_flow_step_wait_kind(arm_step), WAIT_ARM)

        arm_space_step = build_block_flow_step("arm_space_x_neg", 5)
        self.assertEqual(arm_space_step["wait_kind"], WAIT_ARM)
        self.assertEqual(SerialShell._block_flow_step_wait_kind(arm_space_step), WAIT_ARM)

        legacy_base_step = {
            "category": "base",
            "frames": [{"command": "CHS_SET_POS", "cmd": 0x14, "values": [0.0, 0.05, 0.0], "has_float_payload": True}],
        }
        self.assertEqual(SerialShell._block_flow_step_wait_kind(legacy_base_step), WAIT_CHASSIS)

        task_flow_step = {
            "category": "task_flow",
            "wait_kind": WAIT_FLOW,
            "frames": [{"command": "FLOW_WEAPON_GRAB", "cmd": 0x67}],
        }
        self.assertEqual(SerialShell._block_flow_step_wait_kind(task_flow_step), WAIT_FLOW)

    def test_weapon_grab_firmware_start_is_arm_only(self):
        root = Path(__file__).resolve().parents[2]
        source = (root / "Components/Algorithm/Src/Data_Analysis.c").read_text(encoding="utf-8")
        case_body = source.split("case USB_CMD_FLOW_WEAPON_GRAB:", 1)[1].split(
            "case USB_CMD_FLOW_THROW_BLOCK:", 1
        )[0]

        self.assertIn("USB_StartArmTaskFlow", case_body)
        self.assertNotIn("USB_StartTaskFlow(", case_body)

    def test_gripper_opens_during_power_on_initialization(self):
        root = Path(__file__).resolve().parents[2]
        source = (root / "Core/Src/main.c").read_text(encoding="utf-8")
        power_on_body = source.split("/* USER CODE BEGIN 2 */", 1)[1].split(
            "/* USER CODE END 2 */", 1
        )[0]

        self.assertIn("H7_power();", power_on_body)
        self.assertIn(
            "R2_Arm_SetToolActuator(ROBOTARM_TOOL_GRIPPER, 1U);",
            power_on_body,
        )
        self.assertLess(
            power_on_body.index("H7_power();"),
            power_on_body.index("R2_Arm_SetToolActuator"),
        )

    def test_gripper_open_close_angles_are_reversed(self):
        root = Path(__file__).resolve().parents[2]
        source = (root / "Applications/R2_user/Src/R2_arm.c").read_text(
            encoding="utf-8"
        )

        self.assertIn("#define R2_ARM_GRIPPER_OPEN_DEG 120.0f", source)
        self.assertIn("#define R2_ARM_GRIPPER_CLOSE_DEG 0.0f", source)

    def test_weapon_grab_firmware_flow_includes_final_optimize_tail(self):
        root = Path(__file__).resolve().parents[2]
        source = (root / "Applications/Task/Src/Control_Task.c").read_text(encoding="utf-8")
        flow_body = source.split(
            "static const R2_TaskFlowEntry_t s_task_flow_weapon_grab_v1[] = {",
            1,
        )[1].split("};", 1)[0]

        self.assertIn(
            "\n".join(
                [
                    "    R2_TASK_ARM(3U, 1, 1U),",
                    "    R2_TASK_ARM(3U, 1, 1U),",
                    "    R2_TASK_ARM(2U, 5, 1U),",
                    "    R2_TASK_ARM(3U, 1, 1U),",
                    "    R2_TASK_ARM(3U, 1, 1U),",
                ]
            ),
            flow_body,
        )

    def test_weapon_dock_lower_computer_command_and_status(self):
        parsed = UsbStreamParser().feed(build_usb_command("WEAPON_DOCK_TEST"))[0]
        self.assertEqual(parsed.cmd, 0x69)
        self.assertEqual(parsed.payload, b"")

        self.assertEqual(UsbStreamParser().feed(build_usb_command("CHASSIS_MOVE_DONE"))[0].cmd, 0x6A)
        self.assertEqual(UsbStreamParser().feed(build_usb_command("DOCK_DONE"))[0].cmd, 0x6B)

        shell = SerialShell("COM_TEST", 115200)
        writes = []
        shell._write = writes.append
        self.assertTrue(shell._handle_line("weapon_dock_test"))
        parsed = UsbStreamParser().feed(writes[0])[0]
        self.assertEqual(parsed.cmd, 0x69)

        waits = []
        shell._wait_for_task_flow = lambda timeout, expected_flow_id=None: waits.append(
            (timeout, expected_flow_id)
        )
        self.assertTrue(shell._handle_line("weapon_dock_test_wait 12"))
        self.assertEqual(UsbStreamParser().feed(writes[1])[0].cmd, 0x69)
        self.assertEqual(waits, [(12.0, 9)])

        self.assertTrue(shell._handle_line("weapon_dock_status"))
        self.assertEqual(UsbStreamParser().feed(writes[2])[0].cmd, 0x66)

        self.assertTrue(shell._handle_line("weapon_chassis_done"))
        self.assertEqual(UsbStreamParser().feed(writes[3])[0].cmd, 0x6A)

        self.assertTrue(shell._handle_line("weapon_dock_done"))
        self.assertEqual(UsbStreamParser().feed(writes[4])[0].cmd, 0x6B)

        payload = bytearray(32)
        payload[0] = 2
        payload[1] = 9
        payload[2] = 2
        payload[4] = 11
        payload[28] = 1
        decoded = decode_usb_frame(
            UsbStreamParser().feed(pack_usb_frame(0x66, bytes(payload)))[0]
        )["payload"]
        self.assertEqual(decoded["flow_name"], "WEAPON_DOCK_TEST_V1")
        self.assertEqual(decoded["state_name"], "WAIT")
        self.assertEqual(decoded["current_op_name"], "HOST_CHECKPOINT_WAIT")
        self.assertEqual(decoded["host_checkpoint_pending"], 1)

    def test_weapon_dock_lower_computer_flow_contract(self):
        root = Path(__file__).resolve().parents[2]
        source = (root / "Applications/Task/Src/Control_Task.c").read_text(encoding="utf-8")
        flow_body = source.split(
            "static const R2_TaskFlowEntry_t s_task_flow_weapon_dock_test_v1[] = {",
            1,
        )[1].split("};", 1)[0]

        self.assertEqual(flow_body.count("R2_TASK_ARM_ENABLE_HOME()"), 2)
        self.assertIn("R2_TASK_TOOL_ON(ROBOTARM_TOOL_GRIPPER", flow_body)
        self.assertIn("R2_TASK_TOOL_OFF(ROBOTARM_TOOL_GRIPPER", flow_body)
        self.assertIn("R2_TASK_ARM_WORKSPACE(ROBOTARM_WORK_DIR_X_POS)", flow_body)
        self.assertIn("R2_TASK_ARM_WORKSPACE(ROBOTARM_WORK_DIR_X_NEG)", flow_body)
        self.assertIn(
            "\n".join(
                [
                    "    R2_TASK_ARM(3U, 1, 1U),",
                    "    R2_TASK_ARM(3U, 1, 1U),",
                    "    R2_TASK_ARM(2U, 5, 1U),",
                    "    R2_TASK_ARM(3U, 1, 1U),",
                    "    R2_TASK_ARM(3U, 1, 1U),",
                    "",
                    "    /* Host stages pause until the matching host confirmation command arrives. */",
                ]
            ),
            flow_body,
        )
        self.assertIn("R2_TASK_ARM(3U, -90, 1U)", flow_body)
        self.assertIn("R2_TASK_ARM(3U, -5, 1U)", flow_body)
        self.assertIn(
            "    R2_TASK_ARM(2U, -1, 1U),\n"
            "    R2_TASK_ARM(3U, 1, 2U),\n"
            "    R2_TASK_HOST_CHECKPOINT(1U), /* chassis move */",
            flow_body,
        )
        self.assertIn(
            "    R2_TASK_ARM(3U, 1, 4U),\n"
            "    R2_TASK_HOST_CHECKPOINT(2U), /* dock-complete decision */",
            flow_body,
        )
        self.assertEqual(flow_body.count("R2_TASK_HOST_CHECKPOINT"), 2)
        self.assertNotIn("R2_TASK_CLIMB", flow_body)

    def test_block_flow_repeat_count_runs_each_iteration(self):
        shell = SerialShell("COM_TEST", 115200)
        step = build_block_flow_step("arm_j2_cw_1deg", 1)
        step["repeat_count"] = 3
        writes = []
        waits = []

        shell._write_sequence = lambda frames: writes.append(frames)
        shell._wait_after_block_flow_step = (
            lambda step, timeout_s, arm_settle_s: waits.append((step["block_id"], timeout_s, arm_settle_s))
        )

        shell._run_block_flow_steps("demo", [step], 0, 1, timeout_s=5.0, arm_settle_s=0.25)

        self.assertEqual(len(writes), 3)
        self.assertEqual(len(waits), 3)
        self.assertTrue(all(wait == ("arm_j2_cw_1deg", 5.0, 0.25) for wait in waits))

    def test_block_flow_skips_noop_placeholder_step(self):
        shell = SerialShell("COM_TEST", 115200)
        step = {
            "index": 1,
            "block_id": "suction_2_interface_placeholder",
            "category": "tool_interface",
            "placeholder": True,
            "wait_kind": WAIT_NONE,
            "frames": [],
        }
        writes = []
        waits = []

        shell._write_sequence = lambda frames: writes.append(frames)
        shell._wait_after_block_flow_step = (
            lambda step, timeout_s, arm_settle_s: waits.append((step["block_id"], timeout_s, arm_settle_s))
        )

        shell._run_block_flow_steps("demo", [step], 0, 1, timeout_s=5.0, arm_settle_s=0.25)

        self.assertEqual(writes, [])
        self.assertEqual(waits, [])

    def test_block_flow_skips_step_marked_skip_on_run(self):
        shell = SerialShell("COM_TEST", 115200)
        step = build_block_flow_step("tool_s2_on", 1)
        step["skip_on_run"] = True
        writes = []
        waits = []

        shell._write_sequence = lambda frames: writes.append(frames)
        shell._wait_after_block_flow_step = (
            lambda step, timeout_s, arm_settle_s: waits.append((step["block_id"], timeout_s, arm_settle_s))
        )

        shell._run_block_flow_steps("demo", [step], 0, 1, timeout_s=5.0, arm_settle_s=0.25)

        self.assertEqual(writes, [])
        self.assertEqual(waits, [])

    def test_arm_wait_uses_selected_joint_feedback(self):
        step = build_block_flow_step("arm_j2_cw_90deg", 1)
        self.assertEqual(SerialShell._arm_wait_joint_indexes(step), (1,))

    def test_block_flow_arm_wait_uses_run_timeout_not_one_second_step_default(self):
        shell = SerialShell("COM_TEST", 115200)
        step = build_block_flow_step("arm_j2_cw_90deg", 1)
        calls = []

        shell._wait_for_arm_settle = lambda timeout_s, settle_s, step=None: calls.append((timeout_s, settle_s, step))

        shell._wait_after_block_flow_step(step, timeout_s=40.0, arm_settle_s=2.0)

        self.assertEqual(calls[0][0], 40.0)
        self.assertEqual(calls[0][1], 2.0)
        self.assertIs(calls[0][2], step)

    def test_arm_wait_delays_until_selected_joint_feedback_settles(self):
        shell = SerialShell("COM_TEST", 115200)
        step = build_block_flow_step("arm_j2_cw_90deg", 1)
        requests = []
        payloads = [
            self._arm_wait_payload(j2_feedback_deg=0.0),
            self._arm_wait_payload(j2_feedback_deg=-30.0),
            self._arm_wait_payload(j2_feedback_deg=-60.0),
            self._arm_wait_payload(j2_feedback_deg=-60.3),
            self._arm_wait_payload(j2_feedback_deg=-60.1),
            self._arm_wait_payload(j2_feedback_deg=-60.2),
        ]

        def request(timeout_s=1.0):
            requests.append(timeout_s)
            return payloads.pop(0)

        shell._request_arm_status_payload = request

        result = shell._wait_for_arm_settle(2.0, 0.0, step=step)

        self.assertAlmostEqual(result["motor_feedback_deg"][1], -60.2)
        self.assertEqual(len(requests), 6)

    @staticmethod
    def _arm_wait_payload(j2_feedback_deg):
        return {
            "state": 1,
            "state_name": "TARGET_VALID",
            "ik_status": 0,
            "ik_status_name": "OK",
            "ik_reason": 0,
            "ik_reason_name": "OK",
            "error_flags": {
                "ik": False,
                "joint_limit": False,
                "height_unreachable": False,
                "bad_param": False,
                "unsafe": False,
                "unsupported": False,
            },
            "theta_deg": [180.0, -90.0, 0.0],
            "motor_feedback_deg": [0.0, j2_feedback_deg, 45.0],
            "motor_feedback_ok": {"j1": True, "j2": True, "j3": True},
        }

    def test_interrupt_slot_fill_and_clear(self):
        placeholder = build_interrupt_placeholder_step(1, 7, group="demo")
        flow = build_block_flow_data("demo", "2026-07-08 00:00:00", [placeholder])

        self.assertTrue(is_interrupt_placeholder_step(flow["steps"][0]))
        slots = list_interrupt_slots(flow)
        self.assertEqual(slots[0]["slot"], 7)
        self.assertEqual(slots[0]["status"], "empty")

        filled = fill_interrupt_slot(flow, 7, "arm_j2_cw_10deg", note="j2 trim")
        self.assertFalse(is_interrupt_placeholder_step(filled))
        self.assertEqual(filled["interrupt_status"], "filled")
        self.assertEqual(filled["interrupt_group"], "demo")
        self.assertEqual(filled["block_id"], "arm_j2_cw_10deg")
        self.assertEqual(SerialShell._block_flow_step_wait_kind(filled), WAIT_ARM)

        cleared = clear_interrupt_slot(flow, 7)
        self.assertTrue(is_interrupt_placeholder_step(cleared))
        self.assertEqual(cleared["interrupt_status"], "empty")

    def test_test_flow_interrupt_helpers(self):
        flow = build_block_flow_data(
            "demo",
            "2026-07-08 00:00:00",
            [
                build_block_flow_step("climb_all_legs_220", 1),
                build_interrupt_placeholder_step(2, 1, group="demo"),
                build_interrupt_placeholder_step(3, 2, group="demo"),
                build_block_flow_step("climb_all_legs_zero", 4),
            ],
        )

        self.assertEqual(SerialShell._interrupt_bounds(flow), (1, 3))
        self.assertEqual(SerialShell._next_empty_interrupt_slot(flow), 1)
        fill_interrupt_slot(flow, 1, "arm_j2_cw_10deg")
        self.assertEqual(SerialShell._next_empty_interrupt_slot(flow), 2)
        fill_interrupt_slot(flow, 2, "arm_j3_ccw_10deg")
        self.assertIsNone(SerialShell._next_empty_interrupt_slot(flow))

        self.assertEqual(clear_all_interrupt_slots(flow), 2)
        self.assertEqual(SerialShell._next_empty_interrupt_slot(flow), 1)
        self.assertTrue(is_interrupt_placeholder_step(flow["steps"][1]))
        self.assertTrue(is_interrupt_placeholder_step(flow["steps"][2]))

    def test_clear_all_interrupt_slots_syncs_active_test_flow(self):
        flow = build_block_flow_data(
            "demo",
            "2026-07-08 00:00:00",
            [
                build_block_flow_step("climb_all_legs_220", 1),
                build_interrupt_placeholder_step(2, 1, group="demo"),
                build_interrupt_placeholder_step(3, 2, group="demo"),
                build_block_flow_step("climb_all_legs_zero", 4),
            ],
        )
        fill_interrupt_slot(flow, 1, "arm_j2_cw_10deg")
        fill_interrupt_slot(flow, 2, "arm_j3_ccw_10deg")

        shell = SerialShell("COM_TEST", 115200)
        with TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "flow.json"
            SerialShell._write_block_flow_file(path, flow)
            shell.test_flow_path = path
            shell.test_flow_data = flow
            shell.test_flow_name = "demo"
            shell.test_interrupt_bounds = shell._interrupt_bounds(flow)
            shell.test_next_slot = None
            shell.test_executed_slots = {1, 2}

            shell._block_flow_slot_clear_all([str(path)])

            self.assertEqual(shell.test_next_slot, 1)
            self.assertEqual(shell.test_executed_slots, set())
            self.assertTrue(is_interrupt_placeholder_step(shell.test_flow_data["steps"][1]))
            self.assertTrue(is_interrupt_placeholder_step(shell.test_flow_data["steps"][2]))

    def test_s2_flow_requires_completed_s1_prefix(self):
        shell = SerialShell("COM_TEST", 115200)
        s2_flow = build_block_flow_data("s1_up_s2_down_v2", "2026-07-08 00:00:00", [])
        s2_flow["metadata"] = {"requires_s1_prefix": "s1_up_v2"}

        with self.assertRaisesRegex(ValueError, "requires s1_up_v2"):
            shell._check_flow_preconditions(s2_flow)

        s1_flow = build_block_flow_data("s1_up_v2", "2026-07-08 00:00:00", [])
        shell._mark_flow_completed(s1_flow)
        shell._check_flow_preconditions(s2_flow)

    def test_block_command_uses_block_flow_recording_when_active(self):
        shell = SerialShell("COM_TEST", 115200)
        shell.block_flow_name = "mixed_flow"
        captured_args = []

        def fake_block_flow_add(args):
            captured_args.append(args)

        shell._block_flow_add = fake_block_flow_add
        shell._run_block(["climb_all_legs_220", "note"])

        self.assertEqual(captured_args, [["climb_all_legs_220", "note"]])

    def test_climb_flow_save_starts_recording_without_dropping_candidate(self):
        shell = SerialShell("COM_TEST", 115200)
        shell.last_flow_candidate = shell._build_flow_step(1, True, 40.0, None)

        shell._flow_save([])

        self.assertIsNotNone(shell.flow_name)
        self.assertEqual(len(shell.flow_steps), 1)
        self.assertEqual(shell.flow_steps[0]["action_id"], 1)
        self.assertIsNone(shell.last_flow_candidate)

    def test_tool_set_mode_target_frame(self):
        parsed = UsbStreamParser().feed(build_usb_command("TOOL_SET_MODE", [2.0, 1.0, 450.0, 0.25]))[0]
        self.assertEqual(parsed.cmd, 0x32)
        self.assertEqual(len(parsed.payload), 16)
        values = struct.unpack("<4f", parsed.payload)
        self.assertEqual(values[0], 2.0)
        self.assertEqual(values[1], 1.0)
        self.assertAlmostEqual(values[2], 450.0)
        self.assertAlmostEqual(values[3], 0.25)

    def test_decode_climb_downstairs_status_payload(self):
        payload = bytearray(68)
        payload[0] = 4
        payload[1] = 1
        payload[3] = 1
        payload[5] = 1
        payload[6] = 8
        payload[64] = 1
        payload[65] = 0x81
        payload[66] = 0x0F
        payload[67] = 0x03
        struct.pack_into("<I", payload, 8, 2345)
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)["payload"]
        self.assertEqual(decoded["flow_name"], "DOWNSTAIRS")
        self.assertEqual(decoded["state_name"], "DOWN_04_ALL_LEGS_UP_10")
        self.assertTrue(decoded["status_flags"]["motor_output_active"])
        self.assertTrue(decoded["status_flags"]["ready_for_next"])
        self.assertEqual(decoded["leg_reached"], [True, True, True, True])
        self.assertEqual(decoded["drive_reached"], [True, True])
        self.assertIsNone(decoded["summary_state"])
        self.assertIsNone(decoded["summary_state_name"])

    def test_decode_climb_summary_state_payload(self):
        payload = bytearray(69)
        payload[0] = 26
        payload[1] = 1
        payload[2] = 1
        payload[5] = 1
        payload[6] = 8
        payload[64] = 0
        payload[65] = 0x09
        payload[68] = 2
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)["payload"]
        self.assertEqual(decoded["state_name"], "UP_LASER_APPROACH_X_LT_35")
        self.assertEqual(decoded["summary_state"], 2)
        self.assertEqual(decoded["summary_state_name"], "RUNNING")

    def test_decode_climb_recovery_failed_payload(self):
        payload = bytearray(69)
        payload[0] = 30
        payload[1] = 1
        payload[2] = 1
        payload[4] = 0x10
        payload[64] = 1
        payload[68] = 4
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)["payload"]
        self.assertEqual(decoded["state_name"], "RECOVER_CHASSIS_BACKWARD_200")
        self.assertTrue(decoded["error_flags"]["recovery_failed"])
        self.assertEqual(decoded["summary_state_name"], "ERROR")

    def test_usart_remote_frame(self):
        frame = build_remote_frame(
            mode=3,
            source_usb=1,
            chassis=(0.2, 0.0, 0.0),
        )
        self.assertEqual(frame[0], 0xA5)
        self.assertEqual(frame[-1], 0x5A)
        self.assertEqual(len(frame), 43)
        self.assertEqual(frame[1 + 3], 1)
        self.assertEqual(frame[1 + 9], 1)
        self.assertEqual(frame[1 + 10], 0)
        self.assertEqual(frame[1 + 11], 0)
        self.assertEqual(frame[1 + 12], 0)
        self.assertEqual(frame[-2], sum(frame[1:-2]) & 0xFF)

    def test_usart_remote_climb_offsets(self):
        frame = build_remote_frame(
            mode=0,
            climb_enable=1,
            climb_step=1,
            climb_auto=0,
            chassis=(1.0, 2.0, 3.0),
        )
        data = frame[1:41]
        self.assertEqual(len(data), 40)
        self.assertEqual(data[13], 1)
        self.assertEqual(data[14], 1)
        self.assertEqual(data[15], 0)
        self.assertEqual(struct.unpack("<3f", data[16:28]), (1.0, 2.0, 3.0))
        self.assertEqual(bytes(data[28:40]), b"\x00" * 12)


class CrcTests(unittest.TestCase):
    def test_crc16_modbus_order(self):
        body = bytes.fromhex("A5 5A 00 06")
        self.assertEqual(crc16_modbus(body), 0xF982)

    def test_yaw_tune_command_pack(self):
        self.assertEqual(bytes_to_hex(build_usb_command("YAW_TUNE_GET_STATUS")), "A5 5A 00 49 0D C3 FF")
        self.assertEqual(bytes_to_hex(build_usb_command("YAW_TUNE_START", [1.0])), "A5 5A 10 47 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 BD 69 FF")

    def test_direct_command_names_are_case_insensitive(self):
        parsed = UsbStreamParser().feed(build_usb_command("tool_set_mode", [0.0, 1.0, 450.0, 0.0]))[0]
        self.assertEqual(parsed.cmd, 0x32)
        self.assertEqual(struct.unpack("<4f", parsed.payload), (0.0, 1.0, 450.0, 0.0))

        parsed = UsbStreamParser().feed(build_usb_command("ARM_SET_TARGET", [2.0, 1.0, 450.0, 0.25]))[0]
        self.assertEqual(parsed.cmd, 0x23)
        values = struct.unpack("<4f", parsed.payload)
        self.assertEqual(values[0], 2.0)
        self.assertEqual(values[1], 1.0)
        self.assertAlmostEqual(values[2], 450.0)
        self.assertAlmostEqual(values[3], 0.25)


if __name__ == "__main__":
    unittest.main()
