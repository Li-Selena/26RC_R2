import struct
import unittest

from serial_tool.commands import build_climb_test_shortcut_sequence, build_usb_command
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

    def test_known_4float_frame(self):
        frame = build_usb_command("SYS_SWITCH_SOURCE", [1.0])
        self.assertEqual(
            bytes_to_hex(frame),
            "A5 5A 10 02 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 38 4B FF",
        )

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

        arm = bytearray(64)
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
        struct.pack_into("<I", arm, 8, 123)
        struct.pack_into("<I", arm, 16, 7)
        struct.pack_into("<f", arm, 24, 450.0)
        struct.pack_into("<f", arm, 28, 0.5)
        struct.pack_into("<f", arm, 32, 0.1)
        struct.pack_into("<f", arm, 44, 12.0)
        struct.pack_into("<f", arm, 52, 450.0)
        struct.pack_into("<f", arm, 60, 2.0)
        frame = pack_usb_frame(0x26, bytes(arm))
        decoded = decode_usb_frame(UsbStreamParser().feed(frame)[0])["payload"]
        self.assertEqual(decoded["state_name"], "TARGET_VALID")
        self.assertEqual(decoded["ik_status_name"], "HEIGHT_UNREACHABLE")
        self.assertEqual(decoded["ik_reason_name"], "HEIGHT_OUTSIDE_LINK")
        self.assertTrue(decoded["status_flags"]["active"])
        self.assertTrue(decoded["error_flags"]["height_unreachable"])
        self.assertEqual(decoded["tool_name"], "S2")
        self.assertEqual(decoded["tool_state_name"], "USE")
        self.assertAlmostEqual(decoded["target_z_mm"], 450.0)
        self.assertAlmostEqual(decoded["tool_world_mm"][2], 450.0)
        self.assertAlmostEqual(decoded["height_error_mm"], 2.0)

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
        payload[238] = 0x08
        payload[239] = 1
        frame = pack_usb_frame(0x46, bytes(payload))
        decoded = decode_usb_frame(UsbStreamParser().feed(frame)[0])["payload"]
        self.assertEqual(decoded["protocol_version"], 3)
        self.assertAlmostEqual(decoded["chassis"]["target_vel"]["vx"], 0.4)
        self.assertAlmostEqual(decoded["chassis"]["target_pos"]["dx"], 1.25)
        self.assertEqual(decoded["arm"]["state_name"], "TARGET_VALID")
        self.assertEqual(decoded["arm"]["tool_name"], "GRIPPER")
        self.assertEqual(decoded["arm"]["tool_state_name"], "USE")
        self.assertAlmostEqual(decoded["arm"]["target_z_mm"], 450.0)
        self.assertAlmostEqual(decoded["arm"]["theta_rad"][2], 0.3)
        self.assertAlmostEqual(decoded["arm"]["tool_world_mm"][2], 450.0)
        self.assertEqual(decoded["arm"]["output_apply_count"], 6)
        self.assertTrue(decoded["climb_summary_error_flags"]["flow_switch"])
        self.assertTrue(decoded["active_source_stale"])

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
