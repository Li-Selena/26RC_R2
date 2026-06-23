import struct
import unittest

from serial_tool.commands import build_usb_command
from serial_tool.protocol import UsbStreamParser, bytes_to_hex, crc16_modbus, pack_usb_frame
from serial_tool.status import decode_usb_frame
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
        payload[97] = 4
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
        self.assertEqual(decoded["climb"]["state_name"], "STEP_04_ALL_LEGS_DOWN_10")
        self.assertEqual(decoded["climb"]["test_action_name"], "CHASSIS_FORWARD_100")
        self.assertTrue(decoded["climb"]["test_chassis_active"])
        self.assertEqual(decoded["laser"]["distance_mm"]["y_pos"], -1)
        self.assertEqual(decoded["laser"]["update_tick"], 9876)
        self.assertEqual(decoded["yaw_tune"]["state_name"], "RUNNING")
        self.assertEqual(decoded["yaw_tune"]["active_mode_name"], "WORLD_VEL")
        self.assertAlmostEqual(decoded["yaw_tune"]["score"], 8.75)

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

    def test_decode_climb_status_payload(self):
        payload = bytearray(64)
        payload[0] = 4
        payload[1] = 1
        payload[2] = 0
        payload[3] = 1
        payload[4] = 0b100
        payload[5] = 1
        payload[6] = 6
        payload[7] = 10
        struct.pack_into("<I", payload, 8, 2345)
        struct.pack_into("<f", payload, 16, 12.5)
        struct.pack_into("<f", payload, 32, 220.0)
        frame = pack_usb_frame(0x56, bytes(payload))
        parsed = UsbStreamParser().feed(frame)[0]
        decoded = decode_usb_frame(parsed)
        self.assertEqual(decoded["cmd_name"], "CLIMB_GET_STATUS")
        self.assertEqual(decoded["payload"]["state_name"], "STEP_04_ALL_LEGS_DOWN_10")
        self.assertTrue(decoded["payload"]["state_done"])
        self.assertTrue(decoded["payload"]["error_flags"]["test_action"])
        self.assertEqual(decoded["payload"]["active_source_name"], "USB")
        self.assertEqual(decoded["payload"]["fdcan2_motor_online_count"], 6)
        self.assertEqual(decoded["payload"]["test_action_name"], "CHASSIS_FORWARD_100")
        self.assertEqual(decoded["payload"]["elapsed_ms"], 2345)
        self.assertAlmostEqual(decoded["payload"]["leg_pos_mm"][0], 12.5)
        self.assertAlmostEqual(decoded["payload"]["leg_target_mm"][0], 220.0)

    def test_climb_test_action_frame(self):
        frame = build_usb_command("CLIMB_TEST_ACTION", [16.0])
        parsed = UsbStreamParser().feed(frame)[0]
        self.assertEqual(parsed.cmd, 0x57)
        self.assertAlmostEqual(struct.unpack_from("<f", parsed.payload, 0)[0], 16.0)

    def test_usart_remote_frame(self):
        frame = build_remote_frame(mode=3, source_usb=1, chassis=(0.2, 0.0, 0.0), arm_target=(0.0, 0.0, 180.0))
        self.assertEqual(frame[0], 0xA5)
        self.assertEqual(frame[-1], 0x5A)
        self.assertEqual(len(frame), 42)
        self.assertEqual(frame[1 + 3], 1)
        self.assertEqual(frame[1 + 9], 1)
        self.assertEqual(frame[-2], sum(frame[1:-2]) & 0xFF)

    def test_usart_remote_climb_offsets(self):
        frame = build_remote_frame(
            mode=0,
            climb_enable=1,
            climb_step=1,
            climb_auto=0,
            chassis=(1.0, 2.0, 3.0),
            arm_target=(4.0, 5.0, 6.0),
        )
        data = frame[1:40]
        self.assertEqual(len(data), 39)
        self.assertEqual(data[12], 1)
        self.assertEqual(data[13], 1)
        self.assertEqual(data[14], 0)
        self.assertEqual(struct.unpack("<6f", data[15:39]), (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))


class CrcTests(unittest.TestCase):
    def test_crc16_modbus_order(self):
        body = bytes.fromhex("A5 5A 00 06")
        self.assertEqual(crc16_modbus(body), 0xF982)

    def test_yaw_tune_command_pack(self):
        self.assertEqual(bytes_to_hex(build_usb_command("YAW_TUNE_GET_STATUS")), "A5 5A 00 49 0D C3 FF")
        self.assertEqual(bytes_to_hex(build_usb_command("YAW_TUNE_START", [1.0])), "A5 5A 10 47 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 BD 69 FF")

    def test_direct_command_names_are_case_insensitive(self):
        self.assertEqual(
            bytes_to_hex(build_usb_command("tool_set_mode", [0.0])),
            "A5 5A 10 32 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 88 8B FF",
        )
        self.assertEqual(
            bytes_to_hex(build_usb_command("ARM_SET_TARGET", [200.0, 0.0, 180.0])),
            "A5 5A 10 23 00 00 48 43 00 00 00 00 00 00 34 43 00 00 00 00 2D 25 FF",
        )


if __name__ == "__main__":
    unittest.main()
