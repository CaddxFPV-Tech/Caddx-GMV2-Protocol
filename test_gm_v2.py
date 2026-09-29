# Copyright 2026 CaddxFPV
# SPDX-License-Identifier: Apache-2.0

"""Unit tests asserted against the example packets in Appendix 2 of the protocol document.

Run: python -m unittest test_gm_v2 -v
"""

import struct
import unittest

import gm_v2

DOC_EXAMPLE1_HEX = "A5 5A 10 20 32 00 01 00 00 01 24 FA 05 D0 07 00 01 3A"
DOC_EXAMPLE2_HEX = ("5A A5 10 22 00 09 21 01 23 00 28 FB 08 02 23 00 46 FB "
                    "F9 01 01 00 FD FF 02 00 C0 20")


class Crc16Tests(unittest.TestCase):
    def test_matches_document_example1(self):
        frame = bytes.fromhex(DOC_EXAMPLE1_HEX)
        self.assertEqual(gm_v2.crc16(frame[:16]), 0x013A)

    def test_matches_document_example2(self):
        frame = bytes.fromhex(DOC_EXAMPLE2_HEX)
        self.assertEqual(gm_v2.crc16(frame[:26]), 0xC020)


class BuildUplinkFrameTests(unittest.TestCase):
    def test_manual_frame_matches_document_example1(self):
        frame = gm_v2.build_uplink_frame(
            cmd_code=gm_v2.CMD_MANUAL,
            sens=50,
            axes=(
                gm_v2.AxisCommand(gm_v2.WK_LOCK, 0.0),
                gm_v2.AxisCommand(gm_v2.WK_LOCK, -15.0),
                gm_v2.AxisCommand(gm_v2.WK_FOLLOW, 20.0),
            ),
        )
        self.assertEqual(frame, bytes.fromhex(DOC_EXAMPLE1_HEX))

    def test_frame_length_and_crc_self_consistent(self):
        frame = gm_v2.build_uplink_frame(cmd_code=gm_v2.CMD_START, trig=1)
        self.assertEqual(len(frame), gm_v2.FRAME_UP_LEN)
        (crc,) = struct.unpack(">H", frame[16:18])
        self.assertEqual(gm_v2.crc16(frame[:16]), crc)

    def test_command_code_and_trig_layout(self):
        frame = gm_v2.build_uplink_frame(cmd_code=gm_v2.CMD_GYRO_CALIBRATE, trig=5)
        self.assertEqual(frame[3], (gm_v2.CMD_GYRO_CALIBRATE << 3) | 5)

    def test_invalid_axis_keeps_flag_cleared(self):
        frame = gm_v2.build_uplink_frame(axes=(gm_v2.AxisCommand(value_deg=None), None, None))
        self.assertEqual(frame[6], 0x00)  # Roll flag byte: valid=0

    def test_go_zero_with_invalid_value(self):
        frame = gm_v2.build_uplink_frame(
            axes=(None, None, gm_v2.AxisCommand(value_deg=None, go_zero=True)))
        self.assertTrue(frame[12] & 0x02)   # Yaw flag byte: go_zero set
        self.assertFalse(frame[12] & 0x01)  # control value of this axis is invalid

    def test_go_zero_alongside_valid_value(self):
        frame = gm_v2.build_uplink_frame(axes=(None, None, gm_v2.AxisCommand(go_zero=True)))
        self.assertEqual(frame[12] & 0x03, 0x03)  # go_zero and valid both set

    def test_value_clamped_to_axis_range(self):
        self.assertEqual(gm_v2.deg_to_lsb(gm_v2.AXIS_PITCH, 120.0), 8900)
        self.assertEqual(gm_v2.deg_to_lsb(gm_v2.AXIS_YAW, -200.0), -16000)
        self.assertEqual(gm_v2.deg_to_lsb(gm_v2.AXIS_ROLL, 30.0), 3000)


class ParseDownlinkFrameTests(unittest.TestCase):
    def setUp(self):
        self.fb = gm_v2.parse_downlink_frame(bytes.fromhex(DOC_EXAMPLE2_HEX))

    def test_rejects_bad_frame(self):
        frame = bytearray(bytes.fromhex(DOC_EXAMPLE2_HEX))
        frame[-1] ^= 0xFF
        self.assertIsNone(gm_v2.parse_downlink_frame(bytes(frame)))
        self.assertIsNone(gm_v2.parse_downlink_frame(frame[:-1]))
        self.assertIsNone(gm_v2.parse_downlink_frame(b"\x00" * gm_v2.FRAME_DOWN_LEN))

    def test_header_fields(self):
        self.assertEqual(self.fb.fw_version, "2.2")
        self.assertEqual(self.fb.hw_error_bits, 0x00)
        self.assertEqual(self.fb.run_state, 1)
        self.assertEqual(self.fb.run_state_text, "正常")
        self.assertFalse(self.fb.mounted_upside_down)
        self.assertTrue(self.fb.temp_control_ready)

    def test_command_echo(self):
        self.assertEqual(self.fb.cmd_code, 4)
        self.assertEqual(self.fb.cmd_stat, 1)
        self.assertEqual(self.fb.cmd_stat_text, "成功")

    def test_mode_and_angles(self):
        self.assertEqual(self.fb.mode, 1)
        self.assertEqual(self.fb.mode_text, "yaw 跟随")
        self.assertEqual(tuple(round(v, 6) for v in self.fb.cam_angle_deg), (0.35, -12.4, 5.2))
        self.assertEqual(tuple(round(v, 6) for v in self.fb.motor_angle_deg), (0.35, -12.1, 5.05))
        self.assertEqual(tuple(round(v, 6) for v in self.fb.cam_rate_dps), (0.1, -0.3, 0.2))

    def test_hw_error_decoding(self):
        frame = bytearray(bytes.fromhex(DOC_EXAMPLE2_HEX))
        frame[4] = 0x41  # power supply fault | motor stall/overcurrent
        frame[26:28] = struct.pack(">H", gm_v2.crc16(bytes(frame[:26])))
        fb = gm_v2.parse_downlink_frame(bytes(frame))
        self.assertEqual(fb.hw_errors, ("供电异常", "电机堵转/过流"))


class DownlinkStreamParserTests(unittest.TestCase):
    def test_extracts_frame_after_garbage_prefix(self):
        frame = bytes.fromhex(DOC_EXAMPLE2_HEX)
        parser = gm_v2.DownlinkStreamParser()
        out = []
        out += parser.feed(b"\x11\x22\x5A\x00")  # fake sync header
        out += parser.feed(frame[:9])
        out += parser.feed(frame[9:])
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0].fw_version, "2.2")

    def test_byte_by_byte_feed(self):
        frame = bytes.fromhex(DOC_EXAMPLE2_HEX)
        parser = gm_v2.DownlinkStreamParser()
        out = []
        for b in frame:
            out += parser.feed(bytes([b]))
        self.assertEqual(len(out), 1)


class ModeComboTests(unittest.TestCase):
    def test_document_supported_combos(self):
        self.assertTrue(gm_v2.is_mode_combo_supported(gm_v2.WK_LOCK, gm_v2.WK_LOCK, gm_v2.WK_FOLLOW))
        self.assertTrue(gm_v2.is_mode_combo_supported(gm_v2.WK_FOLLOW, gm_v2.WK_LOCK, gm_v2.WK_FOLLOW))
        self.assertTrue(gm_v2.is_mode_combo_supported(gm_v2.WK_FOLLOW, gm_v2.WK_FOLLOW, gm_v2.WK_FOLLOW))

    def test_all_locked_not_supported(self):
        self.assertFalse(gm_v2.is_mode_combo_supported(gm_v2.WK_LOCK, gm_v2.WK_LOCK, gm_v2.WK_LOCK))


if __name__ == "__main__":
    unittest.main()
