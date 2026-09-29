# Copyright 2026 CaddxFPV
# SPDX-License-Identifier: Apache-2.0

"""GM_v2 series gimbal private protocol V1.0 — Python reference implementation.

Protocol at a glance (see docs/gm_v2_protocol_v1.0.md for details):
- Host → gimbal: 18-byte fixed-length frame, sync header A5 5A
- Gimbal → host: 28-byte fixed-length frame, sync header 5A A5
- CRC16: polynomial 0x1021, init 0, no reflection, high byte sent first
- Serial 460800 8N1 TTL, control frames recommended at 50 Hz
- V1.0 exposes angle control only, unit 0.01 deg; angular-rate control is reserved

Python standard library only (3.8+).
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

__all__ = [
    "SYNC_HOST_TO_GIMBAL", "SYNC_GIMBAL_TO_HOST", "PROTO_VERSION",
    "FRAME_UP_LEN", "FRAME_DOWN_LEN",
    "CMD_NONE", "CMD_GYRO_CALIBRATE", "CMD_START", "CMD_STOP", "CMD_MANUAL",
    "WK_LOCK", "WK_FOLLOW",
    "AXIS_ROLL", "AXIS_PITCH", "AXIS_YAW", "AXIS_NAMES",
    "ANGLE_LIMIT_LSB", "SUPPORTED_MODE_COMBOS",
    "AxisCommand", "GimbalFeedback", "DownlinkStreamParser",
    "crc16", "deg_to_lsb", "lsb_to_deg", "is_mode_combo_supported",
    "build_uplink_frame", "parse_downlink_frame",
]

# ---------------- Frame & protocol constants ----------------

SYNC_HOST_TO_GIMBAL = b"\xA5\x5A"   # host → gimbal
SYNC_GIMBAL_TO_HOST = b"\x5A\xA5"   # gimbal → host
PROTO_VERSION = 0x10                # protocol version V1.0

FRAME_UP_LEN = 18
FRAME_DOWN_LEN = 28

# Uplink command codes (cmd.code, protocol Note 3)
CMD_NONE = 0
CMD_GYRO_CALIBRATE = 1
CMD_START = 2
CMD_STOP = 3
CMD_MANUAL = 4

# Per-axis work mode (protocol Note 8)
WK_LOCK = 0      # locked: no mechanical following, holds pointing by gyro stabilization
WK_FOLLOW = 1    # follow: mechanically follows the vehicle

AXIS_ROLL, AXIS_PITCH, AXIS_YAW = 0, 1, 2
AXIS_NAMES = ("Roll", "Pitch", "Yaw")

# Angle control ranges, unit 0.01 deg (protocol Note 10)
ANGLE_LIMIT_LSB = {
    AXIS_ROLL: (-6000, 6000),
    AXIS_PITCH: (-8900, 8900),
    AXIS_YAW: (-16000, 16000),
}

# Three-axis mode combinations supported by V1.0 (roll, pitch, yaw);
# other combinations return cmd.stat = 2 (protocol Note 8)
SUPPORTED_MODE_COMBOS = frozenset({
    (WK_LOCK, WK_LOCK, WK_FOLLOW),      # yaw follow
    (WK_FOLLOW, WK_LOCK, WK_FOLLOW),    # roll + yaw follow
    (WK_FOLLOW, WK_FOLLOW, WK_FOLLOW),  # three-axis follow
})

# Hardware fault bitmap (downlink frame hw_err, protocol Note 17)
HW_ERROR_FLAGS = (
    (0x01, "供电异常"),
    (0x02, "IMU 通讯异常"),
    (0x04, "IMU 数据异常"),
    (0x08, "位置传感器异常"),
    (0x10, "输入信号异常"),
    (0x20, "参数存储异常"),
    (0x40, "电机堵转/过流"),
    (0x80, "限角保护触发"),
)

RUN_STATES = {0: "初始化中", 1: "正常", 2: "已停止", 3: "保护中"}               # protocol Note 18
GIMBAL_MODES = {0: "已停止", 1: "yaw 跟随", 2: "roll+yaw 跟随", 3: "三轴跟随"}  # protocol Note 22
CMD_RESULTS = {0: "未定义", 1: "成功", 2: "失败"}                              # protocol Note 21


# ---------------- CRC & unit conversion ----------------

def crc16(data: bytes) -> int:
    """CRC16: polynomial 0x1021, init 0, no reflection; bit-identical to the C function in the protocol document."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def deg_to_lsb(axis: int, deg: float) -> int:
    """Angle (deg) → 0.01 deg LSB, clamped to the range of the axis (protocol Note 10)."""
    lo, hi = ANGLE_LIMIT_LSB[axis]
    return max(lo, min(hi, int(round(deg * 100))))


def lsb_to_deg(lsb: int) -> float:
    """0.01 deg LSB → angle (deg)."""
    return lsb * 0.01


def is_mode_combo_supported(wk_roll: int, wk_pitch: int, wk_yaw: int) -> bool:
    """Whether the three-axis mode combination is supported by V1.0 (protocol Note 8)."""
    return (wk_roll, wk_pitch, wk_yaw) in SUPPORTED_MODE_COMBOS


# ---------------- Uplink frame building ----------------

@dataclass
class AxisCommand:
    """Per-axis control command.

    wk_mode   : WK_LOCK / WK_FOLLOW
    value_deg : desired camera attitude increment, in deg; None marks this
                axis's control value invalid, so the gimbal keeps the previous
                control value of this axis
    go_zero   : go-to-center trigger; setting it on any axis triggers the
                same action
    """

    wk_mode: int = WK_LOCK
    value_deg: float | None = 0.0
    go_zero: bool = False


def build_uplink_frame(cmd_code: int = CMD_MANUAL, trig: int = 0, sens: int = 0,
                       axes=None) -> bytes:
    """Build an 18-byte uplink frame.

    cmd_code : command code (CMD_*); gyro calibration requires the temperature
               control to be ready and the device stationary (protocol Note 3)
    trig     : command trigger counter; must change when the same command code
               is repeated
    sens     : sensitivity [0, 100]
    axes     : sequence of length 3 (Roll/Pitch/Yaw), items are AxisCommand
               or None (invalid axis)
    """
    if axes is None:
        axes = (AxisCommand(), AxisCommand(), AxisCommand())
    axes = tuple(AxisCommand() if a is None else a for a in axes)
    if len(axes) != 3:
        raise ValueError("axes must be a sequence of length 3 (Roll/Pitch/Yaw)")

    body = bytearray()
    body += SYNC_HOST_TO_GIMBAL
    body.append(PROTO_VERSION)
    body.append(((cmd_code & 0x1F) << 3) | (trig & 0x07))
    body.append(sens & 0xFF)
    body.append(0x00)  # reserved0: zoom steps are not exposed in this version, must be 0
    for i, axis in enumerate(axes):
        valid = axis.value_deg is not None
        value = deg_to_lsb(i, axis.value_deg) if valid else 0
        flags = (0x01 if valid else 0x00) \
            | (0x02 if axis.go_zero else 0x00) \
            | ((axis.wk_mode & 0x03) << 2)
        body += struct.pack("<Bh", flags, value)
    body.append(0x00)  # reserved
    body += struct.pack(">H", crc16(body))  # CRC high byte first
    return bytes(body)


# ---------------- Downlink frame parsing ----------------

@dataclass
class GimbalFeedback:
    """Parsed result of a gimbal feedback frame (28 bytes)."""

    protocol_version: int
    fw_version: str             # e.g. "2.2"
    hw_error_bits: int          # raw hardware fault bitmap
    hw_errors: tuple            # decoded fault descriptions
    run_state: int              # 0 initializing / 1 normal / 2 stopped / 3 protected
    run_state_text: str
    mounted_upside_down: bool   # False upright / True inverted
    temp_control_ready: bool    # temperature-control ready flag
    cmd_code: int               # command code echo
    cmd_stat: int               # 0 undefined / 1 success / 2 failure
    cmd_stat_text: str
    mode: int                   # current work mode
    mode_text: str
    cam_angle_deg: tuple        # camera attitude (roll, pitch, yaw), deg
    motor_angle_deg: tuple      # camera frame angles (roll, pitch, yaw), deg
    cam_rate_dps: tuple         # camera angular rates (roll, pitch, yaw), deg/s


def parse_downlink_frame(frame: bytes) -> GimbalFeedback | None:
    """Parse a 28-byte downlink frame; return None when length / sync header / CRC is invalid."""
    if len(frame) != FRAME_DOWN_LEN or frame[:2] != SYNC_GIMBAL_TO_HOST:
        return None
    (crc_rx,) = struct.unpack(">H", frame[26:28])
    if crc16(frame[:26]) != crc_rx:
        return None

    hw_err = frame[4]
    status = frame[5]
    cmd = frame[6]

    return GimbalFeedback(
        protocol_version=frame[2],
        fw_version=f"{frame[3] >> 4}.{frame[3] & 0x0F}",
        hw_error_bits=hw_err,
        hw_errors=tuple(text for bit, text in HW_ERROR_FLAGS if hw_err & bit),
        run_state=status & 0x03,
        run_state_text=RUN_STATES.get(status & 0x03, "未知"),
        mounted_upside_down=bool(status & 0x04),
        temp_control_ready=bool(status & 0x08),
        cmd_code=(cmd >> 3) & 0x1F,
        cmd_stat=cmd & 0x07,
        cmd_stat_text=CMD_RESULTS.get(cmd & 0x07, "未知"),
        mode=frame[7],
        mode_text=GIMBAL_MODES.get(frame[7], "未知"),
        cam_angle_deg=tuple(lsb_to_deg(v) for v in struct.unpack("<3h", frame[8:14])),
        motor_angle_deg=tuple(lsb_to_deg(v) for v in struct.unpack("<3h", frame[14:20])),
        cam_rate_dps=tuple(v * 0.1 for v in struct.unpack("<3h", frame[20:26])),
    )


class DownlinkStreamParser:
    """Serial byte-stream parser: feed received bytes into feed() and get back CRC-validated feedback frames.

    Internally it slices fixed-length frames at 5A A5 sync headers and confirms
    complete frames by CRC, matching the receive state machine in Appendix 3 of
    the protocol document.
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list:
        out = []
        for byte in bytes(data):
            self._buf.append(byte)
            n = len(self._buf)
            if n == 1:
                if byte != 0x5A:
                    self._buf.clear()
            elif n == 2:
                if byte != 0xA5:
                    self._buf.clear()
                    if byte == 0x5A:
                        self._buf.append(byte)
            elif n == FRAME_DOWN_LEN:
                fb = parse_downlink_frame(bytes(self._buf))
                if fb is not None:
                    out.append(fb)
                self._buf.clear()
        return out
