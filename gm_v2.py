# Copyright 2026 JerryLamMV
# SPDX-License-Identifier: Apache-2.0

"""GM_v2 系列云台私有协议 V1.0 —— Python 参考实现。

协议速览（详见 docs/gm_v2_protocol_v1.0_cn.md）:
- 上位机→云台: 18 字节定长帧, 同步头 A5 5A
- 云台→上位机: 28 字节定长帧, 同步头 5A A5
- CRC16: 多项式 0x1021, 初值 0, 不反射, 发送时高字节在前
- 串口 460800 8N1 TTL, 控制帧建议 50 Hz 周期发送
- V1.0 仅开放角度控制, 单位 0.01 度; 角速度控制为保留功能

仅依赖 Python 标准库（3.8+）。
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

# ---------------- 帧与协议常量 ----------------

SYNC_HOST_TO_GIMBAL = b"\xA5\x5A"   # 上位机 → 云台
SYNC_GIMBAL_TO_HOST = b"\x5A\xA5"   # 云台 → 上位机
PROTO_VERSION = 0x10                # 协议版本 V1.0

FRAME_UP_LEN = 18
FRAME_DOWN_LEN = 28

# 上行命令码 (cmd.code, 协议注释 3)
CMD_NONE = 0
CMD_GYRO_CALIBRATE = 1
CMD_START = 2
CMD_STOP = 3
CMD_MANUAL = 4

# 单轴工作模式 (协议注释 8)
WK_LOCK = 0      # 锁定: 该轴不做机械跟随, 依靠陀螺增稳保持指向
WK_FOLLOW = 1    # 跟随: 该轴随载机机械运动

AXIS_ROLL, AXIS_PITCH, AXIS_YAW = 0, 1, 2
AXIS_NAMES = ("Roll", "Pitch", "Yaw")

# 角度控制量程, 单位 0.01 度 (协议注释 10)
ANGLE_LIMIT_LSB = {
    AXIS_ROLL: (-6000, 6000),
    AXIS_PITCH: (-8900, 8900),
    AXIS_YAW: (-16000, 16000),
}

# V1.0 支持的三轴模式组合 (roll, pitch, yaw), 其它组合返回 cmd.stat = 2 (协议注释 8)
SUPPORTED_MODE_COMBOS = frozenset({
    (WK_LOCK, WK_LOCK, WK_FOLLOW),      # yaw 跟随
    (WK_FOLLOW, WK_LOCK, WK_FOLLOW),    # roll + yaw 跟随
    (WK_FOLLOW, WK_FOLLOW, WK_FOLLOW),  # 三轴跟随
})

# 硬件故障位图 (下行帧 hw_err, 协议注释 17)
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

RUN_STATES = {0: "初始化中", 1: "正常", 2: "已停止", 3: "保护中"}               # 协议注释 18
GIMBAL_MODES = {0: "已停止", 1: "yaw 跟随", 2: "roll+yaw 跟随", 3: "三轴跟随"}  # 协议注释 22
CMD_RESULTS = {0: "未定义", 1: "成功", 2: "失败"}                              # 协议注释 21


# ---------------- CRC 与单位换算 ----------------

def crc16(data: bytes) -> int:
    """CRC16: 多项式 0x1021, 初值 0, 不反射; 与协议文档的 C 函数逐位一致。"""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def deg_to_lsb(axis: int, deg: float) -> int:
    """角度(度) → 0.01 度 LSB, 超出该轴量程时按量程限幅（协议注释 10）。"""
    lo, hi = ANGLE_LIMIT_LSB[axis]
    return max(lo, min(hi, int(round(deg * 100))))


def lsb_to_deg(lsb: int) -> float:
    """0.01 度 LSB → 角度(度)。"""
    return lsb * 0.01


def is_mode_combo_supported(wk_roll: int, wk_pitch: int, wk_yaw: int) -> bool:
    """三轴模式组合是否为 V1.0 支持的组合（协议注释 8）。"""
    return (wk_roll, wk_pitch, wk_yaw) in SUPPORTED_MODE_COMBOS


# ---------------- 上行帧组包 ----------------

@dataclass
class AxisCommand:
    """单轴控制命令。

    wk_mode   : WK_LOCK / WK_FOLLOW
    value_deg : 期望的相机姿态增量, 单位度; None 表示本轴控制值无效,
                云台沿用该轴上一次的控制值
    go_zero   : 回中触发, 任一轴置位都会触发同一动作
    """

    wk_mode: int = WK_LOCK
    value_deg: float | None = 0.0
    go_zero: bool = False


def build_uplink_frame(cmd_code: int = CMD_MANUAL, trig: int = 0, sens: int = 0,
                       axes=None) -> bytes:
    """构造 18 字节上行帧。

    cmd_code : 命令码 (CMD_*); 陀螺校准要求温控就绪且设备静止（协议注释 3）
    trig     : 命令触发计数, 同命令码重复发送时必须改变
    sens     : 灵敏度 [0, 100]
    axes     : 长度 3 的序列 (Roll/Pitch/Yaw), 元素为 AxisCommand 或 None（无效轴）
    """
    if axes is None:
        axes = (AxisCommand(), AxisCommand(), AxisCommand())
    axes = tuple(AxisCommand() if a is None else a for a in axes)
    if len(axes) != 3:
        raise ValueError("axes 必须为长度 3 的序列 (Roll/Pitch/Yaw)")

    body = bytearray()
    body += SYNC_HOST_TO_GIMBAL
    body.append(PROTO_VERSION)
    body.append(((cmd_code & 0x1F) << 3) | (trig & 0x07))
    body.append(sens & 0xFF)
    body.append(0x00)  # reserved0: 本版本变焦档不开放, 必须填 0
    for i, axis in enumerate(axes):
        valid = axis.value_deg is not None
        value = deg_to_lsb(i, axis.value_deg) if valid else 0
        flags = (0x01 if valid else 0x00) \
            | (0x02 if axis.go_zero else 0x00) \
            | ((axis.wk_mode & 0x03) << 2)
        body += struct.pack("<Bh", flags, value)
    body.append(0x00)  # reserved
    body += struct.pack(">H", crc16(body))  # CRC 发送时高字节在前
    return bytes(body)


# ---------------- 下行帧解析 ----------------

@dataclass
class GimbalFeedback:
    """云台应答帧 (28 字节) 解析结果。"""

    protocol_version: int
    fw_version: str             # 如 "2.2"
    hw_error_bits: int          # 原始故障位图
    hw_errors: tuple            # 解码后的故障描述元组
    run_state: int              # 0 初始化中 / 1 正常 / 2 已停止 / 3 保护中
    run_state_text: str
    mounted_upside_down: bool   # False 正装 / True 倒装
    temp_control_ready: bool    # 温控就绪标志
    cmd_code: int               # 命令码回显
    cmd_stat: int               # 0 未定义 / 1 成功 / 2 失败
    cmd_stat_text: str
    mode: int                   # 当前工作模式
    mode_text: str
    cam_angle_deg: tuple        # 相机姿态角 (roll, pitch, yaw), 度
    motor_angle_deg: tuple      # 相机框架角 (roll, pitch, yaw), 度
    cam_rate_dps: tuple         # 相机角速度 (roll, pitch, yaw), 度/秒


def parse_downlink_frame(frame: bytes) -> GimbalFeedback | None:
    """解析 28 字节下行帧; 长度 / 同步头 / CRC 不合法时返回 None。"""
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
    """串口字节流解析器: 把串口收到的字节逐段喂入 feed(), 返回校验通过的应答帧。

    内部按 5A A5 同步头做定长截取, 以 CRC 校验确认完整帧, 截取逻辑与协议文档
    附录 3 的接收状态机一致。
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
