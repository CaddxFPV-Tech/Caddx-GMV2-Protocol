# Copyright 2026 JerryLamMV
# SPDX-License-Identifier: Apache-2.0

"""离线示例：不需要硬件。

示例数据取自协议文档附录 2：
  示例 1 —— 上位机手动控制帧（yaw 跟随，灵敏度 50%，pitch -15°，yaw +20°）
  示例 2 —— 云台应答帧（命令 4 执行成功，yaw 跟随，状态正常，固件 V2.2）
"""

import gm_v2

if __name__ == "__main__":
    frame = gm_v2.build_uplink_frame(
        cmd_code=gm_v2.CMD_MANUAL,
        sens=50,
        axes=(
            gm_v2.AxisCommand(gm_v2.WK_LOCK, 0.0),
            gm_v2.AxisCommand(gm_v2.WK_LOCK, -15.0),
            gm_v2.AxisCommand(gm_v2.WK_FOLLOW, 20.0),
        ),
    )
    print("上行帧  :", frame.hex(" ").upper())
    print("文档预期: A5 5A 10 20 32 00 01 00 00 01 24 FA 05 D0 07 00 01 3A")

    reply = bytes.fromhex(
        "5A A5 10 22 00 09 21 01 23 00 28 FB 08 02 23 00 46 FB F9 01 01 00 FD FF 02 00 C0 20"
    )
    fb = gm_v2.parse_downlink_frame(reply)
    print("下行帧解析:")
    print("  固件版本:", fb.fw_version)
    print("  硬件故障:", "0x%02X" % fb.hw_error_bits, fb.hw_errors or "无")
    print("  运行状态:", fb.run_state_text,
          "| 安装:", "倒装" if fb.mounted_upside_down else "正装",
          "| 温控就绪:", fb.temp_control_ready)
    print("  命令应答: code=%d %s" % (fb.cmd_code, fb.cmd_stat_text))
    print("  工作模式:", fb.mode_text)
    print("  姿态角  : %+.2f %+.2f %+.2f 度" % fb.cam_angle_deg)
    print("  框架角  : %+.2f %+.2f %+.2f 度" % fb.motor_angle_deg)
    print("  角速度  : %+.2f %+.2f %+.2f 度/秒" % fb.cam_rate_dps)
