# Copyright 2026 CaddxFPV
# SPDX-License-Identifier: Apache-2.0

"""Offline example: no hardware required.

The example data comes from Appendix 2 of the protocol document:
  Example 1 — host manual control frame (yaw follow, sensitivity 50%, pitch -15 deg, yaw +20 deg)
  Example 2 — gimbal acknowledgement frame (command 4 succeeded, yaw follow, state normal, firmware V2.2)
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
    print("uplink frame :", frame.hex(" ").upper())
    print("expected     : A5 5A 10 20 32 00 01 00 00 01 24 FA 05 D0 07 00 01 3A")

    reply = bytes.fromhex(
        "5A A5 10 22 00 09 21 01 23 00 28 FB 08 02 23 00 46 FB F9 01 01 00 FD FF 02 00 C0 20"
    )
    fb = gm_v2.parse_downlink_frame(reply)
    print("downlink frame parsed:")
    print("  firmware version:", fb.fw_version)
    print("  hw faults       :", "0x%02X" % fb.hw_error_bits, fb.hw_errors or "none")
    print("  run state       :", fb.run_state_text,
          "| mount:", "inverted" if fb.mounted_upside_down else "upright",
          "| temp control ready:", fb.temp_control_ready)
    print("  cmd ack         : code=%d %s" % (fb.cmd_code, fb.cmd_stat_text))
    print("  work mode       :", fb.mode_text)
    print("  attitude angles : %+.2f %+.2f %+.2f deg" % fb.cam_angle_deg)
    print("  frame angles    : %+.2f %+.2f %+.2f deg" % fb.motor_angle_deg)
    print("  angular rates   : %+.2f %+.2f %+.2f deg/s" % fb.cam_rate_dps)
