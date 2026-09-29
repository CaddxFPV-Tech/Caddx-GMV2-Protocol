# Copyright 2026 CaddxFPV
# SPDX-License-Identifier: Apache-2.0

"""Serial send/receive example: sends a manual control frame periodically at 50 Hz and prints gimbal feedback.

Requires pyserial: pip install pyserial
Usage: python serial_demo.py COM5        (Linux / macOS: /dev/ttyUSB0)
"""

import sys
import time

import serial

import gm_v2


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit(f"usage: python {sys.argv[0]} <port, e.g. COM5 or /dev/ttyUSB0>")
    port = sys.argv[1]

    with serial.Serial(port, 460800, bytesize=serial.EIGHTBITS,
                       parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                       timeout=0.02) as ser:
        parser = gm_v2.DownlinkStreamParser()
        # Example: yaw follow, roll / pitch locked, pitch -15 deg, yaw +20 deg
        axes = (
            gm_v2.AxisCommand(gm_v2.WK_LOCK, 0.0),
            gm_v2.AxisCommand(gm_v2.WK_LOCK, -15.0),
            gm_v2.AxisCommand(gm_v2.WK_FOLLOW, 20.0),
        )
        frame = gm_v2.build_uplink_frame(cmd_code=gm_v2.CMD_MANUAL, sens=50, axes=axes)

        period = 1.0 / 50.0
        next_tx = time.perf_counter()
        print(f"opened {port}, press Ctrl+C to exit")
        while True:
            now = time.perf_counter()
            if now >= next_tx:
                ser.write(frame)
                next_tx += period
            for fb in parser.feed(ser.read(64)):
                print(fb)
            idle = next_tx - time.perf_counter()
            if idle > 0:
                time.sleep(min(idle, 0.002))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
