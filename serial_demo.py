# Copyright 2026 JerryLamMV
# SPDX-License-Identifier: Apache-2.0

"""串口收发示例：50 Hz 周期发送手动控制帧，并打印云台应答。

依赖 pyserial: pip install pyserial
用法: python serial_demo.py COM5        （Linux / macOS: /dev/ttyUSB0）
"""

import sys
import time

import serial

import gm_v2


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit(f"用法: python {sys.argv[0]} <串口名，如 COM5 或 /dev/ttyUSB0>")
    port = sys.argv[1]

    with serial.Serial(port, 115200, bytesize=serial.EIGHTBITS,
                       parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                       timeout=0.02) as ser:
        parser = gm_v2.DownlinkStreamParser()
        # 示例：yaw 跟随，roll / pitch 锁定，pitch -15°，yaw +20°
        axes = (
            gm_v2.AxisCommand(gm_v2.WK_LOCK, 0.0),
            gm_v2.AxisCommand(gm_v2.WK_LOCK, -15.0),
            gm_v2.AxisCommand(gm_v2.WK_FOLLOW, 20.0),
        )
        frame = gm_v2.build_uplink_frame(cmd_code=gm_v2.CMD_MANUAL, sens=50, axes=axes)

        period = 1.0 / 50.0
        next_tx = time.perf_counter()
        print(f"已打开 {port}，按 Ctrl+C 退出")
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
