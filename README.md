# GM_v2 Series Gimbal Private Protocol · Python Reference Implementation

**English** | [简体中文](README_CN.md)

Open-source materials and a reference implementation for the GM_v2 series gimbal private protocol V1.0, intended for third-party developers who need to control a GM_v2 gimbal over a serial link.

- Protocol document (authoritative): [docs/gm_v2_protocol_v1.0.md](docs/gm_v2_protocol_v1.0.md) (bilingual EN/CN)
- Protocol version: V1.0 (firmware V2.2.x and later)
- License: [Apache-2.0](LICENSE)

## Protocol at a Glance

| Item | Host → Gimbal | Gimbal → Host |
|---|---|---|
| Frame length | 18 bytes, fixed | 28 bytes, fixed |
| Sync header | `A5 5A` | `5A A5` |
| Checksum | CRC16 (polynomial 0x1021, init 0, high byte first) | same |
| Payload | command code, sensitivity, 3-axis angle commands | firmware version, hardware faults, status, attitude / frame angles / angular rates |

- Serial: 460800, 8N1, TTL, full-duplex
- Communication logic: acknowledgement-based; send control frames periodically at 50 Hz
- Control values are in 0.01 deg units (signed, little-endian); V1.0 exposes angle control only
- Link-loss protection: with no valid downlink frame for 200 ms the gimbal freezes commands; after 2 s it clears the commands and slowly returns to neutral

## Repository Layout

```
├── docs/gm_v2_protocol_v1.0.md      Protocol document V1.0, bilingual EN/CN (frames / commands / CRC / coordinate system / example packets / C example)
├── gm_v2.py                         Python reference implementation (build / parse / CRC / stream parser), stdlib only
├── example.py                       Offline example: reproduces the example packets in Appendix 2 of the protocol document
├── serial_demo.py                   Serial example: 50 Hz periodic control + feedback printing (requires pyserial)
└── test_gm_v2.py                    Unit tests: asserted against the example packets of the protocol document
```

## Quick Start

No installation needed — copy `gm_v2.py` into your project (Python standard library only, 3.8+).

```python
import gm_v2

# Manual control: yaw follow, roll / pitch locked, pitch -15 deg, yaw +20 deg, sensitivity 50%
frame = gm_v2.build_uplink_frame(
    cmd_code=gm_v2.CMD_MANUAL,
    sens=50,
    axes=(
        gm_v2.AxisCommand(gm_v2.WK_LOCK, 0.0),
        gm_v2.AxisCommand(gm_v2.WK_LOCK, -15.0),
        gm_v2.AxisCommand(gm_v2.WK_FOLLOW, 20.0),
    ),
)
ser.write(frame)                     # send periodically at 50 Hz

fb = gm_v2.parse_downlink_frame(rx)  # parse the 28-byte feedback frame
print(fb.cam_angle_deg)              # camera attitude (roll, pitch, yaw) in deg
```

See [serial_demo.py](serial_demo.py) for full serial send/receive usage.

Run the tests (asserted against the two example packets in Appendix 2 of the protocol document):

```bash
python -m unittest test_gm_v2 -v
```

## Usage Notes

- V1.0 supports only three mode combinations: yaw follow / roll+yaw follow / three-axis follow. All other combinations are rejected by the gimbal (acknowledgement `cmd.stat = 2`).
- When the same command code is sent repeatedly, `trig` (trigger counter) must change; clear the command code after the acknowledgement is received.
- Gyro calibration requires the temperature control to be ready (acknowledgement `status.tca = 1`) and the device to stay still; it usually lasts several seconds.
- The gimbal has no magnetometer and receives no vehicle heading, so yaw is a relative angle (referenced to power-on or the most recent zero-position calibration), not an absolute heading.

## License

[Apache-2.0](LICENSE)
