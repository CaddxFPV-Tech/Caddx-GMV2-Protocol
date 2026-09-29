# GM_v2 系列云台私有协议 · Python 参考实现

[English](README.md) | **简体中文**

GM_v2 系列云台私有协议 V1.0 的开源资料与参考实现，面向需要通过串口直连控制 GM_v2 云台的第三方开发者。

- 协议文档（权威）: [docs/gm_v2_protocol_v1.0.md](docs/gm_v2_protocol_v1.0.md)（中英双语）
- 协议版本: V1.0（对应固件 V2.2.x 及以上）
- 许可证: [Apache-2.0](LICENSE)

## 协议速览

| 项目 | 上位机 → 云台 | 云台 → 上位机 |
|---|---|---|
| 帧长 | 18 字节定长 | 28 字节定长 |
| 同步头 | `A5 5A` | `5A A5` |
| 校验 | CRC16（多项式 0x1021，初值 0，高字节在前） | 同左 |
| 主要内容 | 命令码、灵敏度、三轴角度指令 | 固件版本、硬件故障、状态、姿态角 / 框架角 / 角速度 |

- 串口: 460800, 8N1, TTL，全双工
- 通信逻辑: 应答式，云台收到合法数据包后回复；控制帧建议 50 Hz 周期发送
- 控制值单位 0.01 度（有符号小端），V1.0 仅开放角度控制
- 断链保护: 200 ms 未收到合法下行帧云台冻结指令；2 s 清空指令缓慢回中

## 仓库内容

```
├── docs/gm_v2_protocol_v1.0.md      协议文档 V1.0，中英双语（帧结构 / 命令 / CRC / 坐标系 / 示例包 / C 示例）
├── gm_v2.py                         Python 参考实现（组包 / 解析 / CRC / 流式解析），仅依赖标准库
├── example.py                       离线示例：复现协议文档附录 2 的示例包
├── serial_demo.py                   串口示例：50 Hz 周期控制 + 应答打印（依赖 pyserial）
└── test_gm_v2.py                    单元测试：以协议文档示例包为断言基准
```

## 快速开始

无需安装，直接把 `gm_v2.py` 复制进你的工程即可（仅依赖 Python 标准库，3.8+）。

```python
import gm_v2

# 手动控制：yaw 跟随，roll / pitch 锁定，pitch -15°，yaw +20°，灵敏度 50%
frame = gm_v2.build_uplink_frame(
    cmd_code=gm_v2.CMD_MANUAL,
    sens=50,
    axes=(
        gm_v2.AxisCommand(gm_v2.WK_LOCK, 0.0),
        gm_v2.AxisCommand(gm_v2.WK_LOCK, -15.0),
        gm_v2.AxisCommand(gm_v2.WK_FOLLOW, 20.0),
    ),
)
ser.write(frame)                     # 以 50 Hz 周期发送

fb = gm_v2.parse_downlink_frame(rx)  # 解析 28 字节应答帧
print(fb.cam_angle_deg)              # 相机姿态角 (roll, pitch, yaw)，单位度
```

串口流式收发的完整用法见 [serial_demo.py](serial_demo.py)。

运行测试（以协议文档附录 2 的两个示例包为断言基准）:

```bash
python -m unittest test_gm_v2 -v
```

## 使用注意

- V1.0 仅支持三种模式组合：yaw 跟随 / roll+yaw 跟随 / 三轴跟随，其它组合会被云台拒绝（应答 `cmd.stat = 2`）。
- 同一命令码重复发送时必须改变 `trig`（触发计数），收到应答后命令码清零。
- 陀螺校准前温控需就绪（应答 `status.tca = 1`），校准期间设备保持静止，通常持续数秒。
- 云台无磁力计，也不接收载机航向，Yaw 为相对角（以上电时刻或最近一次零位标定为基准），不是绝对航向。

## License

[Apache-2.0](LICENSE)
