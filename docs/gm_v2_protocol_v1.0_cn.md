# GM_v2 系列云台私有协议

协议版本 V1.0 · 2026.09

## 修订记录

| 日期 | 文档版本 | 对应固件 | 说明 |
|---|---|---|---|
| 2026.09.24 | V1.0 | V2.2.x 及以上 | 首版 |

## 目录

- [接口配置](#接口配置)
- [角度定义](#角度定义)
- [上位机→云台数据包结构](#上位机云台数据包结构)
- [云台→上位机数据包结构](#云台上位机数据包结构)
- [CRC 函数](#crc-函数)
- [附录 1：坐标系定义](#附录-1坐标系定义)
- [附录 2：示例数据包](#附录-2示例数据包)
- [附录 3：示例代码](#附录-3示例代码)

配套 Python 参考实现见仓库根目录（`gm_v2.py`），其组包/解析结果以本文附录 2 的示例包为基准做了断言测试。

---

## 接口配置

- 电平：TTL
- 数据位：8
- 停止位：1
- 校验：无
- 通信方式：全双工
- 通信逻辑：应答式。上位机先发送数据包，云台收到合法数据包后回复
- 波特率：460800
- 频率：50 Hz
- 字节序：小端（CRC 除外）
- 帧长度：上位机→云台 18 字节，云台→上位机 28 字节

连续 200 ms 未收到合法下行帧时，云台冻结当前指令并保持相机姿态；连续 2 s 未收到时清空各轴指令，在保持增稳的前提下缓慢回到中立位置。

## 角度定义

本文中：

| 术语 | 定义 |
|---|---|
| 相机姿态角 | 相机在世界坐标系（NED）下的欧拉角 |
| 相机框架角 | 相机在载机坐标系下的角度，也称为关节角 |
| 相机与载机差值角 | 角度控制的指令值：相机姿态相对**进入当前模式瞬间**姿态的增量 |
| 锁定 | 该轴不做机械跟随，依靠陀螺增稳保持指向 |
| 跟随 | 该轴随载机机械运动 |

## 上位机→云台数据包结构

```c
#pragma pack(1)
typedef struct
{
     uint8_t sync[2];    //注释 1
     uint8_t ver;        //注释 2
     struct
     {
          uint8_t trig:3; //注释 3
          uint8_t code:5;
     } cmd;
     uint8_t sens;       //注释 4
     uint8_t reserved0;  //注释 5
     struct
     {
          uint8_t valid:1;   //注释 6
          uint8_t go_zero:1; //注释 7
          uint8_t wk_mode:2; //注释 8
          uint8_t op_type:1; //注释 9（保留）
          uint8_t        :3;
          int16_t value;     //注释 10
     } gbc[3];               //注释 11
     uint8_t reserved;       //注释 12
     uint8_t crc[2];         //注释 13
} Gcu2GbcPkt_t;
#pragma pack()
```

线上字段按字节定义，不应依赖编译器位域布局：命令字节 bit[2:0] 为
`trig`、bit[7:3] 为 `code`；每个轴的标志字节 bit0 为 `valid`、bit1 为
`go_zero`、bit[3:2] 为 `wk_mode`、bit4 为 `op_type`、bit[7:5] 为保留位。
所有多字节有符号整数均采用 little-endian two's-complement 表示。

上行帧固定偏移：`0..1` 同步头，`2` 协议版本，`3` 命令字节，`4` 灵敏度，
`5` 保留，`6`/`7..8` Roll 标志/数值，`9`/`10..11` Pitch 标志/数值，
`12`/`13..14` Yaw 标志/数值，`15` 保留，`16..17` CRC16。下行帧固定偏移：
`0..1` 同步头，`2` 协议版本，`3` 固件版本，`4` 硬件故障，`5` 状态，
`6` 命令应答，`7` 工作模式，`8..13` 相机姿态角，`14..19` 框架角，
`20..25` 角速度，`26..27` CRC16。

注释 1：数据同步头 0xA5 0x5A

注释 2：协议版本，V1.0 = 0x10

注释 3：命令触发计数（bit[2:0]）与命令码（bit[7:3]）。

        0 - 未定义；1 - 陀螺校准；2 - 启动云台；3 - 停止云台；
        4 - 手动控制；5 - 未定义。

        同命令码重复时必须改变 trig；命令发送一次，主机收到应答后将
        命令码清零。

        陀螺校准：执行前温控状态必须为就绪（下行 status.tca = 1），且
        整个校准过程要求设备保持静止，通常持续数秒。

        启动云台：使能电机与增稳功能。停止云台：关闭电机与增稳功能，
        仅在通电存放时使用。

注释 4：灵敏度，取值范围 [0, 100]，单位 1%。数值越高响应越快，但
        抗振性能下降。

注释 5：保留。本版本不开放变焦档，上位机必须填 0；非 0 时忽略。

注释 6：本轴控制值有效标志。1 表示本轴控制值有效；0 表示沿用该轴
        上一次的控制值。

注释 7：回中触发。该位由 0 变为 1 时触发云台回中，任一轴置位都会
        触发同一动作。

注释 8：本轴工作模式。
        0 - 锁定；1 - 跟随；2、3 - 保留。
        三轴工作模式组合定义如下，V1.0 仅支持：
        001 - yaw 跟随（roll / pitch 锁定）
        101 - roll + yaw 跟随（pitch 锁定）
        111 - 三轴跟随

        其它组合（含 000）返回 cmd.stat = 2，并保持原模式。

注释 9：控制类型，本版本保留：仅开放角度控制，上位机须固定填 0。
        非 0 时忽略该轴并返回 cmd.stat = 2（角速度控制留作 V1.x）。

注释 10：本轴控制值：期望的相机姿态增量（角度控制），单位 0.01 度，
         量程 roll [-6000, 6000]、pitch [-8900, 8900]、
         yaw [-16000, 16000]。超出量程时按对应轴量程限幅。

注释 11：0 - Roll；1 - Pitch；2 - Yaw。

注释 12：保留，必须填 0；非 0 时忽略。

注释 13：CRC16，见 CRC 函数。

## 云台→上位机数据包结构

```c
#pragma pack(1)
typedef struct
{
     uint8_t sync[2];    //注释 14
     uint8_t ver;        //注释 15
     uint8_t fw_ver;     //注释 16
     uint8_t hw_err;     //注释 17
     struct
     {
          uint8_t run_state:2; //注释 18
          uint8_t mount:1;     //注释 19
          uint8_t tca:1;       //注释 20
          uint8_t         :4;
     } status;
     struct
     {
          uint8_t stat:3; //注释 21
          uint8_t code:5;
     } cmd;
     uint8_t mode;           //注释 22
     int16_t cam_angle[3];   //注释 23
     int16_t mtr_angle[3];   //注释 24
     int16_t cam_rate[3];    //注释 25
     uint8_t crc[2];         //注释 26
} Gbc2GcuPkt_t;
#pragma pack()
```

注释 14：数据同步头 0x5A 0xA5。

注释 15：协议版本，V1.0 = 0x10

注释 16：固件版本。bit[7:4] 主版本，bit[3:0] 次版本（V2.2 = 0x22）

注释 17：硬件故障位图：
         0x01 - 供电异常；0x02 - IMU 通讯异常；0x04 - IMU 数据异常；
         0x08 - 位置传感器异常；0x10 - 输入信号异常；
         0x20 - 参数存储异常；0x40 - 电机堵转 / 过流；
         0x80 - 限角保护触发

注释 18：云台状态。0 - 初始化中；1 - 正常；2 - 已停止；3 - 保护中

注释 19：安装方向标志。0 - 正装；1 - 倒装。

注释 20：温控就绪标志。0 - 未就绪；1 - 就绪

注释 21：命令执行状态：0 - 未定义；1 - 成功；2 - 失败。
         bit[7:3] 为命令码回显。

注释 22：当前工作模式：0 - 已停止；1 - yaw 跟随；2 - roll + yaw
         跟随；3 - 三轴跟随。

注释 23：相机姿态角，单位 0.01 度。
         0 - Roll，[-18000, 18000)；1 - Pitch，[-9000, 9000]；
         2 - Yaw，[-18000, 18000)

注释 24：相机框架角，单位 0.01 度，由电机位置反馈计算得到。
         0 - Roll；1 - Pitch；2 - Yaw

注释 25：相机机体角速度，单位 0.1 度/秒。
         0 - Roll；1 - Pitch；2 - Yaw

注释 26：CRC16，见 CRC 函数。

## CRC 函数

```c
uint16_t CalculateCrc16(uint8_t *ptr, uint16_t len)
{
     uint16_t crc = 0;
     while (len-- != 0)
     {
          crc ^= (uint16_t)(*ptr) << 8;
          for (uint8_t i = 0; i < 8; i++)
          {
               if (crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
               else
                    crc = crc << 1;
          }
          ptr++;
     }
     return crc;
}
```

CRC 计算范围从数据同步头开始，到 CRC 字段之前一个字节为止（上位机→
云台数据包为 16 字节，云台→上位机数据包为 26 字节），发送时高字节
在前。

## 附录 1：坐标系定义

世界坐标系：NED（X - 北，Y - 东，Z - 地）。

相机坐标系：X - 光轴向前，Y - 向右，Z - 向下（右手系；相机水平朝前
时与世界坐标系重合）。

旋转顺序：Z → Y → X。

```
                X  （光轴，向前）
                ^
                |
                |
                +----------------> Y  （向右）
               /
              /
             v
            Z  （向下）
```

正方向定义：

| 角度 | 正方向 |
|---|---|
| Roll | 相机绕光轴向右倾为正 |
| Pitch | 光轴抬头为正 |
| Yaw | 俯视时顺时针（右转）为正 |

云台没有磁力计，也不接收载机航向，因此上报的 Yaw 为相对角，不是绝对
航向。Yaw 以上电时刻或最近一次零位标定为基准。

## 附录 2：示例数据包

示例 1 - 手动控制，yaw 跟随，灵敏度 50%，pitch -15.00 度、
yaw +20.00 度：

```
A5 5A 10 20 32 00 01 00 00 01 24 FA 05 D0 07 00 01 3A
```

| 字节 | 数值 | 解析 |
|---|---|---|
| 0 - 1 | A5 5A | 数据同步头 |
| 2 | 10 | 协议 V1.0 |
| 3 | 20 | 命令码 4（手动控制），触发计数 0 |
| 4 | 32 | 灵敏度 50% |
| 5 | 00 | 保留（固定 0） |
| 6 - 8 | 01 00 00 | Roll：有效，锁定，角度 0.00 度 |
| 9 - 11 | 01 24 FA | Pitch：有效，锁定，角度 0xFA24 = -1500 → -15.00 度 |
| 12 - 14 | 05 D0 07 | Yaw：有效，跟随，角度 0x07D0 = +2000 → +20.00 度 |
| 15 | 00 | 保留 |
| 16 - 17 | 01 3A | CRC16 = 0x013A |

示例 2 - 云台应答，命令 4 执行成功，yaw 跟随，状态正常，正装，温控
就绪，固件 V2.2：

```
5A A5 10 22 00 09 21 01 23 00 28 FB 08 02 23 00 46 FB F9 01 01 00 FD FF 02 00 C0 20
```

| 字节 | 数值 | 解析 |
|---|---|---|
| 0 - 1 | 5A A5 | 数据同步头 |
| 2 | 10 | 协议 V1.0 |
| 3 | 22 | 固件 V2.2 |
| 4 | 00 | 无硬件故障 |
| 5 | 09 | 状态正常，正装，温控就绪 |
| 6 | 21 | 命令 4 执行成功 |
| 7 | 01 | 当前模式为 yaw 跟随 |
| 8 - 13 | 23 00 28 FB 08 02 | 姿态角：roll 0.35 度，pitch -12.40 度，yaw 5.20 度 |
| 14 - 19 | 23 00 46 FB F9 01 | 框架角：roll 0.35 度，pitch -12.10 度，yaw 5.05 度 |
| 20 - 25 | 01 00 FD FF 02 00 | 角速度：roll 0.1 度/秒，pitch -0.3 度/秒，yaw 0.2 度/秒 |
| 26 - 27 | C0 20 | CRC16 = 0xC020 |

## 附录 3：示例代码

示例面向带 1 路 UART（460800 8N1）和 20 ms 时基的 MCU。只需适配
`uart_send_bytes()`、`get_tick_ms()` 与串口接收中断。

```c
/* =====================================================================
 *  GM 系列云台私有协议 V1.0 —— 上位机侧示例代码
 * ===================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---------------- 数据包结构（与协议文档一致） ---------------- */
#pragma pack(push, 1)
typedef struct { uint8_t trig:3; uint8_t code:5; } gcu_cmd_t;
typedef struct
{
     uint8_t valid:1;    /* 本轴控制值有效            */
     uint8_t go_zero:1;  /* 回中触发                  */
     uint8_t wk_mode:2;  /* 0 锁定，1 跟随            */
     uint8_t op_type:1;  /* 控制类型，V1.0 固定为 0  */
     uint8_t :3;         /* 保留                     */
     int16_t value;
} gcu_axis_t;

typedef struct
{
     uint8_t sync[2];    /* A5 5A */
     uint8_t ver;
     gcu_cmd_t cmd;
     uint8_t sens;
     uint8_t reserved0;
     gcu_axis_t gbc[3];  /* 0 = Roll，1 = Pitch，2 = Yaw */
     uint8_t reserved;
     uint8_t crc[2];
} Gcu2GbcPkt_t;

typedef struct { uint8_t run_state:2; uint8_t mount:1; uint8_t tca:1; uint8_t :4; } gbc_status_t;
typedef struct { uint8_t stat:3; uint8_t code:5; } gbc_cmd_t;

typedef struct
{
     uint8_t sync[2];    /* 5A A5 */
     uint8_t ver;
     uint8_t fw_ver;
     uint8_t hw_err;
     gbc_status_t status;
     gbc_cmd_t cmd;
     uint8_t mode;
     int16_t cam_angle[3];  /* 0.01 度，   Roll / Pitch / Yaw */
     int16_t mtr_angle[3];  /* 0.01 度，   Roll / Pitch / Yaw */
     int16_t cam_rate[3];   /* 0.1 度/秒， Roll / Pitch / Yaw */
     uint8_t crc[2];
} Gbc2GcuPkt_t;
#pragma pack(pop)

/* ---------------- 常量 ---------------- */
#define GBC_SYNC0 0xA5
#define GBC_SYNC1 0x5A
#define GCU_SYNC0 0x5A
#define GCU_SYNC1 0xA5
#define PROTO_VER 0x10

#define AXIS_ROLL  0
#define AXIS_PITCH 1
#define AXIS_YAW   2

enum { CMD_NONE = 0, CMD_GYRO_CALI = 1, CMD_START = 2, CMD_STOP = 3, CMD_MANUAL = 4 };
enum { WK_LOCK = 0, WK_FOLLOW = 1 };
enum { MODE_STOP = 0, MODE_YAW_FOLLOW = 1, MODE_ROLL_YAW_FOLLOW = 2,
       MODE_THREE_FOLLOW = 3 };

#define ANGLE_TO_LSB(deg) ((int16_t)((deg) * 100.0f))  /* 单位 0.01 度 */

/* ---------------- CRC16 ---------------- */
uint16_t CalculateCrc16(uint8_t *ptr, uint16_t len)
{
     uint16_t crc = 0;
     while (len-- != 0)
     {
          crc ^= (uint16_t)(*ptr) << 8;
          for (uint8_t i = 0; i < 8; i++)
               crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                    : (uint16_t)(crc << 1);
          ptr++;
     }
     return crc;
}

/* ---------------- 平台相关接口（由用户实现） ---------------- */
extern void uart_send_bytes(const uint8_t *data, uint16_t len);
extern uint32_t get_tick_ms(void);

/* ---------------- 发送 ---------------- */
static void send_packet(Gcu2GbcPkt_t *p)
{
     uint16_t crc;
     p->sync[0] = GBC_SYNC0;
     p->sync[1] = GBC_SYNC1;
     p->ver = PROTO_VER;
     crc = CalculateCrc16((uint8_t *)p, sizeof(Gcu2GbcPkt_t) - 2);
     p->crc[0] = (uint8_t)(crc >> 8);
     p->crc[1] = (uint8_t)(crc & 0xFF);
     uart_send_bytes((uint8_t *)p, sizeof(Gcu2GbcPkt_t));
}

/* 一次性命令：1 陀螺校准，2 启动，3 停止。 */
void gimbal_send_command(uint8_t code, uint8_t trig)
{
     Gcu2GbcPkt_t pkt;
     memset(&pkt, 0, sizeof(pkt));
     pkt.cmd.code = code;
     pkt.cmd.trig = trig & 0x07;
     send_packet(&pkt);
}

/* 手动控制帧，建议 50 Hz 调用
 * wk[i]     : WK_LOCK / WK_FOLLOW
 * value[i]  : 角度控制值，单位 0.01 度
 * sens      : 0 ~ 100 (%)
 */
void gimbal_send_manual(const uint8_t wk[3], const int16_t value[3], uint8_t sens)
{
     Gcu2GbcPkt_t pkt;
     memset(&pkt, 0, sizeof(pkt));
     pkt.cmd.code = CMD_MANUAL;
     pkt.cmd.trig = 0;
     pkt.sens = sens;
     for (uint8_t i = 0; i < 3; i++)
     {
          pkt.gbc[i].valid = 1;
          pkt.gbc[i].wk_mode = wk[i];
          pkt.gbc[i].value = value[i];
     }
     send_packet(&pkt);
}

/* 回中：任一轴置位即可触发。 */
void gimbal_send_go_zero(void)
{
     Gcu2GbcPkt_t pkt;
     memset(&pkt, 0, sizeof(pkt));
     pkt.cmd.code = CMD_MANUAL;
     pkt.cmd.trig = 0;
     pkt.gbc[AXIS_YAW].go_zero = 1;
     send_packet(&pkt);
}

/* ---------------- 接收 ---------------- */
static Gbc2GcuPkt_t s_response;
static uint8_t  s_rx_buf[sizeof(Gbc2GcuPkt_t)];
static uint16_t s_rx_len = 0;
static uint32_t s_rx_tick_ms = 0;

static void gimbal_on_packet(uint8_t *buf)
{
     Gbc2GcuPkt_t *p = (Gbc2GcuPkt_t *)buf;
     uint16_t crc = CalculateCrc16(buf, sizeof(Gbc2GcuPkt_t) - 2);
     uint16_t received_crc = ((uint16_t)buf[sizeof(Gbc2GcuPkt_t) - 2] << 8) |
                             buf[sizeof(Gbc2GcuPkt_t) - 1];
     if (p->ver != PROTO_VER || received_crc != crc)
          return;

     s_response = *p;
     s_rx_tick_ms = get_tick_ms();
}

/* 在串口接收中断中调用 */
void gimbal_rx_byte_isr(uint8_t byte)
{
      if (s_rx_len == 0)
      {
          if (byte != GCU_SYNC0) return;
          s_rx_buf[s_rx_len++] = byte;
      }
      else if (s_rx_len == 1)
      {
          if (byte == GCU_SYNC1)
          {
               s_rx_buf[s_rx_len++] = byte;
          }
          else
          {
               s_rx_len = 0;
          }
          if (s_rx_len < 2) return;
     }
     else
     {
          s_rx_buf[s_rx_len++] = byte;
     }
     if (s_rx_len >= sizeof(Gbc2GcuPkt_t))
     {
          s_rx_len = 0;
          gimbal_on_packet(s_rx_buf);
     }
}

static uint8_t gimbal_link_alive(void)
{
     return (uint8_t)((get_tick_ms() - s_rx_tick_ms) < 200);
}

/* ---------------- 解析打印 ---------------- */
void gimbal_print_status(void)
{
     Gbc2GcuPkt_t *p = &s_response;
     printf("固件 V%u.%u 故障 0x%02X 状态 %u 模式 %u 安装 %u 温控 %u 应答 %u 命令 %u\r\n",
            (p->fw_ver >> 4) & 0x0F, p->fw_ver & 0x0F, p->hw_err,
            p->status.run_state, p->mode, p->status.mount, p->status.tca,
            p->cmd.stat, p->cmd.code);
     printf("姿态角 %7.2f %7.2f %7.2f 度\r\n",
            p->cam_angle[0] * 0.01f, p->cam_angle[1] * 0.01f, p->cam_angle[2] * 0.01f);
     printf("框架角 %7.2f %7.2f %7.2f 度\r\n",
            p->mtr_angle[0] * 0.01f, p->mtr_angle[1] * 0.01f, p->mtr_angle[2] * 0.01f);
     printf("角速度 %7.2f %7.2f %7.2f 度/秒\r\n",
            p->cam_rate[0] * 0.1f, p->cam_rate[1] * 0.1f, p->cam_rate[2] * 0.1f);
}

/* ---------------- 周期任务，每 20 ms 调用一次 ---------------- */
void gimbal_task_50hz(void)
{
     /* 示例：yaw 跟随，roll / pitch 锁定，pitch -15 度，yaw +20 度 */
     static const uint8_t wk[3] = { WK_LOCK, WK_LOCK, WK_FOLLOW };
     int16_t sp[3];

     sp[AXIS_ROLL]  = 0;
     sp[AXIS_PITCH] = ANGLE_TO_LSB(-15.0f);
     sp[AXIS_YAW]   = ANGLE_TO_LSB(20.0f);

      if (gimbal_link_alive())
           gimbal_send_manual(wk, sp, 50);
}
```
