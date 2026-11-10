# 两轴步进运动控制器

数控技术课程设计 · 基于 STM32F103 的两轴步进电机运动控制平台，含下位机固件、上位机软件与算法仿真。

支持点位运动、直线插补（两轴联动）、圆弧插补（G02/G03），带梯形加减速、软/硬限位、回零与通信超时保护。

## 硬件与参数

| 项 | 值 |
|---|---|
| 主控 | STM32F103（72 MHz） |
| 驱动器 | 步进驱动器，1600 细分 |
| 丝杠导程 | 5 mm |
| 脉冲当量 δ | 5 / 1600 = **0.003125 mm**（3.125 µm） |
| 脉冲发生 | TIM2 50 kHz 节拍（PSC=71, ARR=19），上限 25 kHz |
| 速度规划 | SysTick 1 ms |
| 最高速度 | 78.1 mm/s（1600 细分） |
| 通信 | USART1 115200 8N1，行文本协议 |

## 目录结构

```
.
├── cnc_project1/            下位机固件（STM32CubeMX + Keil MDK）
│   ├── Core/Src/stepper.c       脉冲发生 + 运动规划 + 圆弧插补
│   ├── Core/Src/protocol.c      串口协议解析与应答
│   ├── Core/Src/main.c          初始化与主循环
│   └── MDK-ARM/cnc_project1.uvprojx
├── host/                    上位机（Python + PySide6 + pyserial）
│   ├── ui_main.py               主界面
│   ├── serial_link.py           串口线程
│   ├── protocol.py              协议编解码
│   ├── arc.py / arc_view.py     圆弧几何与预览画布
│   ├── trace_plot.py            运动曲线
│   └── selftest.py              离线自测（60 项）
├── sim/                     算法仿真（与固件同构）
│   └── arc_sim.py               DDA vs 逐点比较法
└── docs/                    文档与周进度
    ├── 协议说明.md
    ├── 实现原理与代码详解.md
    ├── 源码全解_逐函数手册.md
    ├── 圆弧插补仿真验证.md
    └── 周进度/
```

## 快速开始

**下位机**：Keil MDK 打开 `cnc_project1/MDK-ARM/cnc_project1.uvprojx`，编译后下载。命令行方式：

```bat
UV4.exe -b cnc_project1\MDK-ARM\cnc_project1.uvprojx -j0 -o build.log   :: 编译
UV4.exe -f cnc_project1\MDK-ARM\cnc_project1.uvprojx -j0 -o flash.log   :: 烧录
```

**上位机**：

```bat
cd host
pip install PySide6-Essentials pyserial
python app.py
```

离线自测（不需连板子）：`python host/selftest.py`（60 项应全部通过）

**算法仿真**：

```bat
python sim/arc_sim.py                              :: 默认 R=10 整圆 F=10
python sim/arc_sim.py --r 25 --feed 15 --sweep 180 :: 自定义
```

## 指令一览

| 指令 | 说明 |
|---|---|
| `PING` / `HELP` | 心跳 / 指令表 |
| `MOV X10 F20` | 单轴定长运动 |
| `MOVA X10 F20` | 单轴绝对运动 |
| `JOG X+ F20` / `JOG X-` | 连续点动 / 停止 |
| `LINE X10 Y10 F20` | 两轴直线插补 |
| `G03 X0 Y0 I-8 J0 F8` | 圆弧插补（G03 逆时针 / G02 顺时针） |
| `STOP` / `ESTOP` | 减速停 / 急停 |
| `HOME X` | 回零 |
| `RESET` | 清除报警 |

应答：`OK`、`ERR,<码>`；状态帧 `<ST X=..,..,RUN,H1 Y=.. LIM=0000 ALM=0>` 每 100 ms 一条。
完整定义见 [docs/协议说明.md](docs/协议说明.md)。

## 文档索引

| 文档 | 定位 |
|---|---|
| [协议说明](docs/协议说明.md) | 指令格式、应答、错误码、圆弧语义 |
| [实现原理与代码详解](docs/实现原理与代码详解.md) | 13 章，讲"为什么这么设计" |
| [源码全解_逐函数手册](docs/源码全解_逐函数手册.md) | 逐函数逐行，讲"每一行在干什么" |
| [圆弧插补仿真验证](docs/圆弧插补仿真验证.md) | DDA vs 逐点比较法、实测数据 |
| [周进度](docs/周进度/) | 每周成果记录 |

## 已知限制

- 光电限位开关极性尚待实测确认，`main.c` 中 `LIMITS_ENABLE` 目前为 0（不影响运动功能）
- 圆弧插补为 DDA 方案，逐点比较法目前只有仿真对照，未落地固件
