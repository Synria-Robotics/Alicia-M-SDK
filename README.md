# Alicia-M-SDK

[English Version](README_EN.md) | [中文版](README.md)

<p align="center"><img src="./imgs/Alicia_M.jpg" width="500" /></p>


**Alicia-M SDK** 是用于控制 Alicia-M 系列机械臂的 Python 工具包（含通信驱动、执行层、示例与文档）。

本仓库以实用示例和轻量级控制接口为主，方便在嵌入式/桌面环境里通过串口与板级控制器交互并执行轨迹或示教任务。

[![License](https://img.shields.io/badge/License-GPL--3.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)

--- 

## ✨ 核心功能

| 模块 | 功能 | 状态 |
|---|---|---|
| **通信层** | 串口自动连接、消息收发、帧解析 | ✅ Stable |
| **电机驱动** | 封装电机数据打包与发送 | ✅ Stable |
| **执行层** | 轨迹执行、拖动示教、SparkVis 桥接 | ✅ Stable |
| **工具函数** | 轨迹/角度/单位转换、插值、FPS 统计 | ✅ Stable |
| **示例** | 多个硬件交互与轨迹示例 | ✅ Stable |


## 主要特性

* 关节与笛卡尔级别的运动控制接口（基于 alicia_m_sdk.execution）
* 自动化串口管理与数据解析（`alicia_m_sdk.hardware`）
* 拖动示教与轨迹执行工具（`drag_teaching.py`、`trajectory_executor.py`）
* SparkVis 双向同步桥接（`sparkvis.py`），用于可视化与远程 UI 控制
* 实用工具集（坐标/单位转换、插值、FPS 统计）和友好的日志输出


## 项目结构（概要）

```
├── alicia_m_sdk
│   ├── api
│   │   ├── synria_robot_api.py      # 用户层 API（高层封装）
│   │   └── firmware_version.json    # 固件版本/元信息
│   ├── execution
│   │   ├── drag_teaching.py         # 拖动示教逻辑
│   │   ├── hardware_executor.py     # 硬件执行器（将 motion -> servo 命令）
│   │   ├── sparkvis.py              # SparkVis WebSocket 桥接
│   │   └── trajectory_executor.py   # 轨迹执行器
│   ├── hardware
│   │   ├── comm_manager.py          # 串口发现/管理/重连逻辑
│   │   ├── serial_comm.py           # 串口封装读写
│   │   ├── data_parser.py           # 原始帧到状态/命令的解析
│   │   └── servo_driver.py          # 电机层封包与发送
│   └── utils
│       ├── calculate.py             # 控制计算函数
│       ├── control_utils.py         # 辅助控制工具
│       ├── fps_utils.py             # 帧率统计
│       ├── trajectory_utils.py      # 轨迹生成/插值
│       └── unit_conversion.py       # 单位换算
├── docs                             # 文档（API 说明、示例、安装等）
├── examples                         # 示例脚本：演示读固件、移动、示教等
└── imgs                             # 图片/示意图
```


## 快速开始

1. 安装（推荐在 Python 3.8+ 环境）

```bash
# 在仓库根目录（可选创建 venv）
python3 -m pip install -r requirements.txt || true  # 如果存在 requirements
python3 -m pip install .                          # 可将包安装到当前环境
```

2. 自动串口连接与读取固件版本（示例）：

```bash
cd examples
python3 00_demo_read_version.py
```

3. 控制关节 / 运行轨迹示例：

```bash
python3 05_demo_move_joint.py      # 控制关节运动（查阅脚本内参数）
python3 06_demo_move_cartesian.py  # 笛卡尔运动示例
python3 09_demo_joint_traj.py      # 关节轨迹示例
```

4. SparkVis 可视化桥接（若需 UI）：

```bash
# 启动本地 SparkVis 后端（参考 SparkVis 项目）
# 在本机运行示例桥接
python3 examples/12_demo_sparkvis.py --port /dev/ttyUSB0
```


## 文档

主要文档位于 `docs/`：

* `docs/api_reference.md` — API 参考（关注 `alicia_m_sdk.api` 中暴露的类/方法）
* `docs/examples.md` — 示例说明（每个 examples 脚本的用途和参数）
* `docs/installation.md` — 环境与依赖说明（若存在）


## Examples (demos)

仓库 `examples/` 目录包含若干演示脚本，下面是每个 demo 的简要说明：

- `00_demo_read_version.py` — 连接设备并读取/显示固件版本信息（连接测试）。
- `01_demo_config_mit_params.py` — 配置 MIT / 低层控制参数（用于调试或切换控制行为）。
- `02_demo_monitor_arm_status_simple.py` — 简单的循环状态监控示例（打印关节/状态）。
- `03_demo_read_state.py` — 读取并显示当前关节角、夹爪与末端状态。
- `04_demo_move_gripper.py` — 夹爪开闭控制示例（角度或一键开关）。
- `05_demo_move_joint.py` — 发送关节位置命令，演示单步或多步关节运动。
- `06_demo_move_cartesian.py` — 笛卡尔空间下的末端运动示例（位姿控制）。
- `07_demo_forward_kinematics.py` — 使用 SDK 计算并展示正向运动学结果。
- `08_demo_inverse_kinematics.py` — 逆运动学求解示例（多种求解器或起点策略）。
- `09_demo_joint_traj.py` — 关节空间轨迹执行（插值与时间参数示例）。
- `10_demo_cartesian_traj.py` — 笛卡尔空间轨迹执行示例（跟踪末端轨迹）。
- `11_demo_slider_control_joint.py` — 使用滑块/简单 UI 控制关节实时命令的示例。
- `12_demo_sparkvis.py` — SparkVis 桥接示例：WebSocket UI ↔ 机器人双向同步与数据记录。
- `13_demo_switch_mode.py` — 切换控制模式示例（如位置/力矩/示教模式切换）。
- `14_benchmark_read_joints.py` — 读取关节状态的吞吐/延迟基准测试脚本。
- `15_benchmark_control_frequency.py` — 控制指令发送频率与时序基准测试。
- `16_demo_control_aim_usage.py` — 使用 Control Aim / 高层控制接口的示例用法。
- `17_demo_robot_return_to_zero_test.py` — 将机械臂回到零位 / 安全位置的示例脚本。
- `18_demo_fpv_move_test.py` — 高速或 FPV（第一视角）移动测试示例（用于视觉/运动配合测试）。

说明：上面描述基于脚本名及仓库中实现的功能推断而成，若需对任一 demo 做更精确的参数说明或示例运行命令，我可以逐个打开并摘录脚本开头的用法注释/参数说明并写入 README 或 `docs/examples.md`。



## 常见文件说明（快速索引）

- `alicia_m_sdk/hardware/serial_comm.py`：串口低层读写封装。
- `alicia_m_sdk/hardware/comm_manager.py`：串口自动发现与重连策略。
- `alicia_m_sdk/hardware/data_parser.py`：把原始字节帧解析为状态结构或命令回应。
- `alicia_m_sdk/hardware/servo_driver.py`：把高层角度/速度/扭矩封装为电机协议帧并发送。
- `alicia_m_sdk/execution/trajectory_executor.py`：轨迹插值与执行时序。
- `alicia_m_sdk/execution/drag_teaching.py`：拖动示教逻辑（把示教点记录为轨迹）。
- `alicia_m_sdk/execution/sparkvis.py`：提供 WebSocket 桥接用于 UI 双向同步（查看示例 12）。




