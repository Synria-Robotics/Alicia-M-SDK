# Alicia-M SDK

[English Version](README_EN.md) | [中文版](README.md)

**Alicia-M SDK** 是一个用于控制【Alicia-M】系列机械臂（带夹爪）的 Python 工具包。它基于 `RoboCore` 库构建，提供通过串口通信控制机械臂运动、操作夹爪、读取姿态与状态数据等功能。

## RoboCore: Unified High-Throughput Robotics Library 

本 SDK 由 [Synria Robotics Co., Ltd.](https://synriarobotics.ai) 开发的 [RoboCore (Unified High-Throughput Robotics Library)](https://github.com/Synria-Robotics/RoboCore) 支持。

[![License](https://img.shields.io/badge/License-GPL--3.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)

---

## ✨ 核心功能

| 模块 | 功能 | 状态 |
|---|---|---|
| **建模** | URDF/MJCF 解析, 机器人模型抽象 | ✅ Stable |
| **正向运动学** | 支持 NumPy/PyTorch 后端, 批处理 | ✅ Stable |
| **逆向运动学** | 支持 DLS/Pinv/Transpose 多种求解器, 多起点求解 | ✅ Stable |
| **雅可比矩阵** | 支持解析法/数值法/自动微分法 | ✅ Stable |
| **坐标变换** | SE(3)/SO(3) 刚体变换, 多种格式转换 | ✅ Stable |
| **运动学分析** | 工作空间/奇异点分析 | ✅ Beta |
| **轨迹规划** | 轨迹生成 | 🚧 Alpha |
| **可视化** | 运动学链可视化 | ✅ Stable |
| **配置管理** | 基于 YAML 的配置管理 | ✅ Stable |

## 主要特性

*   **关节控制**：支持设置与读取六个关节的角度，提供平滑插值执行。
*   **末端轨迹**：基于 Cartesian 末端姿态轨迹规划与执行。
*   **夹爪控制**：支持精确角度控制或一键开关。
*   **力矩控制**：开启或关闭关节电机扭矩，实现自由拖动（示教）。
*   **零点设置**：将当前位置设置为新的零点。
*   **状态读取**：实时获取关节角、夹爪角与末端姿态。
*   **自动串口连接**：自动搜索串口或手动指定。
*   **拖动示教**：拖动记录姿态点并执行轨迹。
*   **SparkVis 集成**：支持 WebSocket UI 双向同步与数据记录。
*   **智能日志系统**：支持日志级别过滤，可控制控制台输出详细程度。
*   **RoboCore 集成**：集成高性能运动学和轨迹规划库。

## 项目结构

```
├── alicia_m_sdk
│   ├── api
│   │   ├── synria_robot_api.py      # 用户层 API
│   │   └── firmware_version.json    # 固件版本/元信息
│   ├── execution
│   │   ├── drag_teaching.py         # 拖动示教
│   │   ├── hardware_executor.py     # 硬件执行器
│   │   ├── sparkvis.py              # SparkVis WebSocket 桥接
│   │   └── trajectory_executor.py   # 轨迹执行器
│   ├── hardware
│   │   ├── comm_manager.py          # 串口管理/重连
│   │   ├── serial_comm.py           # 串口通信
│   │   ├── data_parser.py           # 数据解析
│   │   └── servo_driver.py          # 舵机驱动
│   ├── __init__.py
│   └── utils
│       ├── calculate.py             # 计算工具
│       ├── control_utils.py         # 控制工具
│       ├── fps_utils.py             # FPS 统计
│       ├── trajectory_utils.py      # 轨迹工具
│       ├── unit_conversion.py       # 单位转换
│       └── logger/                  # 日志系统
├── docs
│   ├── api_reference.md             # API 参考
│   ├── examples.md                  # 示例说明
│   └── ...                          # 其他文档
├── examples
│   ├── 00_demo_read_version.py      # 读取固件版本
│   ├── 01_demo_switch_mode.py       # 切换控制模式
│   ├── 02_demo_monitor_arm_status_simple.py  # 状态监控
│   ├── 03_demo_read_state.py        # 读取状态
│   ├── 04_demo_move_gripper.py      # 夹爪控制
│   ├── 05_demo_move_joint.py        # 关节运动
│   ├── 07_demo_forward_kinematics.py   # 正向运动学
│   ├── 08_demo_inverse_kinematics.py   # 逆向运动学
│   ├── 09_demo_joint_traj.py        # 关节轨迹
│   ├── 10_demo_cartesian_traj.py    # 笛卡尔轨迹
│   ├── 11_demo_slider_control_joint.py  # 滑块控制
│   └── 12_demo_sparkvis.py          # SparkVis UI 双向同步
```

## 快速开始

1.  **安装**：
```bash
# 克隆仓库
git clone https://github.com/Synria-Robotics/Alicia-M-SDK.git
cd Alicia-M-SDK

# 安装依赖（推荐使用 Python 3.8+）
pip install -e .
```

2.  **运行示例**：
```bash
cd examples
python3 00_demo_read_version.py    # 读取固件版本
python3 03_demo_read_state.py      # 读取状态
python3 04_demo_move_gripper.py    # 夹爪控制
python3 05_demo_move_joint.py      # 关节移动
```

3.  **SparkVis 可视化**（可选）：
```bash
# 启动 SparkVis 桥接进行 UI 双向同步
python3 examples/12_demo_sparkvis.py
```

## 文档

**中文文档：**
*   [API 参考](docs/api_reference.md)
*   [示例说明](docs/examples.md)
*   [MIT 模式使用](docs/mit_mode_usage.md)
*   [状态位解析](docs/status_bit_parsing.md)
*   [高频同步通信](docs/HIGH_FREQUENCY_SYNC_COMMUNICATION.md)

## 示例程序

仓库 `examples/` 目录包含多个演示脚本：

| 示例 | 说明 |
|------|------|
| `00_demo_read_version.py` | 读取固件版本信息 |
| `01_demo_switch_mode.py` | 切换控制模式 |
| `02_demo_monitor_arm_status_simple.py` | 简单状态监控 |
| `03_demo_read_state.py` | 读取关节和末端状态 |
| `04_demo_move_gripper.py` | 夹爪控制示例 |
| `05_demo_move_joint.py` | 关节位置控制 |
| `07_demo_forward_kinematics.py` | 正向运动学计算 |
| `08_demo_inverse_kinematics.py` | 逆向运动学求解 |
| `09_demo_joint_traj.py` | 关节空间轨迹执行 |
| `10_demo_cartesian_traj.py` | 笛卡尔空间轨迹执行 |
| `11_demo_slider_control_joint.py` | 使用滑块控制关节 |
| `12_demo_sparkvis.py` | SparkVis UI 双向同步 |

详细说明请参见 [示例说明文档](docs/examples.md)。

## 许可证

本项目采用 [GPL-3.0 许可证](LICENSE)。

## 联系方式

*   **开发团队**: [Synria Robotics Co., Ltd.](https://synriarobotics.ai)
*   **技术支持**: support@synriarobotics.ai

---

**由 Synria Robotics 用心打造** 🤖




