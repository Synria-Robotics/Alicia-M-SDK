# Alicia-M-SDK

Synria 云擎（Alicia-M）系列 6-DOF 机械臂 Python SDK。

基于 [RoboCore](https://github.com/Synria-Robotics/RoboCore) 构建，通过串口通信协议提供对机械臂的完整控制能力。

## 核心功能

- **关节控制**：6 关节位置/速度控制，支持平滑插值运动
- **夹爪操作**：开合度控制（0~1000），支持独立或协同控制
- **力矩管理**：MIT 模式下的力矩卸载/恢复，支持自由拖动示教
- **状态监测**：实时读取关节角度、速度、力矩、线圈温度等
- **运动学**：正/逆运动学求解（基于 RoboCore）
- **轨迹规划**：关节空间与笛卡尔空间轨迹规划与执行
- **可视化**：WebSocket 实时推送关节状态至 SparkVis

## 主要特性

| 特性 | 说明 |
|------|------|
| **双控制模式** | PV（位置-速度，固件插值）/ MIT（阻抗控制，全参数逐帧） |
| **统一 API** | `set_robot_state()` 自动适配当前模式，屏蔽底层差异 |
| **安全序列** | 使能/模式切换自动发送位置锁定帧，防止关节突跳 |
| **模式同步** | 连接时自动检测固件控制模式，按需切换到目标模式 |
| **异步轮询** | 200Hz 后台状态轮询，缓存读取微秒级返回 |
| **自动发现** | 串口自动扫描、设备类型自动检测（示教臂/操作臂） |

## 项目结构

```
alicia_m_sdk/
├── api/            # 用户 API 层（SynriaRobotAPI 门面类）
├── protocol/       # 协议层（帧结构、编解码、常量定义）
├── hardware/       # 硬件层（串口驱动、设备抽象、状态缓存）
├── control/        # 控制层（关节控制、轨迹执行、示教）
├── types/          # 类型定义（状态、配置、枚举、异常）
├── utils/          # 工具层（单位转换、参数校验）
├── kinematics.py   # 运动学接口（RoboCore 封装）
└── planning.py     # 规划接口（RoboCore 封装）
```

## 快速开始

### 安装

```bash
git clone https://github.com/Synria-Robotics/Alicia-M-SDK.git
cd Alicia-M-SDK
conda create -n msdk python=3.10
conda activate msdk
pip install -e .
```

### 基本使用

```python
import alicia_m_sdk

# 创建并自动连接（自动检测控制模式，串口自动发现）
robot = alicia_m_sdk.create_robot()

# 指定控制模式（若固件当前模式不匹配则自动切换）
robot = alicia_m_sdk.create_robot(control_mode="pv")

# 关节运动（度）
robot.set_robot_state(target_joints=[90, -90, -90, 90, 0, 0], speed=15)

# 夹爪控制
robot.set_robot_state(gripper_value=1000)  # 打开
robot.set_robot_state(gripper_value=0)     # 关闭

# 关节 + 夹爪同时控制
robot.set_robot_state(target_joints=[0, 0, 0, 0, 0, 0], gripper_value=500, speed=15)

# 读取状态
state = robot.get_robot_state("all")
print(state.angles)        # 关节角度 (rad)
print(state.gripper)       # 夹爪值 (0~1000)
print(state.temperatures)  # 线圈温度 (°C)

robot.disconnect()
```

## 示例程序

| 编号 | 文件 | 说明 |
|------|------|------|
| 00 | `demo_read_version.py` | 读取固件版本与设备信息 |
| 01 | `demo_diagnostic.py` | 通信诊断 |
| 02 | `demo_read_status.py` | 读取控制模式与运行状态 |
| 03 | `demo_read_states.py` | 循环读取关节角度、速度、力矩、温度 |
| 04 | `demo_switch_mode.py` | PV / MIT 模式切换 |
| 05 | `demo_disable_enable.py` | 使能 / 失能 |
| 06 | `demo_move_gripper.py` | 夹爪控制（PV） |
| 07 | `demo_move_joint.py` | 关节运动（PV） |
| 08 | `demo_move_full_arm.py` | 关节 + 夹爪协同（PV） |
| 09 | `demo_move_gripper_mit.py` | 夹爪控制（MIT） |
| 10 | `demo_move_joint_mit.py` | 关节运动（MIT） |
| 11 | `demo_move_full_arm_mit.py` | 关节 + 夹爪协同（MIT） |
| 12 | `demo_forward_kinematics.py` | 正运动学计算 |
| 13 | `demo_inverse_kinematics.py` | 逆运动学求解 + 可选执行 |
| 14 | `demo_drag_teaching.py` | 拖动示教录制与 PV 回放 |
| 15 | `demo_sparkvis.py` | SparkVis WebSocket 可视化 |
| 16 | `demo_reset_zero.py` | 零位标定 |

运行示例：

```bash
conda activate msdk
python examples/07_demo_move_joint.py --speed 15
```

## License

MIT License - Synria Robotics
