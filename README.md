# Alicia-M-SDK

Synria 云擎（Alicia-M）系列 6-DOF 机械臂 Python SDK。

基于 [RoboCore](https://github.com/Synria-Robotics/RoboCore) 构建，通过串口通信协议提供对机械臂的完整控制能力。

English summary: Alicia-M-SDK is a Python SDK for Alicia-M robotic arms. The recommended public entry point is `create_robot()` / `SynriaRobotAPI`; advanced users can also import `JointController`, `TrajectoryExecutor`, and `Teleoperation`.

## 核心功能

- **关节控制**：6 关节位置/速度控制，支持平滑插值运动
- **夹爪操作**：开合度控制（0~1000），支持独立或协同控制
- **力矩管理**：MIT 模式下的力矩卸载/恢复，支持自由拖动示教
- **遥操作**：Alicia-D 示教臂实时控制 Alicia-M 操作臂（PV / MIT 双模式）
- **状态监测**：实时读取关节角度、速度、力矩、插补速度、线圈温度（含夹爪电机）
- **运动学**：正/逆运动学求解（基于 RoboCore）
- **轨迹规划**：关节空间与笛卡尔空间轨迹规划与执行
- **可视化**：WebSocket 实时推送关节状态至 SparkVis
- **运行日志**：默认输出终端日志并自动保存到 `./logs/alicia_m_sdk_YYYYMMDD_HHMMSS.log`

## 主要特性

| 特性 | 说明 |
|------|------|
| **双控制模式** | PV（位置-速度，固件插值）/ MIT（阻抗控制，支持直接 PD / 线性轨迹插值） |
| **统一 API** | `set_robot_state()` 自动适配当前模式，屏蔽底层差异 |
| **高级入口** | 顶层导出 `JointController`、`TrajectoryExecutor`、`Teleoperation`，供高级用户扩展 |
| **安全切换** | 模式切换采用失能→切换→使能流程，固件侧处理目标位置初始化 |
| **模式同步** | 连接时自动检测固件控制模式，按需切换到目标模式 |
| **异步轮询** | 200Hz 后台状态轮询，缓存读取微秒级返回 |
| **自动发现** | 串口自动扫描、设备类型自动检测（示教臂/操作臂） |

## 项目结构

```
alicia_m_sdk/
├── api/            # 用户 API 层（SynriaRobotAPI 门面类）
├── hardware/       # 硬件层（串口驱动、设备抽象、状态缓存）
├── execution/      # 高级执行层（关节控制、轨迹执行、遥操作）
├── integrations/   # 外部库适配层（RoboCore FK/IK/trajectory planning）
├── types/          # 类型定义（状态、配置、枚举、异常）
└── utils/          # 工具层（单位转换、参数校验）
```

## 快速开始

### 安装

获取代码：

```
git clone https://github.com/Synria-Robotics/Alicia-M-SDK.git
cd Alicia-M-SDK
```

创建虚拟环境：

```
conda create -n msdk python=3.10
conda activate msdk
```

**方法一（推荐）：从源码安装（开发模式）：**

```bash
pip install -e .
```

源码安装可随时修改代码并立即生效，适合开发和调试。

可选功能：

```bash
pip install -e ".[teleop]"  # Alicia-D -> Alicia-M 遥操作示例
pip install -e ".[torch]"   # RoboCore torch 后端
pip install -e ".[all]"     # 常用可选依赖
```

**方法二：从 PyPI 安装：**

```bash
pip install alicia_m_sdk
```

### 基本使用

```python
import alicia_m_sdk

with alicia_m_sdk.create_robot(control_mode="pv") as robot:
	# 关节运动（默认单位: deg）
	robot.set_robot_state(target_joints=[90, -90, -90, 90, 0, 0], speed=15)

	# 夹爪控制
	robot.set_robot_state(gripper_value=1000)  # 打开
	robot.set_robot_state(gripper_value=0)     # 关闭

	# 读取状态
	state = robot.get_robot_state("all")
	print(state.angles)        # 关节角度 (rad)
	print(state.gripper)       # 夹爪值 (0~1000)
	print(state.temperatures)  # 扩展轮询开启后可用
```

MIT 参数约定：`kp`/`kd` 传 `None` 表示按电机使用 SDK 安全默认值，显式传 `0` 表示真实零增益；标量会广播到 7 个电机，长度 6 的列表表示仅关节、夹爪使用默认值，长度 7 的列表表示逐电机设置。

夹爪类型可使用 `GripperType.MM_50` / `GripperType.MM_100`，也兼容固件值 `0/2`、示例选项 `10/40`、字符串 `"50mm"` / `"100mm"`。

## 示例程序

| 编号 | 文件 | 说明 |
|------|------|------|
| 00 | `00_demo_read_version.py` | 读取固件版本与设备信息 |
| 01 | `01_demo_diagnostic.py` | 自检 |
| 02 | `02_demo_read_status.py` | 读取控制模式与运行状态 |
| 03 | `03_demo_read_states.py` | 循环读取关节角度、速度、力矩、插补速度、温度（含夹爪） |
| 04 | `04_demo_switch_mode.py` | PV / MIT 模式切换 |
| 05 | `05_demo_disable_enable.py` | 使能 / 失能 |
| 06 | `06_demo_move_gripper.py` | 夹爪控制（MIT，逐电机阻抗参数） |
| 07 | `07_demo_move_joint.py` | 关节运动（PV） |
| 08 | `08_demo_move_full_arm.py` | 关节 + 夹爪协同（PV） |
| 09 | `09_demo_move_joint_mit.py` | 关节运动（MIT，逐电机阻抗参数） |
| 10 | `10_demo_move_full_arm_mit.py` | 关节 + 夹爪协同（MIT，逐电机阻抗参数） |
| 11 | `11_demo_forward_kinematics.py` | 正运动学计算 |
| 12 | `12_demo_inverse_kinematics.py` | 逆运动学求解 + 可选执行 |
| 13 | `13_demo_reset_zero.py` | 零位标定 |
| 14 | `14_demo_teleop_mapped.py` | 遥操作 + URDF 限位映射（逐电机阻抗参数） |
| 15 | `15_demo_gripper_params.py` | 0x17 夹爪夹持参数读写 |
| 16 | `16_demo_joint_traj.py` | 关节空间轨迹规划与执行 |
| 17 | `17_demo_mit_torque_switch.py` | MIT 力矩开关测试 |
| 18 | `18_demo_user_settings.py` | 0x02 个性化设置 |

运行示例：

```bash
conda activate msdk
python examples/07_demo_move_joint.py --speed 15
```

### 15 夹爪夹持参数读写

`15_demo_gripper_params.py` 使用公开协议 `0x17` 读取或修改夹爪夹持参数。默认目标为操作臂 `follower`，默认动作是读取全部参数，不会写入。

```bash
# 读取操作臂全部夹爪参数
python examples/15_demo_gripper_params.py --port COM57

# 仅读取指定掩码，例如 0x09 = 目标夹持力 + 最大保持力矩
python examples/15_demo_gripper_params.py --port COM57 --mask 0x09

# 设置目标夹持力为 80N，最大保持力矩为 70N·m
python examples/15_demo_gripper_params.py --port COM57 --target-force 80 --hold-torque 70
```

写入前程序会打印即将写入的参数并等待确认；确认机械臂安全后按 Enter 发送。若需要自动执行，可加 `-y` 跳过确认；默认写入后会读回全部参数，可加 `--skip-readback` 跳过读回。

## License

MIT License - Synria Robotics
