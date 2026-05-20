# Alicia-M-SDK

Alicia-M-SDK 是 Synria Alicia-M 6 自由度机械臂的 Python 控制 SDK。

SDK 将机械臂底层串口协议封装成清晰的 Python 接口。客户无需处理协议帧、寄存器地址、CRC 校验、状态轮询线程等底层细节，只需要通过 `create_robot()` 创建机器人对象，即可完成设备连接、状态读取、关节运动、夹爪控制、模式切换、运动学计算、轨迹规划和维护配置。

最常用入口：

```python
import alicia_m_sdk

robot = alicia_m_sdk.create_robot()
```

## 主要能力

| 能力 | 客户可以完成的事情 |
| --- | --- |
| 设备连接 | 指定串口连接，或让 SDK 自动扫描串口 |
| 状态读取 | 获取关节角、夹爪值、速度、力矩、版本信息和运行状态 |
| 运动控制 | 控制 6 个关节到目标角度，也可以同时控制夹爪 |
| 夹爪控制 | 打开、关闭夹爪，或设置 `0~1000` 范围内的夹爪目标值 |
| PV / MIT 模式 | PV 用于常规点位运动，MIT 用于阻抗控制、拖动示教和遥操作 |
| 运动学与轨迹 | 基于 RoboCore 做 FK、IK、关节轨迹和笛卡尔轨迹规划 |
| 设备维护 | 使能、失能、自检、零位标定、用户设置和夹爪参数读写 |
| 遥操作 | 提供 Alicia-D 到 Alicia-M 的主从映射控制示例 |

## 项目结构

客户通常只需要使用 `alicia_m_sdk.create_robot()` 返回的机器人对象。底层通信、状态轮询和协议解析由 SDK 内部完成。

```text
alicia_m_sdk/
|-- api/            # 面向客户的统一 API 门面，业务文件只有 synria_robot_api.py
|-- _internal/      # 内部辅助实现，例如连接握手
|-- execution/      # 内部关节控制、轨迹执行、遥操作
|-- hardware/       # 内部串口连接、协议帧、设备状态轮询
|-- integrations/   # 内部/高级 RoboCore 运动学和轨迹规划适配
|-- types/          # 可公开的配置、状态、枚举、异常
`-- utils/          # 内部工具和 demo 辅助
```

分层关系可以理解为：

```text
demo代码
  |
  v
SynriaRobotAPI 统一入口
  |
  +-- _internal: 连接握手和内部协调
  +-- execution: 运动控制、夹爪控制、轨迹执行、遥操作
  +-- integrations: RoboCore 运动学和轨迹规划
  +-- hardware: 串口通信、协议编解码、状态轮询
```

这种结构的目标是让客户代码保持简单，复杂的通信和控制细节留在 SDK 内部。

## Demo 导览

示例脚本位于 `examples/` 目录。第一次接入设备时，建议按下面顺序运行：

1. 先运行 `00_demo_read_version.py`，确认 SDK 能连接设备并读取版本。
2. 再运行 `02_demo_read_status.py` 或 `03_demo_read_states.py`，确认状态读取正常。
3. 最后运行 `06_demo_move_gripper.py`、`07_demo_move_joint.py` 等运动示例。

常用运行方式：

```bash
python examples/00_demo_read_version.py --port COM37
python examples/03_demo_read_states.py --port COM37
python examples/07_demo_move_joint.py --port COM37 --speed 15
```

`15_demo_gripper_params.py` 写入 0x17 夹爪力矩和力控参数时，会按协议范围做 SDK 侧校验；默认 `--gripper-type auto` 会尝试读取当前夹爪类型，并按小夹爪或大夹爪的精确范围限制。

完整示例列表：

| 分类 | 文件 | 说明 |
| --- | --- | --- |
| 连接与状态 | `00_demo_read_version.py` | 读取固件版本和设备信息，适合首次连机验证 |
| 连接与状态 | `01_demo_diagnostic.py` | 运行设备自检 |
| 连接与状态 | `02_demo_read_status.py` | 读取控制模式和运行状态 |
| 连接与状态 | `03_demo_read_states.py` | 循环读取关节角、速度、力矩等状态 |
| 基础控制 | `04_demo_switch_mode.py` | 切换 PV / MIT 控制模式 |
| 基础控制 | `05_demo_disable_enable.py` | 失能 / 使能机械臂 |
| 基础控制 | `06_demo_move_gripper.py` | 控制夹爪打开、关闭或移动到指定位置 |
| 基础控制 | `07_demo_move_joint.py` | PV 模式关节运动 |
| 基础控制 | `08_demo_move_full_arm.py` | PV 模式关节和夹爪协同运动 |
| 基础控制 | `09_demo_move_joint_mit.py` | MIT 模式关节运动 |
| 基础控制 | `10_demo_move_full_arm_mit.py` | MIT 模式关节和夹爪协同运动 |
| 运动学 | `11_demo_forward_kinematics.py` | 正运动学计算 |
| 运动学 | `12_demo_inverse_kinematics.py` | 逆运动学计算 |
| 维护设置 | `13_demo_reset_zero.py` | 零位标定 |
| 遥操作 | `14_demo_teleop_mapped.py` | Alicia-D 到 Alicia-M 遥操作 |
| 维护设置 | `15_demo_gripper_params.py` | 夹爪夹持参数读写 |
| 轨迹规划 | `16_demo_joint_traj.py` | 关节空间轨迹规划与执行 |
| 维护设置 | `17_demo_mit_torque_switch.py` | MIT 力矩开关测试 |
| 维护设置 | `18_demo_user_settings.py` | 用户设置读写 |

## 安装

要求 Python `>=3.11`。

从 PyPI 安装：

```bash
pip install alicia-m-sdk
```

安装预发布版本：

```bash
pip install --pre alicia-m-sdk
```

从源码安装：

```bash
git clone https://github.com/Synria-Robotics/Alicia-M-SDK.git
cd Alicia-M-SDK

conda create -n msdk python=3.11
conda activate msdk

pip install -e .
```

可选依赖：

```bash
pip install -e ".[planning]"       # scipy 轨迹规划
pip install -e ".[visualization]"  # matplotlib 轨迹绘图
pip install -e ".[sparkvis]"       # WebSocket / SparkVis
pip install -e ".[teleop]"         # Alicia-D -> Alicia-M 遥操作
pip install -e ".[torch]"          # RoboCore torch 后端
pip install -e ".[all]"            # 常用可选依赖
```

## 串口说明

连接机械臂后，需要把串口名传给 `create_robot(port=...)`。不同系统的串口名称不一样：

| 系统 | 常见串口名 | 示例 |
| --- | --- | --- |
| Windows | `COM1`、`COM37` | `port="COM37"` |
| Linux | `/dev/ttyUSB0`、`/dev/ttyACM0` | `port="/dev/ttyUSB0"` |

Windows 可以在“设备管理器”的“端口 (COM 和 LPT)”中查看串口号。

Linux 可以用下面命令查看串口设备：

```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
```

如果不确定串口名，也可以不传 `port`，SDK 会尝试自动扫描：

```python
robot = alicia_m_sdk.create_robot()
```

## 快速开始

### 1. 连接设备并读取版本

这个示例用于确认电脑、串口、机械臂和 SDK 都已正常工作。

```python
import alicia_m_sdk

# Windows 示例：port="COM37"
# Linux 示例：port="/dev/ttyUSB0"
with alicia_m_sdk.create_robot(port="COM37", sync_control_mode=False) as robot:
    version = robot.get_robot_state("version")
    print(version)
```

如果不确定串口名，可以不传 `port`，SDK 会自动扫描：

```python
with alicia_m_sdk.create_robot() as robot:
    state = robot.get_robot_state("joint_gripper")
    print(state)
```

### 2. 读取机器人状态

状态读取用于确认当前关节角度、夹爪位置、速度、力矩、运行状态等信息。运动前后都建议读取状态，方便确认设备是否处于预期位置。

```python
state = robot.get_robot_state("all")

print(state.angles)   # 6 个关节角，单位 rad
print(state.gripper)  # 夹爪值，范围 0~1000
```

常用状态类型：

| 参数 | 返回内容 |
| --- | --- |
| `"joint"` | 6 个关节角，单位 rad |
| `"joint_gripper"` | 关节角和夹爪值 |
| `"velocity"` | 关节速度 |
| `"torque"` | 关节力矩 |
| `"all"` | 完整 `JointState` 对象 |
| `"version"` | 固件版本和设备信息 |
| `"status"` | 运行状态 |
| `"control_mode"` | 各电机控制模式 |

### 3. 控制关节运动

关节运动是最常用的控制方式。下面示例使用 PV 模式，让机械臂移动到指定关节角度。`joint_format="deg"` 表示输入单位是角度。

```python
import alicia_m_sdk

with alicia_m_sdk.create_robot(port="COM37", control_mode="pv") as robot:
    robot.set_robot_state(
        target_joints=[0, 30, 0, 0, -30, 0],
        joint_format="deg",
        speed=15,
    )
```

### 4. 同时控制关节和夹爪

如果任务需要机械臂移动到目标姿态并调整夹爪，可以在同一个接口中同时传入关节目标和夹爪目标。

```python
with alicia_m_sdk.create_robot(port="COM37", control_mode="pv") as robot:
    robot.set_robot_state(
        target_joints=[0, 30, 0, 0, -30, 0],
        gripper_value=500,
        joint_format="deg",
        speed=15,
        gripper_speed=40,
    )
```

`gripper_value` 的范围是 `0~1000`。

### 5. 单独控制夹爪

只需要打开、关闭或调整夹爪位置时，可以直接使用夹爪接口。

```python
robot.set_gripper_target("open")
robot.set_gripper_target("close")
robot.set_gripper_target(value=500)
```

## 控制模式说明

Alicia-M 支持 PV 和 MIT 两种主要控制模式。普通点位运动优先使用 PV 模式；需要阻抗控制、拖动示教或遥操作时，再使用 MIT 模式。

| 模式 | 适用场景 | 说明 |
| --- | --- | --- |
| PV | 常规点位运动 | SDK 发送目标位置和速度，固件完成运动 |
| MIT | 阻抗控制、拖动示教、遥操作 | SDK 可设置位置、速度、力矩、Kp、Kd 等参数 |

PV 示例：

```python
with alicia_m_sdk.create_robot(port="COM37", control_mode="pv") as robot:
    robot.set_robot_state(target_joints=[0, 30, 0, 0, -30, 0])
```

MIT 示例：

```python
with alicia_m_sdk.create_robot(port="COM37", control_mode="mit") as robot:
    robot.set_robot_state(
        target_joints=[0, 20, 0, 0, -20, 0],
        joint_format="deg",
        kp=50,
        kd=1.0,
    )
```

## 系统控制

系统控制用于管理机械臂的连接状态、使能状态和控制模式。通常在设备初始化、调试、异常恢复或模式切换时使用。

```python
robot.connect()          # 手动连接；auto_connect=True 时通常不需要手动调用
robot.disconnect()       # 断开连接
robot.is_connected()     # 检查连接状态

robot.enable_robot()     # 使能机械臂
robot.disable_robot()    # 失能机械臂
robot.switch_mode("pv")  # 切换到 PV 模式
robot.switch_mode("mit") # 切换到 MIT 模式
```

## 运动学和轨迹

运动学和轨迹接口用于更高层的应用开发，例如根据末端位姿求关节角、生成关节轨迹，或执行规划好的运动路径。使用这些能力需要 RoboCore 和对应模型依赖可用。

```python
pose = robot.get_pose()                         # 读取当前末端位姿
robot.set_pose(target_pose, execute=False)      # 根据目标位姿求解，execute=True 时执行
robot.plan_joint_trajectory(waypoints)          # 根据关节路点规划轨迹
robot.move_joint_trajectory(q_end)              # 执行到目标关节位置的轨迹
```

基础点位运动不需要直接使用这些接口；只有在任务需要末端位姿、IK 或多段轨迹时再使用。

## 常用参数

### `create_robot()`

```python
robot = alicia_m_sdk.create_robot(
    port="COM37",
    control_mode="pv",
)
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `port` | `""` | 串口名；空字符串表示自动扫描 |
| `version` | `"auto"` | 机器人模型版本；默认从固件自动识别 |
| `control_mode` | `None` | `"pv"` / `"mit"`；不传则读取固件当前模式 |
| `control_aim` | `None` | `"leader"` / `"follower"`；不传则自动识别 |
| `sync_control_mode` | `True` | 连接时是否同步控制模式 |
| `backend` | `"cpp"` | RoboCore 后端：`"cpp"` / `"numpy"` / `"torch"` |
| `debug_mode` | `False` | 是否输出 DEBUG 日志 |
| `auto_connect` | `True` | 创建对象后是否自动连接 |
| `extended_polling` | `False` | 是否读取温度、Kp/Kd、插补速度等扩展状态 |

### `set_robot_state()`

| 参数 | 说明 |
| --- | --- |
| `target_joints` | 6 个关节目标角；可不传，只控制夹爪 |
| `gripper_value` | 夹爪目标值，范围 `0~1000` |
| `joint_format` | `"deg"` 或 `"rad"` |
| `speed` | 关节速度参数 |
| `gripper_speed` | 夹爪速度参数 |
| `wait_for_completion` | 是否等待运动完成 |
| `use_interpolation` | MIT 模式下是否使用固件线性插值 |
| `kp` / `kd` | MIT 模式阻抗增益；`None` 表示使用默认安全值 |
| `torque` / `vel_ref` | MIT 模式前馈力矩和目标速度 |

MIT 参数支持 `None`、标量、长度为 6 的关节列表、长度为 7 的电机列表。显式传入 `0` 表示真实零值，不会被默认值覆盖。

## 开发与验证

```bash
python -m pytest
python -m build
python -m twine check dist/*
```

Windows 清理缓存和构建产物：

```bat
clean_sdk.bat /dry-run
clean_sdk.bat
```

默认不会清理 `logs/` 或虚拟环境；如确需清理，可使用 `clean_sdk.bat /logs` 或 `clean_sdk.bat /venv`。

更多文档：

- `docs/API_MAINTENANCE.md`
- `docs/API_REFERENCE.md`
- `docs/ARCHITECTURE.md`
- `docs/PROTOCOL.md`

## License

MIT License - Synria Robotics
