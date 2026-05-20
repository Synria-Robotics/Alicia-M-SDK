# API 参考 / API Reference

推荐入口是 `import alicia_m_sdk`、`alicia_m_sdk.create_robot()` 和返回对象上的 `robot.xxx(...)` 方法。`SynriaRobotAPI` 是 `create_robot()` 返回的用户 API 门面。

Recommended usage is `import alicia_m_sdk`, `alicia_m_sdk.create_robot()`, and methods on the returned `robot` object. `SynriaRobotAPI` is the user-facing facade returned by `create_robot()`.

## 工厂函数

### `create_robot()`

```python
alicia_m_sdk.create_robot(
    port: str = "",              # 串口路径，空=自动发现
    version: str = "auto",       # 硬件版本；默认连接后自动识别
    variant: str = None,         # 变体（自动检测）
    control_aim: str = None,     # "leader"/"follower"（自动检测）
    control_mode: str = None,    # "pv" / "mit" / None=检测固件当前模式
    sync_control_mode: bool = True,  # 连接时是否同步固件控制模式
    baudrate: int = 1_000_000,   # 波特率
    backend: str = "cpp",        # RoboCore 后端: "numpy" / "torch" / "cpp"
    debug_mode: bool = False,    # DEBUG 日志
    auto_connect: bool = True,   # 自动连接
    extended_polling: bool = False,  # 扩展轮询，需新固件支持
) → SynriaRobotAPI
```

`backend="cpp"` 需要 `synria-robocore` 的 C++ 后端可用。需要 torch 后端时请安装 `alicia_m_sdk[torch]`。

---

## SynriaRobotAPI

### 连接管理

| 方法 | 说明 |
|------|------|
| `connect(timeout=5.0)` | 连接机器人 |
| `disconnect()` | 断开连接 |
| `is_connected()` | 检查连接状态 |

支持上下文管理器：

```python
with alicia_m_sdk.create_robot() as robot:
    robot.set_robot_state(target_joints=[0, 0, 0, 0, 0, 0])
```

### 状态查询

| 方法 | 说明 |
|------|------|
| `get_robot_state(info_type)` | 获取机器人状态 |
| `get_pose()` | 获取末端位姿（FK） |
| `get_firmware_version(timeout)` | 获取固件版本 |
| `print_state(continuous, output_format)` | 打印当前状态 |

**info_type 参数**：

| 值 | 返回内容 |
|----|---------|
| `"joint"` | 6 个关节角度 (rad) |
| `"joint_gripper"` | 关节角度 + 夹爪值 |
| `"velocity"` | 关节速度 (rad/s) |
| `"torque"` | 关节力矩 (N·m) |
| `"linear_vels"` | 插补速度 (rad/s)，需扩展轮询 |
| `"temperatures"` | 线圈温度 (°C)，需扩展轮询 |
| `"all"` | 完整 JointState 对象 |
| `"version"` | VersionInfo 对象 |
| `"status"` | RobotStatus 对象 |
| `"control_mode"` | 发送一次性查询，返回各电机控制模式 |

### 运动控制

| 方法 | 说明 |
|------|------|
| `set_robot_state(target_joints, gripper_value, speed=40, gripper_speed=100, ...)` | 点位运动（自动适配 PV/MIT） |
| `go_home(speed=40, gripper_speed=100)` | 回零位 |
| `set_gripper_target(command, value, wait_for_completion)` | 控制夹爪 |

**set_robot_state 参数**：

```python
robot.set_robot_state(
    target_joints=[0, 30, 0, 0, -30, 0],  # 目标角度 (deg)
    gripper_value=500,                      # 夹爪 [0, 1000]
    joint_format='deg',                     # 'deg' / 'rad'
    speed=40,                               # 速度 [0, 400]
    gripper_speed=100,                      # 夹爪速度 [0, 400]
    wait_for_completion=True,               # 是否等待到达
    use_interpolation=True,                  # MIT 模式是否使用固件线性插值
    kp=None, kd=None,                        # MIT 增益: None=默认，0=零增益
    torque=None, vel_ref=None,               # MIT 前馈力矩/速度参考
)
```

MIT 参数支持 `None`、标量、长度 6 或长度 7 的列表。`None` 表示使用默认值，显式 `0` 表示真实零增益/零力矩；长度 6 的列表只覆盖关节，夹爪使用默认值。

### MIT 专用接口

| 方法 | 说明 |
|------|------|
| `send_mit_command(joint_params, gripper)` | MIT 全参数直接发送（低延迟） |
| `initialize_mit_gains(kp, kd, duration, frequency_hz)` | 按固定频率平滑初始化 MIT 增益 |
| `set_linear_interpolation_velocity(velocity_rad_s)` | 写入固件线性轨迹插值速度 |

```python
from alicia_m_sdk import MitParams

# MitParams 字段
MitParams(
    pos_ref=0.0,     # 目标位置 (rad)
    vel_ref=0.0,     # 目标速度 (rad/s)
    t_ref=0.0,       # 前馈力矩 (N·m)
    kp=None,         # 位置增益 [0,500]，None=自动填充默认值
    kd=None,         # 速度增益 [0,5]，None=自动填充默认值
)
```

**Kp/Kd 默认值**：

| 电机组 | 编号 | Kp | Kd |
|--------|------|----|----|
| 大关节 | M0~M2 | 150 | 2.0 |
| 小关节 | M3~M6 | 150 | 2.0 |

### 系统控制

| 方法 | 说明 |
|------|------|
| `enable_robot()` | 使能 |
| `disable_robot()` | 失能 |
| `switch_mode(mode)` | 切换模式 `"pv"` / `"mit"`（失能→切换→使能） |
| `torque_control(command, joints)` | 力矩开关，`"off"` / `"on"`（仅 MIT） |
| `set_extended_polling(enabled)` | 开关扩展轮询；扩展字段需固件支持 |
| `set_zero_position()` | 设置当前位姿为零位 |
| `run_diagnostic(timeout)` | 运行自检并返回 DiagnosticResult |
| `get_user_settings(timeout)` | 读取个性化设置 |
| `set_gripper_type(gripper_type)` | 写入夹爪类型，支持 GripperType / 0/2 / 10/40 / 字符串 |
| `get_gripper_params(mask, aim, timeout)` | 读取夹爪夹持参数，普通用户优先使用 |
| `set_gripper_params(values, aim, timeout, readback, gripper_type)` | 写入夹爪夹持参数，带 SDK 侧范围校验 |
| `send_gripper_param_frame(frame, timeout)` | 发送 0x17 夹爪参数帧，高级调试用途；普通用户优先使用 `get_gripper_params` / `set_gripper_params` |

### 运动学

| 方法 | 说明 |
|------|------|
| `set_pose(target_pose, method, execute, speed)` | IK 运动 |
| `plan_joint_trajectory(waypoints, planner_type)` | 关节轨迹规划 |
| `plan_cartesian_trajectory(waypoints)` | 笛卡尔轨迹规划 |
| `move_joint_trajectory(q_end, duration, method)` | 执行关节轨迹 |
| `move_cartesian_linear(target_pose, duration)` | 执行直线轨迹 |

---

## 数据类型

### JointState

```python
@dataclass
class JointState:
    angles: List[float]                    # 6个关节角度 (rad)
    gripper: float                         # 夹爪值 (0~1000)
    timestamp: float                       # 时间戳
    run_status: int                        # 运行状态字节
    velocities: Optional[List[float]]      # 速度 (rad/s)
    torques: Optional[List[float]]         # 力矩 (N·m)
    kps: Optional[List[float]]             # Kp，扩展轮询
    kds: Optional[List[float]]             # Kd，扩展轮询
    linear_vels: Optional[List[float]]     # 插补速度，扩展轮询
    temperatures: Optional[List[float]]    # 线圈温度，扩展轮询
```

### 枚举

```python
class ControlMode(Enum):
    PV  = "pv"     # 位置-速度模式
    MIT = "mit"    # MIT 阻抗控制模式

class ControlAim(IntEnum):
    LEADER   = 0x01  # 示教臂
    FOLLOWER = 0x02  # 操作臂

class GripperType(Enum):
    MM_50 = "50mm"    # firmware_value=0, option_value=10
    MM_100 = "100mm"  # firmware_value=2, option_value=40
```

夹爪类型输入兼容：`GripperType.MM_50/MM_100`、固件值 `0/2`、示例选项 `10/40`、字符串 `"small"/"large"/"50mm"/"100mm"`。

## 高级能力 / Advanced Capabilities

高级能力仍通过 `robot.xxx(...)` 访问，普通用户不需要直接导入 `execution`、`hardware`、`utils` 或 `integrations` 模块。

```python
pose = robot.compute_forward_kinematics(joints, joint_format="rad")
ik_result = robot.set_pose(target_pose, execute=False)
traj = robot.plan_joint_trajectory(waypoints)
teleop = robot.create_mapped_teleoperation(leader_robot)
```

`JointController`、`TrajectoryExecutor`、`Teleoperation` 等类作为兼容过渡仍可被旧脚本直接导入，但不再作为新用户代码的推荐入口。`Teleoperation` 相关能力需要可选依赖：

```bash
pip install alicia_m_sdk[teleop]
```

---

## 异常体系

```
AliciaSDKError
├── ConnectionError      # 串口连接失败
├── TimeoutError         # 通信超时
├── ProtocolError        # 协议错误
├── ValidationError      # 参数校验失败
├── RobotStateError      # 状态不满足前提
├── HardwareFaultError   # 固件硬件错误
└── MotionError          # 运动执行异常
```

---

## 控制参数范围

| 参数 | 范围 | 单位 |
|------|------|------|
| 关节位置 | [-12.5, +12.5] | rad |
| 速度（用户层） | [0, 400] | 无量纲 |
| 固件速度 | [-10, +10] | rad/s |
| 力矩（大关节 M0~M2） | [-28, +28] | N·m |
| 力矩（小关节 M3~M6） | [-10, +10] | N·m |
| Kp | [0, 500] | — |
| Kd | [0, 5] | — |
| 夹爪 | [0, 1000] | — |
