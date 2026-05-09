# API 参考

## 工厂函数

### `create_robot()`

```python
alicia_m_sdk.create_robot(
    port: str = "",              # 串口路径，空=自动发现
    version: str = "v1_1",       # 硬件版本
    variant: str = None,         # 变体（自动检测）
    control_aim: str = None,     # "leader"/"follower"（自动检测）
    control_mode: str = "pv",    # "pv" / "mit"
    baudrate: int = 1_000_000,   # 波特率
    backend: str = "numpy",      # RoboCore 后端
    auto_connect: bool = True,   # 自动连接
) → SynriaRobotAPI
```

---

## SynriaRobotAPI

### 连接管理

| 方法 | 说明 |
|------|------|
| `connect(timeout=5.0)` | 连接机器人 |
| `disconnect()` | 断开连接 |
| `is_connected()` | 检查连接状态 |

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
| `"all"` | 完整 JointState 对象 |

### 运动控制

| 方法 | 说明 |
|------|------|
| `set_robot_state(target_joints, gripper_value, speed=15, ...)` | 点位运动（自动适配 PV/MIT） |
| `go_home(speed=15)` | 回零位 |
| `set_gripper_target(command, value, wait_for_completion)` | 控制夹爪 |

**set_robot_state 参数**：

```python
robot.set_robot_state(
    target_joints=[0, 30, 0, 0, -30, 0],  # 目标角度 (deg)
    gripper_value=500,                      # 夹爪 [0, 1000]
    joint_format='deg',                     # 'deg' / 'rad'
    speed=15,                               # 速度 [0, 400]
    gripper_speed=40,                       # 夹爪速度 [0, 400]
    wait_for_completion=True,               # 是否等待到达
)
```

### MIT 专用接口

| 方法 | 说明 |
|------|------|
| `send_mit_command(joint_params, gripper)` | MIT 全参数直接发送（低延迟） |
| `initialize_mit_gains(kp, kd, duration=1.0, frequency_hz=50.0)` | 运动前线性初始化 Kp/Kd，降低首次 MIT 命令的跳变风险 |

```cpp
/**
 * @brief 在 MIT 运动开始前线性初始化 Kp/Kd。
 * @details SDK 先读取当前机械臂 Kp/Kd，与目标增益逐电机比较；
 *          随后保持当前位置不变，将 Kp/Kd 按固定频率线性过渡到目标值。
 *          建议在 demo、遥操作或产品流程第一次进入 MIT 运动前调用一次。
 * @param kp 目标位置增益 [0, 500]。None 使用默认值；标量广播；列表逐电机设置。
 * @param kd 目标速度增益 [0, 5]。格式同 kp。
 * @param duration 线性过渡时长，单位秒。
 * @param frequency_hz 过渡帧发送频率，单位 Hz。
 * @return True 表示初始化完成。
 */
robot.initialize_mit_gains(
    kp=[150, 150, 150, 150, 150, 150, 150],
    kd=[2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
)
```

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
| `set_zero_position()` | 设置当前位姿为零位 |

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
    kps: Optional[List[float]]             # MIT 位置增益 Kp
    kds: Optional[List[float]]             # MIT 速度增益 Kd
    linear_vels: Optional[List[float]]     # 线性轨迹插值速度 (rad/s)
    temperatures: Optional[List[float]]    # 线圈温度 (°C)
```

### 枚举

```python
class ControlMode(Enum):
    PV  = "pv"     # 位置-速度模式
    MIT = "mit"    # MIT 阻抗控制模式

class ControlAim(IntEnum):
    LEADER   = 0x01  # 示教臂
    FOLLOWER = 0x02  # 操作臂
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
