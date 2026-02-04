# API 参考文档

本节介绍 Alicia M SDK 的核心类与方法接口。

---

##  初始化接口：`create_robot`

```python
from alicia_m_sdk import create_robot

robot = create_robot(
    port="",                    # 串口（留空自动查找）
    baudrate=1000000,           # 波特率
    version="v1_1",             # 机械臂版本 (可选: v1_0, v1_1)
    variant=None,               # 变体名称 (默认自动选择 follower)
    debug_mode=False,           # 调试模式
    control_aim=None,           # 控制目标（None则使用默认AIM_OPERATION）
    control_mode=None           # 控制模式（None则使用默认PATTERN_PV）
)
```

---

##  控制接口：`alicia_m_sdk.api.synria_robot_api.SynriaRobotAPI`

```python
from alicia_m_sdk import create_robot

robot = create_robot()
```

### 主要方法一览：

#### 连接管理：
- `connect()`  
  连接机械臂并检测固件版本

- `disconnect()`  
  断开机械臂连接并停止更新线程

- `is_connected()`  
  检查机械臂是否连接

#### 运动控制：
- `go_home(speed_deg_s=20, gripper_speed_deg_s=57.3)`  
  移动机械臂到初始位置（HOME位置，所有关节角度为0）
  - 注意：`set_home()` 已弃用，请使用 `go_home()` 代替

- `set_joint_target(target_joints, joint_format='rad', tolerance=0.03, timeout=None, wait_for_completion=True, speeds=None, torques=None)`  
  移动机械臂到目标关节角度（支持新固件直接控制）
  - `target_joints`: 目标关节角度列表（6个或7个元素，第7个为夹爪）
  - `joint_format`: 'rad' 或 'deg'
  - `tolerance`: 到达目标的容差（弧度）
  - `timeout`: 超时时间（秒），None时自动计算
  - `wait_for_completion`: 是否等待到达目标位置
  - `speeds`: 速度设置（rad/s），可为单个值或6个元素列表
  - `torques`: 力矩设置（N·m），可为单个值或6个元素列表

- `set_joint_target_no_wait(target_joints, joint_format='rad', speeds=None, torques=None)`  
  非阻塞版本的关节目标设置，发送指令后立即返回

- `set_joint_target_interplotation(target_joints, joint_format='rad', speed_factor=1.0, T_default=4.0, n_steps_ref=200, visualize=False)`  
  使用插值平滑移动机械臂到目标关节角度（适用于旧固件）

- `set_pose_target(target_pose, backend='numpy', method='dls', display=True, tolerance=1e-4, max_iters=100, multi_start=0, use_random_init=False, speed_factor=1.0, execute=True, joint_limits=None)`  
  使用逆运动学移动末端执行器到目标位姿
  - `target_pose`: [x, y, z, qx, qy, qz, qw] 格式的目标位姿
  - `method`: IK求解方法 ('dls', 'pinv', 'transpose')
  - `multi_start`: 多起点尝试次数，0表示禁用
  - `use_random_init`: 使用随机初始猜测而非当前位姿
  - `execute`: 是否执行运动（False则仅计算IK）

- `move_joint_trajectory(q_end, duration=2.0, method='cubic', num_points=100, visualize=False)`  
  执行平滑的关节轨迹到目标位置
  - `method`: 'linear', 'cubic', 或 'quintic'

- `move_cartesian_linear(target_pose, duration=2.0, num_points=50, ik_method='dls', visualize=False)`  
  执行笛卡尔直线轨迹到目标位姿

#### 状态获取：
- `get_joints(type='follower')`  
  返回当前关节角度（弧度）
  - `type='follower'`: 仅返回角度列表
  - 其他值: 返回 `(angles, button1, button2)` 元组

- `get_pose()`  
  获取当前末端执行器位置与姿态，返回字典包含：
  - `transform`: 4x4变换矩阵
  - `position`: [x, y, z] 位置向量
  - `rotation`: 3x3旋转矩阵
  - `euler_xyz`: [rx, ry, rz] 欧拉角（XYZ顺序）
  - `quaternion_xyzw`: [qx, qy, qz, qw] 四元数

- `get_gripper()`  
  返回当前夹爪开合度（0-100）

- `get_firmware_version(timeout=5.0, send_interval=0.2)`  
  查询机械臂固件版本，使用一问一答机制

- `print_state(continuous=False, output_format='deg', robot_type='follower')`  
  打印当前机械臂信息
  - `continuous`: True为持续打印，False为打印一次
  - `output_format`: 'deg' 或 'rad'
  - `robot_type`: 'follower' 或其他（影响是否显示按钮状态）

#### 夹爪控制：
- `set_gripper_target(command=None, value=None, wait_for_completion=True, timeout=5.0, tolerance=1.0)`  
  控制夹爪位置
  - `command`: 'open' 或 'close' 命令字符串
  - `value`: 夹爪值，范围 0（闭合）到 100（张开）
  - `wait_for_completion`: 是否等待夹爪到达目标
  - `timeout`: 最大等待时间（秒）
  - `tolerance`: 到达目标的容差

#### 系统控制：
- `torque_control(command)`  
  启用或关闭扭矩
  - `command`: 'on' 启用扭矩，'off' 关闭扭矩

- `zero_calibration()`  
  执行归零校准流程：关闭扭矩 → 手动拖动 → 重启扭矩 → 记录零点

- `set_speed(speed_rad_s)`  
  设置机械臂运动速度（弧度/秒，仅新固件）

- `set_acceleration(acceleration)`  
  设置机械臂加速度

---

##  硬件层接口：`alicia_m_sdk.hardware.ServoDriver`

提供底层串口通信、数据解析和电机控制功能。

主要方法包括：
- `connect()` / `disconnect()`
- `get_joint_angles()` / `set_joint_angles(...)`
- `get_joint_state()` / `get_gripper_data()`
- `set_gripper(...)`
- `enable_torque()` / `disable_torque()`
- `set_zero_position()`
- `set_speed(...)` / `set_acceleration(...)`
- `set_joint_and_gripper(...)` - 综合设置关节、夹爪、速度和力矩

**控制模式常量：**
- `AIM_TEACH` / `AIM_OPERATION` - 控制目标
- `PATTERN_PV` - 位置-速度模式
- `PATTERN_PVT` - 位置-速度-力矩模式
- `PATTERN_V` - 速度模式
- `PATTERN_MIT` / `PATTERN_MIT_POSITION` / `PATTERN_MIT_SPEED` / `PATTERN_MIT_TORQUE` - MIT阻抗控制模式

不推荐用户直接使用此类，建议通过 `SynriaRobotAPI` 高级接口操作。


---

##  执行层接口

### `alicia_m_sdk.execution.HardwareExecutor`
负责执行轨迹序列，处理关节角度命令的发送和可视化。

主要方法：
- `execute(joint_traj, visualize=False)` - 执行关节轨迹列表

### `alicia_m_sdk.execution.JointInterpolator`
提供关节空间的插值规划功能。

主要方法：
- `plan(start_angles, target_angles, steps)` - 生成线性插值轨迹

---

##  内部辅助方法

### 私有方法（用户通常不需要调用）：
- `_wait_for_joint_target(target_joints, tolerance_deg, timeout, log_prefix)` - 等待关节到达目标位置
- `_generate_random_q(scale, joint_limits)` - 生成随机关节配置（用于IK测试）

---

##  RoboCore 集成

SDK 集成了 [RoboCore](https://github.com/Synria-Robotics/RoboCore) 库，提供高性能运动学和轨迹规划功能：

### 运动学功能（来自 robocore.kinematics）：
- `forward_kinematics(robot_model, q, backend='numpy', return_end=True)`
- `inverse_kinematics(robot_model, pose, q_init, backend='numpy', method='dls', ...)`
- `jacobian(robot_model, q, backend='numpy', method='analytic')`

### 轨迹规划功能（来自 robocore.planning）：
- `cubic_polynomial_trajectory(q_start, q_end, duration, num_points)`
- `quintic_polynomial_trajectory(q_start, q_end, duration, num_points)`
- `linear_joint_trajectory(q_start, q_end, duration, num_points)`
- `linear_cartesian_trajectory(robot_model, pose_start, pose_end, duration, ...)`
- `trapezoidal_velocity_profile(distance, max_vel, max_acc)`

---

##  使用示例

### 基本连接和运动
```python
from alicia_m_sdk import create_robot
import numpy as np

# 创建机器人实例
robot = create_robot(port="", baudrate=1000000)

# 连接机器人
robot.connect()

# 移动到初始位置（HOME位置）
robot.go_home()

# 关节空间运动（弧度）
target_joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
robot.set_joint_target(target_joints, joint_format='rad')

# 关节空间运动（角度）
target_joints_deg = [10, 20, 30, 40, 50, 60]
robot.set_joint_target(target_joints_deg, joint_format='deg')

# 获取当前状态
joints = robot.get_joints()
pose = robot.get_pose()
gripper = robot.get_gripper()

# 断开连接
robot.disconnect()
```

### 笛卡尔空间运动
```python
# 目标位姿：[x, y, z, qx, qy, qz, qw]
target_pose = [0.3, 0.0, 0.2, 0, 0, 0, 1]

# 使用IK移动到目标位姿
result = robot.set_pose_target(target_pose, method='dls', execute=True)

if result['success']:
    print(f"IK求解成功，迭代次数: {result['iters']}")
else:
    print(f"IK求解失败: {result['message']}")

# 执行直线轨迹
robot.move_cartesian_linear(target_pose, duration=3.0, num_points=100)
```

### 夹爪控制
```python
# 打开夹爪
robot.set_gripper_target(command='open')

# 关闭夹爪
robot.set_gripper_target(command='close')

# 设置夹爪到特定位置（0-100）
robot.set_gripper_target(value=50)
```

### 高级控制模式
```python
from alicia_m_sdk.hardware import ServoDriver

# 创建具有特定控制模式的机器人
robot = create_robot(
    control_aim=ServoDriver.AIM_OPERATION,
    control_mode=ServoDriver.PATTERN_PVT
)

# 使用速度和力矩控制
speeds = [0.1] * 6  # rad/s
torques = [0.5] * 6  # N·m
robot.set_joint_target(
    target_joints=[0, 0, 0, 0, 0, 0],
    speeds=speeds,
    torques=torques
)
```

---

##  重要注意事项

1. **单位约定**：
   - 关节角度：默认弧度（rad），可通过 `joint_format='deg'` 使用角度
   - 速度：弧度/秒（rad/s）
   - 位置：米（m）
   - 力矩：牛顿米（N·m）

2. **固件版本**：
   - 新固件（v1.x）：支持直接速度控制、阻塞等待、MIT模式
   - 旧固件（v0.x）：需要使用插值方法

3. **阻塞与非阻塞**：
   - `set_joint_target(..., wait_for_completion=True)`：阻塞等待到达
   - `set_joint_target_no_wait(...)`：立即返回，不等待
   - `set_joint_target(..., wait_for_completion=False)`：发送后立即返回

4. **容差设置**：
   - `tolerance` 参数控制到达目标的判定标准
   - 默认 5 度（约 0.087 弧度）
   - 过小的容差可能导致永远无法判定到达

5. **超时处理**：
   - 自动计算的超时时间 = 预估运动时间 × 2 + 10秒
   - 可手动指定 `timeout` 参数覆盖

6. **控制模式**：
   - `PATTERN_PV`：位置-速度模式（默认）
   - `PATTERN_PVT`：位置-速度-力矩模式
   - `PATTERN_MIT`：MIT阻抗控制模式（需新固件支持）

---

如需更多细节，请参考源码文档或查看 `logs/` 下日志文件输出。