# Alicia-M-SDK

Synria Alicia-M 6-DOF 机械臂 Python SDK。

SDK 通过串口协议控制 Alicia-M 操作臂，提供连接、状态读取、PV/MIT 控制、夹爪控制、运动学、轨迹规划、遥操作和设备设置等能力。

推荐入口：

```python
import alicia_m_sdk

robot = alicia_m_sdk.create_robot()
```

## 主要特性

- **统一控制入口**：`create_robot()` / `SynriaRobotAPI` 覆盖连接、状态读取、运动控制和设备设置，普通使用场景不需要直接操作底层串口协议。
- **PV / MIT 双模式**：PV 模式适合常规点位运动，MIT 模式适合阻抗控制、拖动示教和遥操作；`set_robot_state()` 会按当前模式自动选择执行路径。
- **关节 + 夹爪协同**：同一接口可同时控制 6 个关节和夹爪，夹爪目标值使用 `0~1000`，并支持 50mm / 100mm 类型配置。
- **实时状态缓存**：后台轮询维护关节角、夹爪、速度、力矩和运行状态；开启扩展轮询后可读取 Kp/Kd、插补速度和温度等字段。
- **RoboCore 集成**：自动加载 `synriard` 中的 Alicia-M URDF 模型，提供 FK、IK、关节轨迹和笛卡尔轨迹规划能力。
- **遥操作支持**：内置 Alicia-D 到 Alicia-M 的主从跟随控制，可结合 URDF 映射、速度限制和 MIT 参数进行实时控制。
- **调试与维护工具**：提供自检、使能/失能、模式切换、零位标定、用户设置读写和夹爪夹持参数读写示例。

## 安装

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

当前要求 Python `>=3.11`。

## 快速开始

```python
import alicia_m_sdk

with alicia_m_sdk.create_robot(port="COM37", control_mode="pv") as robot:
    robot.set_robot_state(
        target_joints=[0, 30, 0, 0, -30, 0],
        gripper_value=500,
        joint_format="deg",
        speed=15,
    )

    state = robot.get_robot_state("all")
    print(state.angles)   # 6 个关节角，单位 rad
    print(state.gripper)  # 夹爪值，范围 0~1000
```

如果不传 `port`，SDK 会自动扫描串口：

```python
robot = alicia_m_sdk.create_robot()
```

## 常用参数

`create_robot()` 常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `port` | `""` | 串口名，空字符串表示自动发现 |
| `control_mode` | `None` | `"pv"` / `"mit"`；不传则跟随固件当前模式 |
| `control_aim` | `None` | `"leader"` / `"follower"`；通常 Alicia-M 使用 follower |
| `version` | `"v1_1"` | 机器人模型版本 |
| `backend` | `"cpp"` | RoboCore 后端：`"cpp"` / `"numpy"` / `"torch"` |
| `extended_polling` | `False` | 是否读取温度、Kp/Kd、插补速度等扩展状态 |

`set_robot_state()` 常用参数：

| 参数 | 说明 |
| --- | --- |
| `target_joints` | 6 个关节目标角 |
| `joint_format` | `"deg"` 或 `"rad"` |
| `gripper_value` | 夹爪目标值，范围 `0~1000` |
| `speed` | 关节速度参数 |
| `gripper_speed` | 夹爪速度参数 |
| `kp` / `kd` | MIT 模式阻抗增益，`None` 表示使用默认安全值 |
| `torque` / `vel_ref` | MIT 模式前馈力矩和目标速度 |

MIT 参数支持标量、长度为 6 的关节列表、长度为 7 的电机列表。显式传入 `0` 表示真实零值，不会被默认值覆盖。

## 常用接口

```python
# 连接和系统控制
robot.connect()
robot.disconnect()
robot.enable_robot()
robot.disable_robot()
robot.switch_mode("pv")
robot.switch_mode("mit")

# 状态读取
robot.get_robot_state("joint")          # 关节角，rad
robot.get_robot_state("joint_gripper")  # 关节角 + 夹爪
robot.get_robot_state("velocity")       # 速度
robot.get_robot_state("torque")         # 力矩
robot.get_robot_state("all")            # JointState 对象
robot.get_robot_state("version")        # VersionInfo 对象
robot.get_robot_state("status")         # RobotStatus 对象
robot.get_robot_state("control_mode")   # 各电机控制模式

# 夹爪
robot.set_gripper_target("open")
robot.set_gripper_target("close")
robot.set_gripper_target(value=500)
robot.set_gripper_type("50mm")
robot.set_gripper_type("100mm")

# 运动学和轨迹
robot.get_pose()
robot.set_pose(target_pose, execute=False)
robot.plan_joint_trajectory(waypoints)
robot.move_joint_trajectory(q_end)
```

## 示例程序

| 编号 | 文件 | 说明 |
| --- | --- | --- |
| 00 | `00_demo_read_version.py` | 读取固件版本和设备信息 |
| 01 | `01_demo_diagnostic.py` | 设备自检 |
| 02 | `02_demo_read_status.py` | 读取控制模式和运行状态 |
| 03 | `03_demo_read_states.py` | 循环读取关节、速度、力矩等状态 |
| 04 | `04_demo_switch_mode.py` | 切换 PV / MIT 控制模式 |
| 05 | `05_demo_disable_enable.py` | 失能 / 使能机械臂 |
| 06 | `06_demo_move_gripper.py` | 控制夹爪 |
| 07 | `07_demo_move_joint.py` | PV 模式关节运动 |
| 08 | `08_demo_move_full_arm.py` | PV 模式关节 + 夹爪协同运动 |
| 09 | `09_demo_move_joint_mit.py` | MIT 模式关节运动 |
| 10 | `10_demo_move_full_arm_mit.py` | MIT 模式关节 + 夹爪协同运动 |
| 11 | `11_demo_forward_kinematics.py` | 正运动学 |
| 12 | `12_demo_inverse_kinematics.py` | 逆运动学 |
| 13 | `13_demo_reset_zero.py` | 零位标定 |
| 14 | `14_demo_teleop_mapped.py` | Alicia-D 到 Alicia-M 遥操作 |
| 15 | `15_demo_gripper_params.py` | 夹爪夹持参数读写 |
| 16 | `16_demo_joint_traj.py` | 关节空间轨迹规划与执行 |
| 17 | `17_demo_mit_torque_switch.py` | MIT 力矩开关测试 |
| 18 | `18_demo_user_settings.py` | 用户设置读写 |

运行示例：

```bash
python examples/00_demo_read_version.py --port COM37
python examples/07_demo_move_joint.py --port COM37 --speed 15
python examples/16_demo_joint_traj.py --port COM37 --plot
```

## 项目结构

```text
alicia_m_sdk/
|-- api/            # 用户 API，SynriaRobotAPI
|-- hardware/       # 串口、协议帧、设备状态
|-- execution/      # 关节控制、轨迹执行、遥操作
|-- integrations/   # RoboCore 运动学和规划适配
|-- types/          # 配置、状态、枚举、异常
`-- utils/          # 转换、校验、日志、计时等工具
```

## 开发

```bash
python -m pytest
python -m build
python -m twine check dist/*
```

## License

MIT License - Synria Robotics
