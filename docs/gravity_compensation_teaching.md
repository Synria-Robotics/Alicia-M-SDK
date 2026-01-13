# 重力补偿拖动示教系统

## 📝 概述

基于 MIT 扭矩模式的重力补偿拖动示教功能，实现"零重力"拖动录制和高频轨迹回放。

### 核心组件

**1. drag_teaching.py**
- `RobotDynamicsModel`: 基于URDF的重力补偿计算（递归牛顿-欧拉法）
- `GravityCompensationTeaching`: 重力补偿示教系统（录制+保存）
- `SimpleDragTeaching`: 简化拖动示教（兼容旧版）

**2. hardware_executor.py**
- `HardwareExecutor`: 轨迹执行器
  - MIT位置模式：200Hz+高频回放
  - PV模式：50Hz低频回放
- `JointInterpolator`: 关节轨迹插值器

### 关键特性
- ✅ MIT扭矩模式实时重力补偿（200Hz）
- ✅ MIT位置模式高频回放（200Hz+）
- ✅ 可配置采样频率（50-500Hz）
- ✅ 结构化JSON数据存储
- ✅ 夹爪位置同步记录

---

## 🚀 快速使用

### 命令行工具

**录制轨迹：**
```bash
python examples/09_demo_gravity_compensation_teaching.py \
    --port /dev/ttyUSB0 \
    --mode record \
    --motion my_motion
```

**回放轨迹：**
```bash
python examples/09_demo_gravity_compensation_teaching.py \
    --port /dev/ttyUSB0 \
    --mode replay \
    --motion my_motion
```

**列出动作：**
```bash
python examples/09_demo_gravity_compensation_teaching.py --mode list
```

### Python API

**完整流程（录制→保存→回放）：**
```python
from alicia_m_sdk import create_robot
from alicia_m_sdk.execution import GravityCompensationTeaching, HardwareExecutor
import json

# 连接机器人
robot = create_robot(port='/dev/ttyUSB0')
robot.connect()

# 创建示教对象
teaching = GravityCompensationTeaching(
    controller=robot,
    urdf_path='Alicia_M_v1_1_gripper_100mm.urdf',
    sample_hz=200.0,
    torque_scale=1.0
)

# 录制
teaching.start_gravity_compensation()
trajectory = teaching.record_trajectory(manual_stop=True)
teaching.stop_gravity_compensation()

# 保存
teaching.save_trajectory(trajectory, "my_motion")

# 回放
joint_traj = [point['q'] for point in trajectory]
gripper_traj = [point['grip'] for point in trajectory]

executor = HardwareExecutor(robot.servo_driver)
executor.execute(
    joint_traj=joint_traj,
    gripper_traj=gripper_traj,
    use_mit_mode=True,
    playback_hz=200.0
)

robot.disconnect()
```

**交互式流程（最简单）：**
```python
teaching = GravityCompensationTeaching(robot, 'Alicia_M_v1_1_gripper_100mm.urdf')
teaching.run_interactive("my_motion")
```

---

## ⚙️ 关键参数

### 录制参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `sample_hz` | 200.0 | 采样频率（Hz），范围50-500 |
| `torque_scale` | 1.0 | 扭矩缩放系数，范围0.5-1.5 |
| `urdf_path` | `Alicia_M_v1_1_gripper_100mm.urdf` | URDF文件路径 |

**torque_scale 调整：**
- `< 1.0`: 补偿不足，机械臂下垂 → 增大值
- `= 1.0`: 理想补偿（推荐）
- `> 1.0`: 过度补偿，机械臂上漂 → 减小值

### 回放参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_mit_mode` | False | True=MIT位置模式（高频），False=PV模式 |
| `playback_hz` | 50.0 | MIT模式回放频率，建议与录制频率一致 |
| `speed_rad_s` | 0.087266 | PV模式速度（rad/s） |

**模式选择：**
- **MIT位置模式**: 200Hz+高频轨迹 → `use_mit_mode=True`
- **PV模式**: <50Hz低频轨迹 → `use_mit_mode=False`

---

## 📁 数据格式

```
example_motions/my_motion/
├── joint_traj.json    # 轨迹数据
└── meta.json          # 元信息
```

**joint_traj.json：**
```json
[
  {
    "t": 0.000,
    "q": [0.0, -0.5, 1.2, 0.0, 0.8, 0.0],
    "grip": 50.0
  },
  ...
]
```

**meta.json：**
```json
{
  "motion": "my_motion",
  "mode": "gravity_compensation",
  "created_at": "2026-01-04 12:34:56",
  "sample_hz": 200.0,
  "count": 2000,
  "duration_sec": 10.0,
  "torque_scale": 1.0
}
```

---

## 🔧 常见问题

### 1. MIT模式初始化失败
```python
# 增加重复次数
robot.servo_driver.initialize_mit_mode(repeat_times=10)
```

### 2. 机械臂下垂/上漂
```bash
# 下垂 → 增大扭矩
python ... --torque-scale 1.1

# 上漂 → 减小扭矩
python ... --torque-scale 0.9
```

### 3. 回放速度不对
```python
# 确保频率匹配
with open('example_motions/my_motion/meta.json') as f:
    meta = json.load(f)
    
executor.execute(..., playback_hz=meta['sample_hz'])
```

### 4. URDF文件未找到
```python
# 使用绝对路径
import os
urdf_path = os.path.join(os.path.dirname(__file__), 'Alicia_M_v1_1_gripper_100mm.urdf')
```

---

## 📊 核心类说明

### RobotDynamicsModel
**功能：** URDF解析和重力扭矩计算

**主要方法：**
- `parse_urdf()`: 解析URDF文件
- `compute_gravity_torques(joint_angles)`: 计算重力补偿扭矩

**算法：** 递归牛顿-欧拉法（RNE）

### GravityCompensationTeaching
**功能：** 重力补偿录制系统

**主要方法：**
- `start_gravity_compensation()`: 启动补偿（MIT扭矩模式）
- `record_trajectory(duration, manual_stop)`: 录制轨迹
- `stop_gravity_compensation()`: 停止补偿
- `save_trajectory(trajectory, motion_name)`: 保存轨迹
- `run_interactive(motion_name)`: 交互式流程

### HardwareExecutor
**功能：** 轨迹回放执行器

**主要方法：**
- `execute(joint_traj, use_mit_mode, playback_hz)`: 统一执行接口
- `_execute_mit_position_mode()`: MIT位置模式（内部）
- `_execute_pv_mode()`: PV模式（内部）

---

## 🎓 工作原理

### 录制阶段
1. URDF模型计算各关节重力扭矩
2. MIT扭矩模式（200Hz）实时发送补偿扭矩
3. 机械臂呈现"零重力"状态，可自由拖动
4. 高频采样记录关节角度和夹爪位置

### 回放阶段
1. 加载JSON轨迹数据
2. MIT位置模式逐帧发送目标位置
3. 高频更新（200Hz）精确重现轨迹

### MIT模式对比

| 特性 | MIT扭矩模式（录制） | MIT位置模式（回放） |
|------|-------------------|-------------------|
| 用途 | 重力补偿 | 轨迹回放 |
| 发送内容 | 扭矩值（6×2字节） | 位置值（6×2字节） |
| 频率 | 200Hz | 200Hz+ |
| 单片机插值 | 无 | 无 |

---

## ✅ 检查清单

**录制前：**
- [ ] 确认URDF文件存在
- [ ] 机械臂处于安全位置
- [ ] USB连接稳定
- [ ] 固件版本 >= v1.0

**回放前：**
- [ ] 轨迹文件存在（joint_traj.json）
- [ ] 回放频率与录制频率一致
- [ ] 回放路径无障碍物

---

## 📚 参考文档

- [API参考](api_reference.md)
- [MIT模式说明](mit_mode_usage.md)
- [更新总结](UPDATE_SUMMARY.md)
- [示例代码](../examples/09_demo_gravity_compensation_teaching.py)

---

**版本:** v2.0.0  
**更新日期:** 2026-01-04  
**许可:** MIT License
