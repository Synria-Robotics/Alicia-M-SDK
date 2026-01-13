# Bug 修复报告：后台线程 KeyError 和数据读取问题

## 问题描述

用户在运行 `05_demo_move_joint.py` 时遇到两个问题：

1. **后台线程崩溃**: `KeyError: 'joint'`
2. **读取数据全是0**: 物理机械臂已运动到目标位置，但读取的关节角度都是0

## 错误日志

```
[Alicia-D-SDK:ERROR] State update thread exception: 'joint'
Exception in thread Thread-1:
Traceback (most recent call last):
  File "/home/ubuntu/robot_ws/src/Alicia_M_SDK/alicia_m_sdk/hardware/servo_driver.py", line 339, in _update_loop
    query_cmd = self.INFO_COMMAND_MAP["joint"]
KeyError: 'joint'
```

```
[位置检测] 当前: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]°
```

## 根本原因

### 问题1: 后台线程使用硬编码指令

在之前的改进中，我们移除了 `INFO_COMMAND_MAP["joint"]` 的硬编码指令，改为使用动态构建的请求帧。但是忘记更新后台线程 `_update_loop()` 中的代码，导致它仍然尝试访问不存在的键。

**位置**: `servo_driver.py:339`

```python
# 旧代码（错误）
query_cmd = self.INFO_COMMAND_MAP["joint"]  # KeyError!
```

### 问题2: control_aim 参数未正确传递

`create_robot()` 函数接收了 `control_aim` 参数，但在创建 `ServoDriver` 实例时没有传递，导致：

- 示例代码指定 `control_aim=ServoDriver.AIM_TEACH` (0x01)
- 但 `ServoDriver` 使用默认值 `AIM_OPERATION` (0x02)
- **发送控制指令给示教臂** (0x81)，但**请求操作臂的数据** (0x02)
- 数据不匹配，读取的都是默认值0

**位置**: `__init__.py:92`

```python
# 旧代码（错误）
servo_driver = ServoDriver(
    port=port, 
    baudrate=baudrate, 
    debug_mode=debug_mode, 
    firmware_version=firmware_version, 
    robot_type=robot_type
    # 缺少 control_aim 参数！
)
```

## 修复方案

### 修复1: 更新后台线程使用动态请求帧

**文件**: `alicia_m_sdk/hardware/servo_driver.py`

**行数**: 337-340

```python
# 修复前
query_cmd = self.INFO_COMMAND_MAP["joint"]

# 修复后
query_cmd = self._build_joint_request_frame(control_aim=self.default_control_aim)
```

### 修复2: 传递 control_aim 参数

**文件**: `alicia_m_sdk/__init__.py`

**行数**: 92-99

```python
# 修复前
servo_driver = ServoDriver(
    port=port, 
    baudrate=baudrate, 
    debug_mode=debug_mode, 
    firmware_version=firmware_version, 
    robot_type=robot_type
)

# 修复后
servo_driver = ServoDriver(
    port=port, 
    baudrate=baudrate, 
    debug_mode=debug_mode, 
    firmware_version=firmware_version, 
    robot_type=robot_type,
    control_aim=control_aim  # 添加这行
)
```

## 验证结果

### 修复前

```bash
# 后台线程崩溃
[Alicia-D-SDK:ERROR] State update thread exception: 'joint'
KeyError: 'joint'

# 数据读取错误
[位置检测] 当前: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]°
[位置检测] 误差: [90.0, 90.0, 90.0, 90.0, 0.0, 0.0]°
```

### 修复后

```bash
# 后台线程正常工作
[RX] 接收数据包: AA 06 01 11 80 01 14 90 E8 6F E9 6F 14 90 FE 7F FE 7F 14 00 00 E6 FF

# 数据读取正确
[位置检测] 当前: [90.0, -90.0, -90.0, 90.0, -0.0, -0.0]°
[位置检测] 误差: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]°, 最大误差: 0.0°

# 位置验证成功
[Alicia-D-SDK:INFO] ✓ 已到达目标位置 (耗时: 0.00s, 最大误差: 0.04°)
```

### 单元测试

```bash
$ python test_fix_verification.py
================================================================================
✅ 所有测试通过！control_aim 参数正确传递
================================================================================

修复总结:
  1. ✅ 修复后台线程使用动态构建的请求帧
  2. ✅ 修复 create_robot() 传递 control_aim 参数
  3. ✅ 示教臂和操作臂模式都可以正常工作
  4. ✅ 后台线程与控制指令使用相同的 control_aim
```

## 影响范围

### 修改文件

1. `alicia_m_sdk/hardware/servo_driver.py` (1行)
   - 第339行：使用动态构建的请求帧

2. `alicia_m_sdk/__init__.py` (1行)
   - 第98行：传递 control_aim 参数

### 测试文件

- `test_fix_verification.py` (新增) - 验证修复

## 数据流验证

### 示教臂模式 (control_aim=AIM_TEACH)

```
初始化: control_aim=0x01 → ServoDriver.default_control_aim=0x01
                                        ↓
后台线程: _build_joint_request_frame(0x01) → [AA 06 01 ...]
                                        ↓
控制指令: _build_send_joint_frame(0x01) → [AA 06 81 ...]
                                        ↓
接收反馈: 解析功能码 0x01 → 示教臂状态
```

### 操作臂模式 (control_aim=AIM_OPERATION)

```
初始化: control_aim=0x02 → ServoDriver.default_control_aim=0x02
                                        ↓
后台线程: _build_joint_request_frame(0x02) → [AA 06 02 ...]
                                        ↓
控制指令: _build_send_joint_frame(0x02) → [AA 06 82 ...]
                                        ↓
接收反馈: 解析功能码 0x02 → 操作臂状态
```

## 兼容性

✅ **完全向后兼容**
- 现有代码无需修改
- 所有API保持不变
- 默认行为保持一致（操作臂）

## 总结

这个Bug是由两个独立的问题组成：

1. **后台线程问题**: 代码重构时遗漏了更新后台线程的代码
2. **参数传递问题**: `create_robot()` 没有正确传递 `control_aim` 参数

修复后，两种控制目标（示教臂和操作臂）都可以正常工作，后台线程与控制指令使用相同的功能码，数据读取正确。

---

**修复日期**: 2026年1月13日  
**状态**: ✅ 已修复并验证  
**影响**: 无向后兼容性问题
