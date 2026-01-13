# 控制目标 (control_aim) 改进说明

## 概述

本次改进将控制目标 (`control_aim`) 与数据发送和解析融合，实现了示教臂和操作臂使用不同功能码的需求。

## 主要改进

### 1. 默认控制目标改为操作臂

**改动位置**: `servo_driver.py` 初始化函数

```python
# 旧代码：默认为示教臂
control_aim = self.AIM_TEACH  # 0x01

# 新代码：默认为操作臂
self.default_control_aim = control_aim if control_aim is not None else self.AIM_OPERATION  # 0x02
```

### 2. 动态构建请求关节信息的指令帧

**新增函数**: `_build_joint_request_frame()`

根据 `control_aim` 动态构建请求帧，不再使用硬编码的指令。

**功能码规则**:
- 示教臂请求: `0x01` (AIM_TEACH)
- 操作臂请求: `0x02` (AIM_OPERATION)

**示例**:
```python
# 请求操作臂关节信息
frame = driver._build_joint_request_frame(control_aim=ServoDriver.AIM_OPERATION)
# 结果: [0xAA, 0x06, 0x02, 0x02, 0x00, 0x01, 0xCE, 0xFF]
#                   ^^^^
#                   功能码 = 0x02 (操作臂)

# 请求示教臂关节信息
frame = driver._build_joint_request_frame(control_aim=ServoDriver.AIM_TEACH)
# 结果: [0xAA, 0x06, 0x01, 0x02, 0x00, 0x01, 0x20, 0xFF]
#                   ^^^^
#                   功能码 = 0x01 (示教臂)
```

### 3. 控制指令帧功能码改进

**写入模式功能码**:
```python
# frame[2] = 0x80 | control_aim
示教臂写入: 0x80 | 0x01 = 0x81
操作臂写入: 0x80 | 0x02 = 0x82
```

### 4. acquire_info() 支持 control_aim 参数

**改进后的函数签名**:
```python
def acquire_info(self, info_type: str, wait: bool = False, 
                 timeout: float = 1.0, control_aim: int = None) -> bool:
```

**使用示例**:
```python
# 使用默认控制目标（操作臂）
driver.acquire_info("joint")

# 显式指定示教臂
driver.acquire_info("joint", control_aim=ServoDriver.AIM_TEACH)

# 显式指定操作臂
driver.acquire_info("joint", control_aim=ServoDriver.AIM_OPERATION)
```

## 使用方式

### 方式1: 初始化时指定默认控制目标

```python
# 创建操作臂实例（默认）
driver_operation = ServoDriver(port="", baudrate=1000000)

# 创建示教臂实例
driver_teach = ServoDriver(
    port="", 
    baudrate=1000000,
    control_aim=ServoDriver.AIM_TEACH  # 指定为示教臂
)
```

### 方式2: 每次调用时覆盖控制目标

```python
# 使用默认控制目标
driver.set_joint_and_gripper(
    joint_angles=[0.0] * 6,
    gripper_value=50.0
)

# 覆盖为示教臂
driver.set_joint_and_gripper(
    joint_angles=[0.0] * 6,
    gripper_value=50.0,
    control_aim=ServoDriver.AIM_TEACH
)

# 覆盖为操作臂
driver.set_joint_and_gripper(
    joint_angles=[0.0] * 6,
    gripper_value=50.0,
    control_aim=ServoDriver.AIM_OPERATION
)
```

## 数据包格式

### 请求模式 (Ask Mode)

| 字段 | 示教臂 | 操作臂 |
|-----|-------|-------|
| 帧头 | 0xAA | 0xAA |
| 指令码 | 0x06 | 0x06 |
| **功能码** | **0x01** | **0x02** |
| 数据长度 | 0x02 | 0x02 |
| 数据类型 | 0x00 | 0x00 |
| 偏移数量 | 0x01 | 0x01 |
| 校验位 | 计算值 | 计算值 |
| 帧尾 | 0xFF | 0xFF |

### 写入模式 (Write Mode)

| 字段 | 示教臂 | 操作臂 |
|-----|-------|-------|
| 帧头 | 0xAA | 0xAA |
| 指令码 | 0x06 | 0x06 |
| **功能码** | **0x81** | **0x82** |
| 数据长度 | 变长 | 变长 |
| 数据... | ... | ... |
| 校验位 | 计算值 | 计算值 |
| 帧尾 | 0xFF | 0xFF |

### 反馈模式 (Feedback Mode)

**写入模式反馈**（简短）:
- 最后一字节状态: `0x01`=接收成功, `0x02`=接收失败
- 不区分示教臂/操作臂，只表示接收状态

**请求模式反馈**（完整状态）:
- 示教臂状态位（frame[2]=0x01 或 0x81）:
  - 0x01: 锁定状态
  - 0x02: 同步状态
  - 0x40: 夹具过高力矩锁定
  - 0x80: 电机错误状态
  
- 操作臂状态位（frame[2]=0x02 或 0x82）:
  - 0x01: 单击
  - 0x02: 双击
  - 0x04: 长按
  - 0x08: 重复长按
  - 0x40: 夹具过高力矩锁定
  - 0x80: 电机错误状态

## 数据解析器 (DataParser) 兼容性

`DataParser` 已经可以正确解析不同设备类型的反馈:

```python
# 解析时会自动识别设备类型
device_code = func_id & 0x7F  # 获取设备类型代码
run_status_text = self._parse_run_status(status_byte, device_code)

# 示教臂状态存储在 _teach_status
teach_status = self.data_parser.get_teach_status()

# 操作臂状态存储在 _oper_status  
oper_status = self.data_parser.get_oper_status()

# 获取当前设备的状态
current_status = self.data_parser.get_current_status()
```

## 向后兼容性

✅ **完全向后兼容**

- 默认行为改为操作臂（更常用）
- 所有现有API保持不变
- 可选参数 `control_aim` 不影响现有代码
- 数据解析器自动识别设备类型

## 测试验证

运行测试验证改进:

```bash
cd /home/ubuntu/robot_ws/src/Alicia_M_SDK
python test_control_aim.py
```

测试覆盖:
- ✅ 默认控制目标为操作臂
- ✅ 动态构建请求帧（示教臂/操作臂）
- ✅ 动态构建控制帧（示教臂/操作臂）
- ✅ 使用默认控制目标
- ✅ 初始化时指定控制目标
- ✅ 功能码正确性验证

## 注意事项

1. **请求关节信息时**必须指定正确的 `control_aim`，否则收到的反馈可能无法正确解析状态位
2. **写入控制指令时**也应指定正确的 `control_aim`，虽然写入反馈只是"成功/失败"，但保持一致性更好
3. 不需要修改请求版本、使能力矩等通用指令，它们不受 `control_aim` 影响
4. `DataParser` 会根据接收到的功能码自动识别设备类型并解析状态位

## 修改文件清单

1. `alicia_m_sdk/hardware/servo_driver.py`
   - 添加 `control_aim` 参数到 `__init__()`
   - 添加 `default_control_aim` 属性（默认 `AIM_OPERATION`）
   - 新增 `_build_joint_request_frame()` 方法
   - 修改 `acquire_info()` 支持 `control_aim` 参数
   - 更新 `set_joint_and_gripper()` 和 `_build_send_joint_frame()` 的默认值
   - 移除硬编码的 "joint" 请求指令

2. `test_control_aim.py` (新增)
   - 完整的测试验证脚本

3. `CONTROL_AIM_IMPROVEMENTS.md` (本文档)
   - 详细的改进说明和使用指南
