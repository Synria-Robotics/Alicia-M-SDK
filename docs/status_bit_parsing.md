# 状态位解析功能说明

## 概述

`data_parser.py` 中已实现完整的状态位解析功能，可以根据设备类型（示教臂或操作臂）自动解析反馈数据中的状态位。

## 功能特点

1. **自动识别设备类型**：根据帧中的功能码（frame[2]）自动判断是示教臂还是操作臂
2. **位掩码解析**：使用位运算从一个字节中提取多个状态标志
3. **详细状态描述**：将状态字节转换为易读的文本描述

## 状态位定义

### 示教臂 (0x01) 状态位

| 位位置 | 十六进制 | 二进制 | 常量名称 | 状态描述 |
|--------|----------|--------|----------|----------|
| bit 0  | 0x01     | 00000001 | TEACH_STATUS_LOCKED | 锁定状态 |
| bit 1  | 0x02     | 00000010 | TEACH_STATUS_SYNC | 同步状态 |
| bit 2  | 0x04     | 00000100 | TEACH_STATUS_RESERVED1 | 待定1 |
| bit 3  | 0x08     | 00001000 | TEACH_STATUS_RESERVED2 | 待定2 |
| bit 4  | 0x10     | 00010000 | TEACH_STATUS_RESERVED3 | 待定3 |
| bit 5  | 0x20     | 00100000 | TEACH_STATUS_RESERVED4 | 待定4 |
| bit 6  | 0x40     | 01000000 | TEACH_STATUS_GRIPPER_TORQUE_LOCK | 夹具过高力矩的运动方向锁定 |
| bit 7  | 0x80     | 10000000 | TEACH_STATUS_MOTOR_ERR | 电机 ERR 状态 |

### 操作臂 (0x02) 状态位

| 位位置 | 十六进制 | 二进制 | 常量名称 | 状态描述 |
|--------|----------|--------|----------|----------|
| bit 0  | 0x01     | 00000001 | OPER_STATUS_SINGLE_CLICK | 单击 |
| bit 1  | 0x02     | 00000010 | OPER_STATUS_DOUBLE_CLICK | 双击 |
| bit 2  | 0x04     | 00000100 | OPER_STATUS_LONG_PRESS | 长按 |
| bit 3  | 0x08     | 00001000 | OPER_STATUS_REPEAT_LONG_PRESS | 重复长按 |
| bit 4  | 0x10     | 00010000 | OPER_STATUS_RESERVED1 | 待定1 |
| bit 5  | 0x20     | 00100000 | OPER_STATUS_RESERVED2 | 待定2 |
| bit 6  | 0x40     | 01000000 | OPER_STATUS_GRIPPER_TORQUE_LOCK | 夹具过高力矩的运动方向锁定 |
| bit 7  | 0x80     | 10000000 | OPER_STATUS_MOTOR_ERR | 电机 ERR 状态 |

## 使用方法

### 1. 在数据解析中自动使用

状态位解析已集成在 `_parse_ask_joint_data()` 函数中，会自动根据设备类型解析状态位：

```python
# 在 _parse_ask_joint_data() 中自动调用
device_code = func_id & 0x7F  # 获取设备类型代码
run_status_text = self._parse_run_status(run_status, device_code)
```

### 2. 直接调用解析方法

也可以直接调用 `_parse_run_status()` 方法：

```python
from alicia_m_sdk.hardware.data_parser import DataParser
import threading

# 创建解析器
lock = threading.Lock()
parser = DataParser(lock=lock)

# 解析示教臂状态
status_byte = 0xC1  # 锁定 + 夹具过高力矩锁定 + 电机错误
device_code = 0x01  # 示教臂
result = parser._parse_run_status(status_byte, device_code)
print(result)  # 输出: "locked,gripper_torque_lock,motor_err"

# 解析操作臂状态
status_byte = 0x05  # 单击 + 长按
device_code = 0x02  # 操作臂
result = parser._parse_run_status(status_byte, device_code)
print(result)  # 输出: "single_click,long_press"
```

### 3. 从关节状态获取

通过 `get_joint_state()` 方法获取的关节状态中包含解析后的状态文本：

```python
joint_state = parser.get_joint_state()
if joint_state:
    print(f"运行状态: {joint_state.run_status_text}")
    # 示例输出: "locked,sync" 或 "single_click" 或 "idle"
```

## 状态组合示例

### 示教臂状态组合

```python
# 状态字节 0x03 = 00000011
# bit 0 (0x01) = 锁定状态
# bit 1 (0x02) = 同步状态
# 解析结果: "locked,sync"

# 状态字节 0xC0 = 11000000
# bit 6 (0x40) = 夹具过高力矩锁定
# bit 7 (0x80) = 电机错误
# 解析结果: "gripper_torque_lock,motor_err"
```

### 操作臂状态组合

```python
# 状态字节 0x0F = 00001111
# bit 0 (0x01) = 单击
# bit 1 (0x02) = 双击
# bit 2 (0x04) = 长按
# bit 3 (0x08) = 重复长按
# 解析结果: "single_click,double_click,long_press,repeat_long_press"

# 状态字节 0xC1 = 11000001
# bit 0 (0x01) = 单击
# bit 6 (0x40) = 夹具过高力矩锁定
# bit 7 (0x80) = 电机错误
# 解析结果: "single_click,gripper_torque_lock,motor_err"
```

## 测试脚本

可以运行测试脚本验证状态位解析功能：

```bash
cd /home/ubuntu/robot_ws/src/Alicia_M_SDK
python3 examples/test_status_parsing_standalone.py
```

测试脚本会输出：
1. 所有状态位常量的定义
2. 示教臂各种状态组合的解析结果
3. 操作臂各种状态组合的解析结果

## 技术细节

### 位运算原理

状态位使用位掩码（bitmask）技术，一个字节（8位）可以表示8个独立的状态标志：

```python
# 检查特定位是否置位
if status_byte & 0x01:  # 检查 bit 0
    print("锁定状态")

if status_byte & 0x80:  # 检查 bit 7
    print("电机错误")

# 多个状态可以同时存在
status_byte = 0x03  # 00000011 = bit 0 + bit 1
# 同时包含"锁定"和"同步"两个状态
```

### 帧结构

状态位位于 Ask 模式（0x06 指令）反馈帧中：

```
[AA] [06] [0x] [LEN] [DATA...] [STATUS] [CRC] [FF]
                                  ↑
                            状态位位置
                    （有效数据的最后一字节，校验位前一位）
```

### 设备识别

设备类型从帧的功能码（frame[2]）中提取：

```python
func_id = frame[2]        # 例如: 0x01 或 0x02
device_code = func_id & 0x7F  # 取低7位得到设备类型
```

## 向后兼容性

为保持向后兼容，保留了旧的常量定义：

```python
# 旧常量（映射到示教臂状态，保留兼容）
RUN_STATUS_LOCKED = 0x01
RUN_STATUS_SYNC = 0x02
RUN_STATUS_TORQUE_LOCKED = 0x04
RUN_STATUS_MOTOR_ERR = 0x08
```

建议新代码使用明确的 `TEACH_STATUS_*` 或 `OPER_STATUS_*` 常量。

## 注意事项

1. **状态可叠加**：多个状态标志可以同时为真（位或运算）
2. **空闲状态**：当所有位都为 0 时，返回 "idle"
3. **待定位**：reserved1-4 为预留位，未来可能定义新功能
4. **设备特定**：示教臂和操作臂的状态位含义完全不同，必须根据设备类型解析
5. **自动解析**：`_parse_ask_joint_data()` 已集成自动解析，无需手动调用

## 更新记录

- **2025-12-25**: 实现完整的状态位解析功能
  - 添加示教臂和操作臂的详细状态位定义
  - 实现 `_parse_run_status()` 方法自动根据设备类型解析
  - 更新 `_parse_ask_joint_data()` 使用新的解析方法
  - 添加测试脚本验证功能
