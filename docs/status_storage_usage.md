# 状态位存储和查询功能说明

## 功能概述

`DataParser` 类现在提供了完整的状态位存储和查询功能。当机械臂发送反馈数据时，状态位会被**自动解析和存储**，您可以随时查询当前状态。

## 核心特性

✅ **自动更新**: 收到机械臂反馈时自动解析和存储状态位  
✅ **线程安全**: 使用锁保护，支持多线程访问  
✅ **类型区分**: 自动识别示教臂和操作臂，存储对应的状态  
✅ **详细状态**: 每个状态位都有独立的布尔值存储  
✅ **实时查询**: 随时查询当前状态，无需等待  

## 状态存储结构

### 示教臂状态 (device_code = 0x01)

```python
{
    "locked": bool,                # 锁定状态
    "sync": bool,                  # 同步状态
    "reserved1": bool,             # 待定1
    "reserved2": bool,             # 待定2
    "reserved3": bool,             # 待定3
    "reserved4": bool,             # 待定4
    "gripper_torque_lock": bool,   # 夹具过高力矩的运动方向锁定
    "motor_err": bool,             # 电机 ERR 状态
}
```

### 操作臂状态 (device_code = 0x02)

```python
{
    "single_click": bool,          # 单击
    "double_click": bool,          # 双击
    "long_press": bool,            # 长按
    "repeat_long_press": bool,     # 重复长按
    "reserved1": bool,             # 待定1
    "reserved2": bool,             # 待定2
    "gripper_torque_lock": bool,   # 夹具过高力矩的运动方向锁定
    "motor_err": bool,             # 电机 ERR 状态
}
```

## API 使用方法

### 1. 获取设备信息

```python
from alicia_m_sdk.api import SynriaRobotAPI

robot = SynriaRobotAPI(port="", baudrate=1000000)
robot.connect()

# 获取设备类型
device_type = robot.data_parser.get_device_type()
print(f"设备类型: {device_type}")  # 输出: "teaching_arm" 或 "operating_arm"

# 获取设备代码
device_code = robot.data_parser.get_device_code()
print(f"设备代码: 0x{device_code:02X}")  # 输出: 0x01 或 0x02
```

### 2. 获取原始状态信息

```python
# 获取原始状态字节
status_byte = robot.data_parser.get_run_status_byte()
print(f"状态字节: 0x{status_byte:02X}")  # 例如: 0x03

# 获取状态文本描述
status_text = robot.data_parser.get_run_status_text()
print(f"状态文本: {status_text}")  # 例如: "locked,sync"
```

### 3. 获取详细状态位

#### 方法A: 根据设备类型获取

```python
# 示教臂状态
teach_status = robot.data_parser.get_teach_status()
print(f"锁定状态: {teach_status['locked']}")
print(f"同步状态: {teach_status['sync']}")
print(f"电机错误: {teach_status['motor_err']}")

# 操作臂状态
oper_status = robot.data_parser.get_oper_status()
print(f"单击: {oper_status['single_click']}")
print(f"双击: {oper_status['double_click']}")
print(f"长按: {oper_status['long_press']}")
```

#### 方法B: 自动获取当前设备的状态

```python
# 根据当前设备类型自动返回对应的状态
current_status = robot.data_parser.get_current_status()

if current_status:
    # 可以直接遍历所有状态位
    for status_name, status_value in current_status.items():
        print(f"{status_name}: {status_value}")
```

### 4. 获取完整状态信息（用于调试）

```python
# 获取所有状态信息
all_info = robot.data_parser.get_all_status_info()

import json
print(json.dumps(all_info, indent=2, ensure_ascii=False))
```

输出示例：
```json
{
  "device_type": "teaching_arm",
  "device_code": "0x01",
  "run_status_byte": "0x03",
  "run_status_text": "locked,sync",
  "teach_status": {
    "locked": true,
    "sync": true,
    "reserved1": false,
    "reserved2": false,
    "reserved3": false,
    "reserved4": false,
    "gripper_torque_lock": false,
    "motor_err": false
  },
  "oper_status": {
    "single_click": false,
    "double_click": false,
    "long_press": false,
    "repeat_long_press": false,
    "reserved1": false,
    "reserved2": false,
    "gripper_torque_lock": false,
    "motor_err": false
  },
  "current_status": {
    "locked": true,
    "sync": true,
    ...
  }
}
```

## 实际应用示例

### 示例1: 检查机械臂是否锁定

```python
def is_arm_locked(robot):
    """检查机械臂是否处于锁定状态"""
    device_code = robot.data_parser.get_device_code()
    
    if device_code == 0x01:  # 示教臂
        status = robot.data_parser.get_teach_status()
        return status['locked']
    elif device_code == 0x02:  # 操作臂
        # 操作臂没有锁定状态，返回 False
        return False
    
    return False

# 使用
if is_arm_locked(robot):
    print("机械臂已锁定，无法移动")
else:
    print("机械臂未锁定，可以移动")
```

### 示例2: 检测操作臂按键

```python
def check_button_press(robot):
    """检测操作臂的按键操作"""
    device_code = robot.data_parser.get_device_code()
    
    if device_code == 0x02:  # 操作臂
        status = robot.data_parser.get_oper_status()
        
        if status['single_click']:
            print("检测到单击")
            return "single_click"
        elif status['double_click']:
            print("检测到双击")
            return "double_click"
        elif status['long_press']:
            print("检测到长按")
            return "long_press"
        elif status['repeat_long_press']:
            print("检测到重复长按")
            return "repeat_long_press"
    
    return None

# 使用
while True:
    robot.acquire_info("joint", wait=True, timeout=1.0)
    button = check_button_press(robot)
    
    if button == "single_click":
        # 执行单击操作
        pass
    elif button == "double_click":
        # 执行双击操作
        pass
```

### 示例3: 监控电机错误

```python
def monitor_motor_error(robot):
    """持续监控电机错误状态"""
    while True:
        # 获取当前状态
        current_status = robot.data_parser.get_current_status()
        
        if current_status and current_status['motor_err']:
            print("⚠️  警告: 检测到电机错误!")
            # 执行错误处理逻辑
            break
        
        time.sleep(0.5)

# 使用（在后台线程中运行）
import threading
error_monitor = threading.Thread(target=monitor_motor_error, args=(robot,), daemon=True)
error_monitor.start()
```

### 示例4: 状态变化触发器

```python
class StatusMonitor:
    """状态变化监控器"""
    
    def __init__(self, robot):
        self.robot = robot
        self.last_status = None
    
    def check_status_change(self):
        """检查状态是否发生变化"""
        current = self.robot.data_parser.get_current_status()
        
        if current != self.last_status:
            # 状态发生变化
            self.on_status_changed(self.last_status, current)
            self.last_status = current.copy()
        
        return current
    
    def on_status_changed(self, old_status, new_status):
        """状态变化回调"""
        if old_status is None:
            print("初始状态:", new_status)
            return
        
        # 检测哪些状态位发生了变化
        for key in new_status:
            if new_status[key] != old_status.get(key, False):
                if new_status[key]:
                    print(f"✓ {key} 变为 True")
                else:
                    print(f"✗ {key} 变为 False")

# 使用
monitor = StatusMonitor(robot)

while True:
    robot.acquire_info("joint", wait=True, timeout=1.0)
    monitor.check_status_change()
    time.sleep(0.1)
```

## 测试脚本

### 1. 独立测试（不需要硬件）

```bash
python3 examples/test_status_storage_standalone.py
```

这个脚本会模拟各种状态字节，测试存储和打印功能。

### 2. 实时监控（需要连接机械臂）

```bash
python3 examples/monitor_arm_status.py
```

这个脚本会实时显示机械臂的状态位变化。

## 状态更新时机

状态位会在以下情况下自动更新：

1. **收到 Ask 模式反馈** (`_parse_ask_joint_data`)
   - 调用 `acquire_info("joint")` 后
   - 后台线程自动查询时

2. **状态更新是实时的**
   - 每次收到机械臂反馈，状态立即更新
   - 无需手动刷新或等待

## 线程安全说明

所有状态查询方法都是**线程安全**的：

```python
# 可以在多个线程中安全调用
thread1 = threading.Thread(target=lambda: print(robot.data_parser.get_teach_status()))
thread2 = threading.Thread(target=lambda: print(robot.data_parser.get_oper_status()))

thread1.start()
thread2.start()
```

## 注意事项

1. **初始状态**: 在首次收到反馈前，所有状态位默认为 `False`

2. **设备类型**: 必须先收到机械臂反馈才能确定设备类型
   ```python
   # 确保先请求一次数据
   robot.acquire_info("joint", wait=True)
   
   # 然后才能获取设备类型
   device_type = robot.data_parser.get_device_type()
   ```

3. **状态持久性**: 状态会一直保持最后收到的值，直到收到新的反馈

4. **None 返回值**: 如果尚未收到任何反馈，某些方法可能返回 `None`

## 常见问题

### Q: 如何判断状态是否已更新？

A: 检查设备类型是否已知：
```python
if robot.data_parser.get_device_type() is not None:
    # 已接收到反馈，状态有效
    status = robot.data_parser.get_current_status()
```

### Q: 状态位多久更新一次？

A: 取决于：
- 手动调用 `acquire_info()` 的频率
- 后台线程的更新间隔（默认 0.5 秒）

### Q: 如何清除状态？

A: 断开并重新连接：
```python
robot.disconnect()
robot.connect()
# 状态会重置为初始值
```

## 相关文档

- [status_bit_parsing.md](./status_bit_parsing.md) - 状态位解析详细说明
- [api_reference.md](./api_reference.md) - API 完整参考

## 更新记录

- **2025-12-25**: 
  - 添加状态位存储功能
  - 添加状态查询 API
  - 实现实时自动更新
  - 添加测试脚本和文档
