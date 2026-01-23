# 示例代码说明

本目录包含使用 Alicia-M SDK 与机械臂交互的演示脚本，涵盖基本控制、运动学计算、轨迹规划、状态监控和可视化功能。

## 如何运行示例

1. 确保您已完成 [安装指南](installation.md) 中的所有步骤
2. 打开终端或命令提示符
3. 进入 `examples` 目录：
   ```bash
   cd examples
   ```
4. 运行某个示例（以读取版本为例）：
   ```bash
   python3 00_demo_read_version.py --port /dev/ttyUSB0
   ```

---

## 示例脚本列表

### 基础功能示例

| 脚本文件 | 功能说明 | 控制频率 |
|----------|----------|----------|
| `00_demo_read_version.py` | 读取机械臂固件版本信息 | - |
| `01_demo_config_mit_params.py` | 配置MIT模式控制参数（仅需运行一次） | - |
| `02_demo_monitor_arm_status_simple.py` | 实时监控机械臂状态位（锁定、同步、按钮等） | 200Hz |

### 运动控制示例

| 脚本文件 | 功能说明 | 控制频率 |
|----------|----------|----------|
| `04_demo_move_gripper.py` | 控制夹爪开合（开、合、部分开合） | - |
| `05_demo_move_joint.py` | 关节空间运动控制（支持角度/弧度输入） | MIT模式 |
| `06_demo_move_cartesian.py` | 笛卡尔空间多点轨迹规划（支持示教录制） | 优化延迟 |

### 运动学示例

| 脚本文件 | 功能说明 | 控制频率 |
|----------|----------|----------|
| `07_demo_forward_kinematics.py` | 正向运动学：读取关节角度计算末端位姿 | - |
| `08_demo_inverse_kinematics.py` | 逆向运动学：指定末端位姿求解关节角度 | - |

### 高级功能示例

| 脚本文件 | 功能说明 | 控制频率 |
|----------|----------|----------|
| `10_demo_sparkvis.py` | SparkVis 可视化界面双向同步与数据记录 | 200Hz |
| `11_demo_slider_control_joint.py` | 滑动条实时控制关节（GUI界面） | 200Hz |

---

## 推荐学习顺序

### 🔰 初学者路径
1. **`00_demo_read_version.py`**  
   - 学习如何连接机械臂并读取基本信息
   - 了解机器人实例创建和连接流程

2. **`04_demo_move_gripper.py`**  
   - 学习最简单的控制：夹爪开合
   - 理解控制指令的基本结构

3. **`05_demo_move_joint.py`**  
   - 掌握关节空间运动控制
   - 了解MIT模式的使用方法

4. **`07_demo_forward_kinematics.py`**  
   - 学习正向运动学计算
   - 理解关节角度与末端位姿的关系

### 🎯 进阶用户路径
5. **`08_demo_inverse_kinematics.py`**  
   - 掌握逆向运动学求解
   - 学习多起点优化和关节限位设置

6. **`06_demo_move_cartesian.py`**  
   - 学习笛卡尔空间轨迹规划
   - 掌握示教录制和轨迹执行

7. **`02_demo_monitor_arm_status_simple.py`**  
   - 了解机械臂状态位监控
   - 学习示教臂和操作臂的状态区别

### 🚀 专业开发路径
8. **`11_demo_slider_control_joint.py`**  
   - 学习高频实时控制（200Hz）
   - 理解GUI与控制线程的分离

9. **`10_demo_sparkvis.py`**  
   - 学习WebSocket双向同步
   - 掌握数据记录和可视化

10. **`01_demo_config_mit_params.py`**  
    - 了解MIT模式底层配置
    - 学习底层串口通信

---

## 详细功能说明

### 00_demo_read_version.py - 读取固件版本

**功能概述：**
- 连接机械臂并读取固件版本信息
- 自动搜索可用串口（可手动指定）

**使用方法：**
```bash
# 自动搜索串口
python 00_demo_read_version.py

# 指定串口
python 00_demo_read_version.py --port /dev/ttyUSB0

# 指定机械臂版本和夹爪型号
python 00_demo_read_version.py --robot_version v1_1 --gripper_type 100mm
```

**适用场景：**
- 验证机械臂连接是否正常
- 检查固件版本兼容性
- 初次使用前的硬件测试

---

### 01_demo_config_mit_params.py - 配置MIT模式

**功能概述：**
- 配置机械臂MIT模式控制参数
- 设置PID增益（Kp, Kd）
- 初始化后电机会回到零点

**使用方法：**
```bash
# 自动搜索串口
python 01_demo_config_mit_params.py

# 指定串口和重复次数
python 01_demo_config_mit_params.py --port /dev/ttyUSB0 --repeat 5
```

**⚠️ 重要注意事项：**
- 运行此脚本会使电机回到零点，请确保周围无障碍物
- 只需在STM32上电后运行一次即可
- 配置完成后可正常使用MIT模式，无需重复配置
- 除非控制板重新上电，否则参数会保持

**适用场景：**
- 首次使用MIT模式前必须运行
- 需要修改MIT控制参数时
- 控制板重新上电后

---

### 02_demo_monitor_arm_status_simple.py - 状态监控

**功能概述：**
- 实时监控机械臂状态位（200Hz）
- 显示锁定状态、同步状态、按钮事件等
- 支持示教臂和操作臂两种设备类型

**状态位说明：**

**示教臂状态位：**
- `locked`: 锁定状态（是否处于力控锁定）
- `sync`: 同步状态（主从臂是否同步）
- `gripper_torque_lock`: 夹爪力矩锁定
- `motor_err`: 电机错误标志

**操作臂状态位：**
- `single_click`: 单击按钮事件
- `double_click`: 双击按钮事件
- `long_press`: 长按按钮事件
- `repeat_long_press`: 重复长按事件
- `gripper_torque_lock`: 夹爪力矩锁定
- `motor_err`: 电机错误标志

**使用方法：**
```bash
# 实时监控模式（200Hz刷新）
python 02_demo_monitor_arm_status_simple.py

# 程序会自动识别设备类型（示教臂/操作臂）
# 按 Ctrl+C 退出监控
```

**适用场景：**
- 调试按钮功能和交互逻辑
- 监控锁定状态变化
- 检测电机错误和异常状态
- 验证主从臂同步功能

---

### 04_demo_move_gripper.py - 夹爪控制

**功能概述：**
- 控制夹爪开合角度（0-100%）
- 支持完全打开、完全闭合、部分开合
- 优化等待时间（每次动作0.3秒）

**使用方法：**
```bash
# 基本使用
python 04_demo_move_gripper.py --port /dev/ttyUSB0

# 指定夹爪型号
python 04_demo_move_gripper.py --gripper_type 100mm
```

**控制序列：**
1. 读取当前夹爪位置
2. 完全打开（100%）
3. 完全闭合（0%）
4. 部分打开（50%）

**适用场景：**
- 测试夹爪功能
- 抓取物体前的准备动作
- 验证夹爪控制精度

---

### 05_demo_move_joint.py - 关节运动控制

**功能概述：**
- 关节空间运动控制（MIT位置模式）
- 支持角度和弧度输入
- 自动关节角度插值

**使用方法：**
```bash
# 基本使用（移动到指定关节角度）
python 05_demo_move_joint.py --port /dev/ttyUSB0

# 指定运动速度
python 05_demo_move_joint.py --interplotation_speed 2.0
```

**⚠️ MIT模式说明：**
- 使用前需先运行 `01_demo_config_mit_params.py`
- MIT模式没有实时位置反馈
- 如需验证到达位置，建议使用非MIT模式（如PATTERN_PV）

**适用场景：**
- 关节空间路径规划
- 快速移动到预设位置
- 重复性高的运动任务

---

### 06_demo_move_cartesian.py - 笛卡尔轨迹规划

**功能概述：**
- 笛卡尔空间多点轨迹规划
- 支持手动拖拽示教录制路径点
- 自动轨迹插值和逆运动学求解
- 支持轨迹可视化

**使用方法：**
```bash
# 基本使用（拖拽示教模式）
python 06_demo_move_cartesian.py --port /dev/ttyUSB0

# 指定运动参数
python 06_demo_move_cartesian.py \
    --move_duration 3.0 \    # 每段移动时间（秒）
    --num_points 200 \        # 轨迹插值点数
    --ik_method dls \         # 逆运动学方法
    --visualize               # 启用可视化
```

**操作流程：**
1. 机械臂移动到初始位置
2. 手动拖拽机械臂到各个路径点
3. 按提示录制路径点
4. 选择执行模式（连续/逐步）
5. 自动执行轨迹

**逆运动学方法：**
- `dls`: 阻尼最小二乘（推荐，稳定性好）
- `pinv`: 伪逆（速度快）
- `lm`: Levenberg-Marquardt（高精度）

**适用场景：**
- 复杂轨迹示教
- 焊接、喷涂等连续轨迹任务
- 轨迹优化和验证

---

### 07_demo_forward_kinematics.py - 正向运动学

**功能概述：**
- 读取当前关节角度
- 计算末端执行器位姿
- 显示位置、旋转矩阵、欧拉角、四元数

**输出信息：**
- 末端位置（x, y, z）单位：米
- 旋转矩阵（3×3）
- 欧拉角（XYZ顺序，弧度和角度）
- 四元数（xyzw格式）
- 齐次变换矩阵（4×4）

**使用方法：**
```bash
python 07_demo_forward_kinematics.py --port /dev/ttyUSB0
```

**适用场景：**
- 验证运动学模型准确性
- 了解当前末端位姿
- 轨迹规划前的位姿分析

---

### 08_demo_inverse_kinematics.py - 逆向运动学

**功能概述：**
- 指定目标末端位姿（位置+姿态）
- 求解关节角度
- 支持多起点优化
- 可选执行移动

**使用方法：**
```bash
# 基本使用（求解但不移动）
python 08_demo_inverse_kinematics.py --port /dev/ttyUSB0

# 求解并执行移动
python 08_demo_inverse_kinematics.py --execute

# 指定目标位姿（7个值：px py pz qx qy qz qw）
python 08_demo_inverse_kinematics.py \
    --end-pose 0.0 -0.5 0.03 0.61 -0.61 0.35 0.35

# 使用多起点优化（提高成功率）
python 08_demo_inverse_kinematics.py --multi-start 10

# 指定求解方法和参数
python 08_demo_inverse_kinematics.py \
    --method dls \           # 求解方法
    --max-iters 500 \        # 最大迭代次数
    --speed_rad_s 0.0349     # 运动速度（弧度/秒）
```

**求解方法：**
- `dls`: 阻尼最小二乘（推荐，稳定）
- `pinv`: 伪逆（快速）
- `transpose`: 雅可比转置（简单）

**多起点优化：**
- 设置 `--multi-start N` 可尝试N个不同初始值
- 建议范围：5-10
- 可显著提高复杂位姿的求解成功率

**适用场景：**
- 指定末端位置的精确控制
- 抓取定位
- 空间路径规划

---

### 10_demo_sparkvis.py - 可视化同步

**功能概述：**
- WebSocket 双向同步（UI ↔ 机器人）
- 实时机器人状态广播（200Hz）
- 数据记录到CSV文件
- 支持UI控制指令

**系统架构：**
```
SparkVis Web UI (浏览器)
    ↕ WebSocket
SparkVis Backend Server (Python)
    ↕ WebSocket
10_demo_sparkvis.py (机器人桥接)
    ↕ Serial
Real Robot (Alicia-M)
```

**使用方法：**

1. **启动后端服务器：**
   ```bash
   cd SparkVis
   python backend_server.py
   ```

2. **启动Web服务器：**
   ```bash
   cd SparkVis
   python -m http.server 8080
   ```

3. **启动机器人桥接：**
   ```bash
   cd examples
   python 10_demo_sparkvis.py --port /dev/ttyUSB0
   ```

4. **打开浏览器：**
   ```
   http://localhost:8080
   ```

**命令行参数：**
```bash
python 10_demo_sparkvis.py \
    --port /dev/ttyUSB0 \              # 串口
    --host localhost \                 # WebSocket主机
    --websocket-port 8765 \            # WebSocket端口
    --output-file data.csv \           # 数据记录文件
    --enable-robot-sync \              # 启用机器人状态同步
    --robot-sync-rate 200.0 \          # 同步频率（Hz）
    --log-source ui                    # 日志来源（ui/robot/both）
```

**适用场景：**
- 远程监控和控制
- 数据采集和分析
- 多用户协同操作
- 轨迹可视化和调试

---

### 11_demo_slider_control_joint.py - 滑动条控制

**功能概述：**
- GUI滑动条实时控制7个关节（6关节+1夹爪）
- 高频控制循环（200Hz）
- 实时位置反馈显示（20Hz）
- 支持回零和重置功能

**界面功能：**
- 7个滑动条：关节1-6（-180°~180°）+ 夹爪（0-100%）
- 控制按钮：启动/停止控制、回零、重置
- 状态显示：控制频率、当前位置
- 快捷键：Ctrl+C 快速退出

**使用方法：**
```bash
# 基本使用
python 11_demo_slider_control_joint.py --port /dev/ttyUSB0

# 启用调试模式
python 11_demo_slider_control_joint.py --debug
```

**操作流程：**
1. 启动程序，连接机械臂
2. 点击"启动控制"按钮
3. 拖动滑动条实时控制关节
4. 需要时点击"回零位"或"重置滑块"
5. 完成后点击"停止控制"

**性能特点：**
- 控制频率：200Hz（5ms控制周期）
- 显示更新：20Hz（50ms刷新）
- 响应延迟：<10ms
- CPU占用：<30%

**适用场景：**
- 手动关节调试
- 位置微调
- 交互式示教
- 功能测试

---

## 控制模式说明

SDK支持多种控制模式，不同demo使用不同模式：

### MIT模式（MIT_POSITION）
- **使用场景**：需要力控或高动态性能
- **特点**：低延迟、高带宽
- **注意**：需要先运行配置工具
- **示例**：05_demo, 06_demo

### PV模式（PATTERN_PV）
- **使用场景**：位置-速度联合控制
- **特点**：精确、稳定
- **示例**：04_demo, 08_demo, 11_demo

### 其他模式
- `PATTERN_MIT`: MIT基础模式
- `PATTERN_JOINT_ANGLE`: 关节角度模式

---

## 常用参数说明

### 串口参数
- `--port`: 串口设备路径
  - Linux: `/dev/ttyUSB0`, `/dev/ttyACM0`
  - macOS: `/dev/cu.usbserial-*`
  - Windows: `COM3`, `COM4`
  - 留空则自动搜索

- `--baudrate`: 波特率（默认：1000000）

### 机器人参数
- `--robot_version`: 机器人版本（默认：v1_1）
- `--gripper_type`: 夹爪型号（默认：100mm）

### 运动参数
- `--speed_rad_s`: 关节速度（弧度/秒）
- `--interplotation_speed`: 速度因子（0.0-1.0）

---

## 故障排查

### 连接问题
**现象**：无法连接机械臂
**解决方案**：
1. 检查串口权限：
   ```bash
   sudo usermod -aG dialout $USER  # Linux
   sudo chmod 666 /dev/ttyUSB0     # 临时授权
   ```
2. 确认串口设备存在：
   ```bash
   ls /dev/tty*  # Linux/macOS
   ```
3. 检查是否有其他程序占用串口
4. 尝试不同的波特率（1000000或921600）

### MIT模式问题
**现象**：MIT模式报错或无响应
**解决方案**：
1. 先运行 `01_demo_config_mit_params.py`
2. 确保STM32未重新上电
3. 检查周围是否有障碍物（配置时会回零）

### 逆运动学失败
**现象**：IK求解失败或不收敛
**解决方案**：
1. 使用多起点优化：`--multi-start 10`
2. 检查目标位姿是否在工作空间内
3. 尝试不同求解方法（dls/pinv/transpose）
4. 增加最大迭代次数：`--max-iters 1000`

### 控制频率不稳定
**现象**：控制频率波动大或达不到200Hz
**解决方案**：
1. 关闭其他占用CPU的程序
2. 检查串口通信是否稳定
3. 减少不必要的打印输出
4. 使用USB 3.0接口

---

## 高级使用技巧

### 1. 批量执行多个动作
```python
# 在05_demo基础上扩展
positions = [
    [0, 0, 0, 0, 0, 0, 0],
    [45, -45, -60, 30, -30, 15, 50],
    [90, -90, -90, 45, -45, 30, 100],
]

for pos in positions:
    robot.set_joint_target(pos, joint_format='deg', speeds=2.0)
    time.sleep(0.3)  # 优化后的等待时间
```

### 2. 轨迹平滑处理
```python
# 在06_demo中使用更多插值点
planner.execute_trajectory(
    waypoints=waypoints,
    num_points=500,  # 增加插值点数，更平滑
    move_duration=5.0  # 增加时间，更稳定
)
```

### 3. 实时数据记录
```python
# 在10_demo中启用数据记录
python 10_demo_sparkvis.py \
    --output-file experiment_$(date +%Y%m%d_%H%M%S).csv \
    --log-source both  # 记录UI和机器人数据
```

### 4. 自定义控制频率
```python
# 修改11_demo的控制频率
# 在JointSliderController.__init__中：
self.control_frequency = 100  # 降低到100Hz以减少CPU占用
```

---

## 性能优化建议

### 提升控制频率
1. 删除控制循环中的打印语句
2. 使用更快的串口波特率（1000000）
3. 减少不必要的状态查询
4. 使用线程池处理异步任务

### 降低延迟
1. 使用USB 3.0端口
2. 禁用USB自动挂起
3. 提高进程优先级：
   ```bash
   sudo nice -n -20 python 11_demo_slider_control_joint.py
   ```

### 提高稳定性
1. 添加异常处理和重试机制
2. 实现软件限位检查
3. 监控控制循环时序
4. 使用看门狗检测超时

---

## 注意事项

### ⚠️ 安全警告
- 首次运行前请确保机械臂周围无障碍物
- MIT模式配置会使电机回零，注意避让
- 高速运动可能导致碰撞，请从低速开始测试
- 紧急情况下可直接断电或按下急停按钮

### 💡 最佳实践
- 始终在try-finally块中调用`robot.disconnect()`
- 使用参数校验避免超出关节限位
- 定期检查固件版本兼容性
- 保存重要的运动轨迹数据

### 📝 开发建议
- 参考示例代码的错误处理模式
- 使用logger而非print进行日志输出
- 添加命令行参数以提高代码复用性
- 编写单元测试验证关键功能

---

## 参考文档

- [API参考手册](api_reference.md)
- [安装指南](installation.md)
- [MIT模式使用说明](mit_mode_usage.md)
- [状态位解析](status_bit_parsing.md)

---

## 技术支持

如有问题或建议，请联系：
- GitHub Issues: [Alicia-M-SDK Issues](https://github.com/synria-robotics/Alicia-M-SDK/issues)
- Email: support@synria-robotics.com
- 文档更新日期：2026年1月4日
