# Python SDK SOP

为规范玄雅科技机器人产品Python SDK的开发流程、代码标准与集成规范，统一开发范式、提升开发效率，保障SDK的可扩展性、可维护性、安全性及兼容性，降低开发成本与协作成本，为后续SDK的迭代升级、测试部署及开发者使用提供明确指导，特制定本规范。本规范适用于玄雅科技所有机器人产品相关Python SDK的设计、开发、测试、文档撰写及维护全流程，所有参与SDK开发的相关人员均需严格遵守。

说明：本文是公司级 Python SDK 通用标准。Alicia-M-SDK 当前项目的 API 收口、目录边界和 demo 规则，以 `docs/API_MAINTENANCE.md` 为具体执行准则；当通用目录示例与当前项目实现不完全一致时，不要求强行回退目录结构。

---
## 1 目的
建立统一的 Python SDK 开发标准，使机械臂产品的软件接口具备以下特征：
- 接口稳定，版本可控
- 对外行为一致，便于客户集成
- 代码结构清晰，便于多人协作
- 文档完善，降低客户上手成本
- 支持仿真、真机、测试三种环境
- 能覆盖基础控制、状态读取、异常处理、日志追踪、示例工程等完整能力

---
## 2. 适用范围
本 SOP 适用于公司所有机械臂相关 Python SDK 项目，包括但不限于：
- 单机械臂控制 SDK
- 双臂 / 多臂控制 SDK
- 夹爪 / 末端执行器扩展 SDK
- 遥操作 / 示教 SDK
- 相机、力传感器、IO 模块等外围设备集成 SDK
- 真机驱动层封装、网络通信层封装、上层任务 API 封装

---
### 3. SDK 分层原则
Python SDK 必须采用分层架构，禁止把通信、控制逻辑、业务流程、示例代码混写。
建议至少分为 5 层：
3.1 设备通信层
负责和机械臂控制器通信。
职责：
- TCP / UDP / Serial / CAN / WebSocket 等通信
- 报文编码与解码
- 连接建立、断开、重连
- 心跳管理
- 超时控制
- 线程安全发送与接收
要求：
- 不直接暴露给最终用户
- 与具体机械臂业务逻辑解耦
- 所有底层异常必须标准化上抛

---
3.2 驱动适配层
负责把通信协议映射成设备级操作。
职责：
- 关节读写
- 模式切换
- 使能 / 失能
- 清错
- 回零
- 读取状态字、错误码、传感器值
- 发送位置 / 速度 / 力矩 / 笛卡尔命令
要求：
- 一台机械臂一个 Driver 类
- 一种末端执行器一个独立 Driver
- 协议字段必须统一命名

---
3.3 核心控制层
负责对外提供稳定统一的机械臂控制接口。
职责：
- Robot 类封装
- Arm、Gripper、Tool、IO 等模块抽象
- 同步 / 异步接口
- 阻塞与非阻塞动作
- 轨迹执行
- 安全状态检查
- 参数校验
要求：
- 这是对外 API 的核心层
- 用户尽量只接触这一层
- 所有高频功能必须在这一层统一定义

---
3.4 应用工具层
负责常用功能扩展。
职责：
- 坐标变换工具
- 插值器
- 轨迹文件读取
- 标定工具
- 录制 / 回放工具
- 日志解析工具
- 设备发现工具
要求：
- 工具类与核心控制层解耦
- 不与硬件强耦合时，尽量设计成纯 Python 工具模块

---
3.5 示例与教程层
负责帮助用户快速上手。
职责：
- 最小运行示例
- 典型任务示例
- 故障排查示例
- 仿真示例
- 多线程 / 异步示例
- 相机 / 夹爪 / 力控等组合示例
要求：
- 示例必须可运行
- 示例代码风格必须和 SDK 主体一致
- 示例不能依赖未文档化的内部接口

---
## 4. 项目立项阶段 SOP
4.1 明确 SDK 定位
在开发前，必须先明确：
- 目标用户是谁
例如：算法工程师、集成商、科研用户、自动化工程师、内部应用工程师
- SDK 控制边界是什么
例如：
只做设备控制
还是也包括视觉、标定、示教、录制、仿真
- 与控制器固件的关系
例如：
SDK 是直接透传协议，还是做高层抽象
- 是否兼容多型号机械臂
例如：
6DOF、7DOF、双臂、不同负载版本

---
4.2 输出立项文档
立项阶段至少应当明确以下事项：
1. SDK 功能范围
2. 通信协议与控制接口对照表
3. API 分层设计草案
4. 版本兼容策略
5. 测试与发布时间表

---
## 5. API 设计标准 SOP
5.1 API 设计总原则
5.1.1 用户优先
API 要围绕用户任务设计，而不是围绕底层寄存器设计。
差的设计：
robot.write_reg(0x31, 0x04)
好的设计：
robot.enable()
robot.set_mode("position")

---
5.1.2 命名稳定
同类功能命名必须统一，禁止混乱。
例如统一使用：
- connect()
- disconnect()
- enable()
- disable()
- clear_error()
- get_state()
- move_joints()
- move_linear()
- stop()
- emergency_stop()
不要同时出现：
- power_on() / enable_motor()
- go_home() / back_to_zero() / return_origin()
必须选一种标准叫法。

---
5.1.3 参数语义明确
每个参数必须明确：
- 单位
- 坐标系
- 数据类型
- 合法范围
- 默认值含义
例如：
move_joints(
    positions: list[float],   # rad
    velocity: float = 1.0,    # rad/s
    acceleration: float = 2.0 # rad/s^2
)
不要只写：
move_joints(pos, vel, acc)

---
5.1.4 尽量显式，不要隐式
例如：
差的示例：
robot.move([0, 1, 2, 3, 4, 5])
好的示例：
robot.move_joints(
    positions=[0, 1, 2, 3, 4, 5],
    velocity=1.0,
    blocking=True
)

---
5.1.5 同步与异步分开
阻塞接口和非阻塞接口必须明确区分。
例如：
- move_joints(..., blocking=True)
或
- move_joints() / move_joints_async()
但必须统一。

---
5.2 API 分级规范
建议分三级 API：
L1：设备级 API
面向高级用户或调试人员
例如：
- read_joint_positions()
- read_joint_velocities()
- read_joint_torques()
- write_joint_command()
L2：控制级 API
面向普通用户
例如：
- move_joints()
- move_linear()
- open_gripper()
- close_gripper()
L3：任务级 API
面向应用开发
例如：
- pick()
- place()
- teach_and_replay()
原则：
- SDK 主包重点提供 L1 + L2
- L3 视产品成熟度决定是否纳入
- 若 L3 引入业务逻辑过多，建议放在 examples 或 app 层

---
## 6. 目录结构标准 SOP
推荐目录结构如下：
robot_sdk/
├── pyproject.toml
├── README.md
├── LICENSE
├── CHANGELOG.md
├── .gitignore
├── src/
│   └── robot_sdk/
│       ├── __init__.py
│       ├── version.py
│       ├── exceptions.py
│       ├── logging.py
│       ├── constants.py
│       ├── types.py
│       ├── transport/
│       │   ├── __init__.py
│       │   ├── tcp.py
│       │   ├── serial.py
│       │   └── base.py
│       ├── protocol/
│       │   ├── __init__.py
│       │   ├── encoder.py
│       │   ├── decoder.py
│       │   └── packets.py
│       ├── drivers/
│       │   ├── __init__.py
│       │   ├── arm_driver.py
│       │   ├── gripper_driver.py
│       │   └── io_driver.py
│       ├── core/
│       │   ├── __init__.py
│       │   ├── robot.py
│       │   ├── arm.py
│       │   ├── gripper.py
│       │   ├── safety.py
│       │   └── state.py
│       ├── utils/
│       │   ├── __init__.py
│       │   ├── transforms.py
│       │   ├── interpolation.py
│       │   └── validators.py
│       └── simulation/
│           ├── __init__.py
│           └── mock_robot.py
├── tests/
│   ├── unit/
│   ├── integration/
│   ├── hardware/
│   └── conftest.py
├── examples/
│   ├── 01_connect.py
│   ├── 02_enable_and_home.py
│   ├── 03_move_joints.py
│   ├── 04_move_linear.py
│   ├── 05_gripper_control.py
│   ├── 06_error_recovery.py
│   └── 07_async_control.py
├── docs/
│   ├── api/
│   ├── tutorials/
│   ├── faq/
│   └── images/
└── scripts/
    ├── release.py
    └── check_version.py

---
## 7. 编码规范 SOP
7.1 Python 版本
必须明确支持版本，例如：
- Python 3.10+
- Python 3.11+
不要支持过多历史版本，否则维护成本高。

---
7.2 类型标注
所有公开 API 必须写类型标注。
例如：
def move_joints(
    self,
    positions: list[float],
    velocity: float = 1.0,
    acceleration: float = 2.0,
    blocking: bool = True,
    timeout: float | None = None,
) -> None:
    ...

---
7.3 Docstring 标准
所有公开类、公开函数必须有 docstring。
推荐 Google 风格或 NumPy 风格，统一即可。
示例：
def connect(self, timeout: float = 5.0) -> None:
    """Connect to the robot controller.

    Args:
        timeout: Maximum wait time in seconds.

    Raises:
        ConnectionError: If controller is unreachable.
    """

---
7.4 日志标准
必须统一日志接口，禁止全项目直接 print()。
要求：
- 使用 logging
- 分级：DEBUG / INFO / WARNING / ERROR
- 日志要带时间、模块名、线程名
- 通信收发日志必须支持开关
- 用户敏感信息不得默认输出

---
7.5 代码格式化
统一工具：
- ruff 或 flake8
- black
- isort
- mypy
建议在 CI 强制检查。

---
## 8. 异常处理 SOP
机械臂 SDK 必须建立统一异常体系，禁止直接把原始协议错误或 Python 原生异常无包装地抛给用户。
建议定义：
class SDKError(Exception): ...
class ConnectionError(SDKError): ...
class TimeoutError(SDKError): ...
class ProtocolError(SDKError): ...
class ValidationError(SDKError): ...
class RobotStateError(SDKError): ...
class SafetyError(SDKError): ...
class MotionError(SDKError): ...
class HardwareFaultError(SDKError): ...
原则：
- 用户输入错误 → ValidationError
- 设备未连接 → ConnectionError
- 通信超时 → TimeoutError
- 控制器返回错误码 → 映射为 HardwareFaultError 或 MotionError
- 危险动作拦截 → SafetyError
同时要求：
- 异常信息必须可读
- 必须包含关键上下文
- 不要只报 “Error code = 17”
应该写成：
Joint limit exceeded on joint 3. Commanded=3.40 rad, limit=[-2.80, 2.80] rad.

---
## 9. 状态模型 SOP
SDK 必须提供统一的状态读取模型。
建议用 dataclass：
from dataclasses import dataclass

@dataclass
class JointState:
    position: float
    velocity: float
    torque: float
    temperature: float | None = None

@dataclass
class RobotState:
    connected: bool
    enabled: bool
    mode: str
    error_code: int | None
    joints: list[JointState]
要求：
- 状态对象字段固定
- 命名统一
- 所有单位固定
- 不同型号若有差异，应通过扩展字段或子类处理

---
## 10. 安全控制 SOP
机械臂 SDK 不是普通软件包，必须把安全放在接口层前面。
10.1 必须实现的安全检查
可能包括：
- 连接状态检查
- 使能状态检查
- 错误状态检查
- 工作模式检查
- 关节限位检查
- 速度上限检查
- 加速度上限检查
- 笛卡尔工作空间检查
- 夹爪开合范围检查
- E-stop 状态检查

---
10.2 危险动作的策略
所有动作接口应明确说明：
- 是否阻塞
- 是否可中断
- 是否自动检查安全
- 是否允许覆盖安全检查
- 是否用于调试模式
例如：
robot.move_joints(..., safety_check=True)
但不建议默认允许用户轻易绕过安全。

---
10.3 急停与停止语义区分
必须区分：
- stop()：正常减速停止
- halt()：快速停机
- emergency_stop()：急停
这三者不可混淆。

---
## 11. 线程与并发 SOP
机械臂 SDK 常涉及状态刷新、心跳、运动等待、异步任务，因此必须明确线程模型。
要求：
11.1 明确线程安全策略
每个核心对象必须声明：
- 是否线程安全
- 哪些方法可并发调用
- 哪些方法必须串行调用

---
11.2 通信层必须加锁
如果存在共享 socket / serial 资源，读写必须采用：
- 互斥锁
- 命令序列号
- 响应匹配机制
避免多个线程抢占导致报文错配。

---
11.3 不要隐式创建失控线程
后台线程只能用于：
- 状态轮询
- 心跳
- 异步事件监听
必须支持：
- 正常启动
- 安全退出
- 超时终止
- 异常上报

---
## 12. 仿真与真机一致性 SOP
这是机械臂 SDK 很关键的一条。
12.1 必须支持仿真对象
建议设计统一接口，使仿真类和真机类保持一致：
robot = RealRobot(...)

或

robot = MockRobot(...)
要求：
- 同一套上层 API 尽量复用
- 示例既可跑真机也可跑仿真
- 便于 CI 与离线开发

---
12.2 真机 / 仿真差异要文档化
例如：
- 仿真不支持真实力矩反馈
- 仿真延时不同
- 某些 IO 无法模拟
不能让用户误以为完全等价。

---
## 13. 测试 SOP
机械臂 SDK 必须采用分层测试，不可只靠人工试跑。
13.1 单元测试
覆盖：
- 参数校验
- 协议编解码
- 状态转换
- 异常映射
- 工具函数
要求：
- 快速执行
- 不依赖真机

---
13.2 集成测试
覆盖：
- 通信链路
- 指令下发流程
- 多模块协同
- 模拟设备返回
要求：
- 用 mock server / fake controller 支撑
- 可在 CI 跑

---
13.3 硬件在环测试
覆盖：
- 真机连接
- 上使能
- 回零
- 小范围点动
- 轨迹执行
- 夹爪动作
- 故障恢复
要求：
- 与生产环境隔离
- 严格限速限位
- 记录日志与视频

---
13.4 回归测试
每次发版前必须验证：
- 核心 API 未破坏
- 老示例可运行
- 老脚本不失效
- 旧版本固件兼容性结论明确

---
## 14. 文档 SOP
SDK 的文档质量，几乎直接决定用户是否愿意用。
14.1 必备文档清单
应当包括：
1. 安装指南
2. 快速开始
3. 连接设备教程
4. 基本控制教程
5. 坐标系说明
6. 单位说明
7. 错误码说明
8. API Reference
9. 基本问题的FAQ
10. 版本兼容表
11. 例程列表
12. 故障排查指南

---
14.2 快速开始应当必须满足开机即跑
理想结构：
安装
pip install your_robot_sdk
最小示例
from your_robot_sdk import Robot

robot = Robot(ip="192.168.1.10")
robot.connect()
robot.enable()
print(robot.get_state())
robot.disconnect()
用户第一次使用，必须尽量少读文档就能成功。

---
14.3 API 文档必须自动生成
建议用：
- MkDocs
- Sphinx
- pdoc
公开 API 与源码 docstring 保持一致，避免双份维护。

---
## 15. 示例工程 SOP
示例不是附属品，而是 SDK 的一部分。
15.1 示例分级
建议按难度编号，例如：
- 01_connect.py
- 02_read_state.py
- 03_move_joint.py
- 04_move_linear.py
- 05_gripper.py
- 06_error_handling.py
- 07_async_motion.py
- 08_teach_replay.py
- 09_multi_robot.py
- 10_simulation.py

---
15.2 示例要求
每个示例必须说明：
- 功能目标
- 适用前提
- 风险提示
- 如何运行
- 预期输出

---
## 16. 版本管理 SOP
16.1 版本号规范
例如：
- 1.0.0
- 1.1.0
- 1.1.3

---
16.2 API 兼容策略
必须明确：
- 哪些接口是稳定接口
- 哪些接口是实验接口
- 弃用接口如何过渡
例如：
- 弃用接口至少保留 2 个 minor 版本
- 使用 warning 提示用户迁移

---
16.3 CHANGELOG 管理
发版应当维护 CHANGELOG.md，至少记录：
- 新增功能
- 修复内容
- 兼容性变化
- 弃用接口
- 已知问题

---
## 17. 发布 SOP
17.1 发版前检查清单
发版前必须完成：
- 单元测试通过
- 集成测试通过
- 关键真机测试通过
- 文档更新完成
- 示例可运行
- 版本号已更新
- CHANGELOG 已更新
- 发布 tag 已创建

---
17.2 发布渠道
建议分三级：
- 内部测试版：公司内部 PyPI / 私有源
- 候选版：给合作客户试用
- 正式版：公开或正式客户版本
例如：
- 1.2.0.dev1
- 1.2.0rc1
- 1.2.0

---
17.3 PyPI 包要求
包必须包含：
- 明确依赖
- Python 版本要求
- License
- README
- 项目主页
- issue 跟踪地址

---
## 18. CI/CD SOP
建议接入 GitHub Actions / GitLab CI。
最少自动化项：
- lint
- format check
- type check
- unit test
- package build
- docs build
更进一步：
- mock integration test
- release tag 自动构建
- wheel 自动发布

---
## 19. Git 协作 SOP
19.1 分支规范
建议：
- main：稳定分支
- develop：开发分支
- feature/*
- fix/*
- release/*

---
19.2 提交规范
建议使用 Conventional Commits：
- feat: add async motion api
- fix: handle reconnect timeout
- docs: update quickstart
- refactor: split transport layer

---
19.3 PR 审查要求
每个 PR 至少检查：
- 是否符合 API 命名规范
- 是否有测试
- 是否破坏兼容性
- 是否补充文档
- 是否有日志与异常处理
- 是否影响线程安全

---
## 20. 机械臂 SDK 特有接口建议
对于机械臂产品，我建议你们把 SDK 的标准能力固定成以下模块。
20.1 连接管理
- connect()
- disconnect()
- reconnect()
- is_connected
20.2 电机与状态
- enable()
- disable()
- clear_error()
- get_state()
- get_joint_states()
20.3 模式管理
- set_mode()
- get_mode()
例如支持：
- position
- velocity
- torque
- impedance
- teach
- idle

---
20.4 基础运动
- move_joints()
- move_linear()
- servo_joints()
- servo_cartesian()
- stop()
- home()

---
20.5 末端执行器
- open_gripper()
- close_gripper()
- set_gripper_position()
- get_gripper_state()

---
20.6 坐标与运动学
- get_tcp_pose()
- get_flange_pose()
- set_tcp()
- inverse_kinematics()
- forward_kinematics()

---
20.7 安全与系统
- emergency_stop()
- reset_estop()
- set_speed_limit()
- set_workspace_limit()
- get_faults()

---
20.8 IO 与外设
- read_digital_input()
- write_digital_output()
- read_analog_input()
- set_tool_voltage()

---
## 21. 推荐的对外 API 风格
建议对外主入口尽量简洁：
from robot_sdk import Robot

robot = Robot(ip="192.168.1.10")
robot.connect()
robot.enable()
robot.home()

robot.move_joints(
    positions=[0.0, -0.5, 1.0, 0.0, 1.2, 0.0],
    velocity=1.0,
    acceleration=2.0,
    blocking=True,
)

pose = robot.get_tcp_pose()
print(pose)

robot.open_gripper()
robot.disconnect()
特点：
- 直观
- 可读
- 参数显式
- 适合文档展示
- 适合客户二次开发

---
## 22. 验收标准 SOP
一个机械臂 Python SDK 是否达到可交付标准，可以用下面 10 条验收。
基础验收
1. 能稳定安装
2. 能稳定连接真机
3. 能完成使能、回零、读取状态
4. 能执行基本运动
5. 能正确处理错误与超时
工程验收
6. 目录结构规范
7. 公开 API 有类型标注和文档
8. 单元测试与集成测试齐全
9. 示例完整且可运行
10. 版本与发布流程可追踪

---
## 23. 协议层稳定
作为机器人与机械臂开发公司，应当构建以下稳定性规范：
- 底层控制协议尽量稳定
- 上层 SDK API 可以持续优化
- 协议升级必须考虑旧固件兼容
- API 变更必须控制破坏性
