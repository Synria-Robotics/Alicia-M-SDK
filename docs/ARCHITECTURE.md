# Alicia-M-SDK v1.0.0 重构架构设计

> 本文档描述 Alicia-M-SDK 的完整重构方案，目标是构建一个**规范、优雅、易用、可扩展**的机器人 SDK。

---

## 一、当前问题总结

### 1.1 架构层面
- **API 层过于臃肿**：`SynriaRobotAPI` 单文件 1100+ 行，承担了运动控制、状态查询、轨迹规划、IK 求解等全部职责，违反单一职责原则
- **协议层缺失**：协议编解码逻辑分散在 `servo_driver.py`、`data_parser.py`、`serial_comm.py` 多个文件中，没有统一的协议抽象层
- **消息定义混乱**：没有独立的消息类型定义，协议字段的魔数（magic number）硬编码在各处
- **通信管理过度设计**：`CommunicationManager` 引入了不必要的优先级队列机制，对简单串口协议来说是过度抽象
- **执行层冗余**：`HardwareExecutor` 和 `TrajectoryExecutor` 职责高度重叠

### 1.2 实现层面
- CRC 校验依赖两个冗余库（`pythoncrc` + `pycrc`），且计算逻辑分散
- 控制模式（PV/MIT）的数据编码逻辑与驱动层耦合
- 方向映射、单位转换等逻辑散布各处，缺乏统一管理
- 状态位解析逻辑不清晰，教学臂/操作臂的状态差异处理混乱

### 1.3 工程层面
- 缺乏类型注解和数据类定义
- 注释语言不统一（中英混杂）
- 配置硬编码，缺少集中管理
- 无异常体系，错误处理不规范

---

## 二、设计目标

| 目标 | 描述 |
|------|------|
| **规范** | 清晰的分层架构，每层职责明确，单向依赖 |
| **优雅** | 统一的接口风格，一致的命名规范，简洁的内部实现 |
| **易用** | 保留原有 API 方法名，提供直观的工厂函数和配置方式 |
| **可扩展** | 协议层可适配不同机型，执行器可插拔，支持新增控制模式 |
| **中文注释** | 全部注释和文档使用中文，公开 API 使用 Google 风格 docstring |
| **异常规范** | 统一异常体系，异常消息可读、含上下文，禁止裸抛原生异常 |

---

## 三、项目目录结构

参照 Alicia-D-SDK v6.1.0 的目录风格，结合通信协议特点重新设计：

```
alicia_m_sdk/
├── __init__.py                     # 包入口：导出 create_robot()、核心类型、RoboCore 接口
│
├── api/                            # 【用户 API 层】面向用户的高层接口
│   ├── __init__.py
│   └── synria_robot_api.py         # SynriaRobotAPI 主类（精简版，委托各子模块）
│
├── protocol/                       # 【协议层】通信协议的编解码（新增，核心改动）
│   ├── __init__.py
│   ├── constants.py                # 协议常量：帧头帧尾、指令ID、功能码、地址表、数据映射范围
│   ├── frame.py                    # 帧结构：Frame 数据类、帧组装/解析、CRC32 校验
│   ├── messages.py                 # 消息定义：各指令ID对应的请求/响应消息数据类
│   └── codec.py                    # 编解码器：消息对象 <-> 二进制帧的双向转换
│
├── hardware/                       # 【硬件层】串口通信与状态管理
│   ├── __init__.py
│   ├── serial_port.py              # 串口驱动：连接、读写、端口发现、线程安全
│   └── device.py                   # 设备抽象：读取线程+轮询线程、StateCache、指令调度
│
├── control/                        # 【控制层】运动控制与轨迹执行（新增，从 API 层分离）
│   ├── __init__.py
│   ├── joint_control.py            # 关节控制：关节运动、夹爪控制、回零、等待完成
│   ├── trajectory_executor.py      # 轨迹执行器：关节空间/笛卡尔空间轨迹的硬件回放
│   └── teleoperation.py            # 遥操作：主从跟随控制
│
├── types/                          # 【类型定义层】全局数据结构（新增）
│   ├── __init__.py
│   ├── state.py                    # 状态类型：JointState、GripperState、RobotStatus 等
│   ├── config.py                   # 配置类型：RobotConfig、连接参数、控制参数
│   ├── enums.py                    # 枚举定义：ControlAim、ControlMode、ErrorCode 等
│   └── exceptions.py              # 异常体系：SDK 统一异常类层级
│
├── utils/                          # 【工具层】通用工具函数
│   ├── __init__.py
│   ├── conversion.py               # 单位转换：角度、速度、力矩的物理量与协议值互转
│   ├── validation.py               # 参数校验：关节限位检查、输入合法性验证
│   ├── timing.py                   # 时间工具：高精度休眠、FPS 统计
│   └── logger.py                   # 日志系统：统一格式化日志
│
└── integrations/                   # 【集成层】外部库适配
    └── robocore/                   # RoboCore FK/IK/Jacobian/trajectory planning 适配
```

```
examples/                               # 示例脚本
├── 00_demo_read_version.py             # 读取固件版本号
├── 01_demo_diagnostic.py               # 自检功能（底层待更新）
├── 02_demo_read_status.py              # 读取机械臂模式/使能状态（底层待更新）
├── 03_demo_read_states.py              # 读关节角度、夹爪、速度、力矩
├── 04_demo_switch_mode.py              # 切换 PV / MIT 模式
├── 05_demo_disable_enable.py           # 失能/使能交互流程
├── 06_demo_move_gripper.py             # 夹爪控制（PV + MIT）
├── 07_demo_move_joint.py               # 关节控制（PV + MIT）
├── 08_demo_move_full_arm.py            # 关节+夹爪协同控制（PV + MIT）
├── 09_demo_move_joint_mit.py           # 关节控制（MIT）
├── 10_demo_move_full_arm_mit.py        # 关节+夹爪协同控制（MIT）
├── 11_demo_forward_kinematics.py       # 正运动学
├── 12_demo_inverse_kinematics.py       # 逆运动学
├── 13_demo_reset_zero.py               # 零位标定
├── 14_demo_teleop_mapped.py            # 遥操作 + URDF 限位映射
├── 15_demo_gripper_params.py           # 夹爪夹持参数读写
├── 16_demo_joint_traj.py               # 关节空间轨迹
├── 17_demo_mit_torque_switch.py        # MIT 力矩开关
└── 18_demo_user_settings.py            # 个性化设置
```

---

## 四、分层架构设计

### 4.1 整体架构图

```
┌──────────────────────────────────────────────────────────────────┐
│                        用户代码 (User Code)                       │
└───────────────────────────────┬──────────────────────────────────┘
                                │
┌───────────────────────────────▼──────────────────────────────────┐
│                     API 层 (SynriaRobotAPI)                      │
│  面向用户的统一接口，委托各子模块执行具体逻辑                          │
│  方法：connect, get_robot_state, set_robot_state, set_pose ...   │
└──┬──────────────┬──────────────┬──────────────┬─────────────────┘
   │              │              │              │
   ▼              ▼              ▼              ▼
┌──────┐   ┌───────────┐  ┌──────────┐  ┌───────────┐
│控制层 │   │运动学接口   │  │规划接口   │  │类型定义层  │
│      │   │           │  │          │  │           │
│关节控制│   │FK/IK/     │  │B-Spline  │  │JointState │
│轨迹执行│   │Jacobian   │  │多段规划    │  │Config     │
│示教模式│   │(RoboCore) │  │笛卡尔规划  │  │Enums      │
└──┬───┘   └───────────┘  └──────────┘  └───────────┘
   │
   ▼
┌──────────────────────────────────────────────────────────────────┐
│                      硬件层 (Hardware)                            │
│  Device: 读取线程 + 状态轮询线程、StateCache、指令调度                │
│  SerialPort: 串口连接、线程安全读写、端口发现                        │
└───────────────────────────────┬──────────────────────────────────┘
                                │
┌───────────────────────────────▼──────────────────────────────────┐
│                       协议层 (Protocol)                           │
│  Constants: 帧头帧尾、指令ID、地址表                                │
│  Frame: 帧结构定义、组装、解析、CRC32                               │
│  Messages: 各指令的请求/响应消息数据类                               │
│  Codec: 消息对象 <-> 二进制帧 双向编解码                             │
└───────────────────────────────┬──────────────────────────────────┘
                                │
                                ▼
                     物理串口 (USB / RS-485)
```

### 4.2 依赖方向（严格单向）

```
API 层 → 控制层 → 硬件层 → 协议层
  │         │
  ├→ 运动学接口（RoboCore）
  ├→ 规划接口（RoboCore）
  └→ 类型定义层 ← 所有层共享（只定义数据结构，无业务逻辑）
```

**原则**：上层依赖下层，下层不得引用上层。类型定义层被所有层共享，但自身无业务依赖。

---

## 五、各层详细设计

### 5.1 协议层 (`protocol/`)

> 这是本次重构最核心的改动。当前代码中协议逻辑分散在 `servo_driver.py`、`data_parser.py`、`serial_comm.py` 中，重构后集中到独立的协议层。

#### 5.1.1 `constants.py` — 协议常量

集中定义所有协议相关的常量，消除魔数：

```python
# === 帧结构 ===
FRAME_HEADER = 0xAA
FRAME_FOOTER = 0xFF
PLACEHOLDER = 0xFE            # 占位符

# === 指令 ID ===
CMD_VERSION        = 0x01     # 版本与设备信息获取
CMD_ZERO_RESET     = 0x03     # 位姿重置（设置当前位姿为初始位姿）
CMD_TORQUE         = 0x05     # 肢体力矩控制（关节力矩开关）
CMD_JOINT_STATE    = 0x06     # 关节与夹具状态控制
CMD_ENABLE         = 0x09     # 机器人部位失能/使能
CMD_MOTOR_PARAM    = 0x11     # 关节控制幅值和驱动参数设置
CMD_ERROR          = 0xEE     # 错误反馈

# === 功能码：bit7 表示读写方向 ===
FUNC_READ_BIT      = 0x00     # bit7=0 → 读取
FUNC_WRITE_BIT     = 0x80     # bit7=1 → 写入

# === 功能码：部位标识（低7位）===
AIM_LEADER         = 0x01     # bit0=1 → 示教臂
AIM_FOLLOWER       = 0x02     # bit1=1 → 操作臂

# === 0x06 指令：自定义数据地址表 ===
ADDR_POSITION      = 0x00     # 当前位置/期望位置  16bit  rad
ADDR_VELOCITY      = 0x01     # 当前速度/期望速度  12bit  rad/s
ADDR_TORQUE        = 0x02     # 当前力矩/额外力矩  12bit  N/m
ADDR_KP            = 0x03     # 位置环 kp          16bit  0~500
ADDR_KD            = 0x04     # 速度环 kd          16bit  0~5
ADDR_LINEAR_VEL    = 0x05     # 线性轨迹速度       12bit  rad/s

# === 0x11 指令：电机参数地址表 ===
MOTOR_PARAM_ACCEL       = 0x05   # 加速度 (float)
MOTOR_PARAM_DECEL       = 0x06   # 减速度 (float)
MOTOR_PARAM_CTRL_MODE   = 0x0B   # 控制模式 (0x01=MIT, 0x02=位置速度)
MOTOR_PARAM_POS_RANGE   = 0x16   # 位置映射范围 (float)
MOTOR_PARAM_VEL_RANGE   = 0x17   # 速度映射范围 (float)
MOTOR_PARAM_TOR_RANGE   = 0x18   # 扭矩映射范围 (float)
# ... 等

# === 0xEE 错误类型 ===
ERR_FRAME_HEADER    = 0x00    # 帧头/帧尾校验错误
ERR_LENGTH_MISMATCH = 0x01    # 数据长度校验错误
ERR_CRC_FAILED      = 0x02    # CRC32 校验不通过
ERR_SYSTEM_MODE     = 0x03    # 系统模式错误（机械臂类型检测失败）
ERR_ANGLE_LIMIT     = 0x04    # 电机角度限位中
ERR_MOTOR_OFFSET    = 0x05    # motorData 偏移与部位数量不符
ERR_ADDR_OVERFLOW   = 0x06    # insID 偏移超过最大地址

# === 数据映射范围（默认值）===
POS_BITS = 16                 # 位置数据位数
VEL_BITS = 12                 # 速度数据位数
TOR_BITS = 12                 # 力矩数据位数
DEFAULT_POS_RANGE = 12.5      # 位置映射范围 ±12.5 rad
DEFAULT_VEL_RANGE = 10.0      # 速度映射范围 ±10.0 rad/s
TOR_RANGE_LARGE   = 28.0     # 大关节力矩映射范围 ±28.0 N·m (M0~M2)
TOR_RANGE_SMALL   = 10.0     # 小关节力矩映射范围 ±10.0 N·m (M3~M6，含夹爪)
KP_RANGE          = 500.0    # Kp 映射范围 [0, 500]
KD_RANGE          = 5.0      # Kd 映射范围 [0, 5]
KP_BITS           = 16       # Kp 数据位数
KD_BITS           = 16       # Kd 数据位数

# === MIT 默认增益 ===
DEFAULT_KP_LARGE  = 150.0    # 大关节默认 Kp (M0~M2)
DEFAULT_KD_LARGE  = 2.0      # 大关节默认 Kd (M0~M2)
DEFAULT_KP_SMALL  = 20.0     # 小关节默认 Kp (M3~M6)
DEFAULT_KD_SMALL  = 1.0      # 小关节默认 Kd (M3~M6)

# === 电机/关节数量 ===
NUM_JOINTS = 6                # 云擎关节数量（不含夹爪）
NUM_MOTORS = 7                # 电机总数（含夹爪）
```

#### 5.1.2 `frame.py` — 帧结构与 CRC

```python
@dataclass
class Frame:
    """通信协议帧结构"""
    cmd_id: int               # 指令 ID
    func_code: int            # 功能码
    data: bytes               # 有效数据
    # 以下字段在解析时自动填充
    checksum: int = 0         # CRC32 校验位（低8位）

    def encode(self) -> bytes:
        """将 Frame 组装为完整的二进制帧（含帧头、校验位、帧尾）"""
        ...

    @classmethod
    def decode(cls, raw: bytes) -> 'Frame':
        """从原始字节流中解析出一个 Frame（含 CRC 校验验证）"""
        ...

def crc32_check(data: bytes) -> int:
    """CRC32 校验，取低 8 位"""
    ...
```

#### 5.1.3 `messages.py` — 消息数据类

为每个指令 ID 定义清晰的请求/响应消息类：

```python
# === 0x01 版本信息 ===
@dataclass
class VersionRequest:
    """查询版本信息的请求"""
    pass  # 无额外参数

@dataclass
class VersionResponse:
    """版本信息响应"""
    serial_number: str        # 16字节 ASCII 唯一序列号
    hardware_version: str     # 4字节硬件版本（如 "v1.0.0"）
    firmware_version: str     # 4字节固件版本（如 "v1.1.0"）

# === 0x06 关节状态 ===
@dataclass
class JointStateRequest:
    """查询关节状态的请求"""
    aim: int                  # 目标部位（AIM_LEADER / AIM_FOLLOWER）
    start_addr: int           # 起始数据地址
    addr_count: int           # 偏移数量（读取几个地址的数据）

@dataclass
class JointStateResponse:
    """关节状态响应"""
    start_addr: int
    addr_count: int
    motor_data: List[List[int]]   # [motor_count][addr_count] 原始数据
    run_status: int               # 运行状态字节

@dataclass
class JointControlRequest:
    """关节控制请求"""
    aim: int
    start_addr: int
    addr_count: int
    motor_data: List[List[int]]   # [motor_count][addr_count] 目标数据

# === 0x05 力矩控制 ===
@dataclass
class TorqueRequest:
    """力矩开关请求"""
    aim: int                  # 目标部位
    start_joint: int          # 起始关节 ID
    joint_count: int          # 关节数量（正方向偏移）

# === 0x03 位姿重置 ===
@dataclass
class ZeroResetRequest:
    """设置当前位姿为零位"""
    aim: int
    start_joint: int
    joint_count: int

# ... 其余指令类似
```

#### 5.1.4 `codec.py` — 编解码器

```python
class MessageCodec:
    """消息编解码器：消息对象 <-> Frame 的双向转换"""

    def encode_version_request(self) -> Frame: ...
    def decode_version_response(self, frame: Frame) -> VersionResponse: ...

    def encode_joint_state_request(self, msg: JointStateRequest) -> Frame: ...
    def decode_joint_state_response(self, frame: Frame) -> JointStateResponse: ...

    def encode_joint_control(self, msg: JointControlRequest) -> Frame: ...

    def encode_torque_request(self, msg: TorqueRequest) -> Frame: ...
    def encode_zero_reset(self, msg: ZeroResetRequest) -> Frame: ...
    # ... 其余指令
```

**设计要点**：
- 编解码器是**无状态**的纯函数集合，方便单元测试
- 所有物理量到协议值的映射（如 rad → 16bit 整数）在此处完成，调用 `utils/conversion.py`
- 错误帧（0xEE）的解析也在此处统一处理

---

### 5.2 硬件层 (`hardware/`)

#### 5.2.1 `serial_port.py` — 串口驱动

纯粹的串口 I/O 封装，不涉及任何协议逻辑：

```python
class SerialPort:
    """串口通信驱动

    串口初始化时设置 read_timeout，保证 read 操作不会无限阻塞。
    这是读线程可安全退出的关键前提。
    """

    # 串口读超时（秒）— 决定读线程最大响应延迟
    READ_TIMEOUT = 0.01   # 10ms，平衡灵敏度与 CPU 占用

    def __init__(self, port: str = "", baudrate: int = 1_000_000):
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=self.READ_TIMEOUT,        # 读超时
            write_timeout=0.1,                # 写超时
        )

    def connect(self) -> bool:
        """连接串口（支持自动发现端口）"""

    def disconnect(self) -> None:
        """断开串口连接"""

    def is_connected(self) -> bool:
        """检查连接状态"""

    def write(self, data: bytes) -> None:
        """线程安全地写入数据（由 Device.send_frame 的 write_lock 保护）"""

    def read_frame(self) -> Optional[bytes]:
        """读取一帧完整数据（基于长度的帧解析，非边界检测）

        解析策略：
        1. 扫描帧头 0xAA（受 read_timeout 约束，无数据时返回 None）
        2. 读取 cmd_id(1B) + func_code(1B) + length(1B)
        3. 按 length 读取有效数据
        4. 读取 checksum(1B) + footer(1B)
        5. 验证 footer == 0xFF

        不依赖 0xFF 做帧边界（数据区可能包含 0xFF）。

        Returns:
            完整帧的原始字节，无数据或超时返回 None（不抛异常）。
        """

    @staticmethod
    def find_ports() -> List[str]:
        """发现可用的串口设备（跨平台）"""
```

#### 5.2.2 `device.py` — 设备抽象

设备层是硬件层的核心。**核心设计原则：写入不阻塞、读取独立线程、状态异步更新**。

> **旧代码的致命问题**：读写共享单线程 + 同步等待响应 → 通信频率可能低至 0-40Hz。
> 理论上串口 1Mbaud 可支持 1000Hz+。必须从架构层面消除阻塞。
>
> **参考 piper_sdk**：控制命令 fire-and-forget（不等响应），读线程持续解析，状态查询在读线程中穿插。

```python
class Device:
    """机器人设备抽象：非阻塞通信、异步状态更新

    线程模型:
    - 写入路径: 调用方线程直接写串口（仅 write_lock 保护）
    - 读取线程: 专用后台线程持续读帧、解析响应、更新状态缓存（纯读取，不写串口）
    - 状态轮询线程: 专用后台线程周期性发送状态查询帧，保证空闲时缓存不过期
    """

    def __init__(self, serial_port: SerialPort, codec: MessageCodec):
        self._port = serial_port
        self._codec = codec
        self._state_cache = StateCache()
        self._write_lock = threading.Lock()    # 仅保护串口写入
        self._stop_event = threading.Event()
        self._read_thread: Optional[Thread] = None
        self._poll_thread: Optional[Thread] = None
        self._last_write_time: float = 0.0     # 轮询线程退避依据
        self._aim: int = AIM_FOLLOWER          # 默认操作臂，connect() 时自动检测更新

    def set_aim(self, aim: int) -> None:
        """设置控制目标部位（connect 自动检测后调用）"""
        self._aim = aim

    # --- 生命周期 ---
    def start(self) -> None:
        """启动读取线程 + 状态轮询线程"""
        self._stop_event.clear()
        self._read_thread = Thread(target=self._read_loop, daemon=True)
        self._poll_thread = Thread(target=self._poll_loop, daemon=True)
        self._read_thread.start()
        self._poll_thread.start()

    def stop(self) -> None:
        """停止所有后台线程（串口 read_timeout 保证线程可退出）"""
        self._stop_event.set()
        if self._read_thread:
            self._read_thread.join(timeout=2.0)   # 最多等 2 秒
        if self._poll_thread:
            self._poll_thread.join(timeout=1.0)

    # --- 写入路径（非阻塞）---
    def send_frame(self, frame: Frame) -> None:
        """发送原始帧（fire-and-forget，不等待响应）"""
        with self._write_lock:
            self._port.write(frame.encode())
            self._last_write_time = time.perf_counter()

    # --- 高层发送接口（控制层调用，封装协议细节）---
    def send_pv(self, aim: int, positions: List[float],
                velocities: List[float]) -> None:
        """发送 PV 控制帧（pos+vel，fire-and-forget）"""
        msg = JointControlRequest(aim, start_addr=0x00, addr_count=2, ...)
        self.send_frame(self._codec.encode_joint_control(msg))

    def send_mit(self, aim: int, params: List[MitParams]) -> None:
        """发送 MIT 全参数帧（pos+vel+torque+kp+kd，fire-and-forget）"""
        msg = JointControlRequest(aim, start_addr=0x00, addr_count=6, ...)
        self.send_frame(self._codec.encode_joint_control(msg))

    def send_linear_velocity(self, aim: int, velocities: List[float]) -> None:
        """发送线性轨迹速度帧（addr=0x05）"""
        msg = JointControlRequest(aim, start_addr=0x05, addr_count=1, ...)
        self.send_frame(self._codec.encode_joint_control(msg))

    def send_and_wait(self, frame: Frame, expected_cmd: int,
                      timeout: float = 1.0) -> Optional[Frame]:
        """发送请求帧并等待响应（仅用于低频操作：版本查询、模式切换等）

        实现:
        1. 注册 Event: self._state_cache.register_pending(expected_cmd)
        2. 发送帧: self.send_frame(frame)
        3. 等待 Event: event.wait(max(timeout, 0))  # 保证 timeout ≥ 0
        4. 读线程收到匹配响应 → set Event → 返回响应帧

        线程安全: Event 注册/触发由 StateCache 内部保护，不阻塞串口 I/O。
        超时: 返回 None，由调用方决定是否重试。
        """

    # --- 状态访问（读缓存，无 I/O）---
    @property
    def joint_state(self) -> JointState:
        """获取最新关节状态（从缓存原子读取，无阻塞）"""
        return self._state_cache.get_joint_state()

    @property
    def robot_status(self) -> RobotStatus:
        """获取最新运行状态"""
        return self._state_cache.get_robot_status()

    # --- 读取线程（内部）---
    def _read_loop(self) -> None:
        """读取线程主循环

        职责: 持续读取串口帧 → 解析 → 更新状态缓存。
        绝不写串口，避免与写入路径竞争。

        状态来源：
        - 轮询线程周期性发送 0x06 读取帧，固件返回完整状态数据
        - 控制帧（0x06 写入）的响应仅含 result 字节（成功/失败），不含状态
        - 因此状态更新**完全依赖轮询线程的查询**

        阻塞防护:
        - SerialPort 初始化时设置 read_timeout（见 serial_port.py）
        - read_frame() 在无数据时最多阻塞 read_timeout 后返回 None
        - 每次循环检查 _stop_event，保证线程可在 read_timeout 内退出
        """
        while not self._stop_event.is_set():
            raw = self._port.read_frame()    # 最多阻塞 read_timeout
            if raw:
                try:
                    parsed = Frame.decode(raw)
                    self._dispatch_response(parsed)
                except ProtocolError:
                    pass  # 丢弃损坏帧，不中断循环

    # --- 状态轮询线程（内部）---
    def _poll_loop(self) -> None:
        """状态轮询线程：周期性发送 0x06 读取帧获取关节状态

        这是状态缓存的**唯一数据来源**（控制帧写入响应仅含 result，不含状态）。
        退避机制: 最近有写入时短暂跳过，避免与高频控制竞争带宽，
        但冷却后立即恢复查询，确保控制期间状态缓存不过期。

        参考 Alicia-D-SDK: 设备端主动推送 + 1ms 轮询间隔。
        由于云擎固件不支持自动推送，改由 SDK 侧周期性查询代替。
        """
        POLL_INTERVAL = 0.005       # 5ms → 200Hz 轮询（空闲时）
        WRITE_COOLDOWN = 0.003      # 最近 3ms 内有写入则跳过本轮

        while not self._stop_event.is_set():
            # 退避: 控制帧正在高频发送时，短暂跳过避免竞争带宽
            elapsed = time.perf_counter() - self._last_write_time
            if elapsed > WRITE_COOLDOWN:
                query = self._codec.encode_joint_state_request(
                    JointStateRequest(aim=self._aim, start_addr=0x00, addr_count=3)
                )
                self.send_frame(query)

            self._stop_event.wait(POLL_INTERVAL)   # 可被 stop() 打断

    # --- 内部分发 ---
    def _dispatch_response(self, frame: Frame) -> None:
        """根据指令 ID 分发响应到对应处理器"""
        # 更新状态缓存（原子写入，见 StateCache）
        # 触发 send_and_wait 的 pending Event（如有匹配）
```

**StateCache**（内部类）：

```python
class StateCache:
    """线程安全的状态缓存

    核心策略: 原子引用替换，避免 deepcopy + 长锁持有。
    - 写入端（读线程）: 构造新 JointState 对象 → 原子替换引用
    - 读取端（API 线程）: 读引用获取快照 → 无锁（JointState 为不可变快照）

    参考 Alicia-D-SDK 的 DataParser 模式，但避免其 RLock+Lock 混用的潜在死锁风险。
    """

    def __init__(self):
        self._lock = threading.Lock()   # 仅保护引用替换和 Event 操作
        self._joint_state: Optional[JointState] = None
        self._version: Optional[VersionResponse] = None
        self._pending_events: Dict[int, threading.Event] = {}  # send_and_wait 用

    def update_joint_state(self, state: JointState) -> None:
        """原子替换关节状态（读线程调用）"""
        with self._lock:
            self._joint_state = state   # 替换引用，不 deepcopy

    def get_joint_state(self) -> Optional[JointState]:
        """获取当前状态快照（API 线程调用，微秒级返回）"""
        with self._lock:
            return self._joint_state    # 返回引用，调用方视为只读快照

    def register_pending(self, cmd_id: int) -> threading.Event:
        """注册等待响应的 Event（send_and_wait 用）"""
        event = threading.Event()
        with self._lock:
            self._pending_events[cmd_id] = event
        return event

    def resolve_pending(self, cmd_id: int, frame: Frame) -> None:
        """触发匹配的 pending Event"""
        with self._lock:
            event = self._pending_events.pop(cmd_id, None)
        if event:
            event._response = frame     # 附带响应数据
            event.set()
```

---

### 5.3 控制层 (`control/`)

从原来过于臃肿的 API 层中分离出来的运动控制逻辑。

> **设计原则**：PV 和 MIT 是两种本质不同的控制范式，不应强行统一为同一套方法签名。
> - PV = 「发后不管」：发送目标+速度，固件自行到达
> - MIT = 「持续控制」：每帧发送完整阻抗参数，SDK 持有控制权
> 
> 参考 piper_sdk：`JointCtrl()` 用于 PV，`JointMitCtrl()` 用于 MIT，接口分离但可协同。

#### 5.3.1 `joint_control.py` — 关节控制

```python
class JointController:
    """关节级运动控制

    提供 PV 模式和 MIT 模式的独立控制方法，以及两种模式共用的通用方法。
    """

    def __init__(self, device: Device, config: RobotConfig):
        self._device = device
        self._config = config
        self._mode = config.control_mode
        ...

    # ========== PV 模式 ==========

    def move_pv(self, target_joints: List[float], speed: float,
                gripper: Optional[float] = None,
                gripper_speed: Optional[float] = None,
                wait: bool = True, timeout: float = 10.0) -> bool:
        """PV 模式点位运动

        发送目标位置+速度，固件内部做加减速插值。
        速度为有符号值，SDK 根据运动方向自动计算符号。

        Args:
            target_joints: 目标角度 (rad), 6 个关节
            speed: 运动速度，无量纲 [0, 400]，映射到 [0, 10] rad/s
            gripper: 夹爪目标值 [0, 1000]，None 表示不控制
            gripper_speed: 夹爪速度 [0, 400]
            wait: 是否阻塞等待到达
        """

    # ========== MIT 模式 ==========

    def send_mit(self, joint_params: List[MitParams],
                 gripper: Optional[float] = None) -> None:
        """MIT 全参数直接发送（标准 MIT 接口）

        发送一帧 6 地址 MIT 帧（线性速度填清零信号）。不做等待，不做插值。
        用于遥操作、力控、轨迹回放等需要持续高频发帧的场景。

        Args:
            joint_params: 7 个电机的 MIT 参数 (pos, vel, torque, kp, kd)
            gripper: 夹爪目标值，为 None 时使用 joint_params[6].pos_ref
        """

    def move_mit(self, target_joints: List[float], speed: float,
                 gripper: Optional[float] = None,
                 gripper_speed: Optional[float] = None,
                 wait: bool = True, timeout: float = 10.0) -> bool:
        """MIT 模式点位运动（简易接口，通过线性轨迹速度实现）

        在 MIT 模式下实现类似 PV 的「发后等待」体验：
        1. 设定线性轨迹速度 (addr=0x05)
        2. 发送全参数 MIT 帧 (pos=目标, vel=0, t=0, kp=默认, kd=默认)
        3. 等待到达目标
        4. 清零线性轨迹速度（防止残留）

        Args:
            参数含义同 move_pv
        """

    # ========== 通用方法 ==========

    def go_home(self, speed: float = 15.0) -> bool:
        """回零位（自动适配当前模式）"""

    def move_gripper(self, value: float, wait: bool = True) -> bool:
        """控制夹爪 [0=关闭, 1000=打开]"""

    # ========== 力矩与使能（0x05 仅 MIT 可用）==========

    def torque_off(self, joints: Optional[List[int]] = None) -> bool:
        """卸载力矩（仅 MIT 模式，发送 0x05 指令）

        原理：将指定关节的 kp=kd 置零，使其自由运动。
        未指定的关节保持原有 kp/kd，继续锁定。

        如果当前为 PV 模式，先自动切换到 MIT。

        Args:
            joints: 需要卸力的关节索引列表，None 表示全部关节
        """

    def torque_on(self, joints: Optional[List[int]] = None) -> bool:
        """恢复力矩（仅 MIT 模式，发送 0x05 指令）

        恢复指定关节的 kp/kd 为默认值。
        首帧以当前位置为 pos_ref，防止突跳。
        """

    def enable(self) -> bool:
        """使能机器人（发送 0x09 指令，任何模式可用）"""

    def disable(self) -> bool:
        """失能机器人（发送 0x09 指令，任何模式可用）"""

    def set_zero_position(self) -> bool:
        """设置当前位姿为零位（发送 0x03 指令）"""

    def switch_mode(self, mode: ControlMode) -> bool:
        """切换控制模式（发送 0x11 指令, addr=0x0B）

        流程: 失能 → 切换模式 → 使能，固件侧处理目标位置初始化。
        """

    def _wait_for_target(self, target: List[float],
                         tolerance: float = 0.05,
                         timeout: float = 10.0) -> bool:
        """轮询状态缓存等待到达目标位置

        实现策略: 固定间隔轮询（非 busy-wait），读状态缓存比较误差。
        不阻塞串口通信——只读内存中的 StateCache，不发 I/O。

        轮询间隔: 10ms（100Hz 检查频率，与状态更新频率匹配）
        超时: 默认 10s，超时返回 False（由调用方决定处理策略）
        """
        POLL_INTERVAL = 0.01  # 10ms
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            state = self._device.joint_state
            if state and all(
                abs(state.angles[i] - target[i]) < tolerance
                for i in range(len(target))
            ):
                return True
            time.sleep(POLL_INTERVAL)
        return False
```

#### 5.3.2 `trajectory_executor.py` — 轨迹执行器

```python
class TrajectoryExecutor:
    """轨迹回放执行器

    PV 模式：逐帧发送 pos+vel，固件负责电机级平滑。
    """

    def __init__(self, device: Device, config: RobotConfig):
        ...

    def execute_pv(self, timestamps: np.ndarray,
                   positions: np.ndarray,
                   velocities: np.ndarray) -> bool:
        """PV 模式轨迹执行

        Args:
            timestamps: 时间序列 (s)
            positions: 关节位置序列 [N, 7] (rad)
            velocities: 关节速度序列 [N, 7] (rad/s)，含符号
        """

    """无需mit模式下的轨迹回放"""

    def stop(self) -> None:
        """紧急停止当前轨迹"""
```

---

### 5.4 类型定义层 (`types/`)

集中定义所有数据结构，被全部层共享。

#### 5.4.1 `state.py` — 状态类型

```python
@dataclass
class JointState:
    """关节状态"""
    angles: List[float]                        # 6个关节角度 (rad)
    gripper: float                             # 夹爪值 (0~1000)
    timestamp: float                           # 时间戳
    run_status: int                            # 原始运行状态字节
    velocities: Optional[List[float]] = None   # 关节速度 (rad/s)
    torques: Optional[List[float]] = None      # 关节力矩 (N·m)

@dataclass
class MitParams:
    """单个电机的 MIT 阻抗控制参数

    τ = kp × (pos_ref - pos_cur) + kd × (vel_ref - vel_cur) + t_ref

    注意：kp/kd 默认为 None，由控制层根据电机编号自动填充安全默认值
    （大关节 kp=150/kd=2，小关节 kp=20/kd=1）。
    显式传入 0 表示零力矩（卸力），不会被自动填充。
    """
    pos_ref: float = 0.0                # 目标位置 (rad)
    vel_ref: float = 0.0                # 目标速度 (rad/s)
    t_ref: float = 0.0                  # 前馈力矩 (N·m)
    kp: Optional[float] = None          # 位置增益 [0, 500]，None=使用默认值
    kd: Optional[float] = None          # 速度增益 [0, 5]，None=使用默认值

@dataclass
class RobotStatus:
    """机器人综合状态（从运行状态字节解析）"""
    is_locked: bool = False
    is_synced: bool = False
    has_motor_error: bool = False
    gripper_torque_locked: bool = False
    # 夹爪臂特有
    single_click: bool = False
    double_click: bool = False
    long_press: bool = False

@dataclass
class VersionInfo:
    """版本信息"""
    serial_number: str
    hardware_version: str
    firmware_version: str
    product_type: str              # 产品类别（AM=云擎, BM=云弈 ...）
    device_type: str               # 设备类型（L=示教臂, F=操作臂 ...）
```

#### 5.4.2 `config.py` — 配置类型

```python
@dataclass
class RobotConfig:
    """机器人配置"""
    # 连接参数
    port: str = ""                       # 串口端口（空=自动发现）
    baudrate: int = 1_000_000            # 波特率
    auto_connect: bool = True            # 自动连接

    # 机器人参数
    version: str = "v1_1"                # 硬件版本
    variant: Optional[str] = None        # 变体（自动检测）
    control_aim: Optional[str] = None    # "leader" / "follower"（自动检测）

    # 控制参数
    control_mode: str = "pv"             # "pv" / "mit"
    num_joints: int = 6                  # 关节数量
    num_motors: int = 7                  # 电机数量（含夹爪）

    # 运动学参数（RoboCore）
    model_format: str = "urdf"
    base_link: str = "base_link"
    end_link: str = "tool0"
    backend: str = "numpy"               # "numpy" / "torch"

    # 关节限位 (rad)
    joint_limits_lower: List[float] = ...
    joint_limits_upper: List[float] = ...

    # 方向映射（电机正方向与关节正方向的对应关系）
    direction_map: List[int] = ...       # [1, -1, 1, 1, -1, 1, 1]
```

#### 5.4.3 `enums.py` — 枚举定义

```python
class ControlAim(IntEnum):
    """控制目标部位"""
    LEADER   = 0x01    # 示教臂
    FOLLOWER = 0x02    # 操作臂

class ControlMode(Enum):
    """控制模式"""
    PV  = "pv"         # 位置-速度模式（固件控制加减速）
    MIT = "mit"        # MIT 阻抗控制模式（每帧发送 pos/vel/torque/kp/kd）

class GripperType(Enum):
    """夹爪类型"""
    MM_50  = "50mm"
    MM_100 = "100mm"

class ErrorCode(IntEnum):
    """协议错误码"""
    FRAME_HEADER    = 0x00
    LENGTH_MISMATCH = 0x01
    CRC_FAILED      = 0x02
    SYSTEM_MODE     = 0x03
    ANGLE_LIMIT     = 0x04
    MOTOR_OFFSET    = 0x05
    ADDR_OVERFLOW   = 0x06
```

#### 5.4.4 `exceptions.py` — 统一异常体系

参照 SOP 第 8 条，建立层级化异常体系，禁止将原始协议错误或 Python 原生异常无包装地抛给用户：

```python
class AliciaSDKError(Exception):
    """SDK 基础异常，所有 SDK 异常的父类"""

class ConnectionError(AliciaSDKError):
    """串口连接失败或已断开"""

class TimeoutError(AliciaSDKError):
    """通信超时（send_and_wait 等待响应超时）"""

class ProtocolError(AliciaSDKError):
    """协议层错误（帧校验失败、帧格式异常等）"""

class ValidationError(AliciaSDKError):
    """参数校验失败（关节超限、速度超范围等）

    示例消息: "关节3超限: 目标=3.40 rad, 限位=[-2.80, 2.80] rad"
    """

class RobotStateError(AliciaSDKError):
    """机器人状态不满足操作前提（未使能、模式不匹配等）"""

class HardwareFaultError(AliciaSDKError):
    """固件返回的硬件错误（0xEE 指令映射）"""
    def __init__(self, error_code: int, detail: str = ""):
        self.error_code = error_code
        super().__init__(f"硬件错误 [0x{error_code:02X}]: {detail}")

class MotionError(AliciaSDKError):
    """运动执行异常（轨迹超时、到达失败等）"""
```

**异常映射规则**：
| 场景 | 异常类型 |
|------|---------|
| 串口未连接 / 打开失败 | `ConnectionError` |
| `send_and_wait` 超时 | `TimeoutError` |
| CRC 校验失败 / 帧格式异常 | `ProtocolError` |
| 关节角度超限 / 速度超范围 / 参数类型错误 | `ValidationError` |
| 未使能就发运动指令 / PV 模式下调用 torque_off | `RobotStateError` |
| 固件 0xEE 错误码 | `HardwareFaultError` |
| 轨迹执行超时 / 未到达目标 | `MotionError` |

**原则**：异常消息必须可读、包含关键上下文。不要只报 `"Error code = 17"`，应写成 `"关节3超限: 目标=3.40 rad, 限位=[-2.80, 2.80] rad"`。

---

### 5.5 API 层 (`api/`)

重构后的 API 层是一个**精简的门面（Facade）**，将具体逻辑委托给控制层、运动学接口、规划接口。

#### 5.5.1 `synria_robot_api.py` — SynriaRobotAPI

```python
class SynriaRobotAPI:
    """Alicia-M 机器人 SDK 主接口

    通过 create_robot() 工厂函数创建实例。
    """

    def __init__(self, config: RobotConfig, robot_model: 'RobotModel' = None):
        # 初始化各内部组件（协议编解码封装在 Device 内部，上层无需感知）
        self._config = config
        self._serial_port = SerialPort(config.port, config.baudrate)
        self._device = Device(self._serial_port, MessageCodec())
        self._joint_ctrl = JointController(self._device, config)
        self._traj_executor = TrajectoryExecutor(self._device, config)
        self._robot_model = robot_model   # 由 create_robot() 传入，已完成初始化

    # ========== 连接管理 ==========

    def connect(self, timeout: float = 5.0) -> bool:
        """连接机器人

        总超时: 整个连接流程（串口打开 + 自动检测 + 首次状态获取）
        受 timeout 参数约束，避免子操作超时累积导致长时间阻塞。

        流程:
        1. 打开串口（自动发现或指定端口）
        2. 启动 Device 后台线程（读取 + 轮询）
        3. 自动检测控制目标（leader/follower）
        4. 等待首次状态缓存填充
        5. 查询固件版本

        失败: 抛出 ConnectionError，内含具体失败原因。
        """

    def disconnect(self) -> None:
        """断开连接: 停止后台线程 → 关闭串口"""

    def is_connected(self) -> bool: ...

    # ========== 状态查询 ==========

    def get_robot_state(self, info_type: str = "joint_gripper") -> ...: ...
    def get_pose(self) -> Optional[Dict]: ...
    def get_firmware_version(self, timeout: float = 5.0) -> Optional[str]: ...

    # ========== 关节控制（通用，自动适配当前模式）==========

    def set_robot_state(self, target_joints=None, gripper_value=None,
                        joint_format='deg', speed=15, gripper_speed=40,
                        wait_for_completion=True, **kwargs) -> bool:
        """点位运动：设置关节角度和/或夹爪位置

        自动根据当前模式选择实现：
        - PV: 发送 pos+vel 帧，固件自行到达
        - MIT: 通过线性轨迹速度 + 全参数帧实现点位运动
        """
        # PV → self._joint_ctrl.move_pv(...)
        # MIT → self._joint_ctrl.move_mit(...)

    def go_home(self, speed=15, gripper_speed=40) -> bool:
        """回零位"""

    def set_gripper_target(self, command=None, value=None,
                           wait_for_completion=True) -> bool:
        """控制夹爪"""

    # ========== MIT 专用接口 ==========

    def send_mit_command(self, joint_params: List['MitParams'],
                         gripper: Optional[float] = None) -> None:
        """MIT 全参数直接发送（低延迟，用于遥操作/力控/高频控制）

        每帧发送 6 地址 MIT 帧（线性速度填清零信号）。不等待、不插值。
        调用方需自行维持高频发送（≥200Hz）。
        """
        # → self._joint_ctrl.send_mit(...)

    # ========== 系统控制 ==========

    def torque_control(self, command: str, joints: List[int] = None) -> bool:
        """力矩开关（仅 MIT 模式，0x05 指令）

        'off': 卸力（kp=kd=0），指定关节自由运动
        'on': 恢复力矩
        如当前为 PV 模式，自动先切换到 MIT。
        """

    def enable_robot(self) -> bool:
        """使能机器人（0x09 指令）"""

    def disable_robot(self) -> bool:
        """失能机器人（0x09 指令）"""

    def switch_mode(self, mode: str) -> bool:
        """切换控制模式（失能→切换→使能）：'pv' / 'mit'"""

    def set_zero_position(self) -> bool:
        """设置当前位姿为零位"""

    # ========== 运动学 ==========

    def set_pose(self, target_pose, method='dls', execute=True,
                 speed=7, **ik_params) -> Dict:
        """通过逆运动学移动到目标位姿"""

    # ========== 轨迹规划与执行 ==========

    def plan_joint_trajectory(self, waypoints, planner_type='b_spline',
                              **kwargs) -> Dict:
        """规划关节空间轨迹"""

    def plan_cartesian_trajectory(self, waypoints, **kwargs) -> Dict:
        """规划笛卡尔空间轨迹"""

    def move_joint_trajectory(self, q_end, duration=2.0,
                              method='cubic', **kwargs) -> bool:
        """执行平滑关节轨迹"""

    def move_cartesian_linear(self, target_pose, duration=2.0,
                              **kwargs) -> bool:
        """执行笛卡尔直线轨迹"""

    def solve_ik_for_trajectory(self, target_poses, q_init=None,
                                **kwargs) -> Dict:
        """批量 IK 求解"""

    # ========== 状态打印 ==========

    def print_state(self, continuous: bool = False,
                    output_format: str = "deg") -> None:
        """打印当前状态（关节角度、夹爪、速度、力矩）"""

    # ========== 废弃方法（保留兼容性）==========

    def set_home(self, **kwargs):
        """已废弃，请使用 go_home()"""
        warnings.warn("set_home() 已废弃，请使用 go_home()", DeprecationWarning)
        return self.go_home(**kwargs)

    def set_pose_target(self, **kwargs):
        """已废弃，请使用 set_pose()"""
        warnings.warn("set_pose_target() 已废弃，请使用 set_pose()", DeprecationWarning)
        return self.set_pose(**kwargs)
```

---

### 5.6 RoboCore 集成接口 (`integrations/robocore`)

对 RoboCore 的薄封装，提供便捷的调用入口。

> **性能与阻塞说明**：
> - FK/IK/Planning 均为**同步阻塞**调用，这与 Alicia-D-SDK 的用法一致
> - FK < 1ms（矩阵乘法），IK 约 5~50ms（迭代求解），规划约 10~100ms（离线计算）
> - **RoboCore 调用不在控制环路内**——控制帧发送是纯协议层操作，与运动学解算完全隔离
> - 轨迹执行时，所有 FK/IK/规划在发帧前已完成（预计算），不会阻塞通信
> - RobotModel 在 `create_robot()` 时**立即初始化**（与 D-SDK 一致），避免首次调用延迟
> - 默认使用 `numpy` 后端，可选 `torch` 后端加速批量 IK

```python
# integrations/robocore/kinematics.py
import robocore as rc
from robocore.kinematics import forward_kinematics, inverse_kinematics, jacobian
from robocore.transform import matrix_to_euler, matrix_to_quaternion
from robocore.utils.backend import to_numpy

def compute_forward_kinematics(robot_model, q: List[float]) -> Dict:
    """正运动学：关节角度 → 末端位姿（同步调用，<1ms）"""
    T = forward_kinematics(robot_model, q, return_end=True)
    position = to_numpy(T[:3, 3])
    rotation = to_numpy(T[:3, :3])
    return {
        'transform': to_numpy(T),
        'position': position,
        'rotation': rotation,
        'euler_xyz': matrix_to_euler(rotation),
        'quaternion_xyzw': matrix_to_quaternion(rotation),
    }

def compute_inverse_kinematics(robot_model, target_pose, q_init=None,
                               method='dls', **kwargs) -> Dict:
    """逆运动学：目标位姿 → 关节角度（同步调用，5~50ms）"""
    # 默认使用解析雅可比 + DLS 方法
    # 支持 num_initial_guesses 多起点求解

def compute_jacobian(robot_model, q: List[float]) -> np.ndarray:
    """雅可比矩阵计算"""
```

```python
# integrations/robocore/planning.py
# 规划器延迟导入（按需加载，避免不使用规划功能时的导入开销）
def plan_joint_trajectory(waypoints, planner_type='b_spline', **kwargs) -> Dict:
    """关节空间轨迹规划（同步调用，离线计算）"""
    from robocore.planning import BSplinePlanner, MultiSegmentPlanner
    # BSplinePlanner: 3/5 阶 B 样条
    # MultiSegmentPlanner: 三次/五次多项式分段

def plan_cartesian_trajectory(waypoints, **kwargs) -> Dict:
    """笛卡尔空间轨迹规划（同步调用，离线计算）"""
    from robocore.planning import SplineCurvePlanner
```

---

### 5.7 工具层 (`utils/`)

#### 5.7.1 `conversion.py` — 单位转换

```python
# 物理量 ↔ 协议值
def rad_to_protocol(value: float, bits: int, range_max: float) -> int:
    """弧度值 → 协议整数值（通用映射）"""
    # [-range_max, +range_max] → [0, 2^bits - 1]

def protocol_to_rad(raw: int, bits: int, range_max: float) -> float:
    """协议整数值 → 弧度值"""

# 常用快捷函数
def pos_to_protocol(rad: float) -> int: ...      # 位置 rad → 16bit
def protocol_to_pos(raw: int) -> float: ...       # 16bit → 位置 rad
def vel_to_protocol(rad_s: float) -> int: ...     # 速度 rad/s → 12bit
def protocol_to_vel(raw: int) -> float: ...       # 12bit → 速度 rad/s
def torque_to_protocol(nm: float) -> int: ...     # 力矩 N/m → 12bit
def protocol_to_torque(raw: int) -> float: ...    # 12bit → 力矩 N/m

# 角度单位互转
def deg_to_rad(deg: float) -> float: ...
def rad_to_deg(rad: float) -> float: ...

# 夹爪量程转换
def gripper_normalize(raw: int, gripper_range: int) -> float: ...
def gripper_denormalize(value: float, gripper_range: int) -> int: ...
```

#### 5.7.2 `validation.py` — 参数校验

```python
def validate_joint_angles(angles: List[float], config: RobotConfig) -> List[float]:
    """校验并裁剪关节角度到安全范围，超限时发出警告"""

def validate_speed(speed, num_joints: int) -> List[float]:
    """校验速度参数（支持标量和列表），返回每关节速度列表"""

def validate_gripper_value(value: float) -> float:
    """校验夹爪值范围 [0, 1000]"""
```

#### 5.7.3 `timing.py` — 时间工具

```python
def precise_sleep(duration: float) -> None:
    """高精度休眠（忙等待 + sleep 混合策略）"""

class FPSCounter:
    """帧率统计器"""
    def tick(self) -> None: ...
    def get_fps(self) -> float: ...
```

#### 5.7.4 `logger.py` — 日志系统

SDK 内部使用标准 `logging` 模块分级输出，面向用户的示例脚本和 API 层状态打印
使用 `robocore.utils.beauty_logger` 提供格式化彩色输出。

```python
import logging
from robocore.utils.beauty_logger import beauty_print, beauty_print_array

def get_logger(name: str, level: int = logging.INFO) -> logging.Logger:
    """获取带格式化的 SDK 内部日志实例

    日志格式: [时间] [模块名] [级别] 消息
    分级: DEBUG / INFO / WARNING / ERROR
    通信收发日志使用 DEBUG 级别，默认不输出
    """

# beauty_logger 转发（供 API 层和示例统一调用）
def print_info(msg: str) -> None:
    beauty_print(msg, type="info")

def print_success(msg: str) -> None:
    beauty_print(msg, type="success")

def print_warning(msg: str) -> None:
    beauty_print(msg, type="warning")

def print_error(msg: str) -> None:
    beauty_print(msg, type="error")

def format_array(arr, precision: int = 5, sign: bool = True) -> str:
    return beauty_print_array(arr, precision=precision, sign=sign)
```

---

### 5.8 包入口 (`__init__.py`)

```python
"""Alicia-M-SDK: Synria 云擎系列机械臂 Python SDK"""

import robocore as rc
from .api.synria_robot_api import SynriaRobotAPI
from .types.state import JointState, MitParams
from .types.config import RobotConfig
from .types.enums import ControlAim, ControlMode
from .types.exceptions import AliciaSDKError, ConnectionError, ValidationError

# RoboCore 转发（供用户直接使用）
from robocore.modeling import RobotModel
from robocore.kinematics import forward_kinematics, inverse_kinematics, jacobian

def create_robot(
    port: str = "",
    version: str = "v1_1",
    variant: str = None,
    control_aim: str = None,
    control_mode: str = "pv",
    baudrate: int = 1_000_000,
    backend: str = "numpy",
    debug_mode: bool = False,
    auto_connect: bool = True,
    **kwargs
) -> SynriaRobotAPI:
    """创建机器人实例的工厂函数

    初始化顺序（与 Alicia-D-SDK create_robot 保持一致）:
    1. 设置 RoboCore 后端（numpy / torch）
    2. 加载机器人 URDF 模型（立即初始化，避免首次调用延迟）
    3. 创建 SynriaRobotAPI 实例
    4. 自动连接（可选）

    Args:
        port: 串口端口路径，空字符串表示自动发现
        version: 机器人硬件版本 ("v1_0", "v1_1")
        variant: 变体标识（None=自动检测）
        control_aim: 控制目标 ("leader"/"follower"/None=自动检测)
        control_mode: 控制模式 ("pv"/"mit")
        baudrate: 串口波特率
        backend: RoboCore 计算后端 ("numpy"/"torch")
        debug_mode: 调试模式
        auto_connect: 是否自动连接

    Returns:
        SynriaRobotAPI 实例
    """
    # 1. 设置 RoboCore 后端
    rc.set_backend(backend)

    # 2. 加载机器人模型（立即初始化）
    from synriard import get_model_path
    model_path = get_model_path("Alicia_M", version=version,
                                variant=variant, model_format="urdf")
    robot_model = RobotModel(str(model_path),
                             base_link="base_link", end_link="tool0")

    # 3. 创建实例
    config = RobotConfig(
        port=port, version=version, variant=variant,
        control_aim=control_aim, control_mode=control_mode,
        baudrate=baudrate, auto_connect=auto_connect, ...
    )
    robot = SynriaRobotAPI(config, robot_model=robot_model)

    # 4. 自动连接
    if auto_connect:
        robot.connect()
    return robot
```

---

## 六、关键设计决策

### 6.1 协议层独立

**决策**：将协议编解码从硬件层完全分离，形成独立的 `protocol/` 模块。

**原因**：
- 当前协议逻辑分散在 `servo_driver.py`（构建帧）、`data_parser.py`（解析帧）、`serial_comm.py`（CRC校验）中
- 协议层应当是纯粹的数据转换，不依赖 I/O 操作
- 独立后可单独对编解码逻辑做单元测试，无需连接硬件

### 6.2 Device 取代 ServoDriver + CommunicationManager + DataParser

**决策**：用一个 `Device` 类统一管理通信调度和状态缓存，取代原来的三个类。

**原因**：
- `CommunicationManager` 的优先级队列对串口协议是过度设计
- `DataParser` 的状态管理与 `ServoDriver` 的后台线程紧密耦合
- 合并为 `Device` 后职责更清晰：「与一台机器人设备通信」

### 6.3 控制层从 API 层分离

**决策**：`JointController`、`TrajectoryExecutor`、`Teleoperation` 独立为控制层。

**原因**：
- 原 `SynriaRobotAPI` 有 1100+ 行，关节控制、轨迹执行、等待逻辑全部混在一起
- 分离后 API 层仅做参数转换和委托调用，控制层负责具体的运动逻辑
- 便于独立测试和复用

### 6.4 类型定义层集中管理

**决策**：所有数据类（`JointState`、`RobotConfig`）和枚举集中到 `types/`。

**原因**：
- 避免循环导入（当前 `JointState` 定义在 `data_parser.py` 中，被多处引用）
- 数据结构定义应当是零依赖的，被所有层共享

### 6.5 CRC32 自行实现

**决策**：移除 `pythoncrc` 和 `pycrc` 依赖，使用 Python 标准库 `binascii.crc32`。

**原因**：
- 协议仅需 CRC32 取低 8 位
- Python 标准库已内置 `binascii.crc32()`，无需第三方依赖
- 减少两个不必要的依赖包

### 6.6 非阻塞通信架构

**决策**：控制命令 fire-and-forget，读写分离，状态查询退避。

**原因**：
- 旧代码的 `CommunicationManager` 将读写放在同一线程，控制命令和状态查询竞争串口 → 频率低至 0-40Hz
- MIT 高频控制（200~1000Hz）要求每帧延迟 < 2ms，不能等待响应

**设计**（参考 piper_sdk + Alicia-D-SDK）：
- **写入路径**：调用方线程直接写串口，仅 `write_lock` 保护原子写入，微秒级返回
- **读取线程**：专用后台线程**纯读取**，持续解析帧、更新状态缓存，**绝不写串口**
- **状态轮询线程**：专用后台线程周期性发送 0x06 读取帧（5ms 间隔，~200Hz），是状态缓存的唯一数据来源；高频控制时短暂退避避免带宽竞争，冷却后立即恢复查询
- **`send_and_wait`**：仅用于低频操作（版本查询、模式切换），通过 `Event` 等待读线程匹配响应，超时值保证 ≥ 0
- **协议封装**：`Device` 提供 `send_pv()`/`send_mit()` 等高层方法，控制层无需直接接触 `MessageCodec`
- **串口读超时**：`SerialPort` 初始化 `read_timeout=10ms`，保证读线程每 10ms 必有机会检查退出信号
- **状态缓存**：`StateCache` 使用原子引用替换（非 deepcopy），读取端微秒级返回，无锁竞争

**频率目标**：
| 场景 | 目标频率 | 实现方式 |
|------|---------|---------|
| MIT 连续控制 | 500~1000 Hz | 帧长 78B @ 1Mbaud ≈ 0.6ms，留余量 |
| PV 点位运动 | 单帧即可 | 固件自行插值 |
| 状态轮询（空闲时） | ~200 Hz | 轮询线程 5ms 间隔查询 |
| 状态轮询（控制时） | ~100 Hz | 轮询线程退避后穿插查询（3ms 冷却） |
| 遥操作 | ≥ 200 Hz | 与 MIT 控制共享写入路径 |

**线程模型总览**：
| 线程 | 职责 | 阻塞防护 |
|------|------|---------|
| 用户线程 | 调用 API、发送控制帧 | `write_lock` 保护写入，微秒级返回 |
| 读取线程 | 纯读串口、解析帧、更新 StateCache | `read_timeout=10ms` 保证可退出 |
| 轮询线程 | 空闲时周期性发送状态查询帧 | `stop_event.wait()` 可打断，写退避 |

### 6.7 模式切换设计

**决策**：模式切换采用「失能 → 切换 → 使能」流程，固件侧处理目标位置初始化。

**流程**：

| 操作 | 流程 |
|------|------|
| 模式切换 | 失能 → 发送 0x11 切换指令 → 读回验证 → 使能 |
| 使能 (0x09) | 直接发送使能指令，固件侧初始化目标位置 |
| 恢复力矩 (0x05) | 直接发送恢复指令，固件侧恢复 kp/kd |

### 6.8 安全校验清单

参照 SOP 第 10 条，所有运动相关 API 在执行前必须通过以下安全校验（在 `JointController` 内部统一执行，抛出 `RobotStateError` 或 `ValidationError`）：

| 校验项 | 校验时机 | 失败行为 |
|--------|---------|---------|
| 串口连接状态 | 所有指令发送前 | 抛出 `ConnectionError` |
| 使能状态 | 运动指令前 | 抛出 `RobotStateError("机器人未使能")` |
| 控制模式匹配 | `torque_off/on` 前 (须 MIT) | 自动切换或抛出异常 |
| 关节角度限位 | `move_pv`/`move_mit` 前 | 裁剪至安全范围 + 发出警告 |
| 速度上限 | `move_pv`/`move_mit` 前 | 裁剪至 [0, 400] + 发出警告 |
| 夹爪范围 | 夹爪控制前 | 裁剪至 [0, 1000] |

**设计原则**：
- 校验由 `JointController` 内部自动执行，用户无需手动调用
- 输入超限时优先**裁剪 + 警告**（而非直接拒绝），降低用户使用门槛
- 可通过 `config.strict_validation = True` 切换为严格模式（超限即抛异常）

### 6.9 保留 API 方法名

**决策**：`SynriaRobotAPI` 的所有公开方法名保持不变。

保留的方法清单：
| 方法名 | 用途 |
|--------|------|
| `connect()` | 连接机器人 |
| `disconnect()` | 断开连接 |
| `is_connected()` | 检查连接状态 |
| `get_robot_state(info_type)` | 获取机器人状态 |
| `get_pose()` | 获取末端位姿 |
| `get_firmware_version()` | 获取固件版本 |
| `go_home()` | 回零位 |
| `set_robot_state()` | 点位运动（PV/MIT 自动适配） |
| `set_gripper_target()` | 控制夹爪 |
| `set_pose()` | IK 运动 |
| `torque_control()` | 力矩开关 |
| `set_zero_position()` | 设置零位 |
| `plan_joint_trajectory()` | 关节轨迹规划 |
| `plan_cartesian_trajectory()` | 笛卡尔轨迹规划 |
| `move_joint_trajectory()` | 执行关节轨迹 |
| `move_cartesian_linear()` | 执行直线轨迹 |
| `solve_ik_for_trajectory()` | 批量 IK 求解 |
| `print_state()` | 打印状态 |

新增方法：
| 方法名 | 用途 |
|--------|------|
| `send_mit_command()` | MIT 全参数直接发送（遥操作/力控） |
| `switch_mode()` | 切换 PV/MIT 模式 |
| `enable_robot()` | 使能机器人 |
| `disable_robot()` | 失能机器人 |

废弃方法保留兼容性包装：
| 废弃方法 | 重定向 |
|----------|--------|
| `set_home()` | → `go_home()` |
| `set_pose_target()` | → `set_pose()` |

---

## 七、依赖关系优化

### 当前依赖
```
pyserial, numpy, scipy, matplotlib, pyyaml, omegaconf,
websockets, pythoncrc, pycrc, synria-robocore, synriard
```

### 重构后依赖

**核心依赖（必需）**：
```
pyserial >= 3.0         # 串口通信
numpy >= 1.21.0         # 数值计算
synria-robocore >= 2.0  # 运动学与轨迹规划
synriard                # 机器人模型描述
```

**可选依赖（按功能分组）**：
```
[planning]
scipy >= 1.7.0          # 高级规划算法

[visualization]
matplotlib >= 3.3.0     # 轨迹可视化

[config]
pyyaml >= 6.0           # YAML 配置文件
omegaconf >= 2.3.0      # 结构化配置
```

**移除的依赖**：
- `pythoncrc` / `pycrc` → 改用标准库 `binascii.crc32`
- `websockets` → SparkVis 集成如需保留，作为可选依赖

---

## 八、通信协议详细设计

> 基于《云擎通讯协议 v1.0.0（公开版）》，结合 SDK 的实际实现需求，详细定义协议层的编解码规范。

### 8.1 帧结构

```
┌──────┬───────┬───────┬──────────┬──────────┬───────┬──────┐
│ 帧头  │指令 ID │功能码  │有效数据长度│ 有效数据  │ 校验位 │ 帧尾  │
│ 0xAA │ 1字节  │ 1字节  │  1字节    │  N字节   │ 1字节  │ 0xFF │
└──────┴───────┴───────┴──────────┴──────────┴───────┴──────┘
```

**校验位计算**：`Check = CRC32(指令ID + 功能码 + 有效数据长度 + 有效数据) & 0xFF`

使用 Python 标准库 `binascii.crc32()` 实现，取结果的低 8 位。

### 8.2 指令 ID 总览

| 指令 ID | 功能 | 当前固件支持 |
|---------|------|------------|
| `0x01` | 版本与设备信息获取 | 已支持 |
| `0x03` | 位姿重置（设置当前位姿为初始位姿） | 已支持 |
| `0x05` | 肢体力矩控制（关节力矩开关） | 已支持 |
| `0x06` | 关节与夹具状态控制（核心指令） | 已支持 |
| `0x09` | 机器人部位失能/使能 | 已支持 |
| `0x11` | 关节控制幅值和驱动参数设置 | 已支持 |
| `0xEE` | 错误反馈 | 已支持 |

> **固件待更新功能**：读取当前模式信息（PV/MIT/重力补偿）、读取电机温度、自检功能、切换重力补偿模式。SDK 架构需预留这些功能的扩展接口。

### 8.3 功能码编码规则

功能码为 1 字节，含义因指令而异，但遵循以下通用规则：

| bit 位 | 含义 | 说明 |
|--------|------|------|
| bit7 | 读写方向 | `0` = 读取/请求，`1` = 写入/控制 |
| bit0~bit6 | 指令特定 | 对于 0x03/0x05/0x06/0x09/0x11：bit0=示教臂，bit1=操作臂 |

**示例**：
- `0x02` = 读取操作臂（bit7=0, bit1=1）
- `0x82` = 写入操作臂（bit7=1, bit1=1）
- `0x01` = 读取示教臂（bit7=0, bit0=1）
- `0x81` = 写入示教臂（bit7=1, bit0=1）
- `0x7E` = 请求版本（0x01 专用）
- `0xFE` = 版本反馈（0x01 专用，0x80 + 0x7E）

### 8.4 指令 0x06 — 关节与夹具状态控制（核心）

这是 SDK 最频繁使用的指令，分为「状态获取」和「状态控制」两类操作。

#### 8.4.1 自定义数据地址表

每个电机可携带多种数据，通过「起始地址」和「偏移数量」选择数据范围：

| 地址 (addr) | 数据内容 | 位数 | 字节数 | 单位 | 物理范围 |
|-------------|---------|------|--------|------|---------|
| `0x00` | 当前位置 / 期望位置 | 16 bit | 2 | rad | [-12.5, +12.5] |
| `0x01` | 当前速度 / 期望速度 | 12 bit | 2 | rad/s | [-10.0, +10.0] |
| `0x02` | 当前力矩 / 额外力矩 | 12 bit | 2 | N/m | 大关节 ±28，小关节 ±10 |
| `0x03` | 位置环 Kp | 16 bit | 2 | — | [0, 500] |
| `0x04` | 速度环 Kd | 16 bit | 2 | — | [0, 5] |
| `0x05` | 线性轨迹速度 | 12 bit | 2 | rad/s | [-10.0, +10.0] |

> 映射范围可通过 0x11 指令的 `addr=0x16/0x17/0x18` 进行修改。以上为云擎实际使用的参数，与协议文档中的默认值（速度 ±30、力矩 ±20）不同。

**12 bit 数据在 2 字节中的存储方式**：高 4 位保留（置零），低 12 位为有效数据。

#### 8.4.2 数据映射公式

所有物理量到协议值的转换使用统一的线性映射：

```
协议值 = (物理值 - 范围下限) / (范围上限 - 范围下限) × (2^位数 - 1)
物理值 = 协议值 / (2^位数 - 1) × (范围上限 - 范围下限) + 范围下限
```

**具体映射**：

| 数据类型 | 物理范围 | 协议范围 | 中点含义 |
|---------|---------|---------|---------|
| 位置 (16bit) | [-12.5, +12.5] rad | [0, 65535] | 32768 ≈ 0 rad |
| 速度 (12bit) | [-10.0, +10.0] rad/s | [0, 4095] | 2048 ≈ 0 rad/s |
| 力矩-大关节 (12bit) | [-28.0, +28.0] N·m | [0, 4095] | 2048 ≈ 0 N·m |
| 力矩-小关节 (12bit) | [-10.0, +10.0] N·m | [0, 4095] | 2048 ≈ 0 N·m |
| Kp (16bit) | [0, 500] | [0, 65535] | — |
| Kd (16bit) | [0, 5] | [0, 65535] | — |

**字节序**：所有多字节数据均使用**小端序**（Little-Endian）。

#### 8.4.3 状态获取帧格式

```
请求帧:
[0xAA] [0x06] [aim] [0x02] [start_addr] [addr_count] [CRC8] [0xFF]

响应帧:
[0xAA] [0x06] [aim|0x80] [len] [start_addr] [addr_count] [motor_data...] [run_status] [CRC8] [0xFF]
```

- `aim`: 部位（`0x01`=示教臂, `0x02`=操作臂），注意此处功能码 bit7=0 为读取
- `start_addr`: 起始数据地址（通常为 `0x00`）
- `addr_count`: 偏移数量，决定每个电机返回几个地址的数据
- `motor_data`: 7 个电机 × `addr_count` 个地址 × 2 字节/地址
- `run_status`: 1 字节运行状态（末尾追加）
- `len`: `2 + 7 × addr_count × 2 + 1`（前缀 + 电机数据 + 运行状态）

**常用查询组合**：

| 场景 | start_addr | addr_count | 每电机字节 | 数据内容 |
|------|-----------|------------|----------|---------|
| 仅位置 | 0x00 | 1 | 2 | pos |
| 位置+速度 | 0x00 | 2 | 4 | pos, vel |
| 位置+速度+力矩 | 0x00 | 3 | 6 | pos, vel, torque |
| 全部（含Kp/Kd） | 0x00 | 5 | 10 | pos, vel, torque, kp, kd |

#### 8.4.4 状态控制帧格式

```
控制帧:
[0xAA] [0x06] [aim|0x80] [len] [start_addr] [addr_count] [motor_data...] [CRC8] [0xFF]

响应帧:
[0xAA] [0x06] [aim|0x80] [0x03] [start_addr|0x80] [addr_count] [result] [CRC8] [0xFF]
```

- `result`: `0x01`=成功, `0x00`=失败

#### 8.4.5 运行状态字节

运行状态为 1 字节，含义取决于设备类型：

**示教臂（握把臂）**：
| bit | 含义 |
|-----|------|
| bit0 | 锁定状态 |
| bit1 | 同步状态 |
| bit2~5 | 待定 |
| bit6 | 夹具过高力矩运动方向锁定 (0x07) |
| bit7 | 电机 err (0x08) |

**操作臂（夹爪臂）**：
| bit | 含义 |
|-----|------|
| bit0 | 单击 |
| bit1 | 双击 |
| bit2 | 长按 |
| bit3 | 重复长按 |
| bit4~5 | 待定 |
| bit6 | 夹具过高力矩运动方向锁定 (0x07) |
| bit7 | 电机 err (0x08) |

#### 8.4.6 数据帧属性字节（前缀1）

响应帧中 `start_addr` 的最高位 (bit7) 用作反馈帧标识：
- `bit7 = 0`: 非反馈帧
- `bit7 = 1`: 反馈帧（用于防止数据混淆）

### 8.5 指令 0x11 — 电机驱动参数设置

该指令用于修改电机底层参数，包括**控制模式切换**：

#### 电机参数地址表

| 参数内容 | 数据类型 | addr | 取值范围 |
|---------|---------|------|---------|
| 加速度 | float | 0x05 | — |
| 减速度 | float | 0x06 | — |
| MID 反馈 ID | uint32_t | 0x08 | — |
| 接收 ID | uint32_t | 0x09 | — |
| **控制模式** | uint32_t | **0x0B** | **0x01=MIT, 0x02=位置速度(PV)** |
| 位置映射范围 | float | 0x16 | 默认 12.5 |
| 速度映射范围 | float | 0x17 | 实际 10.0（协议文档默认 30.0） |
| 扭矩映射范围 | float | 0x18 | 大关节 28.0 / 小关节 10.0（协议文档默认 20.0） |
| 速度环 Kp | float | 0x1A | — |
| 速度环 Ki | float | 0x1B | — |
| 位置环 Kp | float | 0x1C | — |
| 位置环 Ki | float | 0x1D | — |

#### 模式切换帧格式

```
请求帧:
[0xAA] [0x11] [aim|0x80] [0x07] [start_motor] [motor_count] [0x0B] [mode(4字节LE)] [CRC8] [0xFF]

mode = 0x01 0x00 0x00 0x00  → MIT 模式
mode = 0x02 0x00 0x00 0x00  → PV（位置速度）模式
```

### 8.6 其他指令概要

#### 0x01 — 版本信息
- 请求：`功能码=0x7E, 数据=0xFE`（占位符）
- 响应：`功能码=0xFE, 数据=序列号(16B)+硬件版本(4B)+固件版本(4B)`

#### 0x03 — 位姿重置
- 功能码低位指定部位（bit0=示教臂, bit1=操作臂）
- 数据：每个部位 2 字节（起始关节ID + 偏移数量）

#### 0x05 — 力矩控制（仅 MIT 模式可用）
- **原理**：将选定关节的 kp=kd 置零，使其自由运动（卸力）；未选定的关节保持 kp/kd 锁定
- 功能码低位指定部位
- 数据：每个部位 2 字节（起始关节ID + 偏移数量）
- 云擎关节数量：7 个
- **注意**：PV 模式下此指令不可用。如需卸力，须先通过 0x11 切换到 MIT 模式

#### 0x09 — 部位失能/使能（任何模式可用）
- 硬件级使能/失能，与控制模式无关
- 数据：`0x01`=使能, `0x00`=失能
- **安全注意**：使能后必须立即以当前位置发送首帧，防止关节突跳到旧目标位置

#### 0xEE — 错误反馈

| 功能码 (addr) | 错误描述 | 携带数据 |
|--------------|---------|---------|
| 0x00 | 帧头/帧尾校验错误 | 错误帧实际长度 |
| 0x01 | 数据长度校验错误 | 错误帧实际长度 |
| 0x02 | CRC32 校验不通过 | 下位机计算的校验位值 |
| 0x03 | 系统模式错误 | 检测到的机械臂类型 |
| 0x04 | 电机角度限位中 | 限位电机 ID |
| 0x05 | motorData 偏移与部位数量不符 | 当前数据块长度 |
| 0x06 | insID 偏移超过最大地址 | 起始地址+偏移的计算结果 |

---

## 九、PV / MIT 控制模式详细设计

> 参考 [piper_sdk](https://github.com/agilexrobotics/piper_sdk) 的标准实现，重新设计 PV 和 MIT 两种控制模式。
> 
> **废弃旧代码的做法**：旧代码中 MIT 模式仅在初始化时发送 Kp/Kd，之后只发送位置——这不是标准的 MIT 实现。正确做法是：**MIT 每帧都发送全部 5 个参数（pos, vel, kp, kd, torque）**。
> 
> **关键硬件特性**：6 个关节电机中，前 3 个电机（大关节）的减速比是后 3 个电机（小关节）的 4 倍，这影响 MIT 模式下 Kp/Kd 参数的默认值。

### 9.1 模式对比

| 特性 | PV 模式（位置-速度） | MIT 模式（阻抗控制） |
|------|-------------------|-------------------|
| 协议 `0x0B` 值 | `0x02` | `0x01` |
| 速度控制方 | **固件**（内部梯形/S 型加减速） | **SDK**（阻抗控制 + 前馈） |
| 每帧每电机数据 | 4 字节（pos + vel） | 10 字节（pos + vel + torque + kp + kd） |
| 0x06 addr_count | 2 | 5 |
| 典型发送频率 | 低频（~50 Hz，单次指令即可） | 高频（200~1000 Hz，持续发送） |
| 速度含义 | 固件期望运动速度 | MIT vel_ref（阻抗控制器的速度参考） |
| 适用场景 | 常规点位运动、简单控制 | 轨迹回放、遥操作、力控、柔顺控制 |
| 初始化需求 | 无 | 需通过 0x11 切换电机为 MIT 模式 |

**piper_sdk 参考对照**：
- piper PV → `JointCtrl(j1..j6)`: 3 个 CAN 帧（0x155-0x157），每帧 2 个关节，仅位置
- piper MIT → `JointMitCtrl(joint, pos, vel, kp, kd, t)`: 每关节 1 个 CAN 帧（0x15A-0x15F），全 6 地址
- 云擎 PV → 1 个串口帧，7 电机，每电机 pos+vel（4B）
- 云擎 MIT → 1 个串口帧，7 电机，每电机 pos+vel+torque+kp+kd（10B）

### 9.2 PV 模式设计

#### 9.2.1 控制帧结构

```
PV 控制帧 (start_addr=0x00, addr_count=2: 位置+速度):
[0xAA][0x06][aim|0x80][0x1E][0x00][0x02]
  [M0_pos_lo][M0_pos_hi][M0_vel_lo][M0_vel_hi]   ← 电机0 (关节1)
  [M1_pos_lo][M1_pos_hi][M1_vel_lo][M1_vel_hi]   ← 电机1 (关节2)
  ...
  [M6_pos_lo][M6_pos_hi][M6_vel_lo][M6_vel_hi]   ← 电机6 (夹爪)
[CRC8][0xFF]

总帧长: 1+1+1+1 + 30 + 1+1 = 36 字节
数据长度 0x1E = 2(前缀) + 7×4(电机数据) = 30
```

#### 9.2.2 速度参数处理

固件速度为**有符号** [-10, +10] rad/s。SDK 用户使用无量纲速度 [0, 400]（仅表示幅值），SDK 根据运动方向计算符号：

```
用户速度 [0, 400] (无量纲幅值)
  → 物理速度幅值 [0, 10] rad/s
      magnitude = SDK_speed × 10.0 / 400.0
  → 计算方向符号:
      sign[i] = +1 if target[i] > current[i] else -1
  → 有符号速度:
      signed_vel[i] = sign[i] × magnitude    # [-10, +10] rad/s
  → 协议值:
      protocol_vel[i] = float_to_uint(signed_vel[i], -10.0, +10.0, 12)
```

速度为标量时所有关节使用相同幅值（方向各自独立）；为列表时支持逐关节独立幅值。

#### 9.2.3 PV 模式运动流程

```
用户: set_robot_state(joints, speed=15)
  │
  ├─ 转换角度单位 (deg→rad)
  ├─ 校验关节限位
  ├─ 应用方向映射: angle × direction[i]
  │
  ├─ 编码 PV 帧:
  │   pos[i] = float_to_uint(angle_rad, -12.5, +12.5, 16)
  │   signed_vel[i] = sign(target[i] - current[i]) × speed_rad
  │   vel[i] = float_to_uint(signed_vel[i], -10.0, +10.0, 12)
  │
  ├─ 发送单帧 → 固件接管加减速插值 → 固件自行到达目标
  │
  └─ (可选) 轮询等待到达目标位置
```

**PV 的核心特点**：发送一次目标位置+速度即可，固件内部做加减速插值，SDK 不需要持续发帧。

### 9.3 MIT 模式设计

> 标准 MIT 阻抗控制器模型：
> `τ = kp × (pos_ref - pos_cur) + kd × (vel_ref - vel_cur) + t_ref`
> 
> 每帧都发送全部 6 地址，上层可实时调整增益和前馈，实现柔顺控制、力控、遥操作等高级功能。

#### 9.3.1 MIT 控制帧结构（标准全参数帧）

**每一帧都携带全部 5 个参数**，这是 MIT 的标准做法：

```
MIT 控制帧 (start_addr=0x00, addr_count=6: pos+vel+torque+kp+kd+linear_vel):
[0xAA][0x06][aim|0x80][0x48][0x00][0x05]
  [M0: pos(2B) vel(2B) torque(2B) kp(2B) kd(2B)]   ← 10字节/电机
  [M1: pos(2B) vel(2B) torque(2B) kp(2B) kd(2B)]
  ...
  [M6: pos(2B) vel(2B) torque(2B) kp(2B) kd(2B)]
[CRC8][0xFF]

总帧长: 1+1+1+1 + 72 + 1+1 = 78 字节
数据长度 0x48 = 2(前缀) + 7×10(电机数据) = 72
```

**每电机 10 字节布局**（按地址表顺序）：

| 偏移 | 字节 | 地址 | 内容 | 位数 | 物理范围 |
|------|------|------|------|------|---------|
| 0~1 | 2B | 0x00 | pos_ref（目标位置） | 16 bit | [-12.5, +12.5] rad |
| 2~3 | 2B | 0x01 | vel_ref（目标速度） | 12 bit | [-10.0, +10.0] rad/s |
| 4~5 | 2B | 0x02 | t_ref（前馈力矩） | 12 bit | 大关节 ±28 / 小关节 ±10 N·m |
| 6~7 | 2B | 0x03 | kp（位置增益） | 16 bit | [0, 500] |
| 8~9 | 2B | 0x04 | kd（速度增益） | 16 bit | [0, 5] |

#### 9.3.2 MIT 参数数据类

> 定义见 5.4.1 `state.py`，此处不再重复。关键设计：
> - `kp`/`kd` 默认为 `None`，由控制层根据电机编号自动填充安全默认值
> - 显式传入 `0` 表示零力矩（卸力），不会被自动填充
> - 显式传入具体数值表示用户自定义增益

#### 9.3.3 大小关节 Kp/Kd 差异化

由于前 3 个电机减速比是后 3 个的 4 倍，默认 Kp/Kd 需要差异化：

| 电机组 | 电机编号 | 减速比特征 | 默认 Kp | 默认 Kd |
|--------|---------|----------|---------|---------|
| 大关节 | M0~M2 (关节1~3) | 高减速比 (4×) | 150.0 | 2.0 |
| 小关节 | M3~M5 (关节4~6) | 标准减速比 | 20.0 | 1.0 |
| 夹爪 | M6 | — | 20.0 | 1.0 |

SDK 提供默认值，但用户可以在每次调用时覆盖：

```python
# 使用默认 Kp/Kd
robot.set_robot_state(target_joints=[0, 30, 45, 0, -30, 0])

# 自定义 Kp/Kd（高级用法）
robot.send_mit_command(
    joint_params=[
        MitParams(pos_ref=0.0, kp=200.0, kd=3.0),   # 关节1: 更硬
        MitParams(pos_ref=0.5, kp=50.0, kd=0.5),     # 关节2: 更软
        ...
    ]
)
```

#### 9.3.4 MIT 速度控制 — 线性轨迹插值

MIT 模式下还支持一种底层速度控制方式：**线性轨迹插值速度** (addr=0x05)。该功能让固件在 MIT 模式下按给定速度做线性插值到目标位置。

```
线性轨迹速度设定帧 (start_addr=0x05, addr_count=1):
[0xAA][0x06][aim|0x80][0x10][0x05][0x01]
  [M0_linvel_lo][M0_linvel_hi]
  ...
  [M6_linvel_lo][M6_linvel_hi]
[CRC8][0xFF]
```

**重要**：使用线性轨迹速度后，**必须在运动完成后将速度置零**，否则残留的速度值会影响后续运动：

```python
def _move_mit_with_linear_vel(self, target_joints, speed):
    """MIT 模式下使用线性轨迹插值的点位运动"""
    # 1. 设定线性轨迹速度
    self._send_linear_velocity(speed)
    # 2. 发送全参数 MIT 帧（pos=目标, vel=0, t=0, kp=默认, kd=默认）
    self._send_mit_frame(target_joints)
    # 3. 等待到达
    self._wait_for_target(target_joints)
    # 4. 清零线性轨迹速度（防止残留）
    self._send_linear_velocity(zero)
```

**适用场景**：MIT 模式下的简单点位运动（类似 PV 的使用体验，但底层仍在 MIT 模式）。

#### 9.3.5 MIT 无力矩模式（拖动示教 / 零位标定）

通过 **0x05 指令**卸力（原理：将选定关节的 kp=kd 置零）：

```python
def torque_off(self, joints=None):
    """卸力（仅 MIT 模式，0x05 指令）

    选定关节 kp=kd=0 → 自由运动
    未选定关节保持锁定
    """
    if self._mode != ControlMode.MIT:
        self.switch_mode(ControlMode.MIT)
    # 发送 0x05 指令，指定卸力关节
    self._send_torque_command(joints, lock=False)
```

**恢复力矩**：
```python
def torque_on(self, joints=None):
    """恢复力矩（发送 0x05 指令，固件侧恢复 kp/kd）"""
    self._send_torque_command(joints, lock=True)
```

#### 9.3.6 模式切换流程

```
模式切换（PV ↔ MIT）:
  1. 失能所有电机（0x09 失能）
  2. 发送 0x11, addr=0x0B, 值=目标模式
  3. 读回验证（失败重试一次）
  4. 使能所有电机（0x09 使能）
  5. 等待状态缓存刷新（0.2s）

固件侧负责目标位置初始化，SDK 不发送额外控制帧。
```

#### 9.3.7 MIT 遥操作支持

MIT 模式天然适合遥操作（高实时性、可调增益）：

```python
class JointController:
    def send_mit_command(self, joint_params: List[MitParams],
                         gripper: Optional[float] = None) -> None:
        """MIT 直接命令发送（无等待，用于遥操作/实时控制）

        以最低延迟发送全参数 MIT 帧。
        遥操作端以高频率（≥200Hz）调用此方法。
        每帧全部 6 地址都可独立变化。

        Args:
            joint_params: 7 个电机的 MIT 参数
            gripper: 夹爪值 (0~1000)，为 None 时使用 joint_params[6] 的 pos_ref
        """
```

**遥操作典型用法**：
```python
# 遥操作循环
while running:
    leader_state = leader_robot.get_robot_state("joint")
    follower_params = [
        MitParams(pos_ref=leader_state[i], vel_ref=0, t_ref=0,
                  kp=DEFAULT_KP[i], kd=DEFAULT_KD[i])
        for i in range(7)
    ]
    follower_robot.send_mit_command(follower_params)
    precise_sleep(1.0 / 200)  # 200Hz
```

### 9.4 API 层的模式适配

`SynriaRobotAPI.set_robot_state()` 是面向普通用户的统一入口，内部自动适配模式。高级用户直接调用 `send_mit_command()` 获得完整 MIT 控制能力。

```python
# === 普通用户（PV 和 MIT 体验一致）===
robot = create_robot(control_mode="pv")
robot.set_robot_state(target_joints=[0, 30, 0, 0, 0, 0], speed=15)  # PV: pos+vel 帧

robot.switch_mode("mit")
robot.set_robot_state(target_joints=[0, 30, 0, 0, 0, 0], speed=15)  # MIT: 线性轨迹速度 + 全参数帧

# === 高级用户（MIT 全参数控制）===
from alicia_m_sdk.types.state import MitParams

# 遥操作
robot.send_mit_command([
    MitParams(pos_ref=0.5, vel_ref=0, t_ref=0, kp=150, kd=2),   # 大关节
    MitParams(pos_ref=0.3, vel_ref=0, t_ref=0, kp=150, kd=2),
    MitParams(pos_ref=-0.2, vel_ref=0, t_ref=0, kp=150, kd=2),
    MitParams(pos_ref=0.0, vel_ref=0, t_ref=0, kp=20, kd=1),    # 小关节
    MitParams(pos_ref=0.1, vel_ref=0, t_ref=0, kp=20, kd=1),
    MitParams(pos_ref=0.0, vel_ref=0, t_ref=0, kp=20, kd=1),
    MitParams(pos_ref=500, vel_ref=0, t_ref=0, kp=20, kd=1),    # 夹爪
])

# 柔顺控制（降低 kp）
robot.send_mit_command([
    MitParams(pos_ref=0.5, vel_ref=0, t_ref=0, kp=30, kd=0.5),
    ...
])
```

### 9.5 轨迹执行的模式差异

PV 和 MIT 使用 `TrajectoryExecutor` 的不同方法，不做运行时自动切换：

```python
# PV 轨迹执行
executor.execute_pv(timestamps, positions, velocities)
# 内部: 逐帧构建 pos+vel(有符号) 帧 → 发送 → 同步时间戳

# MIT 轨迹执行
executor.execute_mit(timestamps, positions,
                     velocities=planned_velocities,   # 规划器输出 → vel_ref
                     torques=dynamics_feedforward)     # 动力学前馈 → t_ref
# 内部: 逐帧构建全参数 MIT 帧 → 发送 → 同步时间戳
```

**MIT 轨迹执行的优势**：
- 规划器输出的速度（一阶导数）可直接作为 `vel_ref`，改善跟踪性能
- 可选的动力学前馈 `t_ref` 进一步降低跟踪误差
- 实时可调的 `kp`/`kd` 支持变刚度控制

### 9.6 帧结构对比总结

| 字段 | PV 帧 | MIT 帧 |
|------|-------|--------|
| 帧头 | 0xAA | 0xAA |
| 指令 ID | 0x06 | 0x06 |
| 功能码 | aim \| 0x80 | aim \| 0x80 |
| 数据长度 | **0x1E** (30) | **0x48** (72) |
| start_addr | 0x00 | 0x00 |
| addr_count | **0x02** | **0x05** |
| 每电机字节 | **4** (pos+vel) | **10** (pos+vel+torque+kp+kd) |
| 7电机数据 | 28 字节 | 70 字节 |
| 校验位 | CRC8 | CRC8 |
| 帧尾 | 0xFF | 0xFF |
| **总帧长** | **36 字节** | **78 字节** |

### 9.7 控制参数汇总

| 参数 | 范围 | 说明 |
|------|------|------|
| 夹爪开合 | [0, 1000] | 0=完全闭合, 1000=完全打开 |
| 速度（用户层） | [0, 400] | 无量纲幅值，映射到 [0, 10] rad/s |
| 固件速度范围 | [-10, +10] rad/s | 有符号，SDK 根据运动方向计算符号 |
| 位置映射范围 | ±12.5 rad | 16 bit, [0, 65535] |
| 速度映射范围 | ±10.0 rad/s | 12 bit, [0, 4095] |
| 力矩映射-大关节 | ±28.0 N·m | 12 bit, [0, 4095]（M0~M2） |
| 力矩映射-小关节 | ±10.0 N·m | 12 bit, [0, 4095]（M3~M6，含夹爪） |
| Kp 范围 | [0, 500] | 16 bit, [0, 65535] |
| Kd 范围 | [0, 5] | 16 bit, [0, 65535] |
| MIT 大关节 Kp/Kd | 150.0 / 2.0 | 电机 0~2（减速比 4×） |
| MIT 小关节 Kp/Kd | 20.0 / 1.0 | 电机 3~6（标准减速比） |

### 9.8 编码实现参考

```python
# === conversion.py 中的映射函数 ===

def float_to_uint(value: float, min_val: float, max_val: float, bits: int) -> int:
    """物理浮点值 → 无符号协议整数（通用映射）"""
    span = max_val - min_val
    max_uint = (1 << bits) - 1
    result = int((value - min_val) / span * max_uint)
    return max(0, min(max_uint, result))

def uint_to_float(raw: int, min_val: float, max_val: float, bits: int) -> float:
    """无符号协议整数 → 物理浮点值"""
    max_uint = (1 << bits) - 1
    return raw / max_uint * (max_val - min_val) + min_val

# 位置: [-12.5, +12.5] rad ↔ [0, 65535]
def encode_position(rad: float) -> int:
    return float_to_uint(rad, -12.5, 12.5, 16)

def decode_position(raw: int) -> float:
    return uint_to_float(raw, -12.5, 12.5, 16)

# 速度: [-10.0, +10.0] rad/s ↔ [0, 4095]
def encode_velocity(rad_s: float) -> int:
    return float_to_uint(rad_s, -10.0, 10.0, 12)

# 力矩: 大关节 [-28.0, +28.0], 小关节 [-10.0, +10.0] N·m ↔ [0, 4095]
def encode_torque(nm: float, motor_index: int) -> int:
    tor_range = 28.0 if motor_index <= 2 else 10.0
    return float_to_uint(nm, -tor_range, tor_range, 12)

# Kp: [0, 500] ↔ [0, 65535]
def encode_kp(kp: float) -> int:
    return float_to_uint(kp, 0.0, 500.0, 16)

# Kd: [0, 5] ↔ [0, 65535]
def encode_kd(kd: float) -> int:
    return float_to_uint(kd, 0.0, 5.0, 16)
```

---

## 十、示例脚本设计

> 基于 `.claude/功能汇总与目标.md` 的功能需求，设计如下示例脚本。
> 标注「底层待更新」的功能，在代码中预留接口但以占位形式实现。

### 10.1 示例列表与操作流程

#### `00_demo_read_version.py` — 读取固件版本号
```
连接 → 查询版本信息 → 打印序列号/硬件版本/固件版本 → 断开
```

#### `01_demo_diagnostic.py` — 自检功能
```
连接 → 执行自检（底层待更新）→ 打印各关节电机健康状态 → 断开
```
> 底层自检功能尚未更新，预留 `get_robot_state("self_check")` 接口。

#### `02_demo_read_status.py` — 读取机械臂模式与使能状态
```
连接 → 查询当前模式（PV/MIT/重力补偿）→ 查询使能状态 → 打印 → 断开
```
> 底层读取模式信息功能尚未更新，预留接口。

#### `03_demo_read_states.py` — 读取关节状态
```
连接 → 循环打印:
  - 6 个关节角度（deg + rad）
  - 夹爪开合度 (0~1000)
  - 各关节速度 (rad/s)
  - 各关节力矩 (N·m)
  - （后期扩展：电机温度，底层待更新）
按 Ctrl+C 退出
```

#### `04_demo_switch_mode.py` — 切换控制模式
```
连接（PV 模式）
→ 打印当前模式
→ 用户 Enter → 切换为 MIT 模式 → 打印确认
→ 用户 Enter → 切回 PV 模式 → 打印确认
→ 断开
```

#### `05_demo_disable_enable.py` — 失能/使能交互
```
连接
→ 打印当前关节角度
→ 用户 Enter → 失能（卸载力矩，电机自由）
→ 用户手动移动机械臂到新位置
→ 用户 Enter → 在当前位置重新使能（确保不突跳到旧位置）
→ 打印新的关节角度
→ 断开
```

#### `06_demo_move_gripper.py` — 夹爪控制
```
连接
→ 打开夹爪 (1000)
→ 等待完成
→ 关闭夹爪 (0)
→ 等待完成
→ 移动到半开 (500)
→ 断开
注: PV 和 MIT 模式下均可运行
```

#### `07_demo_move_joint.py` — 关节控制
```
连接
→ 回零位 (go_home)
→ 移动到预设关节位置 A
→ 移动到预设关节位置 B
→ 回零位
→ 断开
注: PV 和 MIT 模式下均可运行，用户可通过参数选择模式
```

#### `08_demo_move_full_arm.py` — 关节+夹爪协同控制
```
连接
→ 回零位，夹爪打开
→ 演示1: 仅控制关节（夹爪保持不变）
→ 演示2: 仅控制夹爪（关节保持不变）
→ 演示3: 同时控制关节和夹爪
→ 回零位
→ 断开
注: PV 和 MIT 模式下均可运行
```

#### `09_demo_forward_kinematics.py` — 正运动学
```
连接
→ 读取当前关节角度 → 计算 FK → 打印末端位姿
→ 给定预设关节角度 → 计算 FK → 打印末端位姿
→ 断开
```

#### `10_demo_inverse_kinematics.py` — 逆运动学
```
连接
→ 读取当前末端位姿 → 求解 IK → 打印关节角度
→ 给定目标位姿 → 求解 IK → （可选）执行运动
→ 断开
```

#### `14_demo_teleop_mapped.py` — 遥操作
```
连接
→ 启动 WebSocket 服务 → 实时推送关节状态到 SparkVis
→ Ctrl+C 退出
```

#### `13_demo_reset_zero.py` — 零位标定
```
方案一（MIT 模式，当前可用）:
  连接（MIT 模式）
  → 用户 Enter → 切换 MIT 无力矩（Kp=0, Kd=0）
  → 用户手动将机械臂摆到零位
  → 用户 Enter → 执行零位标定 → 开力矩
  → 断开

方案二（重力补偿模式，固件待支持）:
  连接
  → 用户 Enter → 切换重力补偿模式
  → 用户手动将机械臂摆到零位
  → 用户 Enter → 执行零位标定
  → 断开
```

### 10.2 示例通用模式

所有示例遵循统一的代码结构，使用 `beauty_logger` 进行格式化输出（与 Alicia-D-SDK 保持一致）：

```python
"""XX_demo_xxx.py — 功能简述

演示 xxx 的用法。
"""
import alicia_m_sdk
from robocore.utils.beauty_logger import beauty_print, beauty_print_array

def main():
    beauty_print("Demo: 功能简述", type="module")

    # 创建并连接
    robot = alicia_m_sdk.create_robot(control_mode="pv")
    beauty_print("机器人连接成功", type="success")

    try:
        # --- 读取状态示例 ---
        state = robot.get_robot_state("joint_gripper")
        beauty_print("当前关节角度 (rad):")
        print(f"  joints = {beauty_print_array(state['angles'])}")

        # --- 运动控制示例 ---
        beauty_print("移动到目标位置...", type="info")
        robot.set_robot_state(target_joints=[0, 30, 0, 0, 0, 0], speed=15)
        beauty_print("运动完成", type="success")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")

if __name__ == "__main__":
    main()
```

**beauty_logger 使用规范**：
- `beauty_print(msg, type="module")` — 模块/功能标题（黄色）
- `beauty_print(msg, type="info")` — 一般信息（紫色）
- `beauty_print(msg, type="success")` — 操作成功（绿色）
- `beauty_print(msg, type="warning")` — 警告信息（灰色）
- `beauty_print(msg, type="error")` — 错误信息（红色）
- `beauty_print_array(arr, precision=5, sign=True)` — 格式化数组输出
- 来源：`robocore.utils.beauty_logger`（synria-robocore 包提供）
- **禁止**在示例中直接使用 `print()` 输出状态信息，统一使用 `beauty_print`

---

## 十一、固件待更新功能的扩展预留

以下功能当前固件尚未支持，但 SDK 架构需预留扩展点：

| 功能 | 预留接口 | 所在模块 |
|------|---------|---------|
| 读取当前控制模式 | `get_robot_state("control_mode")` | `device.py` / `synria_robot_api.py` |
| 读取电机温度 | `get_robot_state("temperature")` | `device.py` / `synria_robot_api.py` |
| 自检功能 | `get_robot_state("self_check")` | `device.py` / `synria_robot_api.py` |
| 切换重力补偿模式 | `switch_mode("gravity_comp")` | `joint_control.py` |
| SDK 侧切换 PV/MIT | `switch_mode("pv"/"mit")` | `joint_control.py` |

**扩展策略**：在 `protocol/constants.py` 中为新指令预留常量，在 `protocol/messages.py` 中定义消息类，在 `protocol/codec.py` 中实现编解码。上层通过 `device.py` 的统一接口调用即可。

---

## 十二、实施步骤建议

### 阶段一：基础框架
1. 创建 `types/` 目录，定义所有数据结构和枚举
2. 创建 `protocol/` 目录，实现 `constants.py`、`frame.py`
3. 实现 `utils/conversion.py` 中的单位转换函数

### 阶段二：协议与硬件
4. 实现 `protocol/messages.py` 和 `protocol/codec.py`
5. 实现 `hardware/serial_port.py`
6. 实现 `hardware/device.py`（含读取线程、轮询线程、StateCache）

### 阶段三：控制层
7. 实现 `control/joint_control.py`（含 PV/MIT 双模式支持）
8. 实现 `control/trajectory_executor.py`（含 PV/MIT 轨迹执行差异）
9. 实现 `control/teleoperation.py`（遥操作控制器）

### 阶段四：API 层与接口
10. 实现 `integrations/robocore/`（RoboCore 封装）
11. 实现 `api/synria_robot_api.py`（门面类）
12. 实现 `__init__.py`（包入口和 `create_robot()`）

### 阶段五：收尾
13. 按 10.1 编写全部示例脚本
14. 更新 `pyproject.toml` 依赖
15. 清理旧文件

---

## 十三、文件对照表（旧 → 新）

| 旧文件 | 新文件 | 说明 |
|--------|--------|------|
| `api/synria_robot_api.py` | `api/synria_robot_api.py` | 大幅精简，委托子模块 |
| `hardware/servo_driver.py` | `hardware/device.py` | 统一设备抽象 |
| `hardware/serial_comm.py` | `hardware/serial_port.py` | 纯 I/O 封装 |
| `hardware/data_parser.py` | `protocol/codec.py` + `protocol/messages.py` | 解析逻辑归入协议层 |
| `hardware/comm_manager.py` | _(移除)_ | 合并到 `device.py` |
| `execution/hardware_executor.py` | `control/trajectory_executor.py` | 统一轨迹执行 |
| `execution/trajectory_executor.py` | `control/trajectory_executor.py` | 合并 |
| `execution/drag_teaching.py` | _(已废弃)_ | 拖动示教功能已移除 |
| `execution/sparkvis.py` | _(移除或作为可选插件)_ | 非核心功能 |
| `utils/control_utils.py` | `utils/validation.py` | 重命名，职责更清晰 |
| `utils/unit_conversion.py` | `utils/conversion.py` | 扩展为含协议值转换 |
| `utils/calculate.py` | _(合并到 conversion/validation)_ | 消除碎片文件 |
| `utils/fps_utils.py` | `utils/timing.py` | 扩展为时间工具 |
| `utils/trajectory_utils.py` | _(部分合并到 `integrations/robocore/planning.py`)_ | 精简 |
| `utils/logger/beauty_logger.py` | `utils/logger.py` | SDK 内部日志 + beauty_logger 转发 |
| _(无)_ | `protocol/constants.py` | **新增**：协议常量集中管理 |
| _(无)_ | `protocol/frame.py` | **新增**：帧结构与 CRC |
| _(无)_ | `protocol/messages.py` | **新增**：消息类型定义 |
| _(无)_ | `protocol/codec.py` | **新增**：编解码器 |
| _(无)_ | `types/state.py` | **新增**：状态数据类 |
| _(无)_ | `types/config.py` | **新增**：配置数据类 |
| _(无)_ | `types/enums.py` | **新增**：枚举定义 |
| _(无)_ | `types/exceptions.py` | **新增**：统一异常体系 |
| _(无)_ | `control/joint_control.py` | **新增**：关节控制 |
| _(无)_ | `integrations/robocore/kinematics.py` | **新增**：运动学薄封装 |
| _(无)_ | `integrations/robocore/planning.py` | **新增**：规划薄封装 |
