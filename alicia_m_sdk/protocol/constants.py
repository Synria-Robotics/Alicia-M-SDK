"""协议常量定义

集中管理所有通信协议相关的常量，消除魔数。
基于《云擎通讯协议 v1.0.0（公开版）》。
"""

# === 帧结构 ===
FRAME_HEADER = 0xAA            # 帧头
FRAME_FOOTER = 0xFF            # 帧尾
PLACEHOLDER = 0xFE             # 占位符

# === 指令 ID ===
CMD_VERSION        = 0x01      # 版本与设备信息获取
CMD_ZERO_RESET     = 0x03      # 位姿重置（设置当前位姿为初始位姿）
CMD_TORQUE         = 0x05      # 肢体力矩控制（关节力矩开关）
CMD_JOINT_STATE    = 0x06      # 关节与夹具状态控制
CMD_ENABLE         = 0x09      # 机器人部位失能/使能
CMD_MOTOR_PARAM    = 0x11      # 关节控制幅值和驱动参数设置
CMD_ERROR          = 0xEE      # 错误反馈

# === 0x03 零位标定方式 ===
ZERO_RESET_WEAK    = 0x00      # 弱调零：只改协议零点偏移
ZERO_RESET_STRONG  = 0x01      # 强调零：执行底层硬调零

# === 功能码：bit7 表示读写方向 ===
FUNC_READ_BIT      = 0x00      # bit7=0 → 读取
FUNC_WRITE_BIT     = 0x80      # bit7=1 → 写入

# === 功能码：部位标识（低7位）===
AIM_LEADER         = 0x01      # bit0=1 → 示教臂
AIM_FOLLOWER       = 0x02      # bit1=1 → 操作臂

# === 0x01 版本信息：专用功能码 ===
FUNC_VERSION_REQ   = 0x7E      # 版本请求功能码
FUNC_VERSION_RESP  = 0xFE      # 版本响应功能码 (0x80 + 0x7E)

# === 0x06 指令：自定义数据地址表 ===
ADDR_POSITION      = 0x00      # 当前位置/期望位置  16bit  rad
ADDR_VELOCITY      = 0x01      # 当前速度/期望速度  12bit  rad/s
ADDR_TORQUE        = 0x02      # 当前力矩/额外力矩  12bit  N/m
ADDR_KP            = 0x03      # 位置环 kp          16bit  0~500
ADDR_KD            = 0x04      # 速度环 kd          16bit  0~5
ADDR_LINEAR_VEL    = 0x05      # 线性轨迹速度       12bit  rad/s
ADDR_TEMPERATURE   = 0x06      # 线圈温度           16bit  只读
LINEAR_VEL_CLEAR   = 0xFFFF    # 线性轨迹速度清零信号（全字节 0xFF）

# === 0x11 指令：电机参数地址表 ===
MOTOR_PARAM_ACCEL       = 0x05  # 加速度 (float)
MOTOR_PARAM_DECEL       = 0x06  # 减速度 (float)
MOTOR_PARAM_MID_ID      = 0x08  # MID 反馈 ID (uint32_t)
MOTOR_PARAM_RECV_ID     = 0x09  # 接收 ID (uint32_t)
MOTOR_PARAM_CTRL_MODE   = 0x0B  # 控制模式 (0x01=MIT, 0x02=位置速度)
MOTOR_PARAM_POS_RANGE   = 0x16  # 位置映射范围 (float)
MOTOR_PARAM_VEL_RANGE   = 0x17  # 速度映射范围 (float)
MOTOR_PARAM_TOR_RANGE   = 0x18  # 扭矩映射范围 (float)
MOTOR_PARAM_VEL_KP      = 0x1A  # 速度环 Kp (float)
MOTOR_PARAM_VEL_KI      = 0x1B  # 速度环 Ki (float)
MOTOR_PARAM_POS_KP      = 0x1C  # 位置环 Kp (float)
MOTOR_PARAM_POS_KI      = 0x1D  # 位置环 Ki (float)

# === 0x11 控制模式值 ===
CTRL_MODE_MIT      = 0x01      # MIT 阻抗控制模式
CTRL_MODE_PV       = 0x02      # PV 位置速度模式
CTRL_MODE_SPD      = 0x03      # SPD 速度模式
CTRL_MODE_PSI      = 0x04      # PSI 模式

# 控制模式描述映射
CTRL_MODE_NAMES = {
    CTRL_MODE_MIT: "MIT",
    CTRL_MODE_PV:  "PV (位置速度)",
    CTRL_MODE_SPD: "SPD (速度)",
    CTRL_MODE_PSI: "PSI",
}

# === 0xEE 错误类型 ===
ERR_FRAME_HEADER    = 0x00     # 帧头/帧尾校验错误
ERR_LENGTH_MISMATCH = 0x01     # 数据长度校验错误
ERR_CRC_FAILED      = 0x02     # CRC32 校验不通过
ERR_SYSTEM_MODE     = 0x03     # 系统模式错误（机械臂类型检测失败）
ERR_ANGLE_LIMIT     = 0x04     # 电机角度限位中
ERR_MOTOR_OFFSET    = 0x05     # motorData 偏移与部位数量不符
ERR_ADDR_OVERFLOW   = 0x06     # insID 偏移超过最大地址

# === 0xEE 错误描述映射 ===
ERROR_DESCRIPTIONS = {
    ERR_FRAME_HEADER:    "帧头/帧尾校验错误",
    ERR_LENGTH_MISMATCH: "数据长度校验错误",
    ERR_CRC_FAILED:      "CRC32 校验不通过",
    ERR_SYSTEM_MODE:     "系统模式错误（机械臂类型检测失败）",
    ERR_ANGLE_LIMIT:     "电机角度限位中",
    ERR_MOTOR_OFFSET:    "motorData 偏移与部位数量不符",
    ERR_ADDR_OVERFLOW:   "insID 偏移超过最大地址",
}

# === 0x09 使能/失能数据值 ===
ENABLE_ON          = 0x01      # 使能
ENABLE_OFF         = 0x00      # 失能

# === 数据映射范围（默认值）===
POS_BITS           = 16        # 位置数据位数
VEL_BITS           = 12        # 速度数据位数
TOR_BITS           = 12        # 力矩数据位数
KP_BITS            = 16        # Kp 数据位数
KD_BITS            = 16        # Kd 数据位数

DEFAULT_POS_RANGE  = 12.5      # 位置映射范围 ±12.5 rad
DEFAULT_VEL_RANGE  = 10.0      # 速度映射范围 ±10.0 rad/s
TOR_RANGE_LARGE    = 28.0      # 大关节力矩映射范围 ±28.0 N·m (M0~M2)
TOR_RANGE_SMALL    = 10.0      # 小关节力矩映射范围 ±10.0 N·m (M3~M6，含夹爪)
KP_RANGE           = 500.0     # Kp 映射范围 [0, 500]
KD_RANGE           = 5.0       # Kd 映射范围 [0, 5]

# === MIT 默认增益 ===
DEFAULT_KP_LARGE   = 150.0     # 大关节默认 Kp (M0~M2)
DEFAULT_KD_LARGE   = 2.0       # 大关节默认 Kd (M0~M2)
DEFAULT_KP_SMALL   = 150.0     # 小关节默认 Kp (M3~M6)
DEFAULT_KD_SMALL   = 2.0       # 小关节默认 Kd (M3~M6)

# === 电机/关节数量 ===
NUM_JOINTS         = 6         # 云擎关节数量（不含夹爪）
NUM_MOTORS         = 7         # 电机总数（含夹爪）

# === 轮询地址数量 ===
POLL_ADDR_BASIC    = 3         # 基础查询: 位置 + 速度 + 力矩（兼容旧固件）
POLL_ADDR_EXTENDED = 7         # 扩展查询: + kp + kd + 插补速度 + 温度（仅新固件）

# === 数据帧属性字节标志 ===
FEEDBACK_BIT       = 0x80      # 响应帧 start_addr 的 bit7 = 1 表示反馈帧

# === 版本信息字段长度 ===
VERSION_SERIAL_LEN    = 16     # 序列号长度（字节）
VERSION_HARDWARE_LEN  = 4      # 硬件版本长度（字节）
VERSION_FIRMWARE_LEN  = 4      # 固件版本长度（字节）
