# 通信协议快速参考

本文是 Alicia-M SDK 维护和调试用速查，不替代完整协议。完整帧示例、参数范围和协议细节以 `docs/云擎通讯协议 - 公开版 v1.0.6.md` 为准。

普通 SDK 用户优先阅读 `README.md` 和 `docs/API_REFERENCE.md`，不需要直接拼协议帧。

## 帧结构

```text
[0xAA] [cmd_id] [func_code] [length] [data...] [crc_low8] [0xFF]
  帧头    指令ID    功能码    数据长度   有效数据    CRC32 低 8 位   帧尾
```

校验位为：

```text
CRC32(cmd_id + func_code + length + data) & 0xFF
```

帧头 `0xAA` 和帧尾 `0xFF` 不参与计算；SDK 使用 `binascii.crc32()`。

## 功能码约定

| 项 | 规则 |
|----|------|
| 读写方向 | bit7：`0`=读取，`1`=写入 |
| 示教臂 | bit0：`0x01` |
| 操作臂 | bit1：`0x02` |
| 写示教臂 | `0x81` |
| 写操作臂 | `0x82` |
| 反馈功能码 | 多数反馈会把请求功能码最高位置 1，例如 `0x02 -> 0x82` |
| 多字节数据 | 小端序 |
| 12 bit 数据 | 放在 2 字节槽位中，低 12 位有效 |

## 指令速查

| ID | 功能 | SDK 入口 |
|----|------|----------|
| `0x01` | 设备信息读取 | `get_firmware_version()` |
| `0x02` | 用户个性化设置 | `get_user_settings()` / `set_gripper_type()` |
| `0x03` | 协议层调零 | `set_zero_position()` |
| `0x05` | 机械锁和刚度切换 | `torque_control()` |
| `0x06` | 关节数据读写 | `get_robot_state()` / `set_robot_state()` |
| `0x09` | 使能和失能 | `enable_robot()` / `disable_robot()` |
| `0x11` | 电机参数设置 | `switch_mode()` |
| `0x15` | 清除电机错误 | 暂未封装为普通用户 API |
| `0x16` | 控制锁定 | 暂未封装为普通用户 API |
| `0x17` | 夹爪夹持参数 | `get_gripper_params()` / `set_gripper_params()` |
| `0xFB` | 串口帧率统计 | 调试用途，暂未封装为普通用户 API |
| `0xEE` | 错误反馈 | SDK 自动解析 |

`0xFE` 自检是当前 SDK/固件扩展能力，公开版 v1.0.6 协议原文未列入正式指令总览；SDK 通过 `run_diagnostic()` 暴露。

## 0x06 关节数据

每个地址固定占 2 字节。基础轮询读取 3 个地址，扩展轮询读取 7 个地址。

| 地址 | 内容 | 位数 | 说明 |
|------|------|------|------|
| `0x00` | 位置 `pos` | 16 bit | ±12.5 rad |
| `0x01` | 速度 `vel` | 12 bit | ±10.0 rad/s |
| `0x02` | 力矩 `tor` | 12 bit | 大关节 ±28 / 小关节 ±10 N·m |
| `0x03` | Kp | 16 bit | [0, 500] |
| `0x04` | Kd | 16 bit | [0, 5] |
| `0x05` | 线性插补速度 `linear_vel` | 12 bit | ±10.0 rad/s |
| `0x06` | 线圈温度 | 16 bit | 只读 |

帧模式速查：

| 场景 | addr_count | 内容 |
|------|------------|------|
| PV 写控制 | 2 | `pos + vel` |
| MIT 标准全参数 | 5 | `pos + vel + torque + kp + kd` |
| MIT 线性插补 | 6 | `pos + vel + torque + kp + kd + linear_vel` |
| 扩展状态轮询 | 7 | `pos + vel + torque + kp + kd + linear_vel + temperature` |

写入速度、力矩、线性插补速度时，原始值 `FF FF` 会被下位机按精确 `0` 处理；该特殊值不用于位置、Kp、Kd 或温度字段。

## 0x17 夹爪夹持参数

`0x17` 用于读取或设置夹爪力矩和力控相关参数，一帧只选择一条机械臂。

| 场景 | 数据区 |
|------|--------|
| 读取全部参数 | 空 |
| 读取部分参数 | `mask` 1 字节 |
| 写入参数 | `mask` 1 字节 + 按掩码低位到高位排列的 `float32` |
| 写入并请求保存 | 写入参数数据区末尾追加 1 字节保存标志 |

保存标志为 `0x00` 时仅立即生效；保存标志为非 `0` 时，设备会保存写入后的完整夹爪参数配置。SDK 默认不保存，用户需要显式使用 `set_gripper_params(..., save=True)` 或 15 号 demo 的 `--save`。

| 参数 | 小夹爪/DM4310 | 大夹爪/DM4340 | 默认值（小/大） |
|------|---------------|---------------|-----------------|
| `target_force` | `{0} ∪ [28, 138.9] N` | `{0} ∪ [35, 388.9] N` | `83.3 / 83.3`；0 为零保持力 |
| `open_feedforward` | `[0, 2.5] N*m` | `[0, 7.0] N*m` | `0.8 / 1.1` |
| `close_feedforward` | `[-2.5, 0] N*m` | `[-7.0, 0] N*m` | `-1.0 / -1.5` |
| `hold_torque` | `[0, 2.5] N*m` | `[0, 7.0] N*m` | `2.5 / 3.5`；只表示保持上限 |
| `force_kp` | `[0, 2]` | `[0, 2]` | `0 / 0` |
| `force_ki` | `[0, 2] 1/s` | `[0, 2] 1/s` | `0 / 0` |
| `integral_limit` | `[0, 100] N*s` | `[0, 100] N*s` | `20 / 30` |
| `close_torque_scale` | `[0, 1]` | `[0, 1]` | `0.35 / 0.40` |

正 `target_force` 本身形成目标力前馈，`force_kp/force_ki` 只提供可选误差修正；该 N 值未经力传感器标定。固件始终保证 `target_force * 0.018 <= hold_torque`，必要时会自动下调目标力；下调后低于可靠正区间则归零。

## SDK 维护注意

- `0x02` 仅在 SDK 中用于读取用户设置和写入夹爪类型；普通用户不要手写该帧。
- `0x17` 的 8 个夹爪参数、范围、保存标志和示例以完整协议和 `alicia_m_sdk/gripper_params.py` 为准。
- `0x11` 普通入口主要是 `switch_mode()`；模式 payload 末尾追加非零保存标志时写入 ESC Flash，SDK 默认不保存。加速度、减速度和电机环路参数属于高级调试。
- `0xEE` 错误反馈由 SDK 自动解析；排查时再参考完整协议的错误表。
- 协议常量变更时，同步检查 `alicia_m_sdk/hardware/constants.py`、`alicia_m_sdk/hardware/codec.py` 和本文档。
