# Alicia-M 出货 SOP 测试程序总结

## 目标

`scripts/sop_test.py` 是面向 Alicia-M 机械臂出货验收的标准测试入口。它用于判断当前机械臂是否满足交付条件，包括设备识别、零点准确性、基础状态、自检、使能链路、小行程运动、夹爪动作和可选老化循环。

该程序的定位是“判定是否合格”，不是“自动维修或自动标定”。尤其是零点检查失败时，程序只会输出 FAIL 和误差数据，不会自动写入零点。

## 默认出货流程

默认 `quick` profile 会按以下顺序执行：

1. 安全配置检查：限制速度、小行程幅度、夹爪目标范围和老化参数。
2. 连接并识别设备：记录串口、固件版本、硬件版本、序列号等信息。
3. 零点准确性检查：机械臂放入固定工装或标准姿态后，读取 6 轴角度和夹爪值，与标准值和容差比较。
4. 基础状态读取：记录关节角、夹爪值、速度、力矩、温度、运行状态和控制模式。
5. 固件自检：检查通信位、电机状态和控制模式是否符合预期。
6. 运动前安全门禁：检查电机错误位、温度上限和可选力矩上限。
7. 使能/失能测试：验证基础控制链路。
8. 小行程关节运动：只在零点附近执行保守动作，并检查到位误差。
9. 夹爪测试：执行开、关、中间位动作并检查反馈误差。
10. 安全退出：默认调用 `disable_robot()` 后断开连接。

任何关键步骤失败都会短路后续危险动作，并生成 FAIL 报告。

## 安全措施

程序内置以下安全措施：

- `--max-safe-speed`：限制 SOP 允许的最大速度，默认 `40`。
- `--max-motion-delta-deg`：限制每个关节的小行程幅度，默认 `10 deg`。
- 零点超差即 FAIL，不继续运动。
- 自检失败即 FAIL，不继续运动。
- 运动前检查 `RobotStatus.has_motor_error` 和 `JointState.run_status` 错误位。
- 可配置温度阈值 `--max-temperature-c`。
- 可配置力矩阈值 `--max-abs-torque`。
- 默认退出前自动失能。
- 可使用 `--require-confirmation` 在每个运动步骤前要求操作员输入 `YES`。

只有显式传入 `--keep-enabled-on-exit` 时，程序才不会在退出前自动失能。

## 常用命令

产线默认出货测试：

```bash
python scripts/sop_test.py --port COM37 --profile quick --serial AMxxxx --operator station-1
```

首次验证工装或开放空间调试时，建议启用人工确认并降低速度：

```bash
python scripts/sop_test.py --port COM37 --profile quick --serial AMxxxx --speed 10 --max-safe-speed 20 --require-confirmation
```

指定零点标准值和每关节容差：

```bash
python scripts/sop_test.py --port COM37 --zero-joints-deg 0,0,0,0,0,0 --zero-tolerances-deg 1,1,1,1,1,1
```

执行 120 分钟老化测试：

```bash
python scripts/sop_test.py --port COM37 --profile aging --duration-min 120 --speed 15
```

执行固定轮次老化测试：

```bash
python scripts/sop_test.py --port COM37 --profile aging --aging-cycles 500 --aging-interval-s 0.5
```

## 报告输出

每次测试会输出三种报告，默认保存在 `logs/sop`：

- JSON：完整结构化报告，适合系统归档和数据分析。
- CSV：每台设备一行摘要，适合产线统计。
- Markdown：人工阅读版本，包含 PASS/FAIL、步骤结果和失败原因。

报告中会记录：

- 测试 profile。
- 开始和结束时间。
- 设备序列号、端口、版本信息。
- 操作员或工位信息。
- 每个步骤的 PASS/FAIL、耗时、错误信息和关键 metrics。
- 最终 PASS/FAIL。

## 代码结构

核心实现位于 `alicia_m_sdk/sop.py`：

- `SopConfig`：SOP 参数配置。
- `SopRunner`：测试流程编排器。
- `SopStepResult`：单步骤结果。
- `SopReport`：完整报告。
- `run_sop()`：运行 SOP 的便捷入口。

命令行入口位于 `scripts/sop_test.py`。它只负责参数解析和打印结果，具体测试逻辑由 `alicia_m_sdk.sop` 负责。

## 测试覆盖

单元测试位于 `tests/test_sop.py`，覆盖：

- 正常 quick 流程通过并生成报告。
- 连接失败生成 FAIL 报告。
- 零点超差后短路，不继续运动。
- 固件自检失败。
- 运动命令失败。
- aging profile 固定轮次执行。
- 速度等不安全配置会在连接前失败。
- 运动前检测到电机错误时不会使能。

运行验证：

```bash
python -m pytest
```

