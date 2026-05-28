# Alicia-M SDK API 维护准则

文档状态日期：2026-05-20

本文是 Alicia-M SDK 公开 API、示例代码和目录边界的维护基准。调整公开导出、示例写法、目录结构说明或用户可见方法名时，先检查本文档。

## 1. 维护目标

本项目的 SDK 面向用户使用，公开入口要简洁、稳定、容易形成记忆。

1. 以 `README.md` 和 `docs/API_REFERENCE.md` 的用户入口说明为准。
2. 以 `docs/Python_SDK_SOP.md` 的通用 SDK 规范为约束。
3. `docs/ARCHITECTURE.md` 作为历史设计和架构参考，不强制覆盖当前实现。

维护时优先保证 API 稳定性。内部目录可以继续优化，但用户已经熟悉的重要函数名不主动改变。

## 2. 用户入口

普通用户只推荐一种学习路径：

```python
import alicia_m_sdk

robot = alicia_m_sdk.create_robot(...)
robot.xxx(...)
```

标准使用流程：

1. `import alicia_m_sdk`。
2. 使用 `alicia_m_sdk.create_robot(...)` 创建机器人对象。
3. 通过返回对象的 `robot.xxx(...)` 方法完成连接、状态读取、运动控制和维护设置。
4. 使用 `robot.disconnect()` 或上下文管理器释放连接。

普通文档和 demo 不新增其他用户必学入口。

## 3. API 边界

### 3.1 `api/` 目录

当前 `alicia_m_sdk/api/` 只保留：

- `__init__.py`：兼容性 re-export，不放业务逻辑。
- `synria_robot_api.py`：唯一用户 API 门面文件，定义 `SynriaRobotAPI`。

### 3.2 内部实现层

其他实现按职责放在包内实现层：

- `_internal/`：API 门面需要的内部辅助逻辑，例如连接握手。
- `execution/`：内部运动控制、轨迹执行、遥操作实现。
- `hardware/`：内部串口、协议帧、设备轮询和编解码实现。
- `integrations/`：内部或高级 RoboCore 适配。
- `utils/`：内部工具和 demo 辅助。
- `types/`：可公开的数据类、枚举、异常和配置类型。

### 3.3 `JointController`

`JointController` 不合并进 `synria_robot_api.py`。它继续作为执行层内部实现；旧脚本的兼容导入可以保留，但新用户文档不推荐直接导入。

## 4. 稳定名称

以下名称已经形成用户认知，不得在没有兼容别名和迁移说明的情况下改名：

- `create_robot`
- `get_robot_state`
- `set_robot_state`
- `switch_mode`
- `enable_robot`
- `disable_robot`
- `go_home`
- `set_gripper_target`
- `send_mit_command`
- `initialize_mit_gains`
- `torque_control`
- `run_diagnostic`
- `get_gripper_params`
- `set_gripper_params`

需要扩展行为时，优先保持方法名不变，并增加带保守默认值的可选参数。

## 5. 示例规则

普通 demo 应展示公开 API 风格：

```python
import alicia_m_sdk

robot = alicia_m_sdk.create_robot(...)
robot.set_robot_state(...)
```

普通 demo 不直接导入：

- `alicia_m_sdk.execution`
- `alicia_m_sdk.hardware`
- `alicia_m_sdk.integrations`
- `alicia_m_sdk.utils`

`examples/_demo_helpers.py` 是唯一文档化的 demo 声明适配层，只维护 demo 可使用对象的集中导入和 re-export 清单，例如 CLI 参数入口、打印函数、绘图函数、参数规格和展示元数据。

`examples/_demo_helpers.py` 不放具体 demo 的流程逻辑、参数解析流程、帮助文本构建函数、硬件访问代码或业务拼装函数。它的作用是让普通 demo 避免直接导入 SDK 内部模块，而不是成为新的工具函数目录。

普通 demo 文件只保留 `main()` 作为本文件内函数；脚本专属解析、格式化和说明文本在 `main()` 内组织。若某段逻辑被多个 demo 反复使用，应先判断它属于公开 API、SDK 内部工具还是 demo 声明清单，再按对应目录维护，不直接塞进 `_demo_helpers.py`。

## 6. 源码 Doxygen 注释规则

维护文档本身使用普通 Markdown；修改或新增源码时，注释和 docstring 需要兼容 Doxygen 风格。

- 新增 Python 文件需要模块 docstring，建议包含 `@file` 和 `@brief`。
- 新增或修改公开类、公开函数、重要内部函数时，应补充 `@brief`。
- 参数和返回值较多、类型不直观或行为有条件分支时，使用 `@param` 和 `@return`。
- 重要约束、兼容性说明、硬件风险和迁移注意事项使用 `@note`。
- 普通局部变量不强制逐个注释；协议常量、公开默认值、用户可见配置项需要有清晰注释或文档说明。

示例：

```python
def get_gripper_params(mask=0, aim="follower", timeout=1.0):
    """@brief 读取夹爪夹持参数。

    @param mask 参数掩码，0 表示读取全部参数。
    @param aim 目标部位。
    @param timeout 响应超时时间，单位秒。
    @return 解析后的响应；超时时返回 None。
    """
```

## 7. 当前结构

```text
alicia_m_sdk/
|-- __init__.py                 # 包顶层用户入口和稳定导出
|-- api/
|   |-- __init__.py             # 兼容性 re-export
|   `-- synria_robot_api.py     # 唯一用户 API 门面文件
|-- _internal/                  # 内部辅助实现，如连接握手
|-- execution/                  # 内部运动控制、轨迹、遥操作
|-- hardware/                   # 内部串口、协议、轮询、编解码
|-- integrations/               # 内部/高级 RoboCore 适配
|-- types/                      # 可公开的数据类、枚举、异常
|-- utils/                      # 内部工具和 demo 辅助
|-- diagnostics.py              # 自检实现，由 API 方法封装给用户
|-- user_settings.py            # 用户设置实现，由 API 方法封装给用户
`-- gripper_params.py           # 夹爪参数实现，由 API 方法封装给用户

docs/
`-- API_MAINTENANCE.md          # 本维护准则
```
