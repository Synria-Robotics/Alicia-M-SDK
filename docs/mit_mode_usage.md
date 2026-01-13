# 快速开始：MIT模式配置

## 第一步：配置MIT参数（仅需一次）

```bash
python examples/00_config_mit_params.py
```

**重要提示：**
- ⚠️ 运行此脚本会使电机回到零点
- 请确保机械臂周围没有障碍物
- STM32不断电的情况下，只需配置一次

## 第二步：运行控制程序

```bash
python examples/05_demo_move_joint.py
```

现在可以正常使用MIT模式控制了！

## 何时需要重新配置？

只有在以下情况下需要重新运行配置工具：
- STM32控制板重新上电
- 第一次使用MIT模式

## 两种配置方式对比

### 方式一：手动配置（推荐✅）

```bash
# 只需要在STM32上电后运行一次
python examples/00_config_mit_params.py

# 之后可以多次运行控制程序，不会重复回零
python examples/05_demo_move_joint.py
python examples/05_demo_move_joint.py  # 再次运行，电机不会回零
```

**优点：**
- 只在需要时配置一次
- 控制程序运行时电机不会回零
- 更安全，更可控

### 方式二：自动配置（不推荐❌）

在代码中设置 `auto_init_mit=True`：

```python
robot = alicia_m_sdk.create_robot(
    ...
    auto_init_mit=True  # 每次运行都会使电机回零！
)
```

**缺点：**
- 每次运行控制程序时都会使电机回零
- 不安全，可能碰到障碍物

## 常见问题

**Q: 我修改了目标角度，需要重新配置吗？**  
A: 不需要！配置只是设置MIT模式参数，与目标角度无关。

**Q: 程序提示 "MIT mode requires parameter configuration"，怎么办？**  
A: 运行 `python examples/00_config_mit_params.py` 配置参数即可。

**Q: 为什么要单独配置？**  
A: 因为MIT模式需要特殊的控制参数（Kp、Kd等），这些参数需要写入STM32内存。分离出来可以避免每次运行程序都让电机回零。

## 详细文档

更多信息请参考：`examples/README_MIT_CONFIG.md`
