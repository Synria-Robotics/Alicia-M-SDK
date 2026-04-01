# Alicia-M-SDK

Synria 云擎（Alicia-M）系列 6-DOF 机械臂 Python SDK。

## 安装

```bash
git clone https://github.com/Synria-Robotics/Alicia-M-SDK.git
cd Alicia-M-SDK
conda create -n msdk python=3.10
conda activate msdk
pip install -e .
```

## 控制模式

| 模式 | 说明 | 适用场景 |
|------|------|---------|
| **PV** | 位置-速度模式，固件自行插值 | 常规点位运动 |
| **MIT** | 阻抗控制，每帧发送全部 5 参数 | 遥操作、力控、柔顺控制 |

## 项目结构

```
alicia_m_sdk/
├── api/            # 用户 API 层（SynriaRobotAPI 门面类）
├── protocol/       # 协议层（帧结构、编解码、常量定义）
├── hardware/       # 硬件层（串口驱动、设备抽象、状态缓存）
├── control/        # 控制层（关节控制、轨迹执行、示教模式）
├── types/          # 类型定义（数据类、枚举、异常）
├── utils/          # 工具层（单位转换、参数校验、日志）
├── kinematics.py   # 运动学接口（RoboCore 封装）
└── planning.py     # 规划接口（RoboCore 封装）
```

## License

MIT License - Synria Robotics
