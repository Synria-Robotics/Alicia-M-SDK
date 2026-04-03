## 修改原则：

- **规范**： 清晰的分层架构，每层职责明确，单向依赖
- **优雅**： 统一的接口风格，一致的命名规范，简洁的内部实现
- **易用**： 提供直观的工厂函数和配置方式
- **可扩展**： 协议层可适配不同机型，执行器可插拔，支持新增控制模式
- **中文注释**： 全部注释和文档使用中文，公开 API 使用 Google 风格 docstring
- **异常规范**： 统一异常体系，异常消息可读、含上下文，禁止裸抛原生异常

## 通信协议：
[~/Downloads/Downloads/云擎通讯协议v1.0.0 (公开版).pdf](<../../../Downloads/云擎通讯协议v1.0.0 (公开版).pdf>)

https://docs.sparklingrobo.com/docs/alicia-m-series/protocol/doc_00_intro

## 参考项目：
- 项目架构/文件目录参考https://github.com/Synria-Robotics/Alicia-D-SDK/tree/v6.1.0
- 废弃的原项目（尽量保留api中的方法和方法名，但更改/优化其实现方式）https://github.com/Synria-Robotics/Alicia-M-SDK/tree/v1.0.0-deprecated
- RoboCore库https://github.com/Synria-Robotics/RoboCore