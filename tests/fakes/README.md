# 测试替身（Test Fakes）

存放仅供自动化测试使用的确定性替代实现（Test Doubles）。

- **[`policy/`](policy/README.md)**：确定性 Fake Policy Provider，仅用于验证 gRPC 协议、流式调度与审计逻辑，不进入生产镜像业务路径，也不产生正式实验数据。

目录导航：[上级目录](../README.md)
