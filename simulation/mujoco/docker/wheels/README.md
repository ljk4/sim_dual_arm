# 固定 Ruckig Wheel 安装包

本目录存放预构建的 Ruckig 0.15.3 二进制安装包（适用于 Ubuntu 24.04 / CPython 3.12 / Linux `x86_64`），供仿真容器构建时直接离线校验安装，无需在镜像构建阶段编译 C++ 源码。

| 文件 | 用途说明 |
| --- | --- |
| `ruckig-0.15.3-cp312-cp312-linux_x86_64.whl` | 仿真端在线轨迹生成依赖（内置许可证元数据） |
| `provenance.json` | 上游源码版本、文件字节大小、SHA-256 摘要与目标平台记录 |
| `LICENSE` | Ruckig 上游 MIT 许可证 |

- 安装版本号与 SHA-256 哈希同步锁定于上级目录的 `requirements.lock` 中，由单元测试 [`test_container_build.py`](../../../../tests/unit/test_container_build.py) 自动校验一致性。
- 如需升级版本或迁移至其他 CPU 架构/Python 版本，须重新构建对应平台的 wheel 并同步更新 `provenance.json`、`requirements.lock` 与 `Dockerfile`。

目录导航：[上级目录](../README.md)
