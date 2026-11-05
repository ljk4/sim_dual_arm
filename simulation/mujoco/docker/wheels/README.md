# 固定 Ruckig 安装包

本目录保存 Ruckig 0.15.3 wheel，适用于 Ubuntu 24.04 / CPython 3.12 /
Linux x86_64。Docker 构建直接校验并安装该二进制产物。

| 文件 | 用途 |
| --- | --- |
| `ruckig-0.15.3-cp312-cp312-linux_x86_64.whl` | 仿真轨迹生成依赖，保留包内许可证 |
| `provenance.json` | 上游来源、文件 SHA256、大小与平台记录 |
| `LICENSE` | 上游 MIT 许可证 |

安装版本与 SHA256 统一写在上级 `requirements.lock`。Dockerfile 把锁和 wheel
复制到 `/opt/dependencies`，由同一次 `pip --require-hashes` 安装；
`provenance.json` 留在源码目录供维护核对。

升级版本或更换 Python/CPU/系统平台时，需要生成并验证匹配的新 wheel，同步更新
`provenance.json`、`requirements.lock` 和 Dockerfile 的 COPY 路径。平台不匹配时
构建失败，不自动退回源码编译。产物生成记录见
[DSH-056](../../../../docs/history/README.md#dsh-056)。

目录导航：[上级目录](../README.md)。
