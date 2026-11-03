# Learning 镜像

| Target | 用途 |
| --- | --- |
| `protocol`、`policy` | ROS-free 协议/fake 服务依赖；源码和 SDK 都必须挂载。 |
| `act-system` | Ubuntu 24.04 Python 3.12、FFmpeg、libpython3.12 和 libgomp 运行层。 |
| `act-deps` | 按 `../learning-requirements.lock` 重建固定 94 包环境。 |
| `act` | 跨机器入口；源码、固定 LeRobot、权重和数据只读挂载，输出可写。 |
| `act-dev` | 本机入口；只读挂载已校验的宿主 venv，不重复安装 94 包。 |

默认 `act-system` 使用 `ubuntu:24.04`，便于移植；本机可用已验证的 system-probe
镜像作为 `ACT_SYSTEM_IMAGE`。LeRobot 版本/来源在 `../learning-source.json` 固定，
许可随上游保留；容器离线，训练不下载 backbone 或写 Hub/W&B。

Policy SDK 由 `bash tools/build/policy_sdk.sh` 生成到宿主构建目录。运行时挂载
仓库到 `/workspace:ro`、SDK 到 `/opt/policy-sdk:ro`；入口先校验当前 proto 摘要和
实际模块来源。缺少挂载、SDK 过期或导入回退均拒绝，不能依赖镜像内旧版本。
fake/ACT Compose 均显式选用此校验入口；ACT 训练入口保持独立，不要求 Policy SDK。

目录导航：[上级目录](../README.md)。
