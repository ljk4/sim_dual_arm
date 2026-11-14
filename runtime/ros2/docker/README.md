# Runtime 容器构建与使用

- **基础镜像**：`ros:jazzy-ros-base-noble`（空闲启动 `CMD ["sleep", "infinity"]`，不内置业务源码）。
- **网络与资源**：接入 `ros` 与 `learning` 网络，无需 GPU。

## 常用操作

推荐使用仓库根目录 `./env.sh` 管理容器：

```bash
# 宿主机：构建并启动 runtime 容器，然后进入终端
./env.sh up runtime
./env.sh enter runtime

# 容器内：构建 SDK 或运行测试
scripts/build_sdk.sh
scripts/test.sh -q tests/unit
```

> `scripts/build_sdk.sh` 会一次性编译公共与方案扩展 ROS 接口至 `build/ros-dev`，并生成 Policy SDK 至 `build/generated/policy-sdk`；切换方案无需重新编译。
