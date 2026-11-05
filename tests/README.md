# 测试

- `unit/`：纯 Python 单元测试和共享夹具，默认不依赖 ROS/GPU。
- `integration/`：ROS 服务、原生物理、记录、任务和运行入口的集成检查。
- `doubles/`：test-only fake Policy 与 mock ROS backend，不冒充正式数据。

测试按 `pyproject.toml` 运行纯 Python 部分；进入已有运行或学习容器后执行该端
`run.sh test ...`。仿真端不维护单独测试工具清单，常规环境、场景与运动学验收使用
已有 `environment`、`scene`、`kinematics` 命令；官方 ROS 镜像自带的工具可以直接
使用，缺少的项目测试依赖再按实际需要增加。容器业务入口在宿主执行会明确拒绝。

内核/ROS 回归使用固定插件环境，禁止把 skip 当通过：

```bash
# 进入对应容器后：
runtime/ros2/scripts/run.sh test -q tests/unit
learning/lerobot/scripts/run.sh test -q tests/unit
```

运行和学习入口的 `test` 调用公共 `infrastructure/scripts/test.sh`，由各自环境
提供依赖。仿真入口不提供 `test` 或 SDK 构建命令；ROS SDK 在运行容器生成后
只读挂载到仿真容器。不要修改训练依赖，也不要在宿主启动训练环境。

`run.sh test` 与公共 `test.sh` 固定 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，避免 ROS `launch_testing`
自动插件提前导入未选择的 Torch 测试。
测试产物和缓存写到 `.build/`/`var/`，不提交。

目录导航：[上级目录](../README.md)。
