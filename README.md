# sim_dual_arm

双臂机器人仿真采集、ACT训练与推理工程。已完成50条合格程序示教、独立45/5数据转换、真实ACT训练和20次固定种子闭环执行。当前堆叠成功率为0/20，均在后续跟随偏差超限时受控停止；工程链路已跑通，策略效果尚未达标。详见[本轮结果](docs/simulation-loop-result.md)。

**当前运行入口：[最小仿真闭环操作与结果](docs/minimal-simulation-loop.md)。**
环境使用独立ROS runtime/backend与NumPy2学习进程，源码、SDK、资产和模型只读挂载；配置/模型/日志/台账可追溯。阶段细节见[执行记录](docs/simulation-loop-progress.md)，设计与历史验收见[文档导航](docs/README.md)。

推荐分为 backend-real、backend-sim、runtime、learning 四类环境；真实与仿真后端二选一，应用通过统一 Robot API 接入设备，通过 Policy API 调用模型。

从仓库根构建并运行：

```bash
# 发布/测试镜像（独立容器测试仍使用这些镜像）与 Compose 自动采集
bash tools/build/stage2.sh
bash tools/run/mock_collect.sh
```

自动采集入口现在复用 Compose 开发栈：一次调用启动 mock backend 和带 `--once` 的 runtime，执行脚本动作、生成测试图像，等待真实进程退出后检查 runtime 关键日志排空，再用独立工具容器审计 MCAP，最后有界停止 backend，只按已核对的本次容器/网络删除；同 project label 的外来资源保留并报告，不批量清理。默认生成新的 run id 和隔离项目，不需要 GPU、宿主机 ROS。

结果写在 `var/runs/<run-id>/status.json`：`succeeded` 只表示这次 mock 控制采集通过了进程退出、日志排空和独立审计门槛，不表示物理任务成功或正式训练数据合格。失败时保留 `status.json`、`launcher.json`、`audit.json`（若已执行）、关键日志、退出码和稳定 reason；退出码约定为正常 0、失败 1、配置错误 2、整体超时 124、SIGINT/SIGTERM 130/143。

S2a 仿真环境检查（只验证单 Panda 依赖/资产/EGL，不启动机器人服务）：

```bash
bash tools/build/sim.sh
python3 tools/assets/fetch_panda.py fetch
python3 tools/assets/fetch_panda.py verify
python3 tools/run/sim_check.py --run-id sim-check-001
```

S2b 三相机静态布局可见性检查（同一一次性工具容器，不启动机器人服务）：

```bash
python3 tools/run/sim_check.py --check scene --run-id scene-check-001
python3 tools/run/sim_check.py --check kinematics --run-id kinematics-check-001
```

`--check` 默认 `environment`，保持原单 Panda 环境检查；`scene` 使用
`--scene-config configs/simulation/panda_stack.yaml` 和
`--check-config configs/simulation/scene_check.yaml`，输出三路 RGB PNG、
分割阈值结论和接触表，不表示 S2 整体、IK 或物理堆叠完成。详细输入、报告
和失败语义见 [仿真环境检查](docs/simulation-environment.md) 与
[双 Panda 堆叠场景](docs/panda-stack-scene.md)。

`kinematics` 复用同一 67 布局，对右臂求 6 个关键位姿，检查初始姿态和相邻
关键位姿的关节线性路径净空，并为每个关键姿态生成三路 RGB/分割视野统计。
它冻结左臂、手指和两块，不执行抓取；详见
[关键位姿、IK、路径净空与视野检查](docs/kinematic-scene-check.md)。

开发挂载入口使用 Compose，源码和生成契约分层挂载：

```bash
# 首次或依赖变化：构建开发依赖镜像并生成 ROS 契约
bash tools/build/dev.sh

# ROS 契约修改先停服务再重建；Python/YAML 修改用新 run id 重启即可
bash tools/build/dev.sh contracts
python3 tools/run/mock.py --run-id dev-001 up
python3 tools/run/mock.py --run-id dev-001 status
python3 tools/run/mock.py --run-id dev-001 start --task-id mock-script
python3 tools/run/mock.py --run-id dev-001 end --episode-id <start 返回的 ID> --reason OPERATOR_STOP --wait
python3 tools/run/mock.py --run-id dev-001 down
```

`tools/run/mock_collect.sh`（或 `python3 tools/run/mock_collect.py`）是无需宿主机 ROS 的整次自动采集入口。旧 `--output` 现在明确表示一个原始数据目录 `<raw-root>/<run-id>`，不会再把日志写进同一目录；例如 `--output var/data/raw/my-run`。默认日志仍写到 `var/runs/<run-id>/`，详细迁移和期限说明见 [阶段 1～2 使用指南](docs/stage2-quickstart.md)。

`mock.py` 是无需宿主机 ROS 的人工入口：`up` 等待 runtime 初始 `IDLE` 和公共服务就绪，`status` 同时复查容器存活，`start/end --wait` 等待该 episode 的终态和最终 manifest。`end` 收到 `ENDING` ACK 只表示请求被接受，不代表记录已落盘。人工 start/end 需串行；命令只读取 manifest，不替代独立 MCAP audit，`audit_verified=false`。

开发栈只读挂载仓库到 `/workspace`，生成契约在 `.build/ros-dev/`；MCAP 和事件日志分别写到 `var/data/raw/<run_id>/` 与 `var/runs/<run_id>/`。详见 [部署说明](deploy/README.md) 和 [阶段 1～2 使用指南](docs/stage2-quickstart.md)。

- [阶段 1～2 使用与验证指南](docs/stage2-quickstart.md)

设计入口：

- [逐步实现路线图](docs/implementation-roadmap.md)
- [目录划分与组织规范](docs/repository-layout.md)
- [环境边界与通信接口](docs/architecture-and-interfaces.md)
- [环境与依赖分析](docs/docker-dependency-analysis.md)
- [仿真环境设计](docs/simulation-design.md)
- [全部文档](docs/README.md)

主要实现与契约：

- [Policy API v1 草案](contracts/policy/v1/policy.proto)
- [仿真采集部署绑定示例](deploy/examples/sim_collect.yaml)
- [公共 ROS 接口](contracts/ros/robot_api_msgs/CMakeLists.txt)
- [后端控制核心](libraries/backend_core/src/robot_backend_core/engine.py)
- [runtime 与采集](components/runtime/ros/dual_arm_runtime/dual_arm_runtime/node.py)

数据与实验产物放 var，可重建结果放 .build。mock 记录明确标记为测试数据；旧 learning:protocol 镜像只验证 SDK，实际训练与ACT服务使用独立的 learning:act-dev 环境。
