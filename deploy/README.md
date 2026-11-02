# 部署文件

负责镜像构建、容器组合、设备/网络/挂载绑定。目录规划见 [组织规范](../docs/repository-layout.md)。

- `examples/sim_collect.yaml`：已有的设计示例，不是可执行 Compose 文件。
- `docker/`：ROS 基础层、backend-sim 的 mock 与 `mujoco-dev` target、runtime/test、learning 的 protocol/policy/ACT 系统与依赖 target，以及 `dev/` 开发依赖镜像。`mujoco-dev` 只含 pinned 仿真依赖和 EGL/OpenGL loader；完整双臂场景、训练和真机镜像仍待实现。
- `compose/mock.yaml`：当前 mock 开发组合，包含只读源码/生成契约挂载、按调用者 UID/GID 写入的原始数据和日志目录，以及一次性的 `build` profile builder。
- `compose/mock_collect.yaml`：自动采集的小型覆写，只把 runtime 命令加上 `--once`；镜像、环境、网络和挂载仍来自 `mock.yaml`，不复制第二套栈。
- `compose/mock_audit.yaml`：独立 audit 的小型只读覆写；workspace/build/raw/runs 全部 `read_only: true`，不允许工具容器修改原始记录。
- `compose/native_backend.yaml`、`compose/native_runtime.yaml`：已验收的 MuJoCo/ROS backend 与原生诊断 runtime；源码/SDK/资产只读，私有 network。
- `compose/native_policy.yaml`：叠加在 backend+runtime 上的连续 Policy overlay；新增无 ROS/无 GPU 的 fake policy 容器，源码/配置/SDK 只读、数据/log 可写。入口为 `tools/run/native_policy.py`。
- `compose/native_act_policy.yaml`：独立 ACT overlay；只叠加 backend+runtime，不启动 fake-policy，复用 learning:act-dev 入口与只读 venv/LeRobot/权重 cache/bundle 挂载，运行期 HF cache 写到 `/data/act-cache`，端口健康后才启动 runtime。入口见 [ACT Policy 服务](../docs/act-policy-service.md)。
- `compose/sim_check.yaml`：单个一次性 `sim-check` 服务；源码、生成契约、Panda 资产和摘要只读，只有报告目录 RW，按调用者 UID/GID 运行并要求 `gpus: all`。
- `local/`：当前机器的非公开绑定与覆盖，已通过根 .gitignore 排除。

控制、采集、模型和 episode 逻辑属于 components；deploy 只启动它们。连续 Policy 的完整操作顺序见 [连续 Policy 说明](../docs/native-policy.md)。ACT 数据/训练/模型包入口见 [学习数据与 ACT 训练](../docs/learning-dataset-act.md)。

## 仿真环境检查入口（S2a）

```bash
# 只重建 pinned 仿真依赖 target，原 mock target/default 不变
bash tools/build/sim.sh

# 获取/校验固定 Panda 资产；摘要写到 .build/assets/panda.json
python3 tools/assets/fetch_panda.py fetch
python3 tools/assets/fetch_panda.py verify

# 一次性原生检查：依赖/资产/单 Panda step/QP/NVIDIA EGL 预览
python3 tools/run/sim_check.py --run-id sim-check-001
```

`sim_check.py` 只准备只读挂载和隔离 Compose project，不启动机器人服务；
每次调用用随机容器名和 token label，所有退出分支只回收本次这一个工具容器，
外来同 project 容器保留。报告在
`.build/reports/sim-check/<run-id>/report.json`，宿主同时核对真实退出码和
报告状态。细节见 [仿真环境检查](../docs/simulation-environment.md)。这只是
S2a 环境检查，不是双臂物理场景或正式数据。

## 开发入口

依赖镜像只在依赖变化时重建；ROS 契约（`contracts/ros`）变化时先停服务再显式重建；普通 Python/YAML 修改只需用新 run id 重启服务。

```bash
# 首次或依赖变化：构建开发依赖镜像，并生成契约到 .build/ros-dev
bash tools/build/dev.sh

# 只重建开发依赖镜像，或只重建 ROS 契约
bash tools/build/dev.sh image
bash tools/build/dev.sh contracts

# 人工启动/查看/开始/结束/停止；公共选项放在子命令之前
python3 tools/run/mock.py --run-id dev-001 up
python3 tools/run/mock.py --run-id dev-001 status
python3 tools/run/mock.py --run-id dev-001 start --task-id mock-script
python3 tools/run/mock.py --run-id dev-001 end --episode-id <start 返回的 ID> --reason OPERATOR_STOP --wait
python3 tools/run/mock.py --run-id dev-001 start --task-id mock-script --wait
python3 tools/run/mock.py --run-id dev-001 down

# 仅转发 Compose 的底层入口仍可用于 ps/logs/config
python3 tools/run/dev_compose.py --run-id dev-001 logs -f runtime
```

整次自动采集使用同一组合，叠加 `compose/mock_collect.yaml`：

```bash
bash tools/run/mock_collect.sh
# 可选：显式隔离项目、run id 和挂载根
python3 tools/run/mock_collect.py --project collect-demo-01 --run-id demo-01 \
  --raw-root var/data/raw --runs-root var/runs --probe-backend
```

`collect_run.py` 负责准备、启动、监督、独立 audit 和收尾；`mock_collect.py` / `mock_collect.sh` 只是兼容入口。默认值和期限来自 `configs/deploy/mock_collect.json`（字段带 `_s` 或 `_bytes` 单位；未知字段、bool 冒充数字、NaN/Inf 和非正预算会被拒绝）。`var/runs/<run-id>/status.json` 汇总真实容器退出码、runtime 日志排空、独立 MCAP audit、backend 正常停止和 cleanup；`var/runs/<run-id>/launcher.json` 保存实际镜像 ID、有效配置及哈希、源码摘要和仅作参考的 Git HEAD。两者都不覆写 runtime 的 `resolved/`、`versions.json` 或 runtime 日志。

`mock.py` 复用 `dev_compose.py` 的路径准备、run id 校验和参数转发，不维护另一套 Compose 环境。`up` 先拒绝已有 Compose 项目，只创建挂载父目录；启动后必须实际看到 runtime/backend 运行、初始 `IDLE` 和 start/end 服务可发现才算成功，部分失败只清理经 project/service/run/挂载确认属于本次的容器（移除容器，不删除 bind 产物）。`status` 重新核对容器是否仍运行；缓存状态不能证明已退出的进程健康。`down` 在归属检查后停止指定栈，保留 `var/data/raw` 与 `var/runs` 产物，重复执行报告已停止。

`--timeout`（默认 30 秒）是本条宿主命令从服务发现、RPC 到等待终态的共享期限；`--project`、`--image`、`--build-root`、`--raw-root`、`--runs-root`、`--docker`、`--run-id` 与 `dev_compose.py` 含义相同。操作前会按 Compose project/service 标签、runtime 命令中的 run id、镜像和源码/build/raw/runs 挂载核对实际容器，不按容器名模糊匹配；项目内其他 run、缺失/多个 runtime、旧镜像或不匹配的输出根会被拒绝。人工 start/end 应串行操作；`end` 的 wire 不携带 episode id，检查与发送之间存在间隙。

构建输出根首次必须是空目录或尚不存在。builder 在锁内写入自己的 `workspace.json` 归属标记后，才允许清理 `build/install/log`；已有非空且未归属的目录会被拒绝且不删除任何文件。构建失败只保留失败产物和归属标记，去掉完成标记，可用同一输出根重试；已归属目录可重复构建。缓存使用 `COLCON_HOME=<output>/colcon-home` 和 `ROS_LOG_DIR=/tmp/ros-dev-logs`，不重设 `HOME`，也不依赖 `/root` 可写。

开发镜像不 COPY 项目入口，而是直接执行挂载仓库中的 `deploy/docker/entrypoints/dev_ros.sh`，因此修改入口不需要重建依赖镜像。入口的 check/status 诊断输出到 stderr，被调用应用的 stdout（例如 audit JSON）保持可解析。

源码以只读方式挂载到 `/workspace`，生成契约以只读方式挂载到 `/opt/ros-dev`；只有 `builder` 对 `.build/ros-dev` 有写权限。实际构建/运行命令和路径见 [阶段 2 使用指南](../docs/stage2-quickstart.md)。examples 下的通用绑定文件仍为设计示例，当前执行配方是 configs/runtime/mock_collect.yaml。
