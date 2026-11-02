# 仿真环境检查（S2a/S2b）

本入口有三个一次性检查模式：默认 `environment` 只验证可复现的
MuJoCo/mink 依赖、固定的 Panda 资产和本机 EGL 渲染；`scene` 进一步加载
双 Panda/柱/桌/两块真实场景，检查三相机静态布局可见性；`kinematics` 复用
同一 67 布局，对右臂求 6 个关键位姿并检查离散关节路径净空和关键视角。
三者都不启动机器人服务，也不代表 S2 整体、物理抓取或物理堆叠完成。

## 固定输入

- Python 依赖来自 `deploy/docker/sim-requirements.lock`，由
  `deploy/docker/sim-packages.json` 记录归一输入、版本和 wheel SHA-256。
  镜像用 `pip --require-hashes` 安装，不改成浮动版本。
- Panda 资产来自 `google-deepmind/mujoco_menagerie` 提交
  `f3b6013629c7b94f8ee93fd977b161fa8350f9bc` 的 `franka_emika_panda`
  子目录，许可证 Apache-2.0。来源只写在 `third_party/panda-source.json`，
  checkout 放在已忽略的 `third_party/src/`。
- 实际 mink wheel 为官方 1.3.0；不把分析参考提交当作 wheel 来源。

## 构建依赖镜像

```bash
# 只构建 mujoco-dev target；默认 mock target 不受影响
bash tools/build/sim.sh
```

默认镜像 `sim-dual-arm/backend-sim:mujoco-dev`，基于已验收的
`sim-dual-arm/dev-ros:stage2`。镜像只包含 pinned wheels、EGL/OpenGL loader
和 `/opt/sim/versions.json` 版本记录；不包含项目源码、资产、配置或数据。
NVIDIA 的 EGL vendor 实现由 container toolkit 在运行时提供，镜像不安装
CUDA toolkit、桌面系统或训练框架。Compose 明确使用 `runtime: nvidia`、
`NVIDIA_VISIBLE_DEVICES=all`、`gpus: all` 和
`NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics`；仅有 `nvidia-smi`
可用不等于 EGL/OpenGL 图形库已注入，必须以实际 OpenGL vendor/renderer 为
NVIDIA 为准。运行时注入机制见
[Container Toolkit specialized runtime](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html)。

## 获取与校验资产

```bash
python3 tools/assets/fetch_panda.py fetch
python3 tools/assets/fetch_panda.py verify
```

`fetch` 用 sparse/partial clone 只取得固定子目录并 checkout 固定提交，然后
在 `.build/assets/panda.json` 写内容摘要（相对路径、大小、分块 SHA-256 和
总摘要），摘要不会写进 vendor checkout。已有目录只有在属于本工具的
checkout、origin/提交匹配且整个 checkout 干净（包括未跟踪和忽略文件）时才复用；否则非零退出并保留文件，
不自动 reset/clean。首次克隆失败保留本次 staging 目录供定位。根
`.dockerignore` 排除 `/third_party/src/`，避免下载资产进入 Docker context。

## 一次性检查

```bash
python3 tools/run/sim_check.py
# 可显式指定镜像、build 根、资产/摘要/报告根和 run id
python3 tools/run/sim_check.py \
  --image sim-dual-arm/backend-sim:mujoco-dev \
  --build-root .build/ros-dev \
  --assets-root third_party/src/mujoco_menagerie \
  --summary .build/assets/panda.json \
  --report-root .build/reports/sim-check \
  --run-id sim-check-001
```

入口使用 `deploy/compose/sim_check.yaml` 的单个一次性 `sim-check` 服务，
不用 host network、Docker socket、特权、DISPLAY 或机器人服务。源码、生成
契约、资产和摘要只读；只有报告目录可写，并按调用者 UID/GID 运行。容器内
显式设置 `/workspace/components/backend_sim/src` 的 `PYTHONPATH`，不继承宿主
任意 `PYTHONPATH`。每次调用生成独立的随机容器名和 token label，Compose
`run` 以 `--name`/`--label` 传入；超时、信号和其他退出分支只 inspect 并
删除这一个匹配的容器，外来同项目容器保留，不使用 `compose down` 或按项目
批量删除。清理失败会返回非零。Compose/CLI 诊断通过共享
`BoundedDocker` 流式捕获，每流上限 256 KiB。

容器执行 `python -m robot_backend_sim.probe_environment`：

1. 核对 lock 中的包版本、导入来源和挂载源码未被镜像副本遮蔽；
2. 验证资产摘要与来源提交，加载上游 `scene.xml` 和 `panda.xml`，按真实
   keyframe 初始化并运行少量 MuJoCo step；
3. 确认 DAQP 在 `qpsolvers` 可用，求解一个已知正定 QP 并数值核对；
4. 用 MuJoCo Renderer 经 EGL 渲染 320×240 RGB PNG，记录 OpenGL
   vendor/renderer/version；软件回退或非 NVIDIA vendor 直接失败；
5. 通过公共 `EventLogger` 记录检查与收尾，日志排空后才发布成功报告。

## 报告与退出码

报告目录是 `<report-root>/<run-id>/`，内含 `report.json`、
`artifacts/panda_preview.png` 和 `logs/sim_probe/<instance>/`。报告记录
run/instance、Python/包版本、导入来源、资产提交与总摘要、模型自由度与执行
器数、QP 数值、像素统计、OpenGL 信息和产物相对路径，不写图像数组或环境
变量全集。宿主只读取常规 `report.json`，实际字节上限 2 MiB；FIFO、符号
链接和超限文件会被拒绝，读取不会在子进程期限结束后无界等待。

- `0`：进程真实退出 0，且对应报告为成功、run id 匹配、预览产物存在。
- `1`：检查失败；报告保留稳定 `reason`（如 `opengl_not_nvidia`、
  `asset_verification_failed`）。
- `2`：调用参数或路径错误。
- `124`：整体超时；只清理本次随机命名的工具容器。
- `130`/`143`：SIGINT/SIGTERM 中断并清理本次工具容器。

宿主同时检查真实退出码和报告，不按文件存在推导成功。源码/资产挂载变化
不需要重建依赖镜像；只有依赖或系统 loader 变化才重建。

## 三相机场景可见性检查

`scene` 模式仍使用同一个 `deploy/compose/sim_check.yaml` 服务、同一有界
Docker 启动/退出码/清理路径，只在宿主侧用固定命令覆盖容器 command，不接
受任意模块字符串：

```bash
python3 tools/run/sim_check.py --check scene --run-id scene-check-001
# 也可显式指定两个只读配置；容器内路径固定为 /sim-configs/*
python3 tools/run/sim_check.py \
  --check scene \
  --scene-config configs/simulation/panda_stack.yaml \
  --check-config configs/simulation/scene_check.yaml \
  --run-id scene-check-002
```

`environment` 仍是默认模式，原参数、报告格式和退出码保持不变。`scene`
模式把两个配置文件作为额外只读 bind mount 挂入，容器执行
`python -m robot_backend_sim.check_scene`；源码、资产、摘要和生成契约仍全部
只读，只有报告目录可写。

容器内的检查顺序：

1. 先建立 `EventLogger`，再用 `scene_config`/`visibility` 做全部有型校验；
   未知/重复字段、bool 冒充数字、NaN/Inf、错误 camera/object 名、无效
   seed/布局数/阈值在建 EGL 前失败；
2. 验证 pinned 依赖、挂载源码来源和固定资产摘要，只构建/编译场景一次；
3. 创建一个拥有私有 `MjData` 和 EGL context 的 Renderer。快照是
   `mjSTATE_FULLPHYSICS` 的独立只读拷贝；RGB 与可选分割来自同一快照，
   渲染不写物理 `Scene.data`；
4. 对 seed 11/23/37 正常布局和两个物体各 4 个 XY 角 × 2 个 yaw 极值的
   64 种联合边界布局，逐相机逐对象统计 `objtype==mjOBJ_GEOM` 且
   `objid==目标 geom id` 的实际分割像素，任何一项低于门槛即失败；不会
   自动降门槛，也不会用非黑图或颜色阈值代替实体 ID；
5. 每个布局保存三路原始 RGB PNG，并用首布局/最差像素布局拼一张带
   layout/camera 标注的 contact sheet；布局总数上限 100；
6. 用同一个 Renderer 做 RGB-only 预热 3 组、测量 20 组三视图吞吐，记录
   p50/p95/平均/组数/分辨率/wall time/GL 信息。该数字只是纯渲染吞吐，
   不是 500 Hz 物理 + 20 Hz 相机并发已达标。

## 关键位姿、IK、路径净空与视野检查

```bash
python3 tools/run/sim_check.py --check kinematics --run-id kinematics-check-001
```

`kinematics` 模式仍使用同一个 `deploy/compose/sim_check.yaml` 服务、同一有界
Docker 启动/退出码/清理路径，只在宿主侧用固定命令覆盖容器 command。除
场景与布局配置外，额外把 `configs/simulation/kinematics_check.yaml` 只读挂载
到 `/sim-configs/kinematics_check.yaml`。容器执行
`python -m robot_backend_sim.check_kinematics`，不接受任意模块字符串。

检查在 EGL/大量 IK 前完成严格配置校验，然后复用 `Scene` 和
`SceneRenderer`。每个布局独立 reset/derive 初始快照；只有右臂 7 个 hinge
qpos 可变，左臂、手指和两块冻结。6 个关键位姿通过 mink 1.3
`FrameTask + PostureTask`、`ConfigurationLimit + CollisionAvoidanceLimit`、
`DofFreezingTask` 硬约束和 DAQP 求解。初始姿态和相邻关键位姿的线性关节路径
按最多 0.01 rad 采样，用原生 `mj_geomDistance` 检查有符号净空，门槛 0.002 m；
报告记录最小净空、layout/segment/sample/geom/body、排除的碰撞对类别和依据。
每个 layout 首目标使用配置的右臂数值求解初值，其余 qpos 取该 layout 初始
快照；实际 `initial -> pregrasp` 路径仍从 home 开始检查。plane 等 `rbound=0`
几何不做球包围裁剪。

每个关键姿态从同一诊断快照渲染三路 RGB/分割 PNG，按配置的 phase 门槛统计
source/target/table 实体像素。PNG 总量上限 128 MiB，报告和日志有界；任何
IK、冻结、净空、视野、资源关闭或报告写入失败都使退出码非零。采样净空是
离散证明，不是连续碰撞证明；抓夹始终开启、两块不移动，lift/place 只是空手
关键姿态，S4 才验收真实抓取与搬运。详细参数和限制见
[关键位姿、IK、路径净空与视野检查](kinematic-scene-check.md)。

`scene` 报告是 `<report-root>/<run-id>/report.json`，保留
schema/kind/run/instance、resolved 配置及文件 SHA-256、源码/资产/依赖来源、
逐布局分割结论、吞吐、真实 GL vendor/renderer/version、产物相对路径、
`logs_closed`、`status`/`exit_code`/`reason`。证据在
`artifacts/layouts/<layout>/<camera>.png` 和 `artifacts/contact_sheet.png`；
日志目录名与 `instance_id` 一致。Renderer close、logger close 或报告写入
失败都会使真实退出码非零，成功只在资源收尾、日志排空和报告原子替换后发布。
