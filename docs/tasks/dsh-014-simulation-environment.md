# DSH-014：S2a 仿真依赖、Panda 资产与 EGL 检查入口

前置：S1 已由 DSH-013 验收。此任务只建立可复现依赖、资产获取和本机渲染检查；完整双臂/柱/桌/方块/三相机场景是下一任务。不得把单 Panda 环境检查写成物理堆叠、正式数据或 S2 整体完成。

## 已核对输入

- ROS Jazzy / Python 3.12 / NumPy 1.26.4；默认 dev 镜像和 `.build/ros-dev` 已实际生成，Python 应用从挂载导入。
- 本机 RTX 4050 Laptop 6 GB，当前已有其他 Docker/GPU 应用，不属于本任务范围。
- 主助手已在一次性 ROS 基线容器中完成官方 wheel dry-run。直接需求为 MuJoCo 3.17.0、mink 1.3.0、Pillow 13.0.0、NumPy 1.26.4；共 15 个包，版本及 wheel SHA-256 已确定。
- 归一输入：`/home/ljk/work/sim_dual_arm/.build/reports/s2-dependencies/sim-requirements.lock`、同目录 `resolved-wheels.json`。原报告路径记录在归一 JSON 内。只能复制/核对这些已解析数据，不能改为浮动版本或凭空写测试通过。
- 实际使用的 mink 为官方 wheel 1.3.0，官方标签 `v1.3.0` 对应 `14625beca2ce0918f88d1fc84a3c0cdb591e0729`。此前 `825858...` 是分析参考提交，不可谎称当前 wheel 就来自它。
- Panda 资产固定为 `google-deepmind/mujoco_menagerie` 提交 `f3b6013629c7b94f8ee93fd977b161fa8350f9bc` 的 `franka_emika_panda`；该子目录许可证 Apache-2.0。保留其原始模型、网格、许可和来源，不手写替代机器人。

## 允许文件与预算

新增/修改限：

- `deploy/docker/backend-sim/Dockerfile`：新增 `mujoco-dev` target；原 mock target/default 构建保持可用。
- 新增 `deploy/docker/sim-requirements.lock`、`deploy/docker/sim-packages.json`（归一输入的可维护来源/哈希记录）。
- 新增 `third_party/README.md`、`third_party/panda-source.json`（固定上游 URL/提交/子目录/许可）。下载的 checkout 放已有忽略规则覆盖的 `third_party/src/`，不把整个上游仓库或二进制塞入 Git。
- 新增 `tools/assets/fetch_panda.py`：标准库 + git 的小型专用入口，可含 fetch/verify 子命令；资产摘要记录默认 `.build/assets/panda.json`。
- 新增 `tools/build/sim.sh`、`tools/run/sim_check.py`、`deploy/compose/sim_check.yaml`：薄构建/一次性检查入口，不重写 S1 自动采集监督器，不启动机器人服务。
- 新增 `components/backend_sim/src/robot_backend_sim/__init__.py`、`probe_environment.py`；只有确实有独立职责才增加一个小型 `assets.py`，不预建 teacher/physics/ROS 占位模块。
- 新增 `tests/unit/test_sim_environment.py`；文档限 README、deploy/README、新建 `docs/simulation-environment.md` 和 docs/README 索引。

不得改 ROS/Policy 协议、runtime/记录/日志库业务、旧采集配方或现有验收断言。不要运行 Docker、下载外部源码/依赖、调用其他代理或提交 Git；这些由监督者运行。首轮上限 20 分钟，最多一次 10 分钟定点返工。报告 `.build/reports/dsh-014/report.md`，只报告实际执行的 unit/Ruff/语法结果。

## 依赖镜像与挂载

1. 基于已验收 `sim-dual-arm/dev-ros:stage2` 的稳定依赖层。使用给定 lock，通过 `pip --require-hashes` 安装官方 wheels；只 COPY 稳定依赖清单/记录，不 COPY 项目应用、资产、配置、数据或模型。记录实际 Python/包/系统图形库版本，不编造 image ID。
2. 安装 EGL/OpenGL 的必要系统 loader（如 libegl1/libgl1/libopengl0），NVIDIA vendor 实现由 container toolkit 提供。不要安装桌面系统、Torch/LeRobot/仿真大框架，不把 CUDA toolkit 当作 MuJoCo 必需依赖。保留原 mock target 默认行为，可把新 target 放在它前面。
3. 复用挂载仓库中的 dev_ros.sh 与生成契约；显式将 `/workspace/components/backend_sim/src` 加入容器 PYTHONPATH，不能继承宿主机任意 PYTHONPATH。检查实际 sim 模块导入来源，不能被镜像副本遮蔽。
4. Compose 单个一次性 `sim-check` 服务：源码、SDK、资产和摘要只读；报告目录 RW；按调用者 UID/GID；`gpus: all`，`NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics`，`MUJOCO_GL=egl`/`PYOPENGL_PLATFORM=egl`。无需特权、Docker socket、host network、DISPLAY 或训练服务。
5. 主机入口标准库即可，路径相对仓库解析，支持显式 image/build-root/assets-root/report-root/run-id，拒绝覆盖旧 run。命令用参数列表，整体有有限期限；超时/信号回收本次 CLI/工具容器，不留下后台任务。仅操作本次随机项目/容器，不影响已有应用。不要重设 HOME/home/CODEX_HOME。

## 资产获取与验证

- 固定源描述由一个 JSON 文件维护；校验提交为完整 40 位 SHA，子目录是预期相对路径。git 命令参数列表，不用 shell 拼接。
- 优先 sparse/partial clone，仅取得 Panda 所需模型/网格/许可；明确 checkout 固定提交，不跟随 main。首次失败保留原因与可定位现场；任何清理都只能处理本次明确创建的临时目录，不能递归删除用户任意输出根。
- 已有目录非本工具 checkout、提交不同或有人工修改时明确拒绝，保留文件，不自动 reset/clean。重复运行匹配的干净 checkout 可复用。
- 外部 checkout 与生成摘要分开；不能把摘要写进 vendor checkout，导致下一次 git status 自己判脏。以相对文件路径、大小、分块 SHA-256 生成稳定清单/总摘要，包含模型及许可。验证缺失/修改网格、摘要与提交不匹配时非零；拒绝路径逃逸和不受控符号链接。
- 容器从显式 assets-root 读取模型和只读摘要，不能因宿主绝对路径与容器路径不同就跳过验证。来源、提交、资产摘要、许可证路径写入检查报告。

## 原生环境检查

`python -m robot_backend_sim.probe_environment` 在容器内执行，不依赖宿主 ROS 或 Python 安装：

1. 验证依赖版本/导入来源与资产；加载上游单 Panda `scene.xml`/`panda.xml`，按真实 keyframe 初始化，运行少量真实 MuJoCo step，检查 qpos/qvel/ctrl 有限、实际模型自由度和执行器数合理。
2. 验证 mink 可导入，DAQP 在 qpsolvers 中可用，求解一个很小的已知正定 QP 并数值核对结果；不用从头写 IK。本任务不做双臂示教或控制映射。
3. 通过 MuJoCo Renderer 以 EGL 实际渲染 320×240 RGB Panda 预览 PNG；帧应为 uint8、非均匀、可见模型。记录实际 OpenGL vendor/renderer/version，必须是本机 NVIDIA 硬件，软件回退不能记为本地 GPU 通过。渲染资源在 finally 正确 close。
4. 报告包括 run/instance、版本与包/资产/源码来源、相机尺寸、模型统计、QP 检查、像素摘要、OpenGL 信息、产物路径和稳定失败原因。不要把图像数组、完整模型或环境变量灌入日志。
5. 复用公共 EventLogger，component 可为 sim_probe；记录依赖/资产检查、QP、渲染与收尾，关键写入/close 失败影响退出。模块独占创建自己的报告 run 目录，日志排空后才发布成功报告；失败与不完整证据保留。宿主同时检查真实退出码和对应报告，不仅按文件存在推导成功。

probe 的固定小场景/尺寸可以用清楚命名的检查常量，物理场景可调配置留下一任务，不增加多层配置框架或复制训练参数。不要为了自测在宿主安装 MuJoCo，不允许生产环境故障开关。

## 自测与监督者验收

unit 验证输入描述/提交/路径、已有人工目录保留、固定提交与摘要、文件改变检测、挂载参数/UID/GID/GPU 标志/只读边界、非零退出与报告不匹配、超时清理范围。可用本地临时 git fixture 或 runner 边界，不联网、不依赖真实 GPU，不做源代码字符串快照代替行为。

监督者将实际运行 fetch、构建 target、检查 mounts/导入、pip check、原生 QP 与 NVIDIA EGL，并查看预览。源码/资产挂载修改无需重建依赖镜像；修改依赖才重建。失败保留证据并最多一次定点返工。验收前不得开始完整场景、ROS 物理后端、正式数据或训练。
