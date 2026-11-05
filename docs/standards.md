# 项目规范

统一约定目录、代码、文档、设计、结构、配置、日志与验收方式。适用于后续新增
和修改；既有实现不因写入规范就自动合规。目标与阶段见[当前阶段](project-status.md)，
使用方法见[根 README](../README.md)。

## 1. 文档归属与唯一来源

文档按以下结构维护，旧计划、专题与执行结果已归入对应历史阶段。

```text
README.md                    项目介绍、环境准备、使用指南
docs/
├── README.md                本目录说明与阅读顺序
├── project-status.md        总目标、约束、当前阶段和下一步
├── standards.md             本规范
├── features/
│   ├── README.md            功能索引与编写规则
│   └── <feature>.md         一个功能一文档，说明模块、流程与接口
├── experiments/
│   ├── README.md            重要实践索引与收录规则
│   └── YYYY-MM-DD-<topic>.md 经用户提出或同意收录的尝试与经验
└── history/
    ├── README.md            按 S1～S7 分组的阶段索引与迁移说明
    ├── dsh-000.md           早期设计与基线材料
    └── dsh-NNN.md           一阶段一文档：计划、执行、返工、结果
各项目维护目录/README.md     本目录的用途、主要入口与边界
```

仓库按端与实现组织，仿真器、中间件和学习框架各自一层，避免把具体实现写到
顶层：

```text
simulation/<engine>/        仿真实现，当前 mujoco；其他引擎按需新增
hardware/                   真机 Robot API 边界；驱动尚未实现
runtime/<middleware>/       运行实现，当前 ros2
learning/<framework>/       学习实现，当前 lerobot
infrastructure/             公共 docker、构建/SDK scripts、examples
contracts/ profiles/ libraries/ tests/ docs/ third_party/
var/ .build/                原产物与构建输出，位置不变
```

- 根 README 面向第一次使用项目的人，不堆叠每日进展、会话和验收流水。
- `project-status.md` 是当前阶段的唯一维护位置；已完成部分简记，详细过程、
  数字和失败回到对应历史阶段。
- 同一阶段只维护一份 `dsh-NNN.md`，包括目标/决策、计划、执行、验收和剩余事项，
  不按阶段另建 plan、rework、result。稳定规则提炼到本规范，项目级操作提炼到
  根 README；完整历史仍归原阶段。
- `features/` 按功能详细说明当前实现，一个功能一篇，列出相关模块、流程、接口、
  配置和验证入口；与代码同步维护，不复制阶段流水。
- `experiments/` 记录重要尝试、结果和可复用经验。用户主动提出收录，或助手提出
  建议并经用户同意后才新增正式条目，注明收录依据与证据；不自动积累所有尝试。
- 每个本项目维护的目录保留一个 README，写本地职责、少量实际入口、依赖边界、
  产物去向和上级链接。中间层可用数行，不复制规范全文或阶段流水。
- `.git`、`.build`、`var`、环境缓存、生成目录与第三方 checkout 不逐目录补文档；
  上游 README/许可不改写。`third_party/README.md` 维护本项目来源说明。
- 功能与实践专题仅在上述专用目录维护；其余位置不新增专题文档。
  代码 docstring/必要注释、机器读取的 schema、lock、
  requirements 和自动生成的运行报告不属于额外人工文档。

迁移先记录原路径、hash 和新落点，再合并和更新链接。历史材料注明当时状态，
不将旧命令或预算包装成当前入口。确认内容完整迁移后才移除旧路径；不保留大量
空壳跳转文档。程序、契约与配置决定真实行为，文档不能作为配置加载的替代来源。

## 2. 目录与数据归类

| 位置 | 内容 | 边界 |
| --- | --- | --- |
| `simulation/<engine>/` | 仿真实现：`src/`、`configs/`、`docker/`、`scripts/` | 当前只有 `mujoco/`；不预建其他引擎空目录 |
| `hardware/` | 真机统一 Robot API 边界与后续驱动 | 当前仅说明；不造假驱动或未来空框架 |
| `runtime/<middleware>/` | 运行实现：`ros/`、`configs/`、`docker/`、`scripts/` | 当前只有 `ros2/`；不复制 backend 物理实现 |
| `learning/<framework>/` | 学习实现：`src/`、`configs/`、`docker/`、`scripts/` | 当前只有 `lerobot/`；保持 ROS-free |
| `infrastructure/` | 公共 `docker/`、构建/SDK `scripts/`、`examples/` | 仅公共环境与工具，不放业务、场景或训练算法 |
| `libraries/` | 有明确用途与调用方的共享实现 | 不建任意 utils 或万能上下文 |
| `contracts/` | IDL、proto、schema、轻量类型与语义校验 | 公共格式唯一来源，不依赖物理/学习框架 |
| `profiles/` | 关节、单位、范围、相机与观测/动作语义 | 不放 IP、序列号、账号与机器路径 |
| `tests/` | unit、integration、test-only doubles | mock/fake/synthetic 不冒充正式数据 |
| `third_party/` | 其他固定外部源码 | Panda 模型直接放在 `simulation/mujoco/assets/` |
| `var/` | 数据、模型、日志、预算与实验 | 不进 Git，按 run/episode/实验隔离 |
| `.build/` | 构建、SDK、开发环境、工具与隔离工作树 | 不进 Git，清理前检查在用挂载及复现需要 |

不为目标树预建空模块。ROS 包根、src 和 Python 包目录各有构建用途，不为缩短
路径随意合并。新目录须有实际文件和责任；纯文档整理不批量移动程序或配置。

## 3. 架构与依赖边界

- backend 拥有设备/仿真状态、步进、实际执行与反馈。物理对象有唯一 owner，
  控制、渲染与查询遵守线程所有权，不跨线程直接改状态。
- runtime 拥有观测组装、控制会话、动作接续、记录和收尾，不训练模型，
  不直接导入 backend 的物理实现。
- learning 拥有转换、训练、模型包和 Policy 服务，保持 ROS-free；ROS 记录先由
  ROS 侧审计/导出，再交给学习侧，不把 ROS 依赖带入训练环境。
- 公共语义归 contracts，复用实现归具体 library；组件通过 Robot/Policy 接口
  或校验后的产物通信，不引用对方内部状态。
- 程序示教和物体真值限定在示教边界。正式策略评估不调用 teacher、IK 或真值
  补救，不用脚本替换模型左臂动作。

增加抽象须有实际复用或明确的责任隔离需求。优先显式函数和模块，不为单一实现
引入动态插件、反射分派或多层 manager/factory。

## 4. 接口、动作、时间与反馈

- 语义顺序以 profile 为准。当前 Panda 动作为左 7 关节、左夹爪总开度、右 7
  关节、右夹爪总开度；关节 rad、夹爪 m，时间明确 s/ns，不让调用者猜单位。
- 相机顺序为 global、left_wrist、right_wrist。标定摘要与 profile、数据、模型
  一致；改变语义形成可追溯版本，不能只改名称维持旧 hash。
- 观测身份分配后冻结 state/RGB 引用和时间，迟到帧不能改变同一身份内容。
- 仿真钟、墙钟、观测时刻、执行时刻和动作索引分开。调用/资源监督用墙钟，
  任务/轨迹用仿真钟；停钟、迟到和复位有明确处理。
- 接受 ACK 不等于完成。按 run、episode、会话、command/block 关联 STARTED、
  反馈和终态，幂等重试沿用同一身份；已冻结终态不被迟到非终态覆盖。
- ACT 原始输出、准入目标、实际参考与实测反馈分别保留。插值、接续、限速、
  限加速度、跟踪偏差和停止按配置/契约执行，不用记录补写执行事实。
- 超时/未知 ACK 查询同一请求并按规则停车；矛盾终态必须报告，不能去重掩盖。

接口变更同步维护 IDL/proto/schema、显式双向转换、生成 SDK 与兼容检查。
旧记录按原版本读取，不支持的版本明确拒绝，不静默补默认值冒充原语义。

## 5. 代码可读性与状态所有权

- 使用项目 Python 基线，Ruff 规则以根 `pyproject.toml` 为准。名称具体，
  单位写入字段名或类型说明，避免裸数字和相近缩写。
- 函数处理可解释的一步，模块管理相关责任。按职责、状态所有权和调用关系拆分，
  不按固定行数强拆，也不为每个函数新增一层文件。
- 外部输入校验后转为有类型的内部值。长期状态、跨模块数据、返回结果有明确
  类型，避免可变字典在各层重复解释。
- 明确线程/对象持有哪些状态、锁保护什么、谁创建和关闭资源。回调不做无界
  写盘、整文件 hash 或长时间模型调用，跨线程用快照或有界消息。
- docstring 说明公开接口、输入输出与约束；注释解释原因和边界，不逐行复述语句。
- 异常带来源、字段/请求和稳定原因。必要清理独立尝试并聚合附加错误，保留
  首要失败；不裸 except 后继续、不吞保存失败、不拿默认值冒充正常结果。
- 主流程应能从入口追到配置、业务、结果和收尾，不在 import 时启动任务，
  不用隐式全局状态或多处注册管理同一生命周期。

## 6. 配置与参数

- 模型、训练、推理、任务几何、速度、阈值与预算从配置读取；命令行选择配置或
  显式覆盖少量运行绑定，不在 shell 中再复制算法参数。
- schema 管结构，profile 管语义，业务配置管配方，本地 env 管部署绑定；
  避免多个无法确定优先级的默认来源。
- 未知字段、维数/顺序错误、bool 冒充数值、NaN/Inf 和非法范围启动前拒绝；
  依赖实际状态的校验仍由执行层完成。
- 每次运行保存有效配置与内容 hash。变化使用新 run 或受校验的恢复，不修改
  旧快照或篡改 manifest 适配新机器。
- 私有绑定放忽略的 env、`infrastructure/local` 或实验目录；公开示例先写路径替换、
  挂载核对与输出，不写 token、账号或私密网络信息。

## 7. 日志、记录与结果

- 结构化事件使用既有日志库，带 run/component/instance、顺序、墙钟，以及适用的
  episode/request/command/仿真时间。队列、轮转、关闭期限和错误行为有上限。
- 记录接收/拒绝、ACK、块切换、停止、预算、接触证据和失败原因；关键事件不能
  被普通遥测无声挤掉。大数组、图像和预测单独存产物，事件只存引用。
- 进程退出、runtime、recording、quality 与物理 task 分别报告；文件存在不等于
  记录完整，记录完整不等于任务成功。
- 推理分别保存原始模型输出、准入值、融合/最终传输目标、执行参考与实测反馈。
  时间融合按绝对仿真时刻对齐，记录历史来源；不能覆盖原数组或把融合说成原预测。
- 训练保留 loss/L1/KL、梯度、LR、显存、实际步数、验证覆盖、checkpoint 与退出
  原因。估时用含取数/解码的墙钟差，不以纯计算 step_seconds 代表端到端吞吐。
- 模型与数据绑定配置、profile、schema、预处理、来源、筛选及 hash；记录实际
  源码/镜像，Git HEAD 或进程启动不能单独证明运行版本和行为正确。
- 保存、日志、审计或资源收尾失败必须进入结果；异步快照标观察时间，不能当作
  实时健康状态。不删除失败证据后再汇总。

## 8. 数据与模型实验

- 预先确定 episode、训练/验证划分、种子和接受规则。失败、超时、低质量评估
  计入既定分母，不换种子挑结果或扩预算后沿用旧实验名称。
- 同场景修复回归使用独立名称、预算和前后身份，保留原失败；结果不并入独立
  新种子的泛化指标。一次物理成功也不替代完整质量门禁或成功率评估。
- mock、fake、synthetic、teacher 重放与真实 ACT 分开。离线数学对照、公共入口
  运行和完整物理成功是不同证据，不互相替代。
- 筛帧说明处理的是 query 起点还是时间序列。当前只筛起点，保留原始未来动作
  时间槽；明确验证集和训练统计量处理，防止数据泄漏。
- 新模型/实验使用新目录，旧 bundle/报告不覆盖；恢复检查配置、数据、筛选、
  优化器和预算绑定，目标 20K 不代表已经完成。
- 台账只追加，预检、验证、保存和失败均计费；不清空或换空目录重置。
  增加预算、数据或改变任务约束需明确讨论并记录。

## 9. Docker、依赖与存储

- ROS/控制与 ML 环境独立。系统依赖写进镜像，源码、配置、SDK、数据、资产、
  模型优先挂载；输入只读，输出与缓存有明确可写位置。每个实现恰好一份
  `docker/Dockerfile`、`docker/docker-compose.yml` 和 `scripts/run.sh`，不维护根
  Compose 或统一宿主环境包装。运行变量放各实现的 `configs/container.env`；公共
  DDS 配置和可 source 的 `setup.sh` 放 `infrastructure/docker/`。
- Dockerfile 只安装本端缺少的依赖，并以 `ENTRYPOINT []`、
  `CMD ["sleep", "infinity"]` 空闲启动。各端在自己的 `docker/` 目录直接执行
  `docker compose config/build/up -d/exec`；Compose 的 bind source 相对该文件，
  容器内业务由本端 `scripts/run.sh` 手动启动。构建、启动和进入 shell 都不执行
  数采、训练或推理。SDK 仍由 runtime 容器内的 `run.sh build-ros-sdk` /
  `build-policy-sdk` 显式生成。
- 三份 Compose 固定同一项目名，使 `ros` 与 `learning` 网络在独立启动时共享；
  这是当前 Robot/Policy 通信需要的唯一跨端 Compose 约定。停止单端用 `stop`
  和 `rm -f <服务名>`，不对单端使用 `down --remove-orphans`。当前本机以
  `1000:1000` 写 bind 产物；迁移到其他 UID/GID 时分别核对 `user:`。
- 服务名固定为 `simulation`、`runtime`、`learning`。simulation 与 runtime 共享
  独立 ROS 网络和 `ROS_DOMAIN_ID`；runtime 另加入 learning 网络以解析 Policy
  endpoint；learning 环境不得挂载 ROS SDK、设置 ROS 变量或引用 ROS。runtime 不
  申请 GPU/NVIDIA；simulation 使用 `runtime: nvidia` + `gpus: all` 保证真实
  NVIDIA EGL；learning 的训练/推理使用 NVIDIA GPU。三端均启用 `init: true`。
- 仿真容器只把当前实现目录挂到 `/app:ro`，使用 `/app/scripts/run.sh`；不复刻宿主
  `simulation/<engine>` 的分类，也不挂整个仓库。必要的 contracts、profiles、共享库
  与配置单独只读挂载；Panda 模型放在 `simulation/mujoco/assets/`，通过
  `/app:ro` 一起挂入，不另设资产下载、摘要或挂载。共享数据挂 `/data:rw`，
  ROS SDK 挂 `/opt/ros-dev:ro`。宿主数据位置不因容器命名简化而移动。
  runtime/learning 的现有脚本、SDK 构建和配置仍按仓库根解析跨端路径，因此
  使用 `/workspace:ro`，数据/构建目录挂到 `/workspace/var`、`/workspace/.build`、
  `/data`、`/build`。Policy SDK `/opt/policy-sdk:ro`，固定缓存只读；容器不挂
  Docker socket/CLI。后续缩小挂载时必须先迁移并验证这些实际路径引用。
- 公共 helper `setup.sh` 合并宿主边界与环境准备，可 source，只设置
  PATH/PYTHONPATH/ROS；不保留 `environment.sh`、`exec.sh`、`require-container.sh`
  等旧包装。三端通过 `env_file: ../configs/container.env` 配置容器内路径、
  `PROJECT_SOURCE_PATHS`、ROS/离线设置。`docker compose exec` 不经过镜像入口；
  各端 `run.sh` 自己 source 同一份 helper，再执行真实业务命令。
  runtime 的 PYTHONPATH 必须包含只读 Policy SDK；首次无 SDK 时空闲启动和 shell
  不预检，业务命令再检查。
- 开发运行镜像不 COPY 业务源码、配方、profile、测试或 `run.sh`。COPY 限于锁文件、
  固定依赖清单、必要的最小环境 bootstrap、来源信息和构建产物，并注明用途；运行
  模块来自显式挂载目录，缺失挂载或 SDK 过期必须由具体业务脚本失败，不回退到镜像
  中的旧业务代码。Dockerfile 仅保留依赖环境必需的 `ENV PATH`，用构建期 `ARG` 代替安装期的
  `DEBIAN_FRONTEND` 运行变量，不设置 `WORKDIR`；统一清空继承的 `ENTRYPOINT`
  并默认 `sleep infinity`。工作目录由 Compose 的 `working_dir` 指定；三端的
  `run.sh` 都由用户进入容器后执行。不提供多 target 运行环境或宿主 venv 路线。
- Dockerfile 禁止成段内嵌程序，包括 heredoc、多行 Python/Shell、循环分支以及
  用字符串或编码隐藏脚本。构建逻辑放 `infrastructure/scripts/`，Dockerfile 仅安装和调用；
  apt/pip 等命令可以分行列参数，不能借此塞入业务或构建算法。
- Python/配置变更需重启相应进程，挂载不承诺自动热重载；IDL/proto 变化需重新
  生成 SDK 并核对来源摘要。验证修改生效要检查实际模块来源，而不只看挂载存在。
- 改参数不重建镜像，依赖变化才重建对应层；固定上游版本/hash 和许可，锁文件
  变化说明来源与影响。明确使用官方 `latest` 镜像的组件，每次重建记录实际摘要与
  版本，并重新验证关键功能；之前的验收结果不自动适用于新镜像。
- MuJoCo 等项目 Python 依赖统一维护在 `requirements.lock`，由 pip 使用固定版本
  与 `--require-hashes` 安装；依赖清单变更后显式重建镜像。不另建镜像版本报告或
  运行时锁摘要对比层。环境检查直接验证模块导入、挂载源码、EGL 与实际功能。
- 模块的 `docker/` 目录内使用通用依赖名称（如 `requirements.lock`），不重复添加
  sim、mujoco 等模块/引擎前缀。当前三端均使用本地 `build: .`，Dockerfile 的
  COPY 直接使用本目录相对路径。源码归类
  仍由外层模块目录负责；上游包名与官方环境变量保持原名。
- `configs/container.env` 只声明运行时确实使用的固定设置，不保留未消费的组件
  标签或重复检查开关。修改这些设置只需重新创建容器，不重建依赖镜像。
  Compose 明写镜像、相对 bind 来源和共享网络，避免为早期项目维护宿主变量包装。
- Ruckig 使用随源码提供的固定 wheel，安装锁和来源清单绑定同一产物；保留其平台
  要求、许可证与哈希。普通仿真镜像构建直接安装，不隐式下载源码或退回编译；
  升级版本/更换平台需另行发布并验收匹配的安装包。
- 新机器先核对文件系统、挂载和可用空间。仓库、数据目录、Docker、containerd
  与匿名卷的存储位置分别确认，不能假设随仓库移动。
- 启停/清理只操作已核对归属的 project/run 资源，保留 bind 产物和外来资源，
  不用全局 prune、模糊容器名或批量删除替代归属核对。
- `.build` 中还有在用的冻结训练源与已验证环境，不能一概当可删缓存。
  清理前列目录、核对挂载/复现需要并取得用户授权。

## 9.1 容器执行与替换边界

- 实现相关的 Python 源目录和 ROS 环境选择由各实现的 Compose 或其 `env_file` 显式配置；公共
  `setup.sh` 不按 MuJoCo、LeRobot 等具体名字分派。实现直接运行自己的 Compose，
  公共检查器检查各模块目录，不要求根 Compose 选择器。
- 每端只提供一个清晰的容器内 case 分发脚本 `scripts/run.sh`；进入空闲容器后
  手动执行本端业务命令，参数透传给真实 CLI/SDK 工具。宿主用
  `docker compose exec` 进入已有容器，不按功能名转发，也不在
  `run.sh` 中重复实现 `idle`、`shell`、`exec`。
  必要时只保留确有职责的准备脚本；业务脚本在宿主执行时明确失败，不悄悄创建或
  使用宿主 venv。runtime 额外提供
  `build-ros-sdk`、`build-policy-sdk`、`test`，learning 按需提供 `test`；脚本内部
  调用 `infrastructure/scripts/test.sh` 和真实 SDK 生成器。simulation 直接沿用
  官方 ROS 基础环境，不逐包拆除其自带依赖；项目新增 Python 包统一维护一份哈希
  锁，不另建工具清单或重复安装层。以来源清楚、可维护为目标，不以包数量最少为
  目标。ROS SDK 由 runtime 生成后只读挂载，仿真不维护重复的 SDK 构建入口。
- 仿真 backend 配置/seed 由 simulation 端准备；learning 的 `prepare-policy`
  校验真实 ACT bundle 并输出通用 Policy binding JSON 与 Policy 服务配置；runtime
  的 `prepare` 只消费该 JSON、runtime 模板、seed/run-id，不得导入
  `dual_arm_learning`、MuJoCo 或模型内核。绑定 run-id 必须与当前 run-id 一致；
  endpoint 允许配置 host:port，但端口必须与声明 service.port 一致。runtime 只
  要求通用 bundle/processor digest，直接设置已知 policy 字段；模板残留未解析占位
  必须失败。
- 所有准备输出使用显式目录，已存在即拒绝覆盖。准备产物可附 `run_manifest.json`
  统一记录 run-id、seed、输入/输出 hash 和模型绑定；准备成功不计为运行成功。
- 公共容器测试入口 `infrastructure/scripts/test.sh` 固定
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，避免 ROS 自动插件提前
  导入未选择的 Torch 测试；skip 不等于通过。
- 替换后端：实现同一 Robot API/语义 profile 和输入输出契约，在 `hardware/` 增加
  实际驱动与最小配置，再在 Compose/环境中显式选择；不得让 runtime 导入具体物理
  或驱动实现。替换学习框架：实现同一 Policy API，保持三路 RGB + 16 维实测输入和
  当前 Panda 任务的 60×16 位置动作块语义，learning 环境仍保持 ROS-free。若动作
  语义或时序改变，需同步契约、profile、配置与验收，不能靠目录替换自动兼容。

## 10. 变更、DSH 协作与验收

- 开始前明确问题、范围和验收，一个阶段一个有界目标。DSH 实现前固定输入，
  主助手规划、审查和独立验证，不以 DSH 退出 0 代替项目验收。
- 实际启动 headless 时说明时间、工作树、session、日志和状态；原始事件留
  `var/runs/dsh`，公开计划/决策/结果写同一阶段，不重复抄录工具流水。
- 使用隔离工作树，保留既有修改；隔离输入快照与主分支提交分别说明。
  不自动提交、推送或接管用户已有会话。
- 按变更选择检查。纯文档检查内容、归属、链接和程序文件未变；行为变化做有
  意义的回归，涉及 ROS/物理/容器才运行相应实际验证。
- 区分候选自测、独立验证和公共入口验收，报告实际命令、退出、产物和未测项；
  不拿旧镜像通过证明新源码通过，不弱化规则或删除失败来获得通过。
- API 故障、超时或用户暂停时保留现场，不自动无限重试；通知恢复后从已记录
  状态继续，避免重复实施或重复消耗实验预算。

后续阶段更新当前入口与当期历史文件。功能说明和经确认的实践条目在各自目录维护，
引用阶段证据，不恢复旧专题目录或另建一套阶段结果流水。
