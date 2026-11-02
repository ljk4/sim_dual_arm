# DSH-015 草案：S2b 双 Panda 场景模型、配置与确定性复位

前置：DSH-014 已验收并整合；真实 NVIDIA EGL、固定资产、DAQP、默认入口及338项unit通过。此任务只交付场景模型/配置/复位。S2 的相机/可达性可视验收另交下一任务，不把模型可编译宣称为堆叠任务成功。

## 目标与职责

把固定上游 Panda 组装成可被后续物理后端、teacher 和场景检查共同使用的场景对象；不实现 ROS、策略、采集或训练，不复制上游机器人 XML。

- `configs/simulation/panda_stack.yaml` 是可人工调整的唯一场景参数入口，SI 单位，显式世界坐标（+Y 桌前、+Z 向上），内容分为 physics、column、mounts、table、objects、cameras、reset。关节/位姿/随机化/渲染参数不散在脚本。
- 机械臂基座在圆柱左右，中心 z=1.0m，用短支架连接；桌面 z=.75m；柱上全局相机约1.35m；左右腕各一固定相机，320x240 RGB。右臂工作区域、4cm 方块/6cm底块、XY±3cm/yaw±10°来自已确认需求，可为可达/视野小调名义位置。固定腕相机局部外参，不随目标自动瞄准。
- 原生 `mujoco.MjSpec` attach 两份固定 Menagerie Panda，稳定前缀 left_/right_ 保留原机器人动力学、网格、tendon、equality、执行器。读取 child 前缀前删除 vendor keyframes，再按名称初始化，避免9维keyframe错误扩展。
- 两块都是正常自由体，重力/碰撞启用；不能 mocap、weld、随手改质量来作弊。场景构建只允许 custom柱/支架/桌/相机/物体/光照与必要材质，不手写机器人替代。
- 编译后模型预期 nq32、nv30、nu16、3 named cameras；左/右各7 arm joints+2 finger joints，两个物体各free joint。按名称构建索引，不靠数组位置恰好相邻。
- 独立 `reset(seed)` 使用局部 RNG、显式初始 qpos/ctrl、清空 qvel/act/time、mj_forward，可重复相同种子；不同种子在配置合法域变化，不依赖全局随机状态。物体 z 按桌面+半高，偏航四元数规范化。reset 是重新开始 episode 的操作，不是执行期间物体瞬移接口。
- 测量快照/初始化名称索引必要即可；本任务不抢做16维公共执行映射、控制器或IK。它们按S3/S4独立验收。避免提前出现未使用的接口/策略/工厂层。

## 已验证输入与 API

MuJoCo3.17.0：`parent.attach(child, prefix=..., frame=...)`，`parent.worldbody.add_frame(pos=...,quat=...)`；`spec.delete(key)`。原生 signature 证据来自 DSH-014 的 `acceptance/dependency-api.txt`。主助手已真实运行两份 child attach+两自由块：nq32/nv30/nu16/nkey0；示例与实际输出可读 `/home/ljk/work/sim_dual_arm/.build/reports/dsh/probe_scene_api.py`、主目录 `var/runs/dsh/dsh-014/20261030T121710Z/acceptance/attachment-api/stdout.txt`。这个示例只证明原生 API，不是场景交付，不能照搬到产品并宣称完成。

实际资产在 `/home/ljk/work/sim_dual_arm/third_party/src/mujoco_menagerie`，摘要 `/home/ljk/work/sim_dual_arm/.build/assets/panda.json`，80文件总摘要 `6a6099efd20d3340f8e73c41042df9fcb36b5d6921acdf6784eafb1af8857d43`。固定来源已在 third_party/panda-source.json。不要下载/修改上游文件，不要在宿主安装 MuJoCo。已有仿真镜像 `sim-dual-arm/backend-sim:mujoco-dev`，由监督者原生验收。

稳定名称：left_/right_ + joint1..7、finger_joint1/2、actuator1..8、hand；每臂额外 site `left_tcp/right_tcp`，手局部 [0,0,.1034]（可明确命名配置）。MuJoCo camera 使用本地 -Z 看向前，+Y 向上；腕相机必须是 hand 子相机，固定外参。全局相机固定于柱上，不允许 track/target 自动指向物体。

## 允许文件与实现约束

新增：
- `configs/simulation/panda_stack.yaml`；
- `components/backend_sim/src/robot_backend_sim/scene_config.py`：明确冻结类型、解析及来源明确的校验；
- `components/backend_sim/src/robot_backend_sim/scene.py`：模型组合、名称索引、初始 ctrl 与确定性 reset；
- `tests/unit/test_scene_config.py`、`tests/integration/test_sim_scene.py`（原生物理测试）；
- `docs/panda-stack-scene.md`，仅改 docs/README 索引。

只有确实需要手写的静态 custom MJCF 才新增 `assets/scenes/panda_stack.xml`；优先直接用 MjSpec 加基本几何，所有可调数值留在 YAML，不为一个场景增加模板/工厂/反射框架。不要修改 S1/DSH-014 代码、协议、锁文件、上游 checkout 或公共 Docker 入口。

公开边界建议为 `load_scene_config(path) -> SceneConfig`、`build_scene(config, *, assets_root, summary_path, source_path) -> Scene`、`Scene.reset(seed) -> None`，Scene 明确拥有 model/data/config/name indices。接口可按可读性小调，报告说明。build 验资产一次；reset 不重复扫描网格、不调用 IK、不创建渲染器、不发日志（调用者下一任务拥有一次检查的 EventLogger）。不要把sim assets的主机绝对路径写死在模型或配置。

初始化必须按具名关节写7个rad和两个finger的各半总开度；ctrl初始化7个关节目标和0～255夹爪值。模型原始夹爪控制量与米总开度明确区分，初始化公式有注释；S3再实现统一16维执行映射，当前不加未使用控制方法。`mj_resetData` 后设置完整初值，保证 time/qpos/qvel/ctrl/act/warm-start 等状态不被上一回合污染。两个物体 free qpos 的位置/四元数由 seed 和配置生成。

推荐名义几何：柱半径.12/高1.2m；基座x±.25/y0/z1m，绕Z+90°朝+Y；桌中心xy[0,.65]、边长[1.2,.7]、顶面.75/厚.05m；两块名义xy[.20,.55]/[.36,.55]。这些只是同一已授权几何内的初值，可为真实模型稳定性小调，不能取消柱/降低基座/加固定方块。默认 home 保留上游7维，后续IK/视野阶段再调整待机姿态，不预设“可达已通过”。

相机配置明确全局和两腕的 fixed parent、局部position/quaternion_wxyz/fovy、统一320x240；给清楚的默认值和坐标约定，实际可见性下一任务验收。柱上相机名义约[0,.03,1.35]看向固定桌区[.28,.55,.78]，可以在配置中给计算好的固定四元数；腕相机避开手掌/夹爪视线，使用固定局部外参。不要引入物体真值自动瞄准或 targetbody 模式。

## 验收门槛

配置必须启动前拒绝未知键、布尔代数值、非有限/越界尺寸、无效四元数/重复名称、随机化越过桌边/初始重叠。错误带文件和字段来源。以清楚类型和直接函数保持可读性，不为简单配置写通用反射框架。

监督者在真实依赖容器检查固定资产摘要后编译候选、核对自由度/关节/执行器/相机/接触属性、不同种子以及复位一致性，并运行少量真实物理步进检查稳定。原生测试在无 MuJoCo 的宿主可明确 skip，但不得写作原生已通过；监督者跑容器真测。完整相机/IK可达验收交下一任务。

## 自测、预算与交付

首轮上限20分钟，最多一次10分钟定点返工。不运行Docker/网络/其他代理/host安装，不提交Git。用监督者给定 PROJECT_PYTHON/PROJECT_RUFF 运行 `pytest -p no:cacheprovider tests/unit/test_scene_config.py -q`、Ruff/syntax；主机没有MuJoCo时原生测试可用 importorskip明确跳过，不编造其通过。原生容器验收由主助手运行。

unit 覆盖正常配置与字段来源、布尔/NaN/未知键/无效尺寸/四元数、随机域桌边/物体重叠、配置不可变。native 覆盖真实Mjspec双臂命名/自由度/相机parent、两自由物体/重力/碰撞、reset同seed（在若干真实step及污染qvel/ctrl/act后）恢复一致、不同seed域内变化、默认物体在桌上短时稳定。实际原生资产从 `SIM_ASSETS_ROOT`/`SIM_ASSET_SUMMARY` 环境指定，默认主repo ignored路径可用；原生测试不得在有MuJoCo但资产损坏时静默skip。

报告 `.build/reports/dsh-015/report.md`，写修改范围、实际测试和未执行项。第一次必要检查通过后立即结束，不再循环自我审查/探索缓存，不新增超出本任务的预备接口。
