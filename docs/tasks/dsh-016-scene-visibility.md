# DSH-016：S2 三相机快照、可见性与检查入口

2026-10-31。前置DSH-015已验收。用户恢复按原任务书和协作模式继续；本任务只完成三相机渲染/静态随机布局可见性，下一任务单独检查IK关键姿态和路径碰撞。不得称S2整体或物理堆叠完成。

## 已验证输入与实际问题

- 固定镜像 `sim-dual-arm/backend-sim:mujoco-dev`，ID `sha256:4422f876ae463391f520aacfa3966d8a9c4d9d15a40c7a571cc453133e81b10a`；MuJoCo3.17.0/Python3.12/NumPy1.26.4/Pillow13，已有SDK与资产均可只读挂载。
- 主工作区真实基线在 `var/runs/s2-views/20261030T162500Z/`（UTC目录；本地10月31日）。seed11静态初始姿态：global可见source186/target358像素；left_wrist69/108；right_wrist0/230。实际NVIDIA EGL渲染成功，初始无穿透contact。
- 当前腕相机朝近距离TCP倾斜过多，待机时桌面落在画面边缘。纠正的是固定相机外参；不得根据对象真值逐帧瞄准。
- 可尝试的工程初值（未宣称通过）：两腕保持局部position [0.10,0,0.02]，quat_wxyz [0,0.9975640502598242,0,0.0697564737441253]，fovy85；这使光轴大体沿hand局部+Z并向TCP倾约8°。主助手实际渲染后决定是否保留/调整。
- 真实场景由 `load_scene_config`/`build_scene`/`Scene.reset(seed)`提供。反复reset不重复创建Renderer或hash网格。

## 文件范围和预算

允许新增：
- `configs/simulation/scene_check.yaml`：本任务专用种子/边界布局/可见像素门槛/吞吐测量参数，全部有型校验；
- `components/backend_sim/src/robot_backend_sim/rendering.py`：私有渲染data/EGL与同一快照三视图，分割可选；
- 同目录 `visibility.py`（只有真实职责：有型检查配置、布局与分割可见性统计）；
- 同目录 `check_scene.py`：薄检查编排、EventLogger、resolved配置/版本/报告与PNG证据；
- `tests/unit/test_scene_visibility.py`、`tests/integration/test_scene_visibility_native.py`。

允许修改：
- `configs/simulation/panda_stack.yaml` 仅固定相机position/quat/fovy；不能降低图像分辨率、缩小随机域、改变方块尺寸/质量或取消柱/桌/左臂。
- `components/backend_sim/src/robot_backend_sim/scene.py` 仅相机resolution和offscreen尺寸按实际配置设置；不改变动力学/复位。
- `tools/run/sim_check.py`、`deploy/compose/sim_check.yaml` 复用已验收启动/真实退出码/有界捕获/精确清理，支持 `--check environment|scene`（默认environment）和两个配置路径。
- `tests/unit/test_sim_environment.py` 仅补新入口行为/更新新增挂载，不删除旧保护断言。
- `docs/panda-stack-scene.md`、`docs/simulation-environment.md`、docs/README/README 的当前入口说明。

不修改公共日志库、ROS/runtime/Policy协议、锁或Dockerfile；不下载依赖、不调用Docker/其他代理、不在宿主运行GPU或安装依赖、不提交Git。首轮20分钟，最多一次10分钟定点返工。报告 `.build/reports/dsh-016/report.md`。一次必要自测通过后立刻交付，不循环探索缓存/追加检查。

## 渲染与快照边界

1. Renderer拥有自己的MjData和EGL context，物理Scene.data不被写入。明确的snapshot只包含有限状态的独立拷贝（如原生mjSTATE_FULLPHYSICS数组及sim时间）；多路RGB/分割来自同一个snapshot，输入修改后旧快照仍不变。原生 `mj_getState`/`mj_setState`/`mj_forward`，不能让渲染完成时的新state混入旧图像。当前不承诺跨线程安全，S3由物理拥有者导出快照。
2. 一个Renderer依次切三具名camera，RGB uint8 H×W×3；可选分割返回实体ID/类型，不能把segmentation图当RGB。每次切换恢复渲染flag，不能让前一个camera的分割模式污染后一个RGB。模型最大offscreen/相机resolution尊重已配置分辨率。
3. 返回独立只读数组/快照，保留一个bundle即可；检查循环只积累小元数据/PNG路径，不缓存几十布局的全部数组。不修改model相机外参来自动跟踪目标。
4. 记录真实GL vendor/renderer/version；require本机NVIDIA，软件回退失败。构造、render、close错误清楚传播，所有路径finally close，重复close可安全处理。检查日志close失败不得成功。
5. 后续S3会复用RGB-only渲染，当前RGB-only吞吐测量也确实需要，因此可选segmentation有实际用途；不引入未用插件/工厂/泛型context框架。

## 配置与可见性判据

- scene_check.yaml有schema_version=1；默认工程seed [11,23,37]，并启用两个物体XY随机域各四角、yaw各正负极值的64种联合边界布局（每物体8种）。先Scene.reset(seed)得到初值，再仅在诊断布局副本上设置free体初值；这是场景初始布局检查，不是执行时搬动物体或正式episode。
- 允许在纯Python `visibility.py`中生成布局元数据，输入scene config的两个有名对象；具体snapshot应用由check_scene负责。不加Scene运行期物体传送API。
- 门槛配置为逐camera逐object的min_visible_pixels；初值global两块各64、left/right_wrist两块各32，均为正int。不能因为失败自动降低门槛、只判断非黑图、忽略一个camera或把物体ID/类型通道混用。
- 名称必须与场景camera/object准确对应；未知/重复字段、bool冒充数字、NaN/Inf/过大整数、无效seed/布局数/负阈值等都在建GL前拒绝，带文件/字段。加载后冻结配置。
- 分割统计只数 `objtype==mjOBJ_GEOM` 且 `objid==目标geom id`，以实际像素计数，不依赖颜色阈值。每布局/每camera/每object保留像素面积与通过/失败；任何一项不合格即本检查失败。
- 普通种子与边界布局都保存三路原始RGB PNG；另拼一张标注layout/camera的概览contact sheet（例如首布局加最差像素布局），方便人工查看，不能只交统计。
- 默认RGB-only吞吐预热3组、测量20组，记录三视图一组的p50/p95/平均、处理组数、分辨率、wall时间、GPU信息；不把“纯渲染>20组/秒”等同于端到端500Hz物理+20Hz相机已达标。该项只测量，S3再测并发倍率。
- 独占run目录，最多100个布局，报告/PNG/日志有明确上界，超预算失败而不是无限写。输出report保留schema/kind/run/instance、resolved配置和源码/资产/依赖来源、layout结果、throughput、真实GL、artifact相对路径、logs_closed、status/exit_code/reason。

## 日志、报告和宿主入口

直接复用EventLogger/LogSettings。component=`scene_check`，目录名与instance_id一致；记录start、配置/资产通过、每layout结论、render失败及finalizing。高频数组不写普通事件。日志先于检查建立，失败保留原因；Renderer close、logger close以及report写入失败均影响真实退出码，成功只在资源收尾和日志排空后发布。小型atomic JSON函数可以局部清楚实现；不要复制probe_environment的整套状态机或从components导入tools业务。

`tools/run/sim_check.py --check scene --scene-config ... --check-config ...`沿用一个有界工具容器。建议Compose保留既有sim-check服务，宿主只在scene模式追加固定的 `python -m robot_backend_sim.check_scene ...`命令覆盖；不要接受任意模块字符串。两个配置是额外RO文件挂载，外部配置也可显式使用；容器路径固定、主机绝对路径不写进应用配置。environment模式默认流程和原验收退出码保持。所有kind都检查真实退出码+匹配run report，并按已核对完整容器ID清理。

## 自测与监督者验收

unit：布局数/域/独立性、字段类型/来源、分割实体精确计数与零像素失败、只读快照、报告失败/close传播、入口mount/kind/退出码/旧run保护，避免仅检查源码字符串。无MuJoCo的host只skip确需原生的测试。用PROJECT_PYTHON/PROJECT_RUFF执行针对性unit、Ruff format/check和语法检查，给真实结果。

监督者在固定镜像测试实际RGB/seg多路一致性、输入不变、Renderer关闭/异常、布局门槛、全部实际PNG和源码挂载，查看概览及最差图；另用错误外参/高门槛/日志close故障检查非零退出。用户现有ComfyUI不属于本任务清理范围。失败保留证据后唯一一次定点返工；S2静态视野通过后才另交IK/碰撞/关键姿态，不开始ROS或正式采集。
