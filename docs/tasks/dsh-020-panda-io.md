# DSH-020：双Panda的16维SI执行器与测量映射

2026-11-01。DSH-019已验收整合；本项是S3物理接入的第一块。只交付可复用的具名执行器映射与真实测量读取，不实现ROS后端、轨迹调度、租约、reset协议、teacher或Policy。首轮20分钟，最多一次10分钟返工；写报告后立即退出。

## 目标

把已验收Panda profile绑定到实际Scene/MjModel。16维顺序固定为左7关节/左夹爪总开度/右7关节/右夹爪总开度；关节rad、速度rad/s，夹爪m与m/s。控制只写实际actuator的ctrl，测量只读实际qpos/qvel；不得用目标或合成插值冒充测量。

## 允许文件

- 新增 `components/backend_sim/src/robot_backend_sim/panda_io.py`；
- 新增 `tests/integration/test_panda_io_native.py`；
- `components/backend_sim/src/robot_backend_sim/__init__.py`：修正过期的包说明；
- 若复用夹爪转换确有必要，可将scene.py现有 `_gripper_ctrl` 提升为有说明的公共函数，并同步其已有直接调用/对应测试；不改物理模型、相机、场景参数或重置行为；
- 新增短文档 `docs/panda-io.md`，更新 `docs/simulation-control-design.md`、docs/README、README当前能力与限制。

其余文件不改；不动contracts/profile/依赖/镜像/Compose/工具入口/日志库。不要为一次原生检查新增一个平行启动器或通用插件层。不安装依赖、不运行Docker、不使用非固定MuJoCo缓存、不申请权限、不启动其他代理、不提交Git或修改其他工作树。真实native测试由监督者在既定MuJoCo3.17镜像内执行。

## API与职责

1. 提供小而明确的 `PandaIO(scene, profile)`（允许等价清楚命名）。构造时验证绑定：完整16维名字/单位/顺序、范围、home与scene配置一致；两个7关节、4个指关节和16个actuator均按完整名字取索引，不靠相邻偏移猜测。确认原生joint/actuator范围、夹爪raw ctrlrange 0..255，拒绝错误模型/错误profile/错标定。不要写死当前profile哈希或主机路径。
2. profile三相机顺序、尺寸与校准摘要要与SceneConfig一致。摘要算法沿用profiles/README（name/parent/position/quaternion_wxyz/fovy/width/height）。场景没有ROS光学frame对象，frame_id由已校验profile供后续ROS适配发布，不能声称已验证不存在的frame。可复用diagnostics中的相机描述，避免另一套字段算法。
3. `measure()`返回不可变具名测量值：sim_time_ns、joint_names、positions、velocities。臂取实际joint qpos/dof地址，夹爪取两指位置之和、两指速度之和。不得把qpos地址当作qvel地址，不能缓存目标覆盖测量，不裁剪测量来掩盖超限/跟踪误差。NaN/Inf或非法时间明确失败。每次读取产生独立tuple，之后物理步进不能改写已返回值。
4. `apply_targets(positions)`验证完整16维后再写ctrl。拒绝长度/类型/bool/非有限/越界，错误有位置或名字、稳定ContractError reason；不静默clip。合法关节目标写对应7个actuator；夹爪总开度0..0.08m连续映射为0..255，0.04m必须是127.5，不整数化。任何验证失败时原ctrl和物理状态保持原值。
5. 该对象在创建它的线程使用，measure/apply均拒绝跨线程调用，防止后续ROS回调误读写物理MjData。明确这是本对象的约束，S2现有Scene工具仍可独立工作；整个物理后端的数据所有权由后续控制核心落实。状态读取和ctrl写入不调用mj_step、reset、FK或渲染，不写qpos/qvel/物体位姿/外力，不改模型增益/重力/接触参数。
6. 不在本模块实现插值/速度或加速度调度/跟踪完成判定/事件队列，这些归下一项物理控制核心。允许保持上一次目标以外的控制逻辑也留核心。本项ctrl写入成功只表示目标写入，不表示机械臂到达。

## 真实原生测试

新增native integration使用已固定assets/summary和Scene配置，宿主缺MuJoCo时可按现有约定skip；不造一个假MjData证明物理正确。

- 16维初始测量、具名地址、两指求和、速度dof映射与不可变性；测试可显式准备已知qvel，但不得把测试准备伪装为产品执行。
- 不对称左右目标，检查ctrl正确位置及连续夹爪转换；调用后、mj_step前qpos/qvel/time/物体/外力未变。原生步进后测量确实变化，刚写入目标时测量仍不同，防止teleport/目标替代测量。
- 小幅右臂运动、左臂保持及夹爪动作，用真实mj_step与上游actuator验证有限、稳定的响应。只证明空载小运动，不宣称抓取/堆叠或整个S3通过；保留实测误差供后续控制参数设计。不要通过改增益/重力、焊接或反复重置获得通过。
- 错profile顺序/单位/限位/home/相机摘要或尺寸、丢失/错误actuator、非法目标/NaN/Inf/bool/越界、错误线程均拒绝，失败无部分写入。

宿主自测用PROJECT_PYTHON/PROJECT_RUFF：现有unit、Ruff lint/format，native若skip如实写明。监督者独立核对模型raw ctrl、qpos/qvel地址、全过程状态与实际测量响应，并跑原生Scene回归和公开旧入口回归。报告 `.build/reports/dsh-020/report.md` 说明实际检查/待native验收/修改范围/物理限制；代码保持直接、可读、可定位。

## 原生审查后的补充

固定原生环境已复现：原始camera摘要与加载时四元数归一化后的描述不一致；measure的7/8项strict zip错误；创建线程退出后线程ID复用绕过归属检查。唯一返工额外允许 `scene_config.py`、`tests/unit/test_scene_config.py`，在不可变配置中保留相机原始声明，并核对其解析/归一化后与有效配置一致。摘要仍按既定声明算法，profile实例、有效姿态和物理行为不变，不从source路径重复读文件凑摘要。线程以Thread对象身份识别。相机摘要与7/8项循环问题来自中间快照，首轮正式交付已自行修正为重读source方案；正式首轮16项原生测试通过。额外检查确认其仍依赖源文件与可复用线程ID，返工消除这两项。其余范围和预算不变。
