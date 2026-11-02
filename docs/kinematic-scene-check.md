# 关键位姿、IK、路径净空与三相机视野检查

2026-11-01。本入口是 S2 的运动学诊断，S2 组合验收见 [结果记录](tasks/dsh-017-result.md)。不接 ROS、不执行抓取、
不创建正式数据。它在与 scene 检查相同的 67 个初始布局（3 seeds + 64 个
XY/yaw 边界布局）上，对右臂求 6 个关键位姿，检查右臂关节线性路径的几何净空，
并生成每个关键位姿的三路 RGB 和实体 ID 分割像素统计。

## 固定输入与边界

- 场景与布局继续使用 `configs/simulation/panda_stack.yaml`、
  `configs/simulation/scene_check.yaml` 和 `visibility.build_layouts`，不缩小
  随机域，也不改场景几何/外参/随机化。
- `configs/simulation/kinematics_check.yaml` 是唯一的运动学检查参数来源。
  `kinematics_config.py` 在 EGL、模型构建和大量 IK 之前拒绝未知/重复字段、
  bool 冒充数字、NaN/Inf、超大整数、无效阈值、缺少/重复的
  camera/object/phase 覆盖，以及超过上限的布局、迭代和采样。
- 左臂、左右指和两个物体始终冻结在完整初始快照中的值；只有右臂 7 个 hinge
  qpos 允许变化。IK 使用私有 `mink.Configuration`/MjData，不写
  `Scene.data`。抓夹始终开启，两块不移动，所以 lift/place 只是空手关键姿态，
  不是搬运或堆叠成功。

## 关键位姿

source/target 取每个布局 reset 后的实际物体中心，六目标按配置高度生成：

- `pregrasp = source + [0, 0, pregrasp_height]`
- `grasp_pose = source + [0, 0, grasp_height]`
- `lift_pose = source + [0, 0, lift_height]`
- `place_hover = target + [0, 0, place_hover_height]`
- `place_pose` 的 XY 为 target，z =
  `table_top + 2*target_half_z + source_half_z + place_clearance`
- `retreat = target + [0, 0, retreat_height]`

右 TCP 方向固定为世界向下 `Rz(source_yaw) @ diag(1, -1, -1)`，整个序列不变。
求解器为 mink 1.3 `FrameTask + PostureTask`、`ConfigurationLimit +
CollisionAvoidanceLimit`、`DofFreezingTask` 硬约束和 DAQP；不实现伪逆 IK，
不把 IK dt 当作执行周期。每个目标记录迭代数、位置/旋转残差、7 维右臂解、
非右臂 qpos 最大偏移和明确收敛状态。超迭代/不可达/约束失败都保留原因并使
最终退出非零。

`ik.initial_right_joint_positions` 是每个 layout 首目标的**数值求解初值**；
该 layout 的其余 qpos 仍取自己的完整初始快照，后续 phase 从上一已收敛解
开始。posture 参考同样是该“初始快照 + 配置右臂初值”的 qpos。这个数值初值
不是实际运动起点：`initial -> pregrasp` 路径始终从 `Scene.reset` 的原 home
右臂姿态开始检查，Scene/reset/home 未被修改。

## 净空检查

检查初始姿态，以及 `initial -> pregrasp` 和五个相邻关键位姿之间的右臂关节
线性路径。每段最多 0.01 rad 一个样本、包含两端、最多 2000 个样本；总采样数
也受配置上限约束。每个样本检查 qpos 有限性、右臂关节范围、冻结自由度不变，
并用 `mj_geomDistance` 在有限检测距离内测量有符号距离。

碰撞对按原生规则预先枚举：只保留至少一侧为机器人的有效物理 geom 对；排除
无 contype/conaffinity 的 visual、同 body/同 welded body、parent-child、
显式 `<exclude>`、非机器人静态对，以及两侧都没有可动关节祖先的固定连接对。
报告记录对数和每类排除依据；不使用“全部机器人碰撞关闭”，也不改 margin 或
删除碰撞几何。固定安装处的接触不算自由运动净空。

净空门槛默认 0.002 m。报告保留最小净空及 layout/segment/sample/geom/body；
broadphase 基于原生 bounding sphere 并带检测距离余量，跳过只会发生在两个
`rbound` 都大于 0 且可证明大于检测距离的对上；plane 等 `rbound=0` 的几何
不做球包围裁剪，直接调用 `mj_geomDistance`。若所有对都超出检测距离，报告
把最小净空标记为检测距离下界，最近 geom/body 写 null，而不是伪造一个最近对。

采样检查是离散分辨率证明，不是连续几何碰撞证明；轨迹插值、执行约束和真实
抓取路径留给 S3/S4。

## 关键视角

对 6 个关键姿态，从同一诊断快照渲染 `global`、`left_wrist`、`right_wrist`
三路 RGB 和分割图，保存有界 PNG，并至少生成一张带 layout/phase/camera 标注
的 overview。分割统计使用实际 `objid==geom id` 和 `objtype==mjOBJ_GEOM`，记录
source/target/table 的 geom id 与像素数。

首版门槛全部在配置中按 phase 显式声明：

- `right_wrist`：pregrasp/grasp_pose/lift_pose 看 source，place_hover/
  place_pose/retreat 看 target，至少 64 像素；
- `global`：target 全阶段至少 64；source 除 grasp_pose 外至少 32；
  grasp_pose 的 source 只记录；
- `left_wrist`：每阶段桌面至少 4096 像素；同时记录两块像素和完全遮挡标记。

所有布局初始三视图门槛仍归 scene 检查。这里的空手画面不能替代 S4 对实际
搬运后有效任务区域和遮挡时长的检查。

## 入口、输出与失败语义

```bash
python3 tools/run/sim_check.py --check kinematics --run-id kinematics-check-001
# 也可显式指定三个只读配置
python3 tools/run/sim_check.py \
  --check kinematics \
  --scene-config configs/simulation/panda_stack.yaml \
  --check-config configs/simulation/scene_check.yaml \
  --kinematics-config configs/simulation/kinematics_check.yaml \
  --run-id kinematics-check-002
```

`environment` 仍是默认模式，`scene` 行为不变。宿主只把固定命令
`python -m robot_backend_sim.check_kinematics` 放进同一个一次性容器；三个配置
只读挂载到 `/sim-configs/`，只有报告目录可写。容器接受固定参数，不接受任意
模块字符串。

报告在 `<report-root>/<run-id>/report.json`，包含 schema/kind=`kinematics`/
run/instance、配置有效值及 SHA-256、代码/依赖/资产引用、逐布局目标/残差/
净空/视野/耗时、所有失败、PNG 清单、`logs_closed` 和最终
`status`/`exit_code`/`reason`。summary 的最小净空带 layout/segment；报告预算
按实际 pretty JSON 的 UTF-8 字节数校验，默认 8 MiB，超限即失败而不截断证据。PNG 在 `artifacts/layouts/<layout>/<phase>/`
下，总量上限 128 MiB；PNG 编码后先检查余量，再创建文件，overview 共享同一预算。事件日志复用 32 MiB 有界容量，组件名
`kinematics_check`，目录名与 instance id 一致。

任何净空、IK、冻结、关键视角、资源关闭或报告写入失败都使真实退出码非零；
不会自动降阈值、改外参或重采种子。成功只表示该离散运动学诊断和证据写入完成，
不表示物理抓取或 S3 轨迹执行完成。

人工阅读代码可从 `check_kinematics.run` 开始，再看 `_run_layouts`；单布局顺序为
`_prepare_layout` → `_check_motion` → `_render_key_pose`。`_close_resources` 与
`_publish_report` 单独处理资源关闭和最终结果，配置与 IK/几何逻辑分别在
`kinematics_config.py` 和 `kinematics.py`。
