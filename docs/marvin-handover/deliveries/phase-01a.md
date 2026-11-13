# P01A 交付记录：模型资产整理与碰撞/联动复核

日期：2026-11-12。前置 P00/P01 已交付。本轮完成 Marvin 资产的所属端迁移、模型定位、有效碰撞与联动复核；标准 Robot API 控制、Teacher、采集、训练和真机驱动未接入。工作树改动保留，未暂存、提交或推送。

## 环境与命令

- 仓库：`/home/ljk/work/sim_dual_arm`，分支 `main`；P00 基线 HEAD `55bf341983b985c4746ae3ba245ee00c4f25e6dd`。
- simulation 容器：`sim-dual-arm-simulation-1`，镜像 `sim-dual-arm/simulation:current`，用户 `1000:1000`，MuJoCo `3.17.0`，Python `3.12.3`，`MUJOCO_GL=egl`。
- 挂载：`simulation/mujoco` → `/app:ro`，`shared` → `/shared:ro`，`var` → `/data`。
- 完整检查（工作目录 `simulation/mujoco/docker`）：

```bash
docker compose exec -T -w /app simulation bash -lc \
  'source /shared/common/tools/docker/setup.sh && scripts/run.sh marvin --run-id p01a-20261112'
```

退出码 0。报告：`var/reports/sim-check/p01a-20261112/report.json`（容器 `/data/reports/sim-check/p01a-20261112/report.json`），状态 `succeeded`，耗时 `1.542 s`。

## 资产移动与路径

| 项目 | 结果 |
| --- | --- |
| 移动前 | `shared/scenarios/marvin_handover/assets/marvin/` |
| 移动后 | `simulation/mujoco/assets/marvin/` |
| 内容 | `README.md`、`urdf/Marvin_Stand_M6_2026.4.8.urdf`、`meshes/*.STL` 共 23 个二进制 STL；源文件未改写 |
| 重复副本 | `/shared/scenarios/marvin_handover/` 下 STL 数量为 0，旧 `assets/` 目录不存在 |
| Panda | `simulation/mujoco/assets/franka_emika_panda/` 位置与内容未改 |

定位实现：`physics/assets.py` 的 `robot_model_path(assembly)` 从本端 `assets/` 根解析固定版本模型。`config/marvin_scene_config.py` 的加载器在读取 YAML 后按 `robot.assembly: marvin` 解析 `urdf_path`；场景配方已删除 `robot.urdf`，没有新增资产选择字段或挂载。

实际解析：

- 宿主机：`/home/ljk/work/sim_dual_arm/simulation/mujoco/assets/marvin/urdf/Marvin_Stand_M6_2026.4.8.urdf`，文件存在，23 个 STL；宿主机无 MuJoCo，未在宿主机编译。
- 容器：报告 `scene.robot.asset_path` 为 `/app/assets/marvin/urdf/Marvin_Stand_M6_2026.4.8.urdf`；配方仍从 `/shared/scenarios/marvin_handover/configs/simulation/scene/marvin_handover.yaml` 读取。

```bash
# 宿主机，工作目录 /home/ljk/work/sim_dual_arm；退出码 0
PYTHONPATH=simulation/mujoco/src python3 -c \
  "from robot_backend_sim.physics.assets import robot_model_path; p=robot_model_path('marvin'); print(p, p.is_file())"

# 容器，工作目录 simulation/mujoco/docker；退出码 0
docker compose exec -T simulation bash -lc \
  'ls /app/assets/marvin/urdf/Marvin_Stand_M6_2026.4.8.urdf; \
   ls /app/assets/marvin/meshes/*.STL | wc -l; \
   find /shared/scenarios/marvin_handover -name "*.STL" | wc -l; \
   test -e /shared/scenarios/marvin_handover/assets && echo duplicate || echo no-duplicate'
```

输出分别为宿主机路径 `True`，容器 URDF 存在、STL `23`、`/shared` 下 STL `0`、`no-duplicate`。

## 碰撞复核

原 `_add_self_exclusions` 按链路编号生成 88 项同臂排除。逐对测量源碰撞凸包后，只有 `Arm_*5_Link`/`Arm_*7_Link` 存在结构性重叠：复位位姿的凸包间距为 `-0.0609 m`；在 800 个关节限位内随机位姿中 800/800 为负，`Arm_*6` 位于两者之间。其余链路配对不是永久重叠。P01A 保留 2 项排除（左右各一），编译模型 `nexclude=2`；同臂其余配对、跨臂配对和四指碰撞全部保留。

底座网格仍仅渲染。P01A 不补代理：任务桌面位于 x≥0.40 m，复位、低幅运动和本轮碰撞探针均不要求底座参与；若 P02B 候选路径进入底座中轴约 0.15 m 范围，应先补经验证的代理几何，不启用 216k 面粉碎凸包。该边界与判断依据同时写入资产说明。

有效碰撞探针使用限位内的确定性 qpos 姿态，`mj_forward` 后读取接触；姿态只是检测证据，不是任务或抓取位姿。

| 探针 | 姿态 | 检测到的接触（示例） |
| --- | --- | --- |
| 同臂自碰 | 右臂 `(0.74432, 0.33872, 0.80518, -2.47078, -1.67433, -0.17901, 1.49686)`，左臂复位 | `Arm_R1`–`Arm_R9`、`Arm_R2`–`Arm_R9`、`Arm_R1`–`Arm_R10`、`Arm_R2`–`Arm_R10` |
| 双臂互碰 | 左 `(1.54525, -1.69055, -0.10567, -1.90513, 1.01637, -0.50605, 0.98071)`；右 `(-0.76212, -1.99988, 1.04755, -2.23433, -0.36641, 0.23354, -0.98902)` | `Arm_L5`–`Arm_R5`、`Arm_L8`–`Arm_R1`、`Arm_L9`–`Arm_R2` 等 |
| 臂–桌面 | 右臂 `(2.39295, 1.59484, -2.25148, 0.13239, 1.23707, -0.07975, -0.32199)` | `Arm_R5`/`Arm_R6`/`Arm_R7`–`table` |
| 臂–箱体 | 右臂 `(2.78133, 1.94594, -0.18191, -1.08069, 2.10245, -1.00087, 0.74346)` | `Arm_R5`/`Arm_R6`/`Arm_R7`–`box` |
| 手指–物块 | 左爪指关节各 `0.02 m`，物块置于两指内侧面中点 | `Arm_L10`–`block`、`Arm_L11`–`block` |

复位位姿在 2 项排除生效后无机器人自接触；`cross_arm_exclusions=0`。底座保持仅渲染：报告记录 `base_collision.enabled=false`，代理触发条件为 P02B 路径进入底座中轴约 0.15 m 范围。同臂探针同时覆盖 `Arm_R9`（腕/手）和 `Arm_R10`（手指），手指–物块探针覆盖 `Arm_L10`/`Arm_L11`。桌面/物块/箱体的原有检查继续成立：物块与桌面 4 个接触点，箱口向下射线命中 `box_bottom`（geom 25，距离 `0.308 m`），物块稳定后箱底 4 个接触点。

## 联动约束

编译模型 `neq=2`、`ntendon=2`。两条 equality 均为 `mjEQ_JOINT`：

| 名称 | 对象 | data | solref | solimp |
| --- | --- | --- | --- | --- |
| `Arm_R_finger_coupling` | `Arm_R11_Joint` / `Arm_R10_Joint` | `[0, 1, 0, 0, 0, …]` | `[0.005, 1.0]` | `[0.95, 0.99, 0.001, 0.5, 2.0]` |
| `Arm_L_finger_coupling` | `Arm_L11_Joint` / `Arm_L10_Joint` | `[0, 1, 0, 0, 0, …]` | `[0.005, 1.0]` | `[0.95, 0.99, 0.001, 0.5, 2.0]` |

源 URDF 每侧第 11 关节 mimic 第 10 关节（倍率 1、偏移 0）；MuJoCo 3.17 的 URDF 导入器已生成上述两条约束。P01 又手工添加了同对象的两条 equality，因此原 `neq=4` 是两套重复约束。原四条为：导入的 `Arm_R11_Joint`/`Arm_R10_Joint` 与 `Arm_L11_Joint`/`Arm_L10_Joint`（`solref=[0.02, 1.0]`、`solimp=[0.9, 0.95, 0.001, 0.5, 2.0]`），以及手工的 `Arm_R10_Joint`/`Arm_R11_Joint` 与 `Arm_L10_Joint`/`Arm_L11_Joint`（`solref=[0.005, 1.0]`、`solimp=[0.95, 0.99, 0.001, 0.5, 2.0]`）；两组对象相同、方向相反。P01A 保留导入的源联动、移除手工添加，并把三态测量验证过的 `solref`/`solimp` 写入保留约束。装配器在装配时校验 equality 类型、joint 配对和倍率/偏移；绑定阶段再次校验对象和参数。

## 测量复核

测量来自编译模型和实际步进，不做裁剪。夹爪三态命令 `0 / 0.04 / 0.08 m`，每态 600 步收敛后测量：

| 侧 | 开度（命令 0） | 中点（命令 0.04） | 闭端（命令 0.08） | 最大联动误差 |
| --- | --- | --- | --- | --- |
| 左 | `0.0814349 m` | `0.0414368 m` | `0.0014387 m` | `2.71e-5 m` |
| 右 | `0.0814206 m` | `0.0414225 m` | `0.0014244 m` | `2.71e-5 m` |

- 档案值：开端 `0.0814 m`、闭端 `0.0014 m`；检查容差 `5e-4 m`。实测与档案差：左开端 `3.5e-5`、左闭端 `3.9e-5`、右开端 `2.1e-5`、右闭端 `2.4e-5`。
- 命令范围：执行器 `ctrlrange=[0, 0.08]`；`gripper_ctrl(0.0814)=0`，`gripper_ctrl(0.0014)=0.08`。总开度与闭合量之和为开端测量值。
- TCP：位于 `Arm_*9_Link` 局部 `[0, -0.1596233, 0] m`，到两指内侧面沿手部 z 轴的偏移为 `+0.0407184/-0.0407184 m`（左）和 `+0.0407184/-0.0407041 m`（右）。右臂 TCP 与指面中点的偏差 `1.43e-5 m`，检查容差 `1e-4 m`。
- Profile 的 16 维顺序、关节限位、初值和三路相机摘要继续由 `_profile_checks` 与档案校验通过；机器人语义仍由装配/场景提供，未写入任务环境。

## 模型与渲染结果

编译模型：`nq=25 nv=24 nu=16 ncam=3 nbody=28 ngeom=31 neq=2 ntendon=2 nexclude=2`。机器人 18 个关节自由度（14 hinge + 4 slide）；物块 7 个 qpos / 6 个 dof。

- 源语义对照：23 个 link，质量最大误差 `0 kg`，主惯量最大误差 `8.88e-16`，总 link 质量 `47.449392 kg`；编译网格 24 个（底座拆分 + 22）。
- 静置 1000 步（2.0 s）：无 NaN；最大臂关节漂移 `0.003477 rad`（左右腕关节 7）；物块保持桌面，4 个接触点。
- 低幅运动：14 个臂关节各加 `+0.05 rad`，实测同向 `0.04651..0.05345 rad`；另一臂同序号关节最大漂移 `0.003476 rad`。
- 渲染：NVIDIA EGL，`nvidia=true`。三路 320×240 PNG 位于 `var/reports/sim-check/p01a-20261112/artifacts/renders/{reset,ready}/{global,left_wrist,right_wrist}.png`。
- 物块像素：reset 为 `global 560 / left_wrist 0 / right_wrist 0`；诊断 ready 位姿为 `307 / 1365 / 1355`。ready 仍只是离线渲染位姿。

## 回归

在 simulation 镜像中运行本轮受影响的 Panda/Marvin 测试，未重跑未受影响的整套 P00 基线：

```bash
# 工作目录 simulation/mujoco/docker
docker compose run --rm --no-deps \
  -v /home/ljk/work/sim_dual_arm:/workspace:ro \
  -e PROJECT_ROOT=/workspace -w /workspace simulation bash -lc \
  'source /shared/common/tools/docker/setup.sh && \
   python3 -m pytest -p no:cacheprovider -q \
     tests/unit/test_marvin_scene.py \
     tests/unit/test_scene_config.py \
     tests/unit/test_profile_cameras.py \
     tests/unit/test_sim_environment.py \
     tests/unit/test_contract_consistency.py \
     tests/unit/test_environment_entry.py \
     tests/integration/test_sim_scene.py'
```

结果：`134 passed in 2.78s`，退出码 0。Panda 场景构造/复位与资产探测回归通过；Marvin 场景契约、Profile/相机、资产定位、碰撞排除和 equality 断言通过。

## 未完成项与 P02A 接入点

- 未接入标准 Robot API 控制、Marvin I/O、owner/controller 选择、控制/stream/follower 配方；这些属于 P02A。
- 未验证真实持物、双臂交接路径、候选抓取/放置位姿和环境净空；P02B 用标准控制建立并验证。
- 底座网格仍无碰撞。当前任务区域位于桌前，P01A 不补代理；P02B 若路径进入底座中轴约 0.15 m 范围，先补并验证代理几何。
- 未建立 `binding.yaml`、runtime/learning 配方、Teacher IDL 或采集数据；P03～P05 按实际接入补齐。
- 夹爪 `max_velocity=0.04 m/s`、臂关节 `0.8 rad/s` 仍是首版保守执行值；P02A 需按真实控制验证并补充加速度、jerk、增益和跟踪阈值。
- P02A 接入点：由 simulation 后端装配入口按所选场景/机器人构造 Marvin 场景，使用 `MarvinBinding` 的具名关节/执行器/指面/TCP 地址实现 I/O；复用本阶段的 `gripper_opening`、闭合量映射和 equality 校验，不复制机器人语义到任务代码。
