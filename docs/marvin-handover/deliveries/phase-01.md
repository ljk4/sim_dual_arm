# P01 交付记录：Marvin 模型、场景与档案

日期：2026-11-12。前置 P00 已完成。本轮交付固定根 Marvin 双臂装配、handover 任务环境、三路相机、Marvin 档案/标定和离线模型检查；标准控制接入留在 P02。

## 实际模型导入

- 源资产原样纳入 [`shared/scenarios/marvin_handover/assets/marvin/`](../../../shared/scenarios/marvin_handover/assets/marvin/README.md)：URDF 加 23 个二进制 STL。
- 装配器 [`physics/marvin/assembly.py`](../../../simulation/mujoco/src/robot_backend_sim/physics/marvin/assembly.py) 用本机 MuJoCo `3.17.0` 的 `MjSpec.from_file` 直接导入 URDF，未手工重写关节树。
- `ZJ_Robot_link` 无自由关节，是固定根；`fusestatic=False` 保留 `Arm_*8_Link`/`Arm_*9_Link` 固定腕部 frame，供腕相机和 TCP 使用。
- `ZJ_Robot_link.STL` 有 216,087 面，超过 MuJoCo 单文件 200,000 面上限；装配器在内存中拆成两个 `uservert/userface` 网格，不简化几何。源 STL 未改动。
- URDF 的 `package://.../meshes/*.STL` 在导入后重写为文件基名并设置 `meshdir`；源文件不改写。
- 固定关节 `Arm_R8_Joint` 的无效 limit 不参与自由度或控制限位；编译模型只有 14 个旋转和 4 个滑动关节。

编译模型（完整场景，含物块自由关节）：

```text
nq=25  nv=24  nu=16  ncam=3  nbody=28  ngeom=31  neq=4  ntendon=2  nkey=0
```

机器人自身为 18 个关节自由度（14 hinge + 4 slide）；物块贡献 7 个 qpos / 6 个 dof。检查与绑定全部按名称和不重复地址完成，没有把公开 16 维当作 `nq/nv/nu`。

## 公开/物理映射

| 公开名称 | 物理关节/对象 | 单位 | 来源 |
| --- | --- | --- | --- |
| `left_joint1..7` | `Arm_L1_Joint..Arm_L7_Joint` | rad | URDF 限位与复位值 |
| `left_gripper_width` | `Arm_L10_Joint` / `Arm_L11_Joint` 内侧面间距 | m | 源网格包围盒测量 |
| `right_joint1..7` | `Arm_R1_Joint..Arm_R7_Joint` | rad | URDF 限位与复位值 |
| `right_gripper_width` | `Arm_R10_Joint` / `Arm_R11_Joint` 内侧面间距 | m | 源网格包围盒测量 |

夹爪执行器：

- 左右各一条固定 tendon，系数 `+1` 作用于两个指关节；
- 左右各一个 joint equality，`Arm_*10_Joint == Arm_*11_Joint`；
- 执行器控制量是两指闭合量 `0 .. 0.08 m`；
- 总开度 = 两指内侧面沿手部局部 `z` 轴的垂直间距，不由关节位置之和直接代替。

测量结果（1000 步稳态，左右一致）：

| 闭合命令 | 指关节 q10 | 指关节 q11 | 联动误差 | 总开度 |
| --- | --- | --- | --- | --- |
| 0.00 m | 0.00001 | -0.00001 | 0.000026 m | 0.08143 m |
| 0.04 m | 0.02001 | 0.01999 | 0.000026 m | 0.04144 m |
| 0.08 m | 0.04001 | 0.03999 | 0.000026 m | 0.00144 m |

右臂闭端为 `0.00142 m`，差值与源网格的固定横向偏移一致；Profile 使用开端 `0.0814 m`、闭端 `0.0014 m`。

## 机器人、任务环境与机制拆分

| 归属 | 文件 | 持有内容 |
| --- | --- | --- |
| 通用机制 | [`physics/scene_container.py`](../../../simulation/mujoco/src/robot_backend_sim/physics/scene_container.py) | `MjSpec` 编译、offscreen 尺寸、确定性清除运行状态 |
| 机器人装配 | [`physics/marvin/assembly.py`](../../../simulation/mujoco/src/robot_backend_sim/physics/marvin/assembly.py) | 固定根、关节/执行器、手指联动、TCP/指面 site、碰撞排除、具名地址绑定 |
| 任务环境 | [`physics/marvin/environment.py`](../../../simulation/mujoco/src/robot_backend_sim/physics/marvin/environment.py) | 桌面、长方块、开口箱体、相机安装座、灯光 |
| 场景组合 | [`physics/marvin/scene.py`](../../../simulation/mujoco/src/robot_backend_sim/physics/marvin/scene.py) | 按配置选择机器人/任务、编译、打绑、复位、关节夹爪测量 |
| 配置 | [`config/marvin_scene_config.py`](../../../simulation/mujoco/src/robot_backend_sim/config/marvin_scene_config.py) | `robot.assembly` 与 `task.environment` 独立选择、数值校验 |
| 相机声明 | [`config/camera_config.py`](../../../simulation/mujoco/src/robot_backend_sim/config/camera_config.py) | Panda/Marvin 共用的相机声明与摘要输入 |

Panda 侧只做了机制抽取：`physics/scene.py` 现在继承 `CompiledScene` 并把编译/复位交给共享容器；Panda 的模型形状、限位、场景和业务参数未改。Panda 场景/I/O/运动学/可见性测试通过。

## 场景数值与来源

| 参数 | 数值 | 来源/说明 |
| --- | --- | --- |
| 底座位姿 | 位置 `[0,0,0]`，yaw `0°` | 固定根，URDF 根 frame |
| 任务坐标 | `frame: robot_base` | 桌面/物块/箱体/相机安装座写在机器人基座坐标系，装配时按底座位姿变换；任务环境不读取机器人 I/O |
| 桌面 | 中心 `[0.70, 0] m`，尺寸 `0.60 × 1.00 m`，台面高 `0.78 m` | 首版固定布局；P02 确认可达性 |
| 长方块 | 半尺寸 `[0.03, 0.11, 0.03] m`，中心 `[0.55, 0.12] m`，质量 `0.30 kg` | 首版任务环境配方 |
| 开口箱 | 外尺寸 `0.30 × 0.30 m`，高 `0.22 m`，底厚/壁厚 `0.012 m`，中心 `[0.66, -0.27] m` | 底板 + 四壁，无封口 |
| 相机 | `global` 挂 `camera_mount`；`left_wrist` 挂 `Arm_L9_Link`；`right_wrist` 挂 `Arm_R9_Link`；均 320×240 | 固定局部外参；标定摘要见 Profile |
| 复位位姿 | 7 个臂关节全 0，支架分开 | URDF 零位；双臂展开，避让桌面 |
| 臂执行器 | kp `[2500,2500,1500,1500,400,400,400]`，kv `[150,150,90,90,30,30,30]` | P01 静置与低幅运动验证的初始值；P02 复核 |
| 夹爪执行器 | kp `2000`，kv `50`，力限 `±200` | 三态开度与联动验证的初始值 |
| 关节阻尼/armature | `1.0` / `0.01`（臂）；`0.05` / `0.001`（指） | P01 稳定性配置 |

Profile/相机：

- [`profiles/robots/marvin_handover_v1/profile.json`](../../../shared/scenarios/marvin_handover/profiles/robots/marvin_handover_v1/profile.json)，schema v2；`robot_id=marvin_handover`，`profile_id=marvin_handover_v1`。
- 三路标定文件由场景相机声明生成，摘要分别为：
  `global c7af8f72…872e`、`left_wrist afa2783a…7176`、`right_wrist 8e2fda41…0ee2`。
- 观测/动作档案顺序一致：左 7 关节、左夹爪、右 7 关节、右夹爪；20 Hz、50 ms 领先、60 点动作块等学习规格沿用现有 ACT 契约，不在本轮重新定义。

## 碰撞与渲染验证（分别报告）

碰撞：

- 桌面、物块、箱体底板和四壁均 `contype=1, conaffinity=1`；物块静置与桌面形成 4 个接触点。
- 开口验证：从上方向箱体中心发射射线，命中 `box_bottom`（geom 25），距离 `0.308 m`；没有被封口。
- 箱内承载：把物块放入箱内，稳定后中心 `z=0.82189 m`，与箱底形成 4 个接触点。
- 四只手指均为 mesh collision geom（`contype=1, conaffinity=1`）。
- 同一臂内 88 个非父子 link 排除；左右臂之间 0 个排除，双臂互碰保留。
- 底座网格仅渲染、不参与碰撞：源网格 216k 面凸包不是可用接触面，且底座在桌面后方，不参与抓取/放置。差异记录在资产说明中。

渲染与测量证据：

- 最终离线检查：`var/reports/sim-check/p01-final7-20261112/report.json`，状态 `succeeded`，耗时 1.514 s。
- 三路 320×240 PNG：`var/reports/sim-check/p01-final7-20261112/artifacts/renders/{reset,ready}/{global,left_wrist,right_wrist}.png`。
- 渲染器：NVIDIA EGL，`nvidia=true`。
- 复位（双臂展开）时腕相机看不到物块；诊断 ready 位姿下 `global/left_wrist/right_wrist` 的物块像素分别为 `307 / 1365 / 1355`。该 ready 位姿仅用于离线渲染，不是任务抓取位姿。
- 源语义对照：23 个 link，质量最大误差 `0 kg`，主惯量最大误差 `8.88e-16`；总 link 质量 `47.449392 kg`；编译网格 24 个（底座拆分 + 22）。

静置与低幅运动：

- 静置 1000 步（2 s）无 NaN；最大臂关节漂移 `0.00348 rad`（腕关节 7），物块保持在桌面。
- 14 个臂关节各加 `+0.05 rad` 阶跃，实测同向 `0.0465..0.0535 rad`；另一臂同序号关节最大漂移 `0.00029 rad`（腕部 7 为 `0.00348 rad`）。
- 夹爪三态开度、联动方向和闭端误差见上表。

## 回归

历史日志对应以下测试范围；原容器启动命令未完整记录，此处列完整文件路径，不作为当时精确命令或本轮重跑证明。后续交付须记录实际可执行入口。

```text
tests/unit/test_marvin_scene.py
tests/unit/test_scene_config.py
tests/unit/test_profile_cameras.py
tests/unit/test_contract_consistency.py
tests/unit/test_backend_config.py
tests/unit/test_kinematics_config.py
tests/unit/test_gripper_completion.py
tests/unit/test_contact_unload.py
tests/integration/test_sim_scene.py
tests/integration/test_panda_io.py
tests/integration/test_scene_visibility.py
tests/integration/test_scene_kinematics.py
```

结果：229 passed, 2 failed；两条失败均为 P00 已记录的 `test_scene_kinematics.py` 既有测试/API 不一致，与本次改动无关。全仓 simulation 单元测试（除 learning）为 1008 passed, 1 failed, 7 skipped；唯一失败仍是缺少 `robot_ros_support` 的 `test_mcap_feedback_guard.py`。日志：`var/marvin-handover/p01-final7-regression.log`、`var/marvin-handover/p01-full-unit-final.log`。

## 未完成项与 P02 接入点

- 当前离线成果保留；标准控制前先完成 [P01A](../phase-01-readiness.md) 的单份资产归属/路径整理、同臂碰撞排除收敛和 equality 来源复核。下列路径与数值继续描述本次历史检查，后续整改另存报告。
- 未实现标准 Robot API 控制接入、`owner/controller` 的 Marvin I/O、控制/stream/follower 配方；这些属于 P02。
- 未建立 `binding.yaml` 和 runtime/learning 配方；P02～P05 按实际接入补齐。
- 未验证完整抓取/交接/入箱路径可达性；当前抓取区、交接区和箱位只是首版布局。P02 需用实测 IK/净空确认或调整配方。
- 未验证真实手指接触持物；P01 只确认指面碰撞 geom 可用。P02 建立每指/每物块接触事实。
- `ready` 位姿是诊断位姿，不是任务位姿；P02 定义抓取、交接、退避和放置位姿。
- Profile 的臂关节 `max_velocity=0.8 rad/s`、夹爪 `0.04 m/s` 是首版保守执行值，源 URDF 的臂速度上限为 `3.1416 rad/s`。P02 的控制配方需验证这些值并补充加速度、jerk、增益和跟踪阈值。
- 底座网格不参与碰撞；若 P02/P03 需要底座碰撞，应补一个经验证的代理几何，而不是直接使用 216k 面凸包。
- MuJoCo 在内存编译时会打印空前缀附着警告；模型不落盘保存，不影响编译、步进或渲染。
