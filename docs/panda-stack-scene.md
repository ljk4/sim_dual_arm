# 双 Panda 堆叠场景模型

2026-10-30。本文件描述 S2b 的场景模型、配置和确定性复位。这里只交付可被后续物理后端、teacher 和场景检查复用的场景对象；ROS、16 维执行映射和 S2 整体验收属于后续任务。三相机静态快照/可见性检查入口见 [仿真环境检查](simulation-environment.md#三相机场景可见性检查)。DSH-017 另在冻结的左臂/手指/物体快照上诊断右臂关键位姿、IK 和离散路径净空，不修改本场景的几何、外参或随机化，详见 [关键位姿、IK、路径净空与视野检查](kinematic-scene-check.md)。

## 配置入口

`configs/simulation/panda_stack.yaml` 是唯一可人工调整的场景参数入口。长度用米、关节角用弧度、配置中的偏航角用度；世界坐标为 +Y 指向桌前、+Z 向上。`components/backend_sim/src/robot_backend_sim/scene_config.py` 在启动前完成解析和校验，错误信息包含文件路径与字段路径。

顶层只允许 `physics`、`column`、`mounts`、`table`、`objects`、`cameras`、`reset` 七节：

| 节 | 主要内容 |
| --- | --- |
| `physics` | 时间步、重力、积分器、物体滑动/滚动/扭转摩擦 |
| `column` | 柱半径、高度、颜色 |
| `mounts` | 左右基座中心、绕 Z 安装角、短支架尺寸、每臂 TCP site 的 hand 局部位置 |
| `table` | 桌面中心与边长、顶面高度、厚度 |
| `objects` | 两个自由物体的半尺寸、名义 XY、质量、颜色、XY/yaw 随机域 |
| `cameras` | 全局与左右腕相机的固定 parent、局部位置、`quaternion_wxyz`、fovy、分辨率 |
| `reset` | 上游 7 维 home 关节角和夹爪总开度 |

默认场景：柱半径 0.12 m、高 1.20 m；基座 x=±0.25 m、y=0、z=1.0 m，绕 Z +90°；桌面中心 (0, 0.65)、尺寸 1.2 × 0.7 m、顶面 0.75 m、厚 0.05 m；4 cm 方块与 6 cm 底块名义 XY 分别为 (0.20, 0.55) 和 (0.36, 0.55)。三路相机默认均为 320×240，固定外参，不使用 `targetbody` 或自动瞄准。

MuJoCo 相机沿局部 -Z 看、局部 +Y 向上。全局相机固定在 `column` 上，默认位置 (0, 0.03, 1.35)，配置中的四元数按固定看向桌区 (0.28, 0.55, 0.78) 预先计算。腕相机是 `left_hand`/`right_hand` 的子相机，局部位置 `[0.10, 0, 0.02]`、`quaternion_wxyz [0, 0.9975640502598242, 0, 0.0697564737441253]`、fovy 85°；光轴大体沿 hand +Z 并向 TCP 倾约 8°，是固定外参，不按对象真值逐帧瞄准。相机分辨率同时写入编译模型的 `cam_resolution`，`visual.global_.offwidth/offheight` 也按配置设为同一上限，渲染不会静默降采样。

## 构建与复位

```python
from robot_backend_sim.scene import build_scene
from robot_backend_sim.scene_config import load_scene_config

config = load_scene_config("configs/simulation/panda_stack.yaml")
scene = build_scene(
    config,
    assets_root=assets_root,
    summary_path=summary_path,
    source_path="third_party/panda-source.json",
)
scene.reset(seed=7)
```

`build_scene` 使用固定来源校验只读资产摘要一次，然后通过原生 `mujoco.MjSpec` attach 两份上游 `panda.xml`，前缀为 `left_`/`right_`。attach 前删除 vendor keyframe，避免单臂 9 维 keyframe 被扩成混合 qpos；机器人网格、tendon、equality 和执行器保持上游定义。自定义几何只有柱、左右短支架、桌、两个自由物体和灯光。

`Scene` 拥有 `model`、`data`、`config` 和按名称构建的 `indices`（关节 qpos 地址、执行器、物体 free joint、相机、TCP site）。模型预期 `nq=32`、`nv=30`、`nu=16`、3 个 named camera、0 个 keyframe；左右各 7 个 arm joint、2 个 finger joint、8 个执行器，两个物体各一个 free joint。

`Scene.reset(seed)` 使用局部 `random.Random(seed)`：`mj_resetData` 后显式清零 `time/qpos/qvel/ctrl/act/qfrc_applied/xfrc_applied/qacc_warmstart`，按具名关节写入 7 个 home 角度、两个 finger 各半的总开度、7 个关节 ctrl 和 0～255 夹爪 ctrl，再按配置和种子写入物体 free qpos，最后 `mj_forward`。相同种子逐位一致；不同种子只在配置的 XY±3 cm、yaw±10° 域内变化。复位不扫描网格、不建渲染器、不调用 IK、不发日志。

夹爪配置单位是两指总开度（米），每个 finger joint 取一半；模型执行器 `actuator8` 使用上游 0～255 无量纲命令，初始化使用连续线性公式 `total_opening / 0.08 * 255`，因此 0.04 m 总开度对应 127.5，不做整数舍入或静默裁剪；输入边界已由配置校验保证。公共 16 维执行映射留给 S3。

home 7 维角度按固定上游 `panda.xml` 的逐关节 `<joint range>` 校验，依次为 joint1 ±2.8973、joint2 ±1.7628、joint3 ±2.8973、joint4 [-3.0718, -0.0698]、joint5 ±2.8973、joint6 [-0.0175, 3.7525]、joint7 ±2.8973 rad；不会用一个统一的关节限值代替。原生测试同时核对这组范围与编译后 `model.jnt_range`。

## 校验边界

配置加载会拒绝未知键、重复 YAML 键、布尔冒充数值、NaN/Inf/超出浮点范围的整数、越界几何、无效四元数、重复物体名、错误的相机 parent、不一致的相机分辨率、非法的 home/夹爪值，以及随机化后越过桌边或两个物体初始可重叠的配置。物体重叠检查逐轴计算名义分离减去两对象各自 jitter 半径后的最小中心距（截零），再与偏航后保守圆形 footprint 比较；不会把两个独立 XY jitter 简化为单个合成半径，也不依赖抽样种子。原生测试在无 MuJoCo 的宿主明确 skip；有 MuJoCo 但资产缺失或摘要不符时失败，不静默跳过。材料、抓取接触和可达性未在本任务验收。
