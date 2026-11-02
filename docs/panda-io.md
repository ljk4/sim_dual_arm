# 双 Panda 16 维 SI 执行器与测量映射

2026-11-01。DSH-020 新增 `components/backend_sim/src/robot_backend_sim/panda_io.py`，把已验收的
`panda_stack_v1` profile 绑定到实际双 Panda `Scene`/`MjModel`。它只是 S3 物理接入的
具名执行器/测量层，不是 ROS 后端、轨迹调度器、reset/lease 实现或控制核心。

## 使用

```python
from robot_backend_sim.panda_io import PandaIO
from robot_backend_sim.scene import build_scene
from robot_backend_sim.scene_config import load_scene_config
from robot_contracts.profiles import load_profile

scene = build_scene(config, assets_root=..., summary_path=..., source_path=...)
profile = load_profile(profile_path, schemas_root)
io = PandaIO(scene, profile)       # 构造时校验完整绑定

measurement = io.measure()         # 只读实际 qpos/qvel
io.apply_targets(targets)          # 只写实际 actuator ctrl
```

`measurement` 是不可变 `NamedTuple`：`sim_time_ns`、`joint_names`、`positions`、
`velocities`。每次调用返回独立 tuple；后续 `mj_step`/reset 不会改写已返回的测量。
`positions`/`velocities` 不裁剪，超出 profile 限位时仍如实返回，便于后续控制核心判断
跟踪误差和故障；只有非有限值或非法时间才明确失败。

`apply_targets` 只接受完整 16 维 SI 目标。所有校验先完成，再写 ctrl；任何拒绝都不会
留下部分写入。成功也只表示 16 个目标已写入实际 actuator，不表示到达或跟踪完成。

## 固定顺序与单位

| 下标 | 名称 | 目标/测量单位 | 执行器 |
| --- | --- | --- | --- |
| 0～6 | `left_joint1`～`left_joint7` | rad | `left_actuator1`～`left_actuator7` |
| 7 | `left_gripper_width` | m（两指总开度） | `left_actuator8`（raw 0～255） |
| 8～14 | `right_joint1`～`right_joint7` | rad | `right_actuator1`～`right_actuator7` |
| 15 | `right_gripper_width` | m（两指总开度） | `right_actuator8`（raw 0～255） |

关节速度对应 rad/s；夹爪速度是 `finger_joint1` 与 `finger_joint2` 的 qvel 之和，单位
m/s。夹爪位置同样是两指实际 qpos 之和，不使用目标或上一次命令代替测量。

夹爪总开度 `0～0.08 m` 连续映射到 vendor 的 raw `0～255`；`0.04 m` 恰好为 `127.5`，
不做整数化。转换函数公开为 `robot_backend_sim.scene.gripper_ctrl`，场景 reset 与
`PandaIO` 共用同一实现。

## 构造校验

`PandaIO(scene, profile)` 不保存目标覆盖测量，也不依赖当前 profile 哈希或主机路径。
构造时检查：

- profile 必须是 16 项完整顺序，名称和单位逐项匹配；范围、max_velocity 和 home 与
  `SceneConfig`/`PANDA_JOINT_RANGES`/`FINGER_JOINT_LIMIT` 一致。
- 14 个臂关节、4 个手指关节和 16 个 actuator 全部按完整名称取索引，并分别核对
  qpos 地址、dof 地址、actuator 驱动对象、joint/actuator 原生范围，以及夹爪 actuator
  的 raw `0～255` ctrlrange。scene 已缓存的具名索引也必须与重新按名称解析的结果一致。
- profile 的三相机顺序、尺寸和校准摘要与 `SceneConfig` 一致；摘要对
  `CameraConfig.declared_description_json` 保存的原始 YAML 声明计算，复用
  `robot_contracts.profiles.semantic_hash`。有效 `CameraConfig` 四元数仍是解析后
  归一化的场景值；声明与有效字段必须经同一个单相机校验器互相核对，因此
  `dataclasses.replace` 只改几何却留下旧声明会被拒绝。声明保留原始整数/浮点类型和
  数值，不保留可被外部改写的 dict。MuJoCo 场景没有 ROS 光学 frame 对象，
  `PandaIO.camera_frame_ids` 只发布经过 profile 校验的 frame ID，不声称验证了场景中
  不存在的 frame。
- 模型、profile、相机或 actuator 绑定不一致时抛出带稳定 reason 的 `ContractError`，
  不会构造出半可用对象。

## 线程与所有权

`MjData` 不是线程安全的。`PandaIO` 保存创建它的 Thread 对象；`measure`/`apply_targets` 在
其他线程调用会以 `WRONG_THREAD` 拒绝，即使创建者退出后底层线程 ident 被复用。这是本对象的约束；S2 的 `Scene` 工具仍可独立
使用。整个物理后端的数据所有权（唯一物理循环、ROS 回调转发、reset 和队列）由后续
控制核心落实。

状态读取和 ctrl 写入不调用 `mj_step`、reset、FK 或渲染，不写 qpos/qvel、物体位姿或
外力，也不修改模型增益、重力、接触参数或相机。插值、速度/加速度调度、跟踪完成判定、
事件队列和 lease 均不在本模块内。

## 稳定拒绝 reason

| reason | 含义 |
| --- | --- |
| `PROFILE_MISMATCH` | profile 关节顺序、单位、范围、home 或速度上限不匹配 |
| `MODEL_MISMATCH` | 场景/模型名称、关节类型、原生范围、地址、actuator 或索引不一致 |
| `CAMERA_MISMATCH` | 相机顺序、尺寸或校准摘要与 `SceneConfig` 不一致 |
| `INVALID_TARGET` | 目标长度、类型、bool 或非有限值非法 |
| `JOINT_LIMIT` | 目标超出 profile 限位；不静默裁剪 |
| `INVALID_MEASUREMENT` | 测量时刻或实际 qpos/qvel 非有限/非法 |
| `WRONG_THREAD` | 在创建线程之外调用 `measure`/`apply_targets` |

## 验证与边界

原生集成测试为 `tests/integration/test_panda_io_native.py`，使用固定 Panda assets、
summary 和 `configs/simulation/panda_stack.yaml`，在缺少 MuJoCo 的宿主按既有约定 skip。
监督者在固定 MuJoCo 3.17 镜像内执行。测试覆盖真实初始测量、具名 qpos/dof 地址、两指
求和、不可变快照、非对称 ctrl 映射、写入前后无 physical state 改变、真实 `mj_step`
后测量变化，以及空载小幅右臂/夹爪响应的有限性和稳定性。它不证明抓取、堆叠或整个 S3。

本模块没有实现 ROS 发布、reset/epoch、lease、插值、跟踪完成判定、teacher 或 Policy；
即使 ctrl 写入成功，也仍需下一项物理控制核心验证实际测量跟随。


编译后的相机还逐项核对父体、具名索引、固定模式、分辨率、fovy和本地位姿，
避免有效配置正确而实际渲染模型已经偏离。执行器须使用当前Panda的单位传动gear，
阻止原始ctrl虽在范围内却改变rad/m语义。独立原生验证结果见
[DSH-020验收记录](tasks/dsh-020-result.md)。
