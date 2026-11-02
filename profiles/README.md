# 机器人 profile 与观测/动作语义

`profiles/` 保存可版本化的机器人语义实例；部署地址、容器和设备绑定仍放在
`deploy/`。schema 的唯一维护位置是 `contracts/schemas/`，加载与语义哈希实现位于
`contracts/python/src/robot_contracts/`。

## 目录

| 路径 | 内容 |
| --- | --- |
| `robots/mock_dual_arm_v1/profile.json` | schema v1 历史单相机 CPU mock；`test_only=true` |
| `robots/panda_stack_v1/profile.json` | schema v2 双 Panda、16 维关节、三相机绑定；`test_only=false` |
| `observations/mock_rgb_joint_v1.json` | 旧 mock 的 6 维关节 + 单相机观测绑定 |
| `observations/panda_rgb_joint_v1.json` | 双 Panda 16 维关节 + 三相机观测绑定 |
| `actions/mock_joint_position_v1.json` | 旧 mock 的 6 维 `joint_position` 动作绑定 |
| `actions/panda_joint_position_v1.json` | 双 Panda 16 维 `joint_position` 动作绑定 |

profile 只描述机器人语义，不包含主机路径、容器地址或后端类型。观测/动作文件通过
`validate_bindings` 与 profile 的完整关节顺序和完整相机顺序逐项比对，顺序不同也拒绝。

## schema v1（历史兼容）

v1 仍严格按 `contracts/schemas/v1/robot_profile.schema.json` 读取，保留单个
`camera`、`test_only: true` 和原有字段。加载后内部规范化为一个 `CameraSpec` 的
tuple；语义哈希仍对原始文档计算，因此历史 profile 快照和哈希不变。v1 未声明
`calibration_digest`，内部用空字符串表示“legacy 未知”；mock 为历史回归继续发布既有
校准标识，但 audit 不对 v1 强制校准摘要。未知版本、把 `schema_version=true` 当 1、
重复字段或未知字段都会拒绝。

## schema v2（具名相机集合）

v2 按 `contracts/schemas/v2/robot_profile.schema.json` 读取：

- `cameras` 是 1～8 个具名相机的非空列表，不再有单数 `camera`；`test_only` 必须显式
  为 `true` 或 `false`。
- 每项必须包含 `name`、`width`、`height`、`frame_id`、`calibration_digest`；名称和
  `frame_id` 各自唯一，宽度/高度仍限制在 1～640 / 1～480。相机 `name` 必须是 ROS
  话题 basename：字母或下划线开头，之后仅字母、数字或下划线，最多 64 字符；
  斜杠、空白、点路径等拒绝。
- `calibration_digest` 是 64 位小写 SHA256 十六进制字符串。
- 加载结果 `RobotProfile.cameras` 是唯一的内部相机结构；不存在静默取第一路的
  `.camera` 别名。`camera_names` 返回完整声明顺序。

## 双 Panda 绑定

`panda_stack_v1` 的关节顺序固定为
`[left_joint1..left_joint7, left_gripper_width, right_joint1..right_joint7,
right_gripper_width]`：旋转关节单位 rad，夹爪是两指总开度，单位 m，范围 0～0.08。
关节范围、home 位姿与已验收 scene 配置的 `PANDA_JOINT_RANGES` / reset home 一致；
两指各自的测量/命令映射由后续物理 backend 实现，本 profile 不假装已可执行物理采集。
首版 `max_velocity` 是工程上限：臂 0.8 rad/s、夹爪 0.04 m/s，不是厂商性能标定值。

三相机顺序固定为 `global`、`left_wrist`、`right_wrist`，相机名同时就是
`~/cameras/<name>/frame` 的话题段，对应光学 frame 为
`global_optical_frame`、`left_wrist_optical_frame`、`right_wrist_optical_frame`，
均为 320×240。runtime 按 profile 顺序订阅、组帧和记录；BackendInfo、ObservationIndex
和 audit 都必须使用同一完整顺序。

## 三相机校准摘要

`panda_stack_v1` 的摘要按当前已验收场景文件中每个相机的固定相对安装/针孔描述计算，
不使用主机绝对路径，也不让 `contracts` 依赖 MuJoCo：

```python
{
    "name": "<camera name>",
    "parent": "<scene body>",
    "position": [x, y, z],
    "quaternion_wxyz": [w, x, y, z],
    "fovy": <degrees>,
    "width": <pixels>,
    "height": <pixels>,
}
```

键名与 `configs/simulation/panda_stack.yaml` 的 camera 项一致，数组保留原始浮点值。
对上述对象执行 `robot_contracts.profiles.semantic_hash`（排序键、紧凑分隔符的规范 JSON
后取 SHA256），即得到 `calibration_digest`。当前三路摘要分别对应场景中的
`global` / `left_wrist` / `right_wrist` 固定外参和 60°/85° 针孔视场。

场景加载会在 `CameraConfig.declared_description_json` 中保存上述声明的不可变规范 JSON，
保留 YAML 原始数字类型和数值；`CameraConfig` 的有效 `quaternion_wxyz` 仍是归一化后的
场景值。摘要按声明计算，不按归一化后的有效四元数重算；消费方还会核对声明重新解析后
与有效相机字段完全一致，防止只替换 typed 几何却沿用旧声明。

contracts 只定义和校验摘要值；后续物理 backend 负责在发布前确认实际相机安装、
内参、分辨率与 profile 描述一致，并在不一致时拒绝启动或报告新的标定版本。contracts
不读取 MuJoCo、相机设备或厂商 SDK。
