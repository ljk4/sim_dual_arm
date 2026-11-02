# DSH-018：S3 三相机 profile、runtime 接线与原始记录

2026-11-01。S2 已验收；本任务是 S3 的首项，不是物理控制或学习实现。按原协作模式使用独立 worktree/headless session。首轮25分钟，最多一次10分钟返工；交付报告写好后立即结束，不探索缓存环境。监督者负责真实 Jazzy/MCAP/多容器验收。

## 目标及行为边界

把当前写死单相机的 profile/订阅/记录/审计扩展为具名相机集合，并提供双 Panda 的16维/三相机绑定。旧单相机mock和历史记录继续可读。**保持 runtime 的 test-only mock 限制**，不把它放开成物理后端；MuJoCo接入和测量反馈在下一项实现。

当前 `ObservationBuffer` 已接受多相机tuple，ROS ObservationIndex 已有 camera_names/frame_seqs 数组，不必修改ROS类型。仍采用当前latest-state/causal-frame规则，本项不宣称“延迟渲染的精确快照状态配对”已解决；S3物理适配前必须独立完善该规则。本任务不改控制插值、租约、时钟、Policy或teacher。

## 文件范围

允许修改：
- `contracts/python/src/robot_contracts/models.py`、`profiles.py`；
- runtime `configuration.py`、`node.py`、`audit.py`；
- `tests/doubles/ros/mock_robot_backend/mock_robot_backend/node.py`；
- `tests/integration/test_recording.py`、`test_recording_backpressure.py`；
- `tests/unit/test_configuration.py` 及确实受profile调用变更影响的已有测试；不删旧断言；
- `contracts/README.md`、`profiles/README.md`（不存在则新增）、`docs/architecture-and-interfaces.md`、`docs/episode-recording-format.md`、docs/README、README 的当前能力/边界说明。

允许新增：
- `contracts/schemas/v2/robot_profile.schema.json`；
- `profiles/robots/panda_stack_v1/profile.json`；
- `profiles/observations/panda_rgb_joint_v1.json`、`profiles/actions/panda_joint_position_v1.json`；
- `tests/unit/test_profile_cameras.py`、`tests/integration/test_multicamera_recording.py`。

不改v1 schema/现有mock profile JSON/ROS类型/依赖锁/镜像/Compose/Scene/日志与队列库，不新增通用插件框架，不复制生成契约。不要修改其他工作树、安装依赖、运行Docker、申请权限、启动其他代理或提交Git。需要额外生产文件先在报告说明，不偷偷扩范围。

## 领域模型与配置

1. 新版 profile schema_version=2，保留 robot_id/profile_id/joints/test_only；使用 `cameras` 非空列表（最多8），不再是单个camera。test_only允许明确true或false。camera含name/width/height/frame_id/calibration_digest（SHA256），具名相机和frame_id均唯一，未知字段/重复字段/非法数值或重复名字清楚拒绝。width/height沿用可支持的有界范围，当前三路320×240；不得通过降低分辨率冒充支持。
2. 内部 `RobotProfile.cameras: tuple[CameraSpec,...]` 是唯一相机结构，`CameraSpec`含calibration_digest。v1 load时规范化成一元素tuple，保留**原文档semantic hash**；v1校准摘要未声明可用空字符串表示legacy未知。不要保留会对多相机静默取第一路的 `.camera`别名。所有现有调用方同步迁移。
3. v1严格按原v1 schema读取，v2按新版schema读取，未知版本拒绝。错误含来源文件/字段；不可把schema_version=true当1。profile加载后不可被外部原始dict修改。
4. `validate_bindings`校验完整相机顺序和16维关节顺序。configuration目前先硬编码验证v1 schema再load_profile，需要移除这一重复单版本假设；保持recipe/observation/action其他校验不变，不绕过校验或重写snapshot中的原profile文档。
5. 双Panda绑定使用 `[left_joint1..7, left_gripper_width, right_joint1..7, right_gripper_width]`，关节rad、夹爪总开度m（0～.08），两指测量映射留物理后端。关节范围/home与已验收scene配置/PANDA_JOINT_RANGES一致；max_velocity首版工程值可用臂.8rad/s、夹爪.04m/s，说明这是工程上限。test_only=false；此时只验证绑定，不提供假装可执行的物理采集recipe。
6. 三相机顺序global/left_wrist/right_wrist，光学frame_id具名唯一。校准摘要按 `semantic_hash` 计算当前场景camera描述：name/parent/position/quaternion_wxyz/fovy/width/height（键名与scene_config一致；数组保持原始浮点值）。描述从已验收`configs/simulation/panda_stack.yaml`读取；profiles文档解释此固定相对安装/针孔描述的算法与后续backend一致性校验责任，不硬编码主机路径，也不让contracts依赖MuJoCo。

## ROS、mock、记录与审计

- runtime订阅/记录每个具名camera话题，回调绑定明确相机名，不能有循环lambda晚绑定；BackendInfo检查完整顺序，_topics涵盖全部流。三路frame_seq/frame_id/calibration/图像内容不能串路，ObservationIndex包含全部有效引用。
- mock能按profile发布一或三路流，保持test_only=true，拒绝test_only=false profile，不能拿Panda非测试profile跑伪物理数据。v2使用camera声明的calibration_digest；v1保持既有mock校准行为，便于历史回归。每路使用各自尺寸/frame_id，并能在测试中证明载荷没有全部被替换成一路。
- audit从snapshot原profile规范化获取全部相机，按话题/相机名验证每路尺寸、frame_id、校准（v2）、时间/seq与observation引用；漏一路、未知/重复名字、错校准/跨相机引用等均失败。不是只数总图像条数或检查文件存在。
- 保持MCAP流式读取，不积累图像数组；已有队列上限/记录结果分类/关闭规则不变。不改历史v1 profile快照或manifest，也不把静止左臂判成坏数据。
- 不引入第二套camera解析逻辑：领域模型/profile层提供小而清楚的规范化函数供配置和审计复用，避免runtime倒依赖后端或工具脚本。不要为一个循环建立多层类或泛型注册器。

## 自测和验收

自测用PROJECT_PYTHON/PROJECT_RUFF，至少覆盖：v1 hash/单相机兼容、v2不可变多相机、未知版本/bool/重复相机/未知字段、Panda16维/SI及相机顺序、绑定不匹配拒绝。真实ROS/MCAP集成测试由监督者运行，宿主缺ROS可以skip相应测试；不运行旧缓存native或反复安装。

新增集成测试使用明确test_only=true的三相机fixture，真实消息写入/读取MCAP并audit；验证全部引用和三路内容，以及缺路/错frame_id/错校准等反例。fixture可在tmp_path生成，不添加第二个生产假Panda配置。旧单相机collect、v1/v2历史manifest读取和backpressure必须回归。

监督者另用挂载源码的真实公开采集入口验收三相机mock及audit（不当作正式50条数据），核对完整配置/日志/真实退出与清理；通过后才整合。交付 `.build/reports/dsh-018/report.md`，区分实际host自测与待监督者ROS检查，写明未完成项。一次必要unit/Ruff后立即交付，不把后续物理控制偷加进本任务。
