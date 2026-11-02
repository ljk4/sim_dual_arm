# DSH-018：三相机配置、接线与记录已验收，暂停

2026-11-01。对应 [任务书](dsh-018-multicamera-profiles.md)。19 个允许文件已整合，主暂存区未变，未提交 Git。**按用户要求在本任务完成后暂停**，未分派后续任务。

## 本项完成

- RobotProfile v2 表达具名相机列表；内部统一使用不可变 `cameras` tuple，v1 规范化成单元素 tuple，原文档 semantic hash 保留。没有静默取第一路的 `.camera` 别名。
- 新增双 Panda 的16维 SI profile、三相机 observation/action 绑定与相对安装校准摘要；相机名为话题 basename。Panda profile 的 test_only=false，当前 mock 明确拒绝它。
- runtime 订阅/记录完整相机集合，回调显式绑定路名，BackendInfo 与 ObservationIndex 校验完整顺序。mock 支持多路测试流，保留 test-only 限制。
- MCAP 审计逐路核对 RGB 载荷、步长、frame ID、CameraInfo、校准、序号、时间和观测引用；复用 profile 层相机规范化，仍流式读取。v1历史空记录/仅状态诊断的既有语义保留。

人工阅读：先看 `profiles/README.md` 与 `profiles/robots/panda_stack_v1/profile.json`；领域类型/解析在 `contracts/python/src/robot_contracts/{models,profiles}.py`，ROS接线在 runtime/node.py，帧校验集中在 audit.py 的 `_validate_camera_frame`。

## 实施与修复

本地 headless CLI session `session-7d80d095-eef2-4d3a-8b1c-902856acfc66`。首轮00:18:33开始，397.99s正常结束；唯一返工00:40:33开始，89.61s正常结束，均exit0。

首轮真实公开三相机采集通过，但独立边界/集成验收发现：超大数转换泄漏OverflowError、路径型相机名被接受、旧空/仅状态记录变成partial、新测试把ROS array.array放入set，以及v2步长/CameraInfo不一致被错误接受。一次dsh返工修复，新增相应测试；未降低完整三路要求，也未改旧失败断言来通过。

主助手仅格式化三个Python文件，格式化前后AST完全相同，证据已保存；没有生产逻辑接管。主工作区整合后的Python源码与最终公开验收夹具逐文件一致。

## 独立验收

| 检查 | 结果 |
| --- | --- |
| 纯Python / 静态检查 | 478 unit通过；Ruff lint/format通过 |
| 实际Jazzy/MCAP集成 | 104项通过，无skip；含旧记录、背压、关闭、CLI与新增三相机反例 |
| 独立profile检查 | v1哈希、Panda16维/SI/home/限位/三路校准与场景一致；11项配置/绑定反例通过 |
| 最终公开三相机mock采集 | exit0，日志/审计/清理通过；独立读MCAP为每路52帧、52个完整观测引用，三路内容不同 |
| 旧单相机公开采集 | exit0，MCAP审计与清理通过 |
| 独立元数据反例 | 错RGB步长、CameraInfo尺寸、CameraInfo坐标系均真实写MCAP并被拒绝 |
| 历史v1证据 | 6个原文件hash未变；完整记录可读，原失败记录与未知版本继续拒绝 |
| 非测试profile保护 | 实际mock CLI拒绝Panda非测试profile，返回UNSUPPORTED_CAPABILITY |

首次验收把Pillow场景单测放进不含该依赖的ROS开发镜像，且自动pytest插件使MuJoCo缺依赖的模块skip扩散到目录。该设置错误已保留，未计产品失败。最终按依赖分组、明确ROS测试文件并设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，实际执行104项；没有将skip当作通过，也没有为测试临时修改镜像依赖。

公开三相机验收使用隔离源码副本，Python文件与冻结候选逐文件hash一致；仅把既有mock配置切为明确test_only=true的三相机fixture，未改候选或生产默认配置。该数据不计正式50条。

证据根：`var/runs/dsh/dsh-018/20261031T161801Z/`。保留首轮/返工、源码归档及accepted.patch，验收见 `acceptance/profile-final`、`ros-final`、`metadata-final`、`public-frozen`、`legacy-public-final`、`legacy-fixtures-final`、`backend-guard-final` 和 `format-ast-equivalence.json`。

两轮dsh共487.60s、96 steps、131 tool calls；回报输入103,591、输出72,207、cache-read10,117,120、total10,292,918 tokens。缓存量不代表新生成量或实际费用；主助手费用未知，不计算节省比例。

## 暂停位置

S2已验收；S3只完成本项三相机/16维配置与mock记录接线。runtime仍为test-only；MuJoCo物理后端、测量与参考分离、插值/反馈/query/reset、精确快照状态配对、Policy RPC尚待后续任务。正式数据与ACT训练/推理未开始。

下次从 [S3接口与物理控制核对](../simulation-control-design.md) 的剩余项继续，不把本项mock数据当作物理执行或学习闭环证据。
