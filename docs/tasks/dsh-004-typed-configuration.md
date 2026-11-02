# DSH-004：S1a 类型配置与启动前校验

前置：R1～R3 已验收，监督者提供包含这些修改的隔离快照。目标是消除 R7，不同时做节点职责拆分、日志、三相机或 MuJoCo 实现。保持当前 mock 外部文件格式和正常运行结果。

## 范围与阅读

允许修改：

- `components/runtime/ros/dual_arm_runtime/dual_arm_runtime/configuration.py`
- 同目录 `node.py`：仅将配置读取迁移到有类型属性，不改变生命周期/执行行为。
- `contracts/schemas/v1/runtime_config.schema.json`
- `tests/unit/test_configuration.py`
- `tests/integration/test_runtime_cleanup.py`：仅更新测试 fixture 的类型配置用法。
- `docs/stage2-quickstart.md`：只增加配置字段、校验及快照说明。

必要阅读：上述文件；`contracts/python/src/robot_contracts/{models,profiles}.py`；`configs/runtime/mock_collect.yaml`；`profiles/{actions,observations}/*.json`；`libraries/backend_core/src/robot_backend_core/engine.py` 中 lease 与轨迹校验。不要改其他文件、Git 分支/提交、依赖版本或测试入口；不启动其他代理。

## 设计和行为

1. 使用少量明确的 frozen dataclass 表达 RuntimeRecipe、ScriptTarget、ObservationSettings、ActionSettings 和 Configuration。容器使用 tuple，按具名字段构建；业务代码不再用字符串索引 recipe/observation/action。保留 JSON Schema 校验外部文件，不引入 Pydantic 或通用配置框架。
2. Configuration 的运行字段真正不可变。快照仅用于序列化/审计，可以通过 `snapshot` 属性每次返回独立的 JSON-compatible dict；不得通过修改一次返回的嵌套 dict/list 改变之后的快照或运行配置。建议保存规范 JSON 字符串，再解码返回；不要为了兼容实现通用冻结映射。保留原 profile/observation/action/recipe 摘要与外部字段语义。
3. 启动前校验已知约束：lease 在当前后端支持的 100 ms～10 s 内；renew 为正且小于 lease/2；所有浮点参数有限；超时为正；每个脚本目标维度、有限值、关节位置限位正确；各段从 initial 或前一目标计算平均速度，不能超过 profile.max_velocity；command_timeout 必须大于最长脚本段的秒数。不能取消后端的实时校验。
4. 不能把 NaN/Infinity、bool 冒充 number、float 冒充 ns 整数放过。无效 YAML/缺文件/字段错误用 ContractError（SCHEMA_MISMATCH）报告，消息包含来源路径和字段/目标索引。外部观测/动作 schema 的数值也检查有限值；不要通过后来的 semantic_hash ValueError 才暴露错误。
5. 内部类字段明确对应当前已存在的配方/schema，不添加未实现的模拟器或训练参数。对已知后端限制作具名常量与注释；runtime 不导入 backend_core 执行器。将来后端能力协商是另一个阶段。
6. 保持公开输入配置无需改动，快照仍可被现有 recorder 序列化。不要修改已验收的 R1～R3 流程，也不要保留第二套 dict 业务配置来绕过迁移。

## 验收

- 当前合法配置通过，原有快照摘要/字段可用；纯 Python 全量回归与 Ruff 通过。
- 回归测试覆盖：30 ms lease、过大 lease、renew 边界、NaN/Infinity/bool/ns float、维度/限位/段速度、过短 command_timeout、嵌套快照变更不污染配置/下一次快照、带路径和字段的错误。
- 测试验证实际失败行为，避免只复制 dataclass 定义。数据 fixture 在临时目录生成，不改变正式配置。
- 监督者会独立跑原生 ROS/MCAP 和 R1～R3 集成测试，必要时调整 fixture；你不申请 Docker 权限。

使用监督者给定的 PROJECT_PYTHON 和 PROJECT_RUFF，工作目录必须为本 worktree。自测输出保存在 `.build/reports/dsh-004/`。交付说明修改、测试实际结果和未执行项，不能宣称未运行的 ROS 检查通过。首轮上限 15 分钟，一次定点返工上限 10 分钟，由监督进程实施。
