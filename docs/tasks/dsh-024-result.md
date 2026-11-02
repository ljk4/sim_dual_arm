# DSH-024 结果：有界连续动作块执行

2026-11-01，11 个允许文件已验收整合。执行依据为 [任务书](dsh-024-native-stream.md)
和 [连续动作设计](../continuous-action-design.md)。dsh 实施、一次集中返工，监督者接管局部修复后完成验收。

输入快照 `a7a3f521d79d7d9d2ea8cda9cd901d5c20602e57`，headless session
`session-2a400a7a-9139-4f50-8c63-d701b1709cf8`。本地证据位于
`var/runs/dsh/dsh-024/20261101T071430Z/`；`source-first`、`source-rework`、
`source-primary1`、`source-final` 分别保留交付与修复版本，`accepted.patch` 为整合补丁。

## 交付与维护入口

| 文件 | 职责 |
| --- | --- |
| `components/backend_sim/src/robot_backend_sim/stream_config.py` | 严格不可变的提前量、观测年龄、整体/进度/跟踪期限配置 |
| `configs/simulation/panda_stream.yaml` | 连续执行参数；不改变旧控制配置或限位 |
| `components/backend_sim/src/robot_backend_sim/stream_schedule.py` | 一个活动块、一个待执行后继及有界不可变块事件 |
| `components/backend_sim/src/robot_backend_sim/motion.py` | 在现有 NativeMotion 中准入、交接、跟踪、收尾和刹车；共用唯一真实物理步进 |
| `tests/unit/test_stream_config.py`、`test_stream_schedule.py` | 配置、时间与队列边界 |
| `tests/integration/test_native_stream.py` | 原生连续执行、测量、接触、期限与故障回归 |

调用方在物理循环外 `prepare_chunk`，通过 `chunk_anchor` 获取固定交接参考，
`start_stream` 准入首块，`queue_chunk` 提前排队下一块。参考 p/v 连续，块内使用既有
三次计划；不要求加速度跨块连续。默认停稳接口仍可使用。

ACCEPTED、STARTED、RETIRED 对应准入、真实应用、参考窗口退休。RETIRED 不代表停稳或
抓取成功；最终块进入 FINAL_HOLD 后仍须实测稳定才 COMPLETED。缺块、合法候选迟到、
观测过期和持续跟踪失配经有界刹车与实测停止产生故障终态。精确 API 和时钟说明见
[原生运动文档](../native-motion.md)。

## 验收结果

| 检查 | 结果与证据 |
| --- | --- |
| 最终冻结源码 unit | 643 passed，`unit-final.xml` |
| 固定镜像 native | 81 passed、0 skip，含原 65 项及新增 16 项；`native-final/pytest.xml` |
| 独立行为检查 | 18/18，`independent-primary1/result.json` |
| 主目录源码核对 | 112 个 Python 文件与冻结候选逐字节一致，其中 61 个业务文件 |
| 日志审计 | 105 条事件全部符合既有 run_event schema；序号连续、全部写入、零丢弃、关闭为 complete |
| 静态规范 | Ruff lint 通过，6 个变更 Python 文件 format 通过 |

独立检查通过后只增加集成回归与文档；最终 61 个业务 Python 文件与独立检查版本摘要
全部一致，证明在 `independent-final-source-match.json`，无需对同一业务代码重复跑相同探针。

固定原生镜像 ID 为
`sha256:4422f876ae463391f520aacfa3966d8a9c4d9d15a40c7a571cc453133e81b10a`，
实际使用 MuJoCo 3.17.0 / SciPy 1.17.1 / NumPy 1.26.4 和固定 Panda 资产。
ROS IDL 未变，默认 SDK 摘要仍为
`b5a26e7935020a4d3d00a1633dc0f150eedb4ad7f03af7df9ec8cd1f32493e94`，19 文件。

独立物理证据包括：

- 合法 3 s / 右 q1 增量 0.15 rad 轨迹分为三个 1 s 块，原曲线参考误差最大约 7.36e-14；中间退休时机械臂仍在运动。
- 在第二份真实场景逐步重放实际 ctrl，全部 qpos/qvel（含自由物体）一致，最终真实稳定；终态冻结与后续实时测量保持分离。
- 原闭合目标 0、实测约 0.04 m 的自由块夹持可 CONTACT 完成；被故障执行器挡住的张开目标必须跟踪失败。
- 断供、迟到、取消 pending、跨块/最终 SETTLING 的持续跟踪故障、未来/过期观测、错误流/锚点/profile、停钟/停止超时和整体墙钟期限均得到实际验证。
- 控制入口禁用编译器后仍可完整执行；不在 start/queue/step 重新拟合。高频轨迹独立存为诊断 JSONL，阶段/块事件复用 EventLogger。

## 问题与接管记录

首轮 643 unit、73 native 通过，1 项新测试未解包返回值失败；独立 16 组中 11 组通过。
发现流序号只匹配前缀、对象地址重用导致流/事件身份碰撞、queue 未及时落实墙钟期限、
首块缺少最小提前量及接触掩盖张开失败，交给 dsh 唯一一次集中返工。

返工后的 643 unit 仍通过，但 native 为 72 passed / 7 failed，独立为 7/18。
监督者发现接触方法调用和签名不一致、提取的边界辅助函数未接入实际路径，随后直接修复；
同时修正接触测试的初始提前量，将参考选择/首次应用/边界转换/测量后更新分为清楚的小函数，
并让持续跟踪期限跨最终 SETTLING 生效。保留张开失败与跨最终收尾故障回归后，最终
643 unit / 81 native / 18 独立检查通过。原失败版本与输出全部保留。

这一结果说明 host unit 和交付报告不能替代固定镜像原生验收。本轮需要监督者局部接管，
不将 dsh 的完成声明直接当作完成结果。

## 当前边界与执行成本

本项完成无 ROS 的连续物理执行层。公共 ROS ACK/MCAP、命令租约/幂等、reset/epoch、
三图与测量精确配对、fake Policy 与左臂预测守卫仍是 S3 后续门槛；S3 整体尚未验收。
此处接触夹具只在运行前初始化自由物体，不是 teacher，也不计正式 episode。
正式数据仍为 0，ACT 训练与 20 次堆叠评估尚未开始。

dsh 首轮 749.109 s、返工 249.216 s，合计 107 步 / 118 次工具调用；input 95,232、
output 128,181、cacheRead 13,452,288、total 13,675,701 tokens。计费未知，未计算
节省比例。headless 已退出，验收容器按本次完整名称清理；未提交 Git，主 HEAD/暂存区不变。
