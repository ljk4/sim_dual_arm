# 阶段 1～2 可维护性审查

审查日期：2026-10-30。范围是当前源码、配置、Docker 入口、测试和已有规划。本文是本次审查记录与整改清单；操作命令仍以 [使用指南](stage2-quickstart.md) 为准，公共职责仍以 [接口设计](architecture-and-interfaces.md) 为准。

结论：四类环境的职责划分和仓库顶层分类可以保留。当前实现适合作为短时 mock 联调基线，但应先完成一次收尾行为修正与小范围职责拆分，再进入阶段 3。主要负担来自 runtime 的职责集中、mock 假设进入公共实现，以及文档与实际入口不一致。单纯减少目录层数或增加注释无法解决这些问题。

本轮只更新审查与规划文档，没有修改运行代码、移动业务文件或实现阶段 3。下列源码问题仍待修复。

## 1. 已执行的验证

| 检查 | 本次结果 | 能说明的范围 |
| --- | --- | --- |
| 当前源码纯 Python 测试 | 34 项通过 | 后端控制核心、观测选择和配置的已有用例 |
| Ruff lint / format | 通过，27 个 Python 文件格式正常 | 格式、导入与已启用的静态规则 |
| 已有 ROS test 镜像 | 39 项通过 | 上述单测及原生 MCAP 的 5 项测试；本次未重建镜像 |
| 真实 ROS runtime 中途 SIGINT | 复现退出码 0、没有 manifest | 人工中断没有等待 episode 收尾 |
| 迟到相机帧的纯 Python 探针 | 同一 observation ID 引用不同帧 | 观测身份不稳定 |
| 控制清理故障注入 | 续租等待超时后未调用 stop/release | 清理步骤之间存在不必要的短路 |
| 配置边界探针 | 30 ms lease 通过配置加载 | 与后端至少 100 ms 的限制不一致 |

39 项包含前述 34 项，不是额外的 39 项。SIGINT 探针使用独立 Docker 网络、mock 后端和挂载的当前 runtime 源码；本次创建的容器与网络已清理。探针及退出证据位于本地 `.build/reports/maintainability-review/`。这些复现说明测试存在缺口，不否定原有成功链路。

## 2. 需要优先修复的行为

这里的 P1 表示应在阶段 3 前处理；P2 表示明确安排的维护问题，具体时限见各项。区分“已经复现”和“从代码确认的设计限制”，不将尚未测试的负载问题描述为实测故障。

### R1 · P1：人工中断绕过 episode 收尾（已复现）

位置：[node.py](../components/runtime/ros/dual_arm_runtime/dual_arm_runtime/node.py)，`_start`、`main`（审查时第 357、529～536 行）。episode 线程是 daemon；主线程接到中断后直接销毁 executor/node，没有等待 worker 完成停止、释放控制权和记录提交。

复现：订阅到第一条动作后等待 0.15 秒，向 runtime 子进程发送 SIGINT。进程返回 0，输出目录只有 `.partial-episode-*/recording/recording_0.mcap`，没有 metadata.yaml 和 manifest.json。人工无法从退出码判断记录未完成，也没有结构化的中断原因。

修复方向：提供明确的 shutdown 入口和有界收尾流程；在通信上下文仍可用时发起停止，等待 worker 和 writer 收尾，最后销毁 ROS 资源。协调 rclpy 的信号处理时机；仅在当前 `finally` 里增加 `join()` 不能保证控制服务仍可调用。中断、收尾失败和正常结束应有明确退出语义。

验收：在 PREPARING、动作执行中和记录收尾时分别中断；不会将不完整运行报告为成功；可写存储上的记录保留失败原因；无活跃 worker 访问已销毁 ROS 对象。SIGKILL/断电恢复是另外的问题。

### R2 · P1：同一个 observation ID 的内容会变化（已复现）

位置：[observation.py](../components/runtime/ros/dual_arm_runtime/dual_arm_runtime/observation.py)，`assemble`；[node.py](../components/runtime/ros/dual_arm_runtime/dual_arm_runtime/node.py)，`_publish_observation`。

最小输入为 state(seq=1, t=100)、frame(seq=1, t=90)，第一次 assemble 得到 observation_id=1、frame_seq=1；随后加入 frame(seq=2, t=95)，再次 assemble 得到 observation_id=1、frame_seq=2。两张图都满足因果时间约束，但身份相同、内容不同。

runtime 仅在 ID 变化时发布/记录 ObservationIndex，却每次更新 `_last_observation`。目前脚本不消费图像载荷；接入推理后，这会使实际请求中的图像与记录索引不一致。

修复方向：确定“一次分配 ID 就冻结全部样本引用”的规则。建议同一 state 首次形成有效观测后固定相机引用；若确实需要重新组装，则必须产生新 ID 并同步更新记录。后续载荷缓存必须按这些固定引用读取。

验收：迟到但合法的相机帧不会改变已有 ID 的内容；记录、发布和调用方读取的是同一组引用。

### R3 · P1：续租清理失败阻止后续停止尝试（故障注入复现）

位置：[node.py](../components/runtime/ros/dual_arm_runtime/dual_arm_runtime/node.py)，`_cleanup_control`（审查时第 481～509 行）。等待已有续租、stop 和 release 共用一个 try；第一步超时就跳过其余步骤，随后清空本地 lease。

故障注入将 `_wait_future` 设为抛出 CALL_TIMEOUT，实际调用的控制服务列表为空。后端仍有 lease 到期保护，但客户端没有主动尝试停止，也难以区分哪些清理步骤成功。

修复方向：每个必要的清理动作独立尝试并记录结果；保留首要失败原因与附加清理错误。通信仍可用时，前一步失败不能直接跳过 stop；上下文已经不可用时应明确记录未能完成的操作。

验收：续租等待失败、stop 失败分别注入，后续适用的清理仍被尝试，最终状态不误报成功。

## 3. 职责与可读性问题

### R4 · P2：RuntimeNode 同时拥有过多生命周期

位置：[node.py](../components/runtime/ros/dual_arm_runtime/dual_arm_runtime/node.py)，542 行。行数本身不是判据；问题是 ROS 注册、跨线程状态、续租、服务等待、episode、脚本动作、记录和退出都由同一对象管理。一次修改很容易跨越多个责任。

当前优点是命名大体明确、协议转换显式、没有复杂继承或动态插件机制。应保留这些特点。建议先提取两个具体模块，仍在现有 Python 包内：

| 模块 | 唯一职责 |
| --- | --- |
| `node.py` | 注册 ROS 接口、转发消息、发布状态，作为组装入口 |
| `episode.py`（拟新增） | 一个 episode 的开始、执行、结束和错误收尾 |
| `control_client.py`（拟新增） | lease、动作调用、停止与释放，封装调用超时 |
| `configuration.py` | 外部配置加载、结构与语义验证、有类型的内部配置 |
| `observation.py` | 固定身份的观测选择，后续按需增加有界载荷缓存 |
| `recording.py` / `audit.py` | 写入与完整性检查，明确区分通用规则和 mock 演示断言 |

不为每个模块创建 Python distribution、ROS 节点或容器。暂不建立 session/manager/service/factory 等多层目录。脚本源只有一份简单实现时可先保留为函数，出现第二种动作源后再提取稳定接口。

拆分时应写清：ROS executor 线程与 episode worker 各自拥有哪些状态；哪个锁保护什么；谁创建/关闭 recorder；什么条件下可以销毁 node。长生命周期状态应有类型，长数据对象采用关键字参数，状态/错误码和时长上限采用具名定义。不要通过统一的“万能上下文”对象重新把所有可变状态连接起来。

### R5 · P2：同步写盘阻塞 ROS 回调，审计占用随整段数据增长

位置：[node.py](../components/runtime/ros/dual_arm_runtime/dual_arm_runtime/node.py) 的 `_record`、SingleThreadedExecutor；[recording.py](../components/runtime/ros/dual_arm_runtime/dual_arm_runtime/recording.py) 的 `append`、文件哈希；[audit.py](../components/runtime/ros/dual_arm_runtime/dual_arm_runtime/audit.py) 的 `states/frames`。

传感器回调直接序列化并写 MCAP，续租定时器在同一个 executor 上运行。慢写盘会延迟续租和状态接收；“回调保持非阻塞”的维护约定尚未落实。离线审计又保留所有完整图像对象，并通过 `read_bytes()` 整文件计算哈希。

这些是从代码确认的限制，本次没有进行大图像/长 episode 负载测试。对当前 64×48、约 2.4 秒脚本影响有限；增加真实相机或长时间运行前需要处理。

建议有界写入队列和单一 writer 所有者，明确溢出与排空失败行为；审计只保留引用检查需要的元数据，文件哈希按块读取。低层 lease 看门狗继续独立存在。先增加一次慢写盘行为验证，再选择队列大小，不预先引入消息中间件。

### R6 · P2：通用名称下包含 mock 专属规则

位置：[engine.py](../libraries/backend_core/src/robot_backend_core/engine.py)、[recording.py](../components/runtime/ros/dual_arm_runtime/dual_arm_runtime/recording.py)、[audit.py](../components/runtime/ros/dual_arm_runtime/dual_arm_runtime/audit.py)、[manifest schema](../contracts/schemas/v1/episode_manifest.schema.json)。

`BackendEngine` 不仅管理 lease，还持有 mock positions、执行线性插值，并使用 `mock-system` 时钟域；目前只有 mock 一个使用方。EpisodeRecorder 和 manifest schema 固定 `source=mock/test_only=true`，审计要求所有命令正常完成且关节必须产生运动。

这些约束适合验证当前演示，但不能直接成为真实/仿真后端或通用数据完整性的定义。静止采集、合法取消、任务失败也可能有完整且可用的记录。

建议现在明确标注 mock 实现边界；接入第二个后端时，再从实际共同需求提取 lease/校验逻辑。将“文件完整性”“运行结果”“任务成功标签”区分开。mock 必须运动的断言归测试验收。暂不为未来后端拆出多个抽象库，也不解除当前 test-only 接入限制。

### R7 · P2：内部配置仍是可变字典，验证规则不一致

位置：[configuration.py](../components/runtime/ros/dual_arm_runtime/dual_arm_runtime/configuration.py)、[runtime schema](../contracts/schemas/v1/runtime_config.schema.json)、[engine.py](../libraries/backend_core/src/robot_backend_core/engine.py)。

Configuration 虽然 frozen，内部 recipe/observation/action/snapshot 仍是可变 dict；业务代码散布字符串键和隐式约束。复现中 lease=30 ms、renew=10 ms 可以通过配置加载，后端 acquire 却规定 lease 至少 100 ms。脚本限位/有限值等也主要延迟到动作执行时检查。

建议保留 JSON Schema 验证外部文件，将已验证配置转换成少量有类型的 dataclass；在启动前检查已知参数冲突，错误包含文件/字段和原因。结构规则、机器人限位和后端能力分别有清晰权威来源。依赖当前测量状态的执行检查仍留在后端。

### R8 · P2：人工操作入口和结束语义不完整

位置：[node.py](../components/runtime/ros/dual_arm_runtime/dual_arm_runtime/node.py) 的 `_end_service`、[EndEpisode.srv](../contracts/ros/robot_api_msgs/srv/EndEpisode.srv)、[启动器](../tools/run/mock_collect.py)。

EndEpisode 只确认进入 ENDING，不能表示已经落盘；它目前总把提前结束记为失败。目标接口文档却写成落盘后返回最终结果。默认启动器执行完即删除容器，只提供自动演示，缺少保留现场、只启动服务、查询状态和人工 start/end 的完整使用路径。

建议在阶段 3 前写清当前异步确认语义，以最终状态/manifest 判断结束，并提供最小人工调试入口。先支持查看状态、启动/结束、保留日志和产物，再考虑 GUI。预期的用户停止与记录损坏应有不同结果描述。

## 4. 目录与规划的调整

顶层 `contracts / components / libraries / profiles / configs / deploy / tools / tests / docs` 的归属基本合理，`var` 和 `.build` 的区分也应保留。ROS 的包目录与同名 Python 包各有作用，不建议只为缩短路径而改布局。

需要收敛的是承诺与权威来源：

| 位置 | 本轮判断与后续规则 |
| --- | --- |
| `contracts/` | 保留唯一协议源；v1 仍是项目原型，不宣称所有目标语义已稳定。补充字段用途、错误含义及当前能力比增加类型更重要 |
| `components/runtime/` | 在原包内拆职责，避免每个功能再加一层包/容器 |
| `libraries/backend_core/` | 当前是 mock 使用的纯 Python 实现；共享价值要在第二个后端出现后验证 |
| `profiles/` | 保留机器人/模型语义；说明时间同步阈值影响观测语义，与运行超时的归属不同 |
| `configs/` | 当前真正使用的配方只有 `runtime/mock_collect.yaml`；先保持这一条加载路径清晰 |
| `deploy/examples/sim_collect.yaml` | 设计示例，没有解析器；不能通过编辑它改变当前演示。之后有实际部署解析需求时再实现多层覆盖 |
| `tools/` | 保留薄构建/启动入口；Docker CLI 当前为主路径。Compose 在验证和维护需求明确前作为可选路径 |
| `tests/` | 当前集中在根目录合理，明确 unit/integration/doubles；不立即迁移到每个组件。将来拆独立发布项目时再评估 |
| `assets/`、`third_party/`、未实现组件 | 有实际文件才建立，不按目标树预建空目录 |
| `docs/` | 区分已实现操作、目标设计、历史材料和一次性审查；当前状态与命令不要多处独立维护 |

发现的文档偏差包括：路线图末尾仍要求开始阶段 1；目录文档仍引用不存在的 `libraries/backend_core/ros`；测试放置规则与实际根目录 tests 不一致；启动器被描述为必须调用 Compose；Policy proto 首行仍写未生成 SDK。前四项已在本轮文档中纠正，proto 注释留待协议维护时同步处理，避免本轮改变协议源摘要。

环境边界继续保留，镜像打包方式可随实际开发调整。当前 requirements-dev 同时给运行镜像提供 gRPC/Protobuf 与测试工具，ROS 包的自身声明又没有完整呈现项目 Python 依赖。下一轮维护应明确系统依赖、项目依赖和开发工具的来源，并提供受支持的开发安装/增量构建路径；不用在每次修改配置时重建全部镜像。现有 Docker 成功构建只能证明该镜像配方可用。

离线边界也需要一个小验证：现在 MCAP 审计依赖 ROS，而规划的 learning 不依赖 ROS。应在大规模数据转换前用一份现有 MCAP 验证独立 Python 环境能解码公共消息；若需要导出步骤，将其明确归属于 ROS 侧。先解决这条实际边界，再扩展数据集转换框架。

## 5. 建议的整改顺序与恢复推进条件

在阶段 2 和阶段 3 之间增加“2R：可维护性与人工操作收敛”，每一步单独审阅：

1. **建立变更基线，修复 R1～R3。** 当前 `git status` 报告此目录不是 Git 仓库；先确认项目真正的版本管理位置或建立可回退基线。本轮没有初始化 Git。补上中断、迟到帧和清理失败的行为测试。
2. **拆分 runtime 的 episode 与 control client。** 尽量保持外部 ROS 接口、当前配置和成功演示行为不变；提取过程中记录线程所有权与资源关闭顺序。不要把修复与全仓库改名混在一起。
3. **补齐人工接手路径。** 有类型的配置、启动前错误、当前状态说明、手动 start/end/查看结果、保留日志与失败产物；提供一条可重复运行的检查入口。以人的实际修改任务验证文档。
4. **恢复阶段 3。** 增加真实 RPC 时，基于稳定的观测引用与单一执行入口接入。慢写盘/记录容量及 mock 断言分离最迟在增大图像、延长 episode 或进入阶段 4 前完成。

允许恢复阶段 3 的标准：已有成功链路仍通过；R1～R3 的新增行为验证通过；维护者能按文档修改一段脚本、运行、查看当前状态、中途结束并找到有原因的记录；能从入口定位到配置、episode、控制和记录四个责任。R5～R8 的剩余限制有明确归属和时限。

Gazebo、机器人选型、LeRobot 依赖和真实相机接入继续保持待定。本次审查不需要重新选择技术栈，也不需要增加通用插件系统或分布式调度层。
