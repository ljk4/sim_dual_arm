# DSH-012：S1d 人工操作命令

前置：DSH-011 已验收。用户已要求恢复原规划和协作模式。此任务只完成人工操作；下一任务 DSH-013 再统一自动 collect 与整次运行监督。不要将本任务通过等同于 S1 完成。

## 目标和允许范围

维护者无需宿主机 ROS，即可使用 Compose 开发栈启动服务、查看当前状态、开始一个 episode、结束指定的当前 episode、等待其记录收尾、停止栈。代码直接使用现有公共 API，不复制控制、episode 或 audit 业务。

允许新增：

- `components/runtime/ros/dual_arm_runtime/dual_arm_runtime/cli.py`：小型 ROS 客户端，`python -m dual_arm_runtime.cli`。
- `tools/run/mock.py`：仅标准库的人工主机入口。复用 `dev_compose.py` 的准备/参数转发能力，不维护另一份 Compose 环境/路径/身份校验规则。
- `tests/unit/test_mock_cli.py`、`tests/integration/test_runtime_cli.py`；必要的小型 CLI 专属测试夹具放 `tests/integration/runtime_cli_*.py`。

允许修改：`tools/run/dev_compose.py` 的最小可复用接口；runtime `node.py` 只增加服务启动时初始 IDLE 状态发布；`README.md`、`deploy/README.md`、`docs/stage2-quickstart.md` 的操作说明。不改其他运行业务、协议源、日志/记录库、依赖或 Compose 打包；不重写旧 mock_collect，本任务暂保留它。

必要阅读：上述入口、runtime 的 `_start_service/_end_service/publish_status`，EpisodeRunner 的 start/end/finish，`robot_ros_support.qos`，DSH-010/011 结果。按需读，不遍历所有历史报告。

首轮最多 20 分钟，最多一次 10 分钟定点返工。不运行 Docker、不联网安装依赖、不调用其他代理、不提交 Git。按任务头提供的 Python/Ruff 绝对路径自测，实际 Docker/ROS 验收由监督者进行。报告 `.build/reports/dsh-012/report.md`，列明实际检查与未检查项。

## 主机命令和所有权

命令示例（参数可统一放子命令前，文档必须与实际 argparse 一致）：

```bash
python3 tools/run/mock.py --run-id manual-001 up
python3 tools/run/mock.py --run-id manual-001 status
python3 tools/run/mock.py --run-id manual-001 start --task-id mock-script
python3 tools/run/mock.py --run-id manual-001 end --episode-id <start返回的ID> --reason OPERATOR_STOP --wait
python3 tools/run/mock.py --run-id manual-001 start --task-id mock-script --wait
python3 tools/run/mock.py --run-id manual-001 down
```

公共选项保持 `--project/--image/--build-root/--raw-root/--runs-root/--docker/--run-id`，默认值来自 dev_compose；增加有限正数 `--timeout`（默认 30 s，拒绝 bool/NaN/Inf/非正值）。主机命令使用参数列表，不用 shell，不读取或打印全部环境。单次命令的服务发现、RPC、状态等待共享 monotonic 截止时间；主机外层 subprocess 同样有界，不能每次重试重置总预算。超时明确非零；不能把 RPC 超时解释为请求肯定未执行。

通过 Compose `exec -T runtime bash /workspace/deploy/docker/entrypoints/dev_ros.sh python -m dual_arm_runtime.cli ...` 调用客户端，明确 source ROS/生成 install 和源码路径。不要假定 Docker exec 会继承原 entrypoint 在 shell 中导出的 PYTHONPATH。客户端不触发 dev_compose 的“新 run”检查，不能为了执行 status/start 而新建另一个 runtime。

操作前解析所选 Compose 项目的实际 runtime/backend 容器，并核对 Compose project/service 标签、实际 runtime 命令中的 run id，以及源码、runs、raw 挂载与本次选项相符；不能只按名字模糊匹配。已有项目的其他 run、缺失/多个 runtime、旧镜像路径或不匹配的输出根要拒绝。`up` 若项目已存在容器，明确拒绝，不能替换正在运行的另一 run。新启动只创建父目录，不能预建 runtime 的独占 run 目录。

`up` 成功需 runtime/backend 实际 running，且公共客户端收到 runtime 初始状态、服务可发现；Compose up 退出 0 不等于 ready。启动部分失败时，只停止本次可确认归属的容器，保留输出；所有清理也有期限，不清理其他项目或历史数据。`down` 在正常归属检查后有界停止指定栈，保留挂载产物；重复 down 可幂等报告已停止。该命令只说明停止操作结果，暂不写全局 succeeded。

## 客户端响应与等待语义

- 使用明确的 rclpy Context、SingleThreadedExecutor 和 finally 清理，所有等待都按 monotonic 期限 spin；不要 busy wait。使用已有 INFO_QOS 读取 RuntimeStatus。runtime 初始化后发布一次 IDLE，避免首个 episode 前 status 无值。
- 每次 stdout 恰好输出一个可解析 JSON 对象，诊断走 stderr；统一字段至少包括 `operation/run_id/success/reason`，相关时包含 `episode_id`、`ack`、`status`、`manifest`。CLI 的 `success` 只表示该操作达成所约定边界，不叫 task_success。
- `status` 报告最新 runtime 状态和实际容器存活情况；宿主客户端返回前复查容器状态。缓存的 IDLE 不能证明已退出进程仍存活。容器不存在/已退出/服务不可用时非零且给实际退出码（已知时），不能输出健康。当前状态消息没有心跳时间，文档明确它是最后发布的状态，不虚构新鲜时间或终态。
- `start` 无 `--wait`：返回服务的 PREPARING ACK 和 episode id，不等完成；重复活动 start 由服务拒绝。带 `--wait`：只等待该 ACK episode id 的 IDLE/FAULT，不接受旧 episode 的终态。
- `end` 必须提供非空 `--episode-id`；先取最新活动状态核对目标，错 id 不发送 EndEpisode。成功 ACK 为 ENDING，不能宣称已落盘。带 `--wait` 时继续等待该 id 的终态和可读最终 manifest。
- EndEpisode 现有 wire 不能原子携带 episode id，检查与发送有间隙；本轮不改协议。文档要求人工 start/end 串行操作，不声称能够原子停止并发操作者选定的 episode；观测到 id 切换时报告不确定/不匹配，不能把另一 episode 结果算成成功。
- 终态 manifest 必须来自 `/data/raw/<run_id>/` 内该 episode，核对 run/episode id、schema_version=2 和三维结果；禁止读取 status 给出的任意外部路径/符号链接跳出边界，缺失/损坏/不完整或 FAULT 非零。正常 completed 或人工 stopped 且记录完整可分别作为相应等待操作完成；runtime=failed/interrupted 不算正常完成。task=not_evaluated 原样返回，不推导任务成功。
- CLI 只读取最终 manifest，不冒充独立 MCAP audit；报告明确 `audit_verified=false`，native audit 与全局结果留 DSH-013。读 JSON 有合理大小上限，主机摘要不得携带图像或高频日志。
- 进程/ROS/JSON 错误尽量转为有稳定 reason 的非零 JSON 结果；服务拒绝、超时、目标错配各自可区分。用户可依次完成两个 episode，第二次不会误读上次 IDLE。

## 可读性与验收

小型命名函数或少量明确 dataclass 表达参数、容器身份和客户端结果；不用通用插件框架、不用动态命令分派 eval、不把整个流程塞进一个 main。避免反射任意对象成 JSON、递归自动重试及宽泛异常后继续成功。主机只做部署/公共客户端调用，不能生成动作或直接修改记录。

纯 Python 行为测试应覆盖命令转发与实际参数、fresh-run 检查不误伤查询、项目/run/mount 不匹配拒绝、退出容器不健康、期限/非有限值拒绝、up 部分失败清理范围和 down 幂等。需要替身时按 subprocess 边界注入明确 runner，不以字符串源码检查代替行为。

ROS 行为测试按现有 integration 组织，可在真实 RuntimeNode/公开服务下验证初始 IDLE、start ACK、正常等待、End ACK/人工停止、重复 start、错 episode、不匹配旧终态、超时/服务失联，以及第二次 episode。不要改已有断言或删除测试以通过。监督者另用实际 Compose/mount 运行公开主机命令，核对退出码、manifest、日志、故障与清理。

完成后停止写入，报告变更范围、自测命令/结果、未验证项和已知限制，等待监督验收；不要开始 DSH-013 或 MuJoCo。
